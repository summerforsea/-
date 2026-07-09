#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

// ROS 2 基础依赖
#include "rclcpp/rclcpp.hpp"
// 标准 CAN 消息依赖
#include "can_msgs/msg/frame.hpp"

// Linux SocketCAN 原生头文件
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

class SocketCANNode : public rclcpp::Node
{
public:
    SocketCANNode() : Node("socket_can_node")
    {
        // 1. 初始化 SocketCAN (参考你提供的示例)
        setup_socket_can();

        // 2. 创建发布者：将收到的 CAN 数据发布到 ROS 话题
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);

        // 3. 创建订阅者：将收到的 ROS 话题数据发送到 CAN 总线
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "can_tx", 10, std::bind(&SocketCANNode::can_tx_callback, this, std::placeholders::_1));

        // 4. 开启独立线程接收 CAN 数据（因为 read 是阻塞的）
        rx_thread_ = std::thread(&SocketCANNode::receive_thread, this);
    }

    ~SocketCANNode()
    {
        if (rx_thread_.joinable()) rx_thread_.join();
        close(sock_);
    }

private:
    int sock_;
    std::thread rx_thread_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    void setup_socket_can()
    {
        struct sockaddr_can addr;
        struct ifreq ifr;

        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 创建 SocketCAN 原生套接字
        strcpy(ifr.ifr_name, "can0"); // 动态获取接口名
        ioctl(sock_, SIOCGIFINDEX, &ifr); // 获取接口索引

        addr.can_family = PF_CAN; // 把网卡绑定到套接字上
        addr.can_ifindex = ifr.ifr_ifindex;
        bind(sock_, (struct sockaddr *)&addr, sizeof(addr));
        
        RCLCPP_INFO(this->get_logger(), "SocketCAN 初始化成功");
    }

    // 接收线程：CAN -> ROS 2
    void receive_thread()
    {
        struct can_frame frame;
        while (rclcpp::ok()) {
            int nbytes = read(sock_, &frame, sizeof(struct can_frame));
            if (nbytes > 0) {
                auto msg = can_msgs::msg::Frame();
                msg.header.stamp = this->now();
                msg.id = frame.can_id;
                msg.dlc = frame.can_dlc;
                std::copy(frame.data, frame.data + 8, msg.data.begin());
                can_pub_->publish(msg);
            }
        }
    }

    // 回调函数：ROS 2 -> CAN
    void can_tx_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        struct can_frame frame;
        frame.can_id = msg->id;
        frame.can_dlc = msg->dlc;
        std::copy(msg->data.begin(), msg->data.end(), frame.data);
        
        if (write(sock_, &frame, sizeof(struct can_frame)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 发送失败");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SocketCANNode>());
    rclcpp::shutdown();
    return 0;
}