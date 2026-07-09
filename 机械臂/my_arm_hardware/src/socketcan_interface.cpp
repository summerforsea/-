#include "my_arm_hardware/socketcan_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp" 

// 引入 Linux 底层操作网卡和 CAN 总线所需的 C 语言库
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>

namespace my_arm_hardware
{

// 【初始化】系统启动时执行一次
hardware_interface::CallbackReturn SocketCanArmInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // 根据 URDF 里写了几个关节，就给我们的数组分配几个位置，并初始化为 0.0
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // 减速比：电机侧编码器 -> 关节侧角度，J1~J6
  reduction_ratios_ = {-30.0, -50.0, 30.0, 30.0, 30.0, 30.0};

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 【状态导出】告诉系统，真实位置数据存放在哪里
std::vector<hardware_interface::StateInterface> SocketCanArmInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // 绑定内存地址：把 hw_states_[i] 的内存地址交给 ROS 2
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

// 【命令导出】告诉系统，目标指令应该写到哪里
std::vector<hardware_interface::CommandInterface> SocketCanArmInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // 绑定内存地址：把 hw_commands_[i] 的内存地址交给 ROS 2
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

// 【激活硬件】打开底层的 CAN 接口，并发送使能指令
hardware_interface::CallbackReturn SocketCanArmInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  struct sockaddr_can addr;
  struct ifreq ifr;

  // 1. 创建原生的 Linux Socket
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  std::strcpy(ifr.ifr_name, "can0"); // 你的硬件网卡名
  ioctl(sock_, SIOCGIFINDEX, &ifr);
  addr.can_family = PF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(sock_, (struct sockaddr *)&addr, sizeof(addr)); // 绑定网卡

  // 2. 极其重要：设置为非阻塞模式！
  // 这样如果读不到 CAN 报文，程序也不会卡死在这里，保证高频控制的流畅度
  int flags = fcntl(sock_, F_GETFL, 0);
  fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

  // 3. 根据你的协议，给每一个电机发送 Enable (使能) 指令，唤醒电机
  for (size_t i = 0; i < info_.joints.size(); i++) {
    uint32_t node_id = i + 1; // 假设你的电机拨码 ID 分别是 1, 2, 3...
    struct can_frame frame;
    
    // 协议规定：高 4 位是 NodeID，低 7 位是指令 0x01 (Enable)
    frame.can_id = (node_id << 7) | 0x01; 
    frame.can_dlc = 8; // 数据长度永远是 8 字节
    
    uint32_t enable_cmd = 1; // 发送 1 代表使能
    std::memcpy(&frame.data[0], &enable_cmd, sizeof(uint32_t)); // 安全拷贝进 data
    send(sock_, &frame, sizeof(struct can_frame), 0); // 发射！
  }
  
  // 等待 50 毫秒，给步进电机驱动板一点反应时间
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 【停用硬件】安全退出
hardware_interface::CallbackReturn SocketCanArmInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (sock_ >= 0) close(sock_); // 关闭文件描述符，释放硬件资源
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 【主循环 - 读】高频执行，把电机发给 PC 的报文拆解出来
hardware_interface::return_type SocketCanArmInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  struct can_frame frame;
  
  // 只要网卡缓冲区里有数据，就一直死循环读，直到读空为止
  while (recv(sock_, &frame, sizeof(struct can_frame), 0) > 0) {
    
    uint8_t cmd = frame.can_id & 0x7F;        // 用位操作分离出低 7 位的指令码
    uint32_t node_id = frame.can_id >> 7;     // 用位操作分离出 NodeID
    
    // 如果收到的是 0x23 (Get Position 的回复报文)
    if (cmd == 0x23) { 
      int joint_index = node_id - 1; // ID 1 对应数组的第 0 位
      
      // 防止越界导致的段错误
      if (joint_index >= 0 && joint_index < static_cast<int>(info_.joints.size())) {
        float current_position = 0.0f;
        // 把 data 里的前四个字节，拼成一个 float 浮点数
        std::memcpy(&current_position, &frame.data[0], sizeof(float));
        
        // 电机侧编码器值 / 减速比 = 关节角度，这样 ROS 2 看到的就是真实的关节位置
        hw_states_[joint_index] = (static_cast<double>(current_position) / reduction_ratios_[joint_index])*2.0*M_PI; // 转成弧度

        // 调试：每 5 秒打印一次原始电机值和换算后的关节角度，方便确认单位
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);

        // 2. 传入这个本地时钟对象
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("SocketCanArmInterface"),
          steady_clock, 5000,
          "READ Motor[%d]: raw=%.4f -> joint=%.4f rad (%.1f deg)",
          node_id, current_position, hw_states_[joint_index],
          hw_states_[joint_index] * 180.0 / M_PI);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

// 【主循环 - 写】高频执行，把 PC 算好的目标发给电机
hardware_interface::return_type SocketCanArmInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 遍历每一个关节
  for (size_t i = 0; i < info_.joints.size(); i++) {
    uint32_t node_id = i + 1; 
    struct can_frame frame;
    
    // 协议规定：指令 0x05 是 Set Position (设置目标位置)
    frame.can_id = (node_id << 7) | 0x05; 
    frame.can_dlc = 8;
    
    // 关节角度 * 减速比 = 电机目标位置，因为电机在减速器前端
    float target_position = static_cast<float>((hw_commands_[i] / (2.0 * M_PI)) * reduction_ratios_[i]);
    
    // 放入 data[0] 到 data[3]
    std::memcpy(&frame.data[0], &target_position, sizeof(float));
    
    // 协议精髓：把第五个字节设为 1，电机收到后会立刻把它的当前位置回复给我们！
    frame.data[4] = 1; 
    
    // 后面的字节清零，保持干净
    frame.data[5] = 0; frame.data[6] = 0; frame.data[7] = 0;
    
    // 发射给总线
    send(sock_, &frame, sizeof(struct can_frame), 0);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace my_arm_hardware

// 宏定义：把上面写的这个类，正式注册为一个能够被独立调用的插件
PLUGINLIB_EXPORT_CLASS(
  my_arm_hardware::SocketCanArmInterface, hardware_interface::SystemInterface)