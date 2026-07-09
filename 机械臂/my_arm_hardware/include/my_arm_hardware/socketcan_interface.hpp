#ifndef MY_ARM_HARDWARE__SOCKETCAN_INTERFACE_HPP_
#define MY_ARM_HARDWARE__SOCKETCAN_INTERFACE_HPP_

// 这些是 ros2_control 要求的标准系统接口头文件
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>

namespace my_arm_hardware
{

// 继承自 SystemInterface，代表这是一个完整的系统级硬件接口
class SocketCanArmInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SocketCanArmInterface)

  // 1. 初始化函数：系统启动时调用，用于分配内存、解析 URDF 模型配置
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  // 2. 导出状态接口：把电机真实的当前位置（我们从 CAN 读上来的）暴露给系统
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  // 3. 导出指令接口：接收系统（MoveIt）计算好的目标位置
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // 4. 激活函数：点击 start 时调用。我们要在这里打开底层的 SocketCAN 通信
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  // 5. 停用函数：关闭底层的 CAN 通信
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  // 6. 主循环 - 读：以 100Hz 的频率疯狂调用，从 CAN 总线上抓取电机发回的状态
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  // 7. 主循环 - 写：以 100Hz 的频率疯狂调用，把目标位置发给 CAN 总线
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int sock_ = -1; // Linux 下 SocketCAN 的文件描述符，类似于一个文件句柄

  // 这两个数组非常关键！它们就是 ROS 2 系统和底层硬件交互的“内存共享区”
  // hw_commands_ 存放大脑下发的目标；hw_states_ 存放电机真实的反馈
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // 减速比：电机侧编码器值 -> 关节角度，J1 到 J6
  std::vector<double> reduction_ratios_;
};

}  // namespace my_arm_hardware

#endif  // MY_ARM_HARDWARE__SOCKETCAN_INTERFACE_HPP_