#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import ikpy.chain
import numpy as np

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander_node')
        
        # 1. 挂载 Action Client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        # 2. 载入机械臂的物理模型 (请确保路径正确)
        # 如果你的底座不需要参与 IK 计算（比如固定在地面的底座），可以将 active_links_mask 配置为 True/False 数组
        urdf_path = "/home/sunrise/YYC/jixiebi/jixiebi_ws/src/my_arm_hardware/urdf/jixiebi.urdf"
        self.arm_chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
        self.get_logger().info(f"成功加载机械臂模型，包含 {len(self.arm_chain.links)} 个连杆/关节")

    def calculate_ik(self, x, y, z):
        """
        逆运动学核心函数：输入目标坐标(米)，输出关节角度(弧度)
        """
        target_position = [x, y, z]
        
        # 调用 ikpy 的底层求解器
        # 默认它不仅考虑位置，还会尽量保持末端姿态，这里我们先写死只管位置
        ik_solution = self.arm_chain.inverse_kinematics(target_position)
        
        # ⚠️ 避坑指南：
        # ikpy 返回的数组包含了虚拟的 base_link 和末端 tcp_link，所以长度通常比 6 个电机多。
        # 对于标准的 6 轴 URDF，真正的 1~6 号电机角度通常在索引 1 到 6。
        motor_angles = ik_solution[1:7].tolist() 
        
        self.get_logger().info(f"目标坐标: {target_position} -> 结算角度: {motor_angles}")
        return motor_angles

    def grab_target(self, x, y, z, duration_sec=3):
        """
        组合动作：算 IK -> 发送执行指令
        """
        self.get_logger().info(f'准备抓取坐标: X={x}, Y={y}, Z={z}')
        
        # 翻译坐标为角度
        target_angles = self.calculate_ik(x, y, z)
        
        # 下面是我们之前写好的发包逻辑
        self._action_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = target_angles
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('轨迹被底层拒绝！')
            return
        self.get_logger().info('正在移动机械臂前往目标坐标...')

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()

    # 假设目标在正前方 0.3 米，偏左 0.1 米，高度 0.2 米的位置
    node.grab_target(x=0, y=0.1, z=0.2, duration_sec=4)


    rclpy.spin(node)

if __name__ == '__main__':
    main()