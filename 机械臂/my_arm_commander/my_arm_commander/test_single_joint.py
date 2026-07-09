#!/usr/bin/env python3
"""
单关节测试脚本：
只控制 J1，命令它转 0.5 rad（关节角度）。
如果减速比和单位都对，J1 关节应该实际转动 0.5 rad ≈ 28.6°
如果转的角度不对，根据实际转了多少反推单位。
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SingleJointTest(Node):
    def __init__(self):
        super().__init__('single_joint_test')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.get_logger().info('等待 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('已连接！')

    def send_joint_command(self, joint_name, angle_rad, duration_sec=2):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [angle_rad]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'发送命令: {joint_name} = {angle_rad:.4f} rad ({angle_rad*180/3.14159:.1f} deg)')
        self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = SingleJointTest()

    import time
    # 先归零（确保电机在安全位置）
    node.get_logger().info('=== 归零 ===')
    node.send_joint_command('joint1', 0.0, duration_sec=2)
    time.sleep(3)

    # 测试：命令关节转 0.5 rad ≈ 28.6°
    # 如果减速比和单位正确，J1 输出端应该转 28.6°
    # 电机原始命令 = 0.5 * 30 = 15（如果单位是弧度）
    node.get_logger().info('=== 测试: J1 目标 0.5 rad ===')
    node.send_joint_command('joint1', 0.5, duration_sec=2)
    time.sleep(3)

    # 回到 0
    node.get_logger().info('=== 归零 ===')
    node.send_joint_command('joint1', 0.0, duration_sec=2)
    time.sleep(3)

    # 测一下 π rad = 180°（电机那边应该转 30*3.14 = 94.2 如果单位是弧度）
    node.get_logger().info('=== 测试: J1 目标 3.14 rad (180 deg) ===')
    node.send_joint_command('joint1', 3.14159, duration_sec=3)
    time.sleep(4)

    node.get_logger().info('=== 归零 ===')
    node.send_joint_command('joint1', 0.0, duration_sec=2)

    rclpy.spin(node)

if __name__ == '__main__':
    main()
