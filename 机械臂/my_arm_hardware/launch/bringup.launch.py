import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_path = get_package_share_directory('my_arm_hardware')
    urdf_file = os.path.join(pkg_path, 'urdf', 'jixiebi.urdf')
    yaml_file = os.path.join(pkg_path, 'controllers.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # ==========================================
    # 新增：打开 Linux 底层 can0 网卡
    # ==========================================
    # 注意：这里假设你的步进电机波特率是 1 Mbps (1000000)。如果是 500k，请改为 500000。
    setup_can_cmd = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'],
        output='screen'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, yaml_file],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        setup_can_cmd,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])