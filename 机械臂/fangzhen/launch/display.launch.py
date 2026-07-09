import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 包名和文件名 (请确保你的实际包名叫 fangzhen，且 URDF 文件名叫 jixiebi.urdf)
    package_name = 'fangzhen'
    urdf_name = 'jixiebi.urdf'
    
    # 获取包的安装路径
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    # 假设你把 rviz 配置放在了 rviz 文件夹下，如果没有该文件，这行可能会报错
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz') 

    # 读取 URDF 文件内容
    with open(urdf_model_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 启动 robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # 启动 joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # 启动 rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        )
    ])