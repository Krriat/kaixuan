from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package = Path(get_package_share_directory('xiaogong_kaixuan'))
    description = (package / 'urdf/xiaogong-kaixuan.urdf').read_text(encoding='utf-8')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', parameters=[{'robot_description': description}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(package / 'config/display.rviz')]),
    ])
