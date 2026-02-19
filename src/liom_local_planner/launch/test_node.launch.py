from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('liom_local_planner')
    yaml_filename = os.path.join(package_dir, 'maps', 'map.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'planner.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
        
        Node(
            package='liom_local_planner',
            executable='liom_test_node',
            name='liom_test_node',
            output='screen'
        )
    ])