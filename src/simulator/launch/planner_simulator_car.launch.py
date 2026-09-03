import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue  # 导入ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --------------------------
    # 1. 基础路径配置（原有仿真配置）
    # --------------------------
    simulator_package_dir = get_package_share_directory('simulator')
    yaml_filename = os.path.join(simulator_package_dir, 'maps', 'c.yaml')
    rviz_config = os.path.join(simulator_package_dir, 'rviz', 'planner.rviz')
    lifecycle_nodes = ['map_server']

    # --------------------------
    # 2. 汽车模型（URDF）配置（使用ParameterValue方式）
    # --------------------------
    car_description_dir = get_package_share_directory('car_description')
    default_urdf_path = os.path.join(car_description_dir, 'urdf', 'car_model.urdf')
    
    # 声明URDF路径参数
    declare_urdf_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='URDF模型文件的绝对路径'
    )
    
    # 第二种方式：用ParameterValue显式声明类型为字符串（推荐）
    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),  # 读取URDF文件内容
        value_type=str  # 明确指定参数类型为字符串
    )

    # --------------------------
    # 3. 节点配置（合并所有节点）
    # --------------------------
    return LaunchDescription([
        declare_urdf_arg,  # 声明URDF路径参数

        # 原有仿真节点
        Node(
            package='simulator',
            executable='clock_node',
            name='clock',
            output='screen'
        ),
        Node(
            package='simulator',
            executable='transform',
            parameters=[{'use_sim_time': True},
                        {'pose_x': 0.0}, {'pose_y': 0.0}, {'pose_yaw': 0.0}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': yaml_filename},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_simulator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        ),
        Node(
            package='simulator',
            executable='planner_bridge',
            name='planner_bridge',
            parameters=[
                {'use_sim_time': True},
                {'planner_id': "custom"}
            ],
            output='screen'
        ),

        # 汽车模型相关节点（使用显式类型的robot_description）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},  # 引用显式类型的参数
                {'use_sim_time': True}
            ]
        )
    ])
