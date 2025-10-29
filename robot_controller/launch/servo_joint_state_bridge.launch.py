from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='True'
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='False'
    )

    use_rviz = LaunchConfiguration('rviz')

    # Use robot_description package to get xacro and rviz config
    robot_desc_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(robot_desc_share, 'urdf', 'differential_drive_robot_version1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(robot_desc_share, 'config', 'display.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    servo_bridge_node = Node(
        package='robot_controller',
        executable='servo_joint_state_bridge',
        name='servo_joint_state_bridge',
        parameters=[
            {'bind_host': '0.0.0.0'},
            {'bind_port': 3333},
            {'left_joint_name': 'Revolute 2'},
            {'right_joint_name': 'Revolute 4'},
            {'angle_units': 'degrees'},
            {'left_sign': 1.0},
            {'right_sign': 1.0},
        ]
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        gui_arg,  # retained if you wish to mirror the display.launch style
        robot_state_publisher_node,
        servo_bridge_node,
        rviz_node,
    ])
