"""Spawn the mobile manipulator in Gazebo (gz sim) with ros2_control.

ros2 launch tb3_omx_description gazebo.launch.py
Controllers: joint_state_broadcaster, diff_drive_controller, arm_controller, gripper_controller
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('tb3_omx_description')
    xacro_file = os.path.join(pkg, 'urdf', 'tb3_omx.urdf.xacro')
    world = LaunchConfiguration('world')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true']), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', world]}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'tb3_omx',
                            '-x', '0', '-y', '0', '-z', '0.01'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                             '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                             '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'], output='screen')

    jsb = spawner('joint_state_broadcaster')
    after_jsb = [spawner('diff_drive_controller'), spawner('arm_controller'), spawner('gripper_controller')]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty.sdf')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),
        gz_sim, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=after_jsb)),
    ])
