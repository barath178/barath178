"""RViz: view UR5 fixed manipulator and move every joint with sliders.

    ros2 launch ur5_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("ur5_description")
    robot_description = xacro.process_file(
        os.path.join(pkg, "urdf", "ur5.urdf.xacro")
    ).toxml()

    gui = LaunchConfiguration("gui")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="joint_state_publisher_gui sliders"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(pkg, "config", "display.rviz")]),
    ])
