"""Gazebo (gz sim) : spawn UR5 fixed manipulator in the material-handling world
with ros2_control controllers.

    ros2 launch ur5_description gazebo.launch.py
    ros2 launch ur5_description gazebo.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("ur5_description")
    controllers = os.path.join(pkg, "config", "ros2_controllers.yaml")
    world = os.path.join(pkg, "worlds", "material_handling.sdf")
    robot_description = xacro.process_file(
        os.path.join(pkg, "urdf", "ur5.urdf.xacro"),
        mappings={"controllers_file": controllers},
    ).toxml()

    # let Gazebo resolve package://ur5_description/meshes/...
    resource_path = os.path.dirname(pkg)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world}", "on_exit_shutdown": "true"}.items(),
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}],
               output="screen")
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "ur5",
                            "-x", "0", "-y", "0", "-z", "0", "-Y", "0"])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    jsb = spawner("joint_state_broadcaster")
    rest = [spawner(c) for c in ['arm_controller', 'gripper_controller']]

    rviz = Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", os.path.join(pkg, "config", "gazebo.rviz")],
                parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        gz_sim, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=rest)),
        rviz,
    ])
