import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("drok_real_arm_bridge")
    mapping = os.path.join(share, "config", "real_mapping.yaml")

    dry_run = LaunchConfiguration("dry_run")
    default_max_speed = LaunchConfiguration("default_max_speed")

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("default_max_speed", default_value="30"),

        # Proven legacy feedback reader. Its incorrect old /joint_states output is remapped away.
        Node(
            package="drok_real_arm_bridge",
            executable="motor_angle_publisher",
            name="motor_angle_publisher",
            output="screen",
            remappings=[("/joint_states", "/legacy_joint_states_unused")],
        ),

        # Publish logical joints using the inverse of the verified real command mapping.
        Node(
            package="drok_real_arm_bridge",
            executable="real_joint_state_mapper.py",
            name="real_joint_state_mapper",
            output="screen",
            parameters=[mapping],
        ),

        # Exact real-robot command mapping from DROK_ARM_EEcontrol/main.
        Node(
            package="drok_real_arm_bridge",
            executable="moveit_to_rmd_bridge.py",
            name="moveit_to_rmd_bridge",
            output="screen",
            parameters=[mapping, {
                "dry_run": ParameterValue(dry_run, value_type=bool),
                "default_max_speed": ParameterValue(default_max_speed, value_type=int),
            }],
        ),
    ])
