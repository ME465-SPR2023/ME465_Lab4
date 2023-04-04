from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    sim_share = get_package_share_directory("ME465_Sim")
    description_share = get_package_share_directory("ME465_Description")
    lab4_share = get_package_share_directory("ME465_Lab4")
    slam_share = get_package_share_directory("ME465_SLAM")
    return LaunchDescription([
        DeclareLaunchArgument(
            name="visualization",
            default_value="true",
            description="Run Rviz",
        ),
        DeclareLaunchArgument(
            name="node",
            default_value="true",
            description="Run the lab 4 node",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "load-map-launch.py"),
            ),
            launch_arguments={"map_file": os.path.join(slam_share, "map", "sim_edited.yaml")}.items(),
        ),
        Node(
            package="ME465_Lab4",
            executable="lab4_node",
            condition=IfCondition(LaunchConfiguration("node")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(lab4_share, "lab4.rviz")],
            output="own_log",
            condition=IfCondition(LaunchConfiguration("visualization")),
        ),
    ])
