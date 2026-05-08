"""Launch travel_node with the bundled KITTI parameter file.

Usage:
  ros2 launch travel_ros travel_run.launch.py

  # Override the input topic from the command line:
  ros2 launch travel_ros travel_run.launch.py input_topic:=/velodyne_points

  # Disable the auto-launched RViz pane:
  ros2 launch travel_ros travel_run.launch.py rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("travel_ros")
    default_params = PathJoinSubstitution([pkg_share, "config", "kitti_params.yaml"])
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "travel.rviz"])

    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/kitti/points",
        description="Source PointCloud2 topic feeding TRAVEL.",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to a ROS 2 parameter file for travel_node.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Auto-launch RViz with travel.rviz config.",
    )

    travel_node = Node(
        package="travel_ros",
        executable="travel_node",
        name="travel_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("~/input", LaunchConfiguration("input_topic"))],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="travel_rviz",
        output="log",
        arguments=["-d", default_rviz],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        input_topic_arg,
        params_arg,
        rviz_arg,
        travel_node,
        rviz_node,
    ])
