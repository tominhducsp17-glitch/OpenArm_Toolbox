"""Start node "trajectory" (q, dq reference) and node "controller" (calls openarm_control, outputs tau_ff).

    ros2 launch openarm_controller trajectory_controller.launch.py [trajectory_file:=/path/file.csv] [controller_type:=...]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "trajectory_file",
            default_value=PathJoinSubstitution([FindPackageShare("openarm_trajectory"), "trajectories", "tissue_wipe_x2.csv"]),
            description="CSV: t, q1..q7, dq1..dq7[, ddq1..ddq7] of one arm"),
        DeclareLaunchArgument("arm", default_value="right", description="arm of the trajectory file"),
        DeclareLaunchArgument("rate_hz", default_value="400.0"),
        DeclareLaunchArgument("controller_type", default_value="ctc_feedforward"),
        DeclareLaunchArgument(
            "controller_params",
            default_value=PathJoinSubstitution([FindPackageShare("openarm_controller"), "config", "controller.yaml"])),
    ]
    arm = LaunchConfiguration("arm")
    trajectory = Node(
        package="openarm_trajectory", executable="trajectory_node", name="trajectory", output="screen",
        parameters=[{
            "trajectory_file": LaunchConfiguration("trajectory_file"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "joint_names": [["openarm_", arm, f"_joint{j}"] for j in range(1, 8)],
        }])
    controller = Node(
        package="openarm_controller", executable="controller_node", name="controller", output="screen",
        parameters=[LaunchConfiguration("controller_params"),
                    {"controller_type": LaunchConfiguration("controller_type")}])
    return LaunchDescription(args + [trajectory, controller])
