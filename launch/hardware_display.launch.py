import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "biped_bike_robot"
    pkg_share = get_package_share_directory(package_name)

    urdf_file = os.path.join(pkg_share, "urdf", "biped_bike_robot.urdf")
    rviz_config = os.path.join(pkg_share, "config", "rviz_config.rviz")
    dxl_config = os.path.join(pkg_share, "config", "dynamixel_hardware.yaml")

    with open(urdf_file, "r", encoding="utf-8") as infp:
        robot_description_config = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "torque_on_start",
                default_value="true",
                description="Enable torque on all configured position motors.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="true",
                description="Send tick 2048 to all position motors when the bridge starts.",
            ),
            DeclareLaunchArgument(
                "max_abs_position_rad",
                default_value="0.35",
                description="Ignore GUI joint commands whose absolute position exceeds this limit.",
            ),
            DeclareLaunchArgument(
                "log_joint_states",
                default_value="true",
                description="Log mapped joint commands for debugging.",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_commands",
                default_value="true",
                description="Forward /joint_states GUI commands to Dynamixel motors.",
            ),
            DeclareLaunchArgument(
                "enable_trajectory_commands",
                default_value="true",
                description="Forward JointTrajectory commands to Dynamixel motors.",
            ),
            DeclareLaunchArgument(
                "enable_velocity_commands",
                default_value="true",
                description="Forward wheel velocity commands to Dynamixel motors.",
            ),
            DeclareLaunchArgument(
                "max_wheel_velocity_rad_s",
                default_value="2.0",
                description="Clamp each physical wheel command to this speed.",
            ),
            DeclareLaunchArgument(
                "wheel_command_timeout_sec",
                default_value="0.5",
                description="Stop wheels when velocity commands stop arriving.",
            ),
            DeclareLaunchArgument(
                "log_telemetry",
                default_value="false",
                description="Log Dynamixel PWM/load/voltage/position telemetry to CSV.",
            ),
            DeclareLaunchArgument(
                "telemetry_log_path",
                default_value="",
                description="CSV path for Dynamixel telemetry. Empty uses src/biped_bike_robot/motor_logs.",
            ),
            DeclareLaunchArgument(
                "telemetry_rate_hz",
                default_value="5.0",
                description="Dynamixel telemetry logging rate.",
            ),
            DeclareLaunchArgument(
                "telemetry_duration_sec",
                default_value="10.0",
                description="Stop telemetry logging after this many seconds. Use 0 to log until shutdown.",
            ),
            DeclareLaunchArgument(
                "telemetry_motor_ids",
                default_value="",
                description="Comma-separated Dynamixel IDs to log. Empty logs all active motors.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description_config,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package=package_name,
                executable="dxl_joint_state_bridge.py",
                name="dxl_joint_state_bridge",
                output="screen",
                parameters=[
                    {
                        "config_path": dxl_config,
                        "torque_on_start": LaunchConfiguration(
                            "torque_on_start"
                        ),
                        "center_on_start": LaunchConfiguration(
                            "center_on_start"
                        ),
                        "max_abs_position_rad": LaunchConfiguration(
                            "max_abs_position_rad"
                        ),
                        "log_joint_states": LaunchConfiguration(
                            "log_joint_states"
                        ),
                        "enable_joint_state_commands": LaunchConfiguration(
                            "enable_joint_state_commands"
                        ),
                        "enable_trajectory_commands": LaunchConfiguration(
                            "enable_trajectory_commands"
                        ),
                        "enable_velocity_commands": LaunchConfiguration(
                            "enable_velocity_commands"
                        ),
                        "max_wheel_velocity_rad_s": LaunchConfiguration(
                            "max_wheel_velocity_rad_s"
                        ),
                        "wheel_command_timeout_sec": LaunchConfiguration(
                            "wheel_command_timeout_sec"
                        ),
                        "log_telemetry": LaunchConfiguration("log_telemetry"),
                        "telemetry_log_path": LaunchConfiguration(
                            "telemetry_log_path"
                        ),
                        "telemetry_rate_hz": LaunchConfiguration(
                            "telemetry_rate_hz"
                        ),
                        "telemetry_duration_sec": LaunchConfiguration(
                            "telemetry_duration_sec"
                        ),
                        "telemetry_motor_ids": LaunchConfiguration(
                            "telemetry_motor_ids"
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
