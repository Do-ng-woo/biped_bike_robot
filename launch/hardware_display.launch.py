import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
                "torque_off_on_shutdown",
                default_value="true",
                description="Disable torque when the hardware bridge exits.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="false",
                description="Send tick 2048 to all position motors when the bridge starts.",
            ),
            DeclareLaunchArgument(
                "startup_ready_posture_on_start",
                default_value="true",
                description="Send a forward-lean ready posture to position motors when the bridge starts.",
            ),
            DeclareLaunchArgument(
                "startup_forward_lean_deg",
                default_value="10.0",
                description="Deprecated; startup ready now uses the shared ready posture.",
            ),
            DeclareLaunchArgument(
                "startup_shoulder_pitch_deg",
                default_value="-70.0",
                description="Shoulder pitch angle for startup ready posture.",
            ),
            DeclareLaunchArgument(
                "max_abs_position_rad",
                default_value="2.2",
                description="Ignore GUI joint commands whose absolute position exceeds this limit.",
            ),
            DeclareLaunchArgument(
                "log_joint_states",
                default_value="true",
                description="Log mapped joint commands for debugging.",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_commands",
                default_value="false",
                description="Forward /joint_states GUI commands to Dynamixel motors.",
            ),
            DeclareLaunchArgument(
                "use_joint_state_gui",
                default_value="false",
                description="Start joint_state_publisher_gui for manual RViz sliders.",
            ),
            DeclareLaunchArgument(
                "publish_present_joint_states",
                default_value="true",
                description="Publish actual Dynamixel present positions as /joint_states.",
            ),
            DeclareLaunchArgument(
                "present_joint_state_rate_hz",
                default_value="15.0",
                description="Actual Dynamixel joint state publish rate.",
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
            DeclareLaunchArgument(
                "enable_opencr_imu",
                default_value="false",
                description="Read the OpenCR virtual IMU sensor from Dynamixel ID 200.",
            ),
            DeclareLaunchArgument(
                "opencr_imu_rate_hz",
                default_value="30.0",
                description="OpenCR IMU publish rate.",
            ),
            DeclareLaunchArgument(
                "opencr_imu_topic",
                default_value="/opencr/imu",
                description="Topic for OpenCR sensor_msgs/Imu output.",
            ),
            DeclareLaunchArgument(
                "opencr_imu_frame_id",
                default_value="body_link",
                description="Frame ID for OpenCR IMU messages.",
            ),
            DeclareLaunchArgument(
                "enable_imu_tf",
                default_value="false",
                description="Rotate base_link in RViz using /opencr/imu.",
            ),
            DeclareLaunchArgument(
                "imu_mount_roll_deg",
                default_value="-90.0",
                description="Roll correction from OpenCR IMU frame to base_link.",
            ),
            DeclareLaunchArgument(
                "imu_mount_pitch_deg",
                default_value="0.0",
                description="Pitch correction from OpenCR IMU frame to base_link.",
            ),
            DeclareLaunchArgument(
                "imu_mount_yaw_deg",
                default_value="0.0",
                description="Yaw correction from OpenCR IMU frame to base_link.",
            ),
            DeclareLaunchArgument(
                "imu_rpy_remap",
                default_value="YRp",
                description="RPY remap after mount correction. Uppercase reverses sign.",
            ),
            DeclareLaunchArgument(
                "imu_pivot_x",
                default_value="-0.066549",
                description="Base-link pivot x used for IMU visualization.",
            ),
            DeclareLaunchArgument(
                "imu_pivot_y",
                default_value="-0.076779",
                description="Base-link pivot y used for IMU visualization.",
            ),
            DeclareLaunchArgument(
                "imu_pivot_z",
                default_value="0.018299",
                description="Base-link pivot z used for IMU visualization.",
            ),
            DeclareLaunchArgument(
                "imu_zero_yaw_on_start",
                default_value="true",
                description="Use the startup IMU yaw as RViz forward direction.",
            ),
            DeclareLaunchArgument(
                "imu_yaw_zero_samples",
                default_value="10",
                description="Number of first IMU samples used for yaw zeroing.",
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
                condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
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
                        "torque_off_on_shutdown": LaunchConfiguration(
                            "torque_off_on_shutdown"
                        ),
                        "center_on_start": LaunchConfiguration(
                            "center_on_start"
                        ),
                        "startup_ready_posture_on_start": LaunchConfiguration(
                            "startup_ready_posture_on_start"
                        ),
                        "startup_forward_lean_deg": LaunchConfiguration(
                            "startup_forward_lean_deg"
                        ),
                        "startup_shoulder_pitch_deg": LaunchConfiguration(
                            "startup_shoulder_pitch_deg"
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
                        "publish_present_joint_states": LaunchConfiguration(
                            "publish_present_joint_states"
                        ),
                        "present_joint_state_rate_hz": LaunchConfiguration(
                            "present_joint_state_rate_hz"
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
                        "enable_opencr_imu": LaunchConfiguration(
                            "enable_opencr_imu"
                        ),
                        "opencr_imu_rate_hz": LaunchConfiguration(
                            "opencr_imu_rate_hz"
                        ),
                        "opencr_imu_topic": LaunchConfiguration(
                            "opencr_imu_topic"
                        ),
                        "opencr_imu_frame_id": LaunchConfiguration(
                            "opencr_imu_frame_id"
                        ),
                    }
                ],
            ),
            Node(
                package=package_name,
                executable="imu_base_tf.py",
                name="imu_base_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_imu_tf")),
                parameters=[
                    {
                        "imu_topic": LaunchConfiguration("opencr_imu_topic"),
                        "parent_frame": "world",
                        "child_frame": "base_link",
                        "mount_roll_deg": LaunchConfiguration("imu_mount_roll_deg"),
                        "mount_pitch_deg": LaunchConfiguration("imu_mount_pitch_deg"),
                        "mount_yaw_deg": LaunchConfiguration("imu_mount_yaw_deg"),
                        "rpy_remap": LaunchConfiguration("imu_rpy_remap"),
                        "pivot_x": LaunchConfiguration("imu_pivot_x"),
                        "pivot_y": LaunchConfiguration("imu_pivot_y"),
                        "pivot_z": LaunchConfiguration("imu_pivot_z"),
                        "zero_yaw_on_start": LaunchConfiguration(
                            "imu_zero_yaw_on_start"
                        ),
                        "yaw_zero_samples": LaunchConfiguration(
                            "imu_yaw_zero_samples"
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
