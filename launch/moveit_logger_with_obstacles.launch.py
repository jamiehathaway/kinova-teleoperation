from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")

    use_kortex_api = LaunchConfiguration("use_kortex_api")
    output_csv = LaunchConfiguration("output_csv")
    num_random_targets = LaunchConfiguration("num_random_targets")
    samples_per_target = LaunchConfiguration("samples_per_target")
    sample_period_ms = LaunchConfiguration("sample_period_ms")
    settle_time_ms = LaunchConfiguration("settle_time_ms")

    logger_launch = Node(
        package="kinova_teleoperation",
        executable="kinova_moveit_current_logger",
        output="screen",
        parameters=[
            {
                "move_group_name": "arm",
                "use_kortex_api": use_kortex_api,
                "robot_ip": robot_ip,
                "output_csv": output_csv,
                "num_random_targets": num_random_targets,
                "samples_per_target": samples_per_target,
                "sample_period_ms": sample_period_ms,
                "settle_time_ms": settle_time_ms,
                "enable_environment_obstacles": True,
                "obstacle_frame": "base_link",
                "obstacle_apply_wait_ms": 1000,
                # Table + 3 walls around the back and sides.
                "table.size": [1.2, 1.2, 0.06],
                "table.pose": [0.0, 0.0, -0.09],
                "back_wall.size": [0.06, 1.2, 1.0],
                "back_wall.pose": [-0.55, 0.0, 0.50],
                "left_wall.size": [1.2, 0.06, 1.0],
                "left_wall.pose": [0.0, 0.55, 0.50],
                "right_wall.size": [1.2, 0.06, 1.0],
                "right_wall.pose": [0.0, -0.55, 0.50],
            }
        ],
    )

    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kinova_gen3_lite_moveit_config"),
                    "launch",
                    "robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.2.10"),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("use_kortex_api", default_value="false"),
            DeclareLaunchArgument(
                "output_csv",
                default_value="/workspaces/kinova-teleoperation/log/moveit_current_log.csv",
            ),
            DeclareLaunchArgument("num_random_targets", default_value="20"),
            DeclareLaunchArgument("samples_per_target", default_value="50"),
            DeclareLaunchArgument("sample_period_ms", default_value="20"),
            DeclareLaunchArgument("settle_time_ms", default_value="300"),
            moveit_stack,
            logger_launch,
        ]
    )
