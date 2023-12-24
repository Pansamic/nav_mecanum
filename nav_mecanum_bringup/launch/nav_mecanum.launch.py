from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("nav_mecanum_description"), "urdf", "nav_mecanum.urdf.xacro"]
            )
        ]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("nav_mecanum_bringup"), "config", "ekf.yaml"])
    nav_mecanum_kernel_params = PathJoinSubstitution(
        [FindPackageShare("nav_mecanum_bringup"), "config", "params.yaml"]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("nav_mecanum_description"), "rviz", "nav_mecanum.rviz"]
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"]
            )
        )
    )

    kernel_node = Node(
        package="nav_mecanum_bringup",
        executable="nav_mecanum_kernel",
        name="nav_mecanum_kernel",
        parameters=[nav_mecanum_kernel_params],
        output="both"
    )

    localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom/filtered")]
        )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    return LaunchDescription(
        [
            kernel_node,
            robot_state_pub_node,
            sllidar_launch,
            localization_node
        ]
    )
