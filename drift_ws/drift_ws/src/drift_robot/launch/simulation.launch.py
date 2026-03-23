import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Logs go to ~/Desktop/drift_assignment_ws/logs/ regardless of where the
# launch file is installed (install/ tree vs source tree).
_LOG_DIR = os.path.join(
    os.path.expanduser("~"), "Desktop", "drift_assignment_ws", "logs"
)
_METRICS_CSV = os.path.join(_LOG_DIR, "tidy_metrics.csv")
_BAG_DIR = os.path.join(_LOG_DIR, "tidy_bag")


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("drift_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    world = PathJoinSubstitution([package_share, "worlds", "home.world"])
    robot_xacro = PathJoinSubstitution([package_share, "urdf", "drift_robot.urdf.xacro"])
    waypoint_file = PathJoinSubstitution([package_share, "config", "waypoints.yaml"])

    # 1. Gazebo ----------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": world, "gui": gui}.items(),
    )

    # 2. Robot state publisher -------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_xacro]), value_type=str
                ),
            }
        ],
        output="screen",
    )

    # 3. Spawn robot -----------------------------------------------------------
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "drift_robot",
            "-topic", "robot_description",
            "-x", "0.5",    # SE quadrant of Room 1 — clear of all furniture
            "-y", "-2.5",   # and all pickup objects (>1 m margin)
            "-z", "0.02",   # gentle drop onto ground plane
        ],
        output="screen",
    )

    # 4. Bag recorder — logs /odom and /scan for measurable output -------------
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--output", _BAG_DIR,
            "/odom",
            "/scan",
            "/camera/image_raw",
        ],
        output="screen",
    )

    # 5. Mission node — full pick-and-place mission, delayed until spawn done --
    mission = Node(
        package="drift_robot",
        executable="navigator",
        #executable="waypoint_cleaner",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_cleaner = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=2.0, actions=[mission])],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bag_recorder,
            delayed_cleaner,
        ]
    )
