#!/usr/bin/env python3
"""
Gazebo Harmonic + MoveIt2 Integration Launch File

This launch file starts:
1. Gazebo Harmonic simulation (empty world with no gravity)
2. Robot with ros2_control
3. MoveIt2 for motion planning
4. RViz2 visualization

Harmonic has the ogre-next fix that resolves Mesa d3d12 rendering issues.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Configuration
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_name = LaunchConfiguration("robot_name", default="fanuc_arm")

    # Get package directory
    pkg_share_dir = get_package_share_directory("fanuc_arm_config")

    # URDF processing
    xacro_file = os.path.join(pkg_share_dir, "config", "lrmate200ic5l_with_sg2.urdf.xacro")
    initial_positions_file = os.path.join(pkg_share_dir, "config", "initial_positions.yaml")

    # Build robot_description by running xacro
    # Using Gazebo Harmonic plugin naming (gz-sim8)
    robot_description_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_sim_time:=true",
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
            # Harmonic uses simplified plugin names without lib prefix and .so suffix
            "gz_control_lib:=gz_ros2_control-system",
            " ",
            "gz_control_name:=gz_ros2_control::GazeboSimROS2ControlPlugin",
            " ",
            "hardware_plugin:=gz_ros2_control/GazeboSimSystem",
            " ",
            "gz_sensors_lib:=gz-sim-sensors-system",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_cmd, value_type=str)}

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("lrmate200ic5l_with_sg2", package_name="fanuc_arm_config")
        .robot_description(file_path="config/lrmate200ic5l_with_sg2.urdf.xacro")
        .robot_description_semantic(file_path="config/lrmate200ic5l_with_sg2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Static transform publisher for world frame (fixes MoveIt2 world frame issue)
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Joint State Publisher (basic - no GUI needed for simulation)
    joint_state_publisher_node = Node(
        package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher", parameters=[{"use_sim_time": use_sim_time}]
    )

    # Get world file path
    world_file = os.path.join(pkg_share_dir, "worlds", "empty_no_gravity.sdf")

    # Gazebo server with controller parameters
    # ros_gz_sim is built from source for Harmonic compatibility
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]),
        launch_arguments={
            "gz_args": f"-r -v 2 {world_file}"  # Start unpaused with verbose level 2
        }.items(),
    )

    # Clock bridge
    # ros_gz_bridge is built from source for Harmonic compatibility
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawn entity into Gazebo
    # ros_gz_sim is built from source for Harmonic compatibility
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
        ],
    )

    # Controller spawners - connect to Gazebo's built-in controller manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
        output="screen",
    )

    fanuc_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fanuc_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
        output="screen",
    )

    # MoveIt2 move_group node with enhanced configuration
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            {"publish_planning_scene": True},
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"trajectory_execution.allowed_goal_duration_margin": 2.0},
        ],
        output="screen",
    )

    # RViz2 with MoveIt2 interface
    rviz_config = PathJoinSubstitution([FindPackageShare("fanuc_arm_config"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # Event-driven sequencing - much more reliable than fixed timers
    delayed_spawn_entity = TimerAction(period=5.0, actions=[gz_spawn_entity])

    # Start joint state broadcaster after robot spawns
    delayed_load_joint_state_broadcaster = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])

    # Start arm controller after joint state broadcaster succeeds
    start_fanuc_controller_after_joint_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[fanuc_arm_controller_spawner],
        )
    )

    # Start MoveIt2 after arm controller succeeds
    start_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fanuc_arm_controller_spawner,
            on_exit=[run_move_group_node],
        )
    )

    # Start RViz2 after MoveIt2 has time to initialize
    delayed_rviz = TimerAction(period=15.0, actions=[rviz_node])

    return LaunchDescription(
        [
            # Environment variables
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                    ":",
                    PathJoinSubstitution([FindPackageShare("fanuc_arm_config"), ".."]),
                ],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=[
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                    ":",
                    "/opt/ros/humble/lib",
                    ":",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            # Core nodes
            static_transform_publisher,
            robot_state_publisher_node,
            joint_state_publisher_node,
            gazebo,
            clock_bridge,
            # Event-driven sequence
            delayed_spawn_entity,
            delayed_load_joint_state_broadcaster,
            start_fanuc_controller_after_joint_broadcaster,
            start_moveit_after_controllers,
            delayed_rviz,
        ]
    )
