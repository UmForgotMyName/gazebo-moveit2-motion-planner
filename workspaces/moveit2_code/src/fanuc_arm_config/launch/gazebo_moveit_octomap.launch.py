#!/usr/bin/env python3
"""
Gazebo Harmonic + MoveIt2 Perception Pipeline (no octomap_server)

Goal:
- Gazebo publishes a simulated D405 PointCloud2
- MoveIt move_group consumes it via occupancy_map_monitor + PointCloudOctomapUpdater
- RViz MotionPlanning shows the internal octomap voxels and planning avoids obstacles

Critical requirement:
- A real TF frame named "world" must exist if octomap_frame == "world" and RViz Fixed Frame == "world"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Keep RMW consistent (Fast DDS) because rviz2 throws Fast CDR / typesupport
    # errors when mixed with other RMWs for large PointCloud2 messages.
    rmw_impl = LaunchConfiguration("rmw_implementation", default="rmw_fastrtps_cpp")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    robot_name = LaunchConfiguration("robot_name", default="fanuc_arm")

    # Keep consistent with your SRDF virtual joint and RViz fixed frame
    octomap_frame = LaunchConfiguration("octomap_frame", default="world")

    # Publish world->base_link so "world" actually exists in TF
    publish_world_tf = LaunchConfiguration("publish_world_tf", default="true")

    pkg_share_dir = get_package_share_directory("fanuc_arm_config")

    xacro_file = os.path.join(pkg_share_dir, "config", "lrmate200ic5l_with_sg2.urdf.xacro")
    initial_positions_file = os.path.join(pkg_share_dir, "config", "initial_positions.yaml")

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

    moveit_config = (
        MoveItConfigsBuilder("lrmate200ic5l_with_sg2", package_name="fanuc_arm_config")
        .robot_description(file_path="config/lrmate200ic5l_with_sg2.urdf.xacro")
        .robot_description_semantic(file_path="config/lrmate200ic5l_with_sg2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time_param},
            {"ignore_timestamp": True},
        ],
        output="screen",
    )

    # This is the missing piece in your current launch file.
    # Without it, "world" does not exist in TF, RViz cannot use Fixed Frame: world,
    # and MoveIt cannot transform the point cloud into the octomap_frame.
    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(publish_world_tf),
        output="screen",
    )

    world_file = os.path.join(pkg_share_dir, "worlds", "octomap_demo_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]),
        launch_arguments={"gz_args": f"-r -v 4 --render-engine-server ogre2 --render-engine-gui ogre2 {world_file}"}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", robot_name, "-allow_renaming", "true"],
        parameters=[{"use_sim_time": use_sim_time_param}],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Gazebo publishes RGB on d405/image (not /d405/rgb/image_raw)
            "/d405/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/d405/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/d405/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/d405/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        parameters=[{"use_sim_time": use_sim_time_param}],
        output="screen",
    )

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
            "--service-call-timeout",
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
            "--service-call-timeout",
            "60",
        ],
        output="screen",
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.sensors_3d,
            {"octomap_frame": octomap_frame},
            {"octomap_resolution": 0.05},
            {"publish_planning_scene": True},
        ],
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("fanuc_arm_config"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time_param},
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        output="screen",
        additional_env={
            "QT_QPA_PLATFORM": "xcb",
            "LIBGL_ALWAYS_SOFTWARE": "true",
            "LIBGL_ALWAYS_INDIRECT": "0",
        },
    )

    delayed_spawn_entity = TimerAction(period=5.0, actions=[gz_spawn_entity])
    delayed_load_joint_state_broadcaster = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])

    start_fanuc_controller_after_joint_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[fanuc_arm_controller_spawner],
        )
    )

    start_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fanuc_arm_controller_spawner,
            on_exit=[run_move_group_node],
        )
    )

    delayed_rviz = TimerAction(period=15.0, actions=[rviz_node])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="fanuc_arm"),
            DeclareLaunchArgument("octomap_frame", default_value="world"),
            DeclareLaunchArgument("publish_world_tf", default_value="true"),
            DeclareLaunchArgument("rmw_implementation", default_value="rmw_fastrtps_cpp"),
            SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
            SetEnvironmentVariable(name="WAYLAND_DISPLAY", value=""),
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
            # Force all nodes (rviz2, ros_gz_bridge, move_group) onto the same RMW.
            SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value=rmw_impl),
            robot_state_publisher_node,
            world_to_base_tf,
            gazebo,
            clock_bridge,
            camera_bridge,
            delayed_spawn_entity,
            delayed_load_joint_state_broadcaster,
            start_fanuc_controller_after_joint_broadcaster,
            start_moveit_after_controllers,
            delayed_rviz,
        ]
    )
