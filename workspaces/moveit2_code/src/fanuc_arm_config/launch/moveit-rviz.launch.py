from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_description_kinematics = {
        "robot_description_kinematics.fanuc_arm.kinematics_solver": "fanuc200ic5l_w_sg2_fanuc_arm/IKFastKinematicsPlugin",
        "robot_description_kinematics.fanuc_arm.kinematics_solver_search_resolution": 0.005,
        "robot_description_kinematics.fanuc_arm.kinematics_solver_timeout": 0.1,
    }

    moveit_config = (
        MoveItConfigsBuilder("lrmate200ic5l_with_sg2", package_name="fanuc_arm_config")
        .robot_description(file_path="config/lrmate200ic5l_with_sg2.urdf.xacro")
        .robot_description_semantic(file_path="config/lrmate200ic5l_with_sg2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value="moveit.rviz", description="RViz config")
    sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock")

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution([FindPackageShare("fanuc_arm_config"), "config", rviz_base])

    # MoveIt2 move_group node - connects to existing controllers
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            {"publish_planning_scene": True},
            {"planning_scene_monitor.publish_geometry_updates": True},
            {"planning_scene_monitor.publish_state_updates": True},
            {"planning_scene_monitor.publish_transforms_updates": True},
        ],
        output="screen",
    )

    # Static transform publisher for world frame
    static_tf_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # RViz2 with MoveIt2 interface
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            sim_time_arg,
            # Start static transform first
            static_tf_world,
            # Start MoveIt2 first, then RViz after a longer delay to ensure MoveIt2 is fully ready
            run_move_group_node,
            TimerAction(period=8.0, actions=[rviz_node]),
        ]
    )
