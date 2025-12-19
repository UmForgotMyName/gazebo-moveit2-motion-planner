# Troubleshooting and debugging

This guide is meant to be operational. Start from the top and stop when you find the problem.

## Quick checklist
Inside the container:
```bash
ros2 topic list | sort
ros2 node list | sort
ros2 service list | sort
ros2 action list | sort
```

If Gazebo or RViz will not open under WSLg, jump to:
- [WSLg display and GPU](wslg-gpu.md)

## Controllers

### Confirm controllers are running
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Expected (typical):
- `joint_state_broadcaster` active
- `fanuc_arm_controller` active

If a controller is inactive:
- Look at launch logs
- Verify joint names match between URDF, ros2_control config, and MoveIt configs

### Check joint states
```bash
ros2 topic echo /joint_states --once
```

If joint states are all zeros or never update:
- The `joint_state_broadcaster` is not active
- Gazebo ros2_control plugin is not loaded
- Joint names mismatch (URDF vs YAML)

## MoveIt planning vs execution

### Planning works but execute fails
Common causes:
- MoveIt is mapped to the wrong FollowJointTrajectory action
- `moveit_controllers.yaml` does not match the controller name and namespace
- The planning group contains joints that are not controllable

Check controller mapping:
- `workspaces/moveit2_code/src/fanuc_arm_config/config/moveit_controllers.yaml`

### Bypass MoveIt and test the controller directly
```bash
ros2 action send_goal /fanuc_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

If this fails, focus on ros2_control and Gazebo integration first.

## Gazebo issues

### Robot is visible but does not move
Common causes:
- Missing or unrealistic joint limits or effort limits in URDF
- Controller is not loaded or not activated
- Joint interfaces not exported

Verify:
- URDF joint limits in `urdf/fanuc200ic5l_w_sg2.urdf`
- ros2_control config in `config/ros2_controllers.yaml`

## Octomap issues

### Pointcloud topic exists?
```bash
ros2 topic echo /d405/points --once
```

If it is empty or missing:
- The camera plugin is not publishing
- `ros_gz_bridge` is not bridging this topic
- Topic names differ from what MoveIt expects

### TF chain exists?
```bash
ros2 run tf2_tools view_frames
```

If the pointcloud frame is not connected to the MoveIt planning frame, Octomap will not update.

### Sensor config to verify
- `workspaces/moveit2_code/src/fanuc_arm_config/config/sensors_3d.yaml`

## Logs
If something is unclear, get the logs that matter:
```bash
docker logs ros2-moveit2 --tail 200
```

Inside the container:
```bash
ros2 run rqt_console rqt_console
```
