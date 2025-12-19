# Perception and Octomap pipeline (D405)

This guide explains how the simulated D405 pointcloud becomes an Octomap inside MoveIt for collision-aware planning.

## Data flow
1. Gazebo publishes camera topics (RGB, depth, pointcloud)
2. `ros_gz_bridge` bridges Gazebo messages into ROS 2 topics
3. TF connects the pointcloud frame into the planning frame
4. MoveIt `occupancy_map_monitor/PointCloudOctomapUpdater` consumes the cloud
5. PlanningSceneMonitor updates the Octomap
6. RViz visualizes the Octomap and pointcloud (if enabled)

## Key configuration
MoveIt 3D sensor config:
- `workspaces/moveit2_code/src/fanuc_arm_config/config/sensors_3d.yaml`

Typical knobs you may tune:
- `max_range`
- `point_subsample`
- `max_update_rate`
- `filtered_cloud_topic`

## Topics to verify
Pointcloud:
```bash
ros2 topic list | grep -i d405
ros2 topic echo /d405/points --once
```

TF:
```bash
ros2 run tf2_tools view_frames
```

## Common causes of "Octomap not updating"
- Topic name mismatch between `sensors_3d.yaml` and actual topic
- Pointcloud frame not connected to the MoveIt planning frame
- Pointcloud is empty or extremely sparse due to camera placement
- Update throttled too hard (max_update_rate too low)

## Performance tips
If RViz or planning becomes sluggish:
- Increase `point_subsample`
- Lower `max_update_rate`
- Reduce camera publish rate (in sim) if needed
