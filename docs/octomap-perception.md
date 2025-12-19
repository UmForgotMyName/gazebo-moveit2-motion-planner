# Perception and Octomap pipeline (D405)

This guide explains how the simulated D405 pointcloud becomes an Octomap inside MoveIt for collision-aware planning.

## Data flow
1. Gazebo publishes camera topics (RGB, depth, pointcloud)
2. `ros_gz_bridge` bridges Gazebo messages into ROS 2 topics
3. TF connects the pointcloud frame into the MoveIt planning frame
4. MoveIt `occupancy_map_monitor/PointCloudOctomapUpdater` consumes the cloud
5. PlanningSceneMonitor updates the Octomap
6. RViz visualizes the pointcloud and Octomap (if enabled)

## Key configuration
MoveIt 3D sensor config:
- `workspaces/moveit2_code/src/fanuc_arm_config/config/sensors_3d.yaml`

Common knobs:
- `max_range`
- `point_subsample`
- `max_update_rate`
- `filtered_cloud_topic`

Note: `sensors_3d.yaml` only matters if your MoveIt launch actually loads it.

## Verify topics
```bash
ros2 topic list | grep -i d405
ros2 topic echo /d405/points --once
```

## Visualize simulated RGB and depth
Install:
```bash
apt-get update && apt-get install -y ros-humble-image-view
```

RGB:
```bash
ros2 run image_view image_view --ros-args -r image:=/d405/image
```

Depth:
```bash
ros2 run image_view image_view --ros-args -r image:=/d405/depth_image
```

If images are black or empty, it usually indicates one of:
- Camera plugin not publishing in Gazebo
- Bridge not relaying the correct message type/topic
- Topic names or remaps differ from what you expect
- Camera is pointed at empty space or occluded

## Verify TF connectivity
Generate the TF frame graph:
```bash
ros2 run tf2_tools view_frames
```

Quick sanity check for a specific transform:
```bash
ros2 run tf2_ros tf2_echo <planning_frame> <pointcloud_frame>
```

## Common causes of "Octomap not updating"
- Topic name mismatch between `sensors_3d.yaml` and the actual pointcloud topic
- Pointcloud frame not connected to the MoveIt planning frame through TF
- Pointcloud is empty or sparse due to camera placement or clip range
- Update throttled too hard (`max_update_rate` too low) or subsample too high

## Performance tips
If RViz or planning becomes sluggish:
- Increase `point_subsample`
- Lower `max_update_rate`
- Reduce the camera publish rate in sim if needed
