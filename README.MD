# ROS2 MoveIt2 Fanuc Robot Simulation with Gazebo Harmonic

## Overview
This project implements a complete robotic manipulation system using ROS2 Humble and MoveIt2, featuring a Fanuc LR Mate 200iC/5L industrial robot arm with SG2 gripper. The system provides full integration between MoveIt2 motion planning and **Gazebo Harmonic** physics simulation, allowing for realistic robot behavior testing and development.

### Why Gazebo Harmonic?
- **ogre-next fix**: Harmonic includes the ogre-next rendering fix that resolves Mesa d3d12 crashes on WSL2/Docker Desktop
- **Latest LTS**: Supported until 2028, providing long-term stability
- **Improved sensors**: Better camera/depth sensor rendering stability
- **Reference**: [GitHub Issue #2502](https://github.com/gazebosim/gz-sim/issues/2502)

## Key Features
- **Complete MoveIt2 Integration**: Full motion planning capabilities with OMPL planners
- **Gazebo Harmonic Simulation**: Real-time robot simulation with ogre-next rendering fix
- **IKFast Inverse Kinematics**: High-performance analytical IK solver
- **ros2_control Integration**: Seamless communication between MoveIt2 and Gazebo controllers
- **D405 RGBD Camera**: Intel RealSense depth camera for octomap generation
- **Octomap Integration**: 3D occupancy mapping for obstacle avoidance
- **Interactive Motion Planning**: RViz2-based interface for trajectory planning and execution
- **Docker-based Development**: Containerized environment with WSL2 GPU support (Mesa d3d12)

## Getting Started

Want to see the robot in action immediately? Follow these two simple steps:

### 1. Start the System
```bash
# Clone the repository and navigate to it
git clone <repository-url>
cd gazebo-moveit2-motion-planner

# Enable X11 forwarding for GUI applications (from WSL2 Ubuntu)
xhost +local:docker

# Build and start the Docker container (this takes a while on first build)
docker compose up --build -d
```

### 2. Launch the Robot Simulation
```bash
# Enter the container
docker exec -it ros2-moveit2 bash

# Inside container: Launch basic MoveIt2 + Gazebo integration
ros2 launch fanuc_arm_config gazebo_moveit_integration.launch.py

# OR: Launch with camera and octomap
ros2 launch fanuc_arm_config gazebo_moveit_octomap.launch.py
```

**That's it!** You should now see Gazebo with the robot simulation and RViz2 with the motion planning interface. You can drag the interactive markers to plan and execute robot movements.

## System Architecture

### Core Components
- **fanuc_arm_config**: Robot configuration package containing URDF, SRDF, and MoveIt2 config files
- **fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin**: Custom IKFast-generated inverse kinematics plugin

### Key Technologies
- **ROS2 Humble**: Latest LTS version of ROS2
- **MoveIt2**: Advanced motion planning framework
- **Gazebo Harmonic (gz-sim 8.x)**: Physics simulation with ogre-next rendering fix
- **OMPL**: Open Motion Planning Library for path planning
- **IKFast**: Analytical inverse kinematics solver
- **Octomap**: 3D occupancy mapping from point clouds

## Prerequisites

### System Requirements
- **Windows 11** with WSL2 + Docker Desktop (WSL Integration enabled)
- Ubuntu 22.04 WSL2 distribution
- Docker Desktop with WSL Integration
- NVIDIA GPU (RTX series recommended) with driver 580+
- At least 8GB RAM and 20GB disk space

### GPU Requirements (WSL2 + Docker Desktop)
The system uses Mesa d3d12 driver for GPU acceleration:
- NVIDIA GPU with DirectX 12 support
- WSL2 with `/dev/dxg` device available
- Docker Desktop with WSL Integration enabled

### Setup Display for WSLg (Wayland-first)
```bash
# From WSL2 Ubuntu terminal (NOT PowerShell):
xhost +local:docker
```
WSLg provides a Wayland compositor; the compose file is set for Wayland-first (`WAYLAND_DISPLAY=wayland-0`, `XDG_RUNTIME_DIR=/run/user/0`, `QT_QPA_PLATFORM=wayland`) with Xwayland fallback via `DISPLAY=:0`. This aligns with Microsoft’s WSLg guidance and matches community fixes for RViz “failed to create drawable” on WSLg when GLX-only is used.

If you see `Could not find the Qt platform plugin "wayland"` from Gazebo or RViz, ensure the `qtwayland5` runtime is installed (added in the Dockerfile) so the Qt Wayland platform plugin is available.

If Qt complains about runtime directory permissions, check inside the container:
```bash
ls -ld /run/user/0
ls -l /run/user/0/wayland-0
```
The entrypoint now creates `/run/user/0` (0700) and symlinks the WSLg socket (`/mnt/wslg/runtime-dir/wayland-0`) there so Qt/Wayland accepts it. When exporting `QT_QPA_PLATFORM` manually, quote a fallback if you use one: `export QT_QPA_PLATFORM='wayland;xcb'`.

## Quick Start

### Interactive Motion Planning

1. **Open RViz2**: Should launch automatically with the container
2. **Use Interactive Markers**: 
   - Drag the orange interactive markers to set target poses
   - The robot will display the planned path in real-time
3. **Plan Trajectories**: Click "Plan" to compute a motion plan
4. **Execute Movements**: Click "Execute" to run the planned trajectory

### Expected Behavior
- **Visual Feedback**: You should see the robot move in both RViz2 and Gazebo
- **Joint State Updates**: Joint positions update in real-time during movement
- **Collision Avoidance**: The planner automatically avoids self-collisions
- **Smooth Trajectories**: Movements follow smooth, dynamically-feasible paths

### Verify GPU Acceleration
```bash
# Inside the container:
glxinfo -B
# Should show: "Device: D3D12 (NVIDIA GeForce RTX ...)"
```
# If GLX fails, force software once to confirm it is a driver path issue:
# LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d /root/ws_moveit/install/fanuc_arm_config/share/fanuc_arm_config/config/moveit_octomap.rviz

### Monitoring Robot State
```bash
# Check joint states (should show real positions, not all zeros)
docker exec -it ros2-moveit2 bash -c "ros2 topic echo /joint_states --once"

# Monitor controller status
docker exec -it ros2-moveit2 bash -c "ros2 control list_controllers"

# View available action servers
docker exec -it ros2-moveit2 bash -c "source /opt/ros/humble/setup.bash && ros2 action list"
```

### References
- Microsoft WSLg GUI apps guidance (Wayland-first, host sockets in `/mnt/wslg`): https://learn.microsoft.com/windows/wsl/tutorials/gui-apps
- WSLg README (Wayland compositor, GLX routed through Xwayland): https://github.com/microsoft/wslg#applications
- ROS Answers threads on RViz “failed to create drawable” under WSLg recommend Wayland or software GL fallback: https://answers.ros.org (search for the error string)

## Troubleshooting

### Common Issues

1. **Robot Not Moving in Gazebo**
   - **Symptom**: Motion planning works in RViz2 but robot doesn't move in Gazebo
   - **Solution**: Ensure joint effort limits are properly set in URDF (should not be 0)

2. **MoveIt2 Cannot Execute Trajectories**
   - **Symptom**: Planning works but execution fails
   - **Solution**: Check that planning group only includes controllable joints

3. **Display Issues**
   - **Symptom**: GUI windows don't appear
   - **Solution**: Ensure X11 forwarding is enabled: `xhost +local:docker`

4. **Controller Errors**
   - **Symptom**: Controllers fail to start
   - **Solution**: Check that ros2_control hardware interfaces are properly configured

### Debugging Commands
```bash
# Check container logs
docker logs ros2-moveit2

# Access container for debugging
docker exec -it ros2-moveit2 bash

# Test direct controller communication
docker exec -it ros2-moveit2 bash -c "source /opt/ros/humble/setup.bash && ros2 action send_goal /fanuc_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}'"
```

## Development

### Project Structure
```
ros2-moveit2/
├── docker/                          # Docker configuration
│   └── ros2-moveit2.dockerfile     # Main container definition
├── workspaces/moveit2_code/src/     # ROS2 workspace
│   ├── fanuc_arm_config/           # Robot configuration
│   │   ├── config/                 # MoveIt2 and controller configs
│   │   ├── launch/                 # Launch files
│   │   ├── urdf/                   # Robot description files
│   │   └── worlds/                 # Gazebo world files
│   ├── fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin/  # IKFast plugin
│   └── hello_moveit/               # Example code
├── interfaces/robot_interfaces/     # Custom ROS2 interfaces
└── docker-compose.yml             # Container orchestration
```

### Key Configuration Files
- **URDF**: `workspaces/moveit2_code/src/fanuc_arm_config/urdf/fanuc200ic5l_w_sg2.urdf`
- **SRDF**: `workspaces/moveit2_code/src/fanuc_arm_config/config/lrmate200ic5l_with_sg2.srdf`
- **Controllers**: `workspaces/moveit2_code/src/fanuc_arm_config/config/ros2_controllers.yaml`
- **MoveIt2 Controllers**: `workspaces/moveit2_code/src/fanuc_arm_config/config/moveit_controllers.yaml`

## TODO / Roadmap

### High Priority
- [x] ~~Fix MoveIt2-Gazebo integration for trajectory execution~~
- [x] ~~Resolve joint control issues~~
- [ ] Create custom launch file to launch all components together
- [ ] Implement /compute_ik service with IKFast plugin
- [ ] Create path planning service using MoveGroupInterface

### Medium Priority
- [ ] Add real-time obstacle detection using RGBD sensor data
- [ ] Implement point cloud to Octomap conversion
- [ ] Add end-effector goal position targeting
- [ ] Create pick-and-place demonstration

### Low Priority
- [ ] Add force/torque control
- [ ] Implement advanced collision checking
- [ ] Add custom planning pipelines
- [ ] Create web-based monitoring interface

## Appendix: IKFast Plugin Development Guide

This section provides detailed instructions for creating IKFast plugins for MoveIt2, which is useful for other robot models.

### Converting URDF to IKFast MoveIt2 Plugin

### Step 1: Convert URDF to Collada
  **1.1 Set Up ROS2 Noetic Docker Container**
  ```bash
  docker run -it -v /path/to/your/folder:/root/catkin_ws ros:noetic
  ```

  **1.2 Install Required Dependencies**
  ```bash
  apt-get update
  apt-get install -y git cmake build-essential
  ```

  **1.3 Install and Build collada_urdf**
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/ros/collada_urdf.git
  rosdep install --from-paths src --ignore-src -r -y
  apt-get install libfcl-dev
  catkin_make_isolated --pkg collada_urdf
  source ~/catkin_ws/devel_isolated/setup.bash
  ```

  **1.4 Convert URDF to Collada**
  ```bash
  rosrun collada_urdf urdf_to_collada urdf/fanuc200ic5l_w_sg2.urdf fanuc200ic5l_w_sg2.dae
  ```

  **1.5 Round Numbers in Collada File**
  ```bash
  apt-get install ros-noetic-moveit-kinematics
  rosrun moveit_kinematics round_collada_numbers.py $MYROBOT_NAME.dae $MYROBOT_NAME.rounded.dae 5
  ```

### Step 2: Generate IKFast C++ File

   **2.1 Start ROS Indigo container:**
   ```bash
   docker run -it -v /path/to/your/folder:/root/catkin_ws ros:indigo bash
   ```

   **2.2 Install SymPy 0.7.1:**
   - Download sympy-0.7.1.tar.gz from the official website
   - Extract to /urdf folder
   - Install using:
   ```bash
   python setup.py install
   ```

   **2.3 Follow the MoveIt IKFast documentation:**
   Follow the steps for "Identify Link Numbers and Generate IK Solver" at:
   [https://moveit.readthedocs.io/en/latest/doc/ikfast_tutorial.html](https://moveit.readthedocs.io/en/latest/doc/ikfast_tutorial.html)

3. Create IKFast Plugin for MoveIt2

   ### Step 3: Create IKFast Plugin for MoveIt2

   **3.1 Clone the IKFast package template:**
   - Go to [https://github.com/moveit/moveit2](https://github.com/moveit/moveit2)
   - Navigate to the `moveit_ikfast` package template
   - Copy the IKFast template into your own workspace

   **3.2 Modify the copied files:**
   - Open all relevant files within the template and ensure all variables (such as robot names and namespaces) are updated to reflect your project's naming conventions.

   **3.3 Follow CMake changes for ROS2 compatibility:**
   - Reference: [https://github.com/moveit/moveit2/issues/2144](https://github.com/moveit/moveit2/issues/2144)
   - This includes modifications to the `CMakeLists.txt` and other necessary files for ROS2 compatibility.

   **3.4 Update the plugin.xml:**
   - In the plugin.xml file, remove the "lib/lib" prefix from the library path. The path should now be just the name of the library, not including the "lib" directory.

   **3.5 Build and Source the Package:**
   ```bash
   # Build the package
   colcon build --packages-select your_ikfast_package
   
   # Source your workspace
   source install/setup.bash
   ```

   **3.6 Verify the IKFast Plugin is Loaded:**
   - Use MoveIt2 Setup Assistant to ensure that the IKFast plugin shows up in the list of available kinematics plugins.
   - You should be able to use the plugin with MoveIt2 for path planning and inverse kinematics.
