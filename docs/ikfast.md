# IKFast Plugin Development Guide

This section provides detailed instructions for creating IKFast plugins for MoveIt2, which is useful for other robot models.

## How MoveIt loads it
MoveIt selects a kinematics plugin per planning group from `kinematics.yaml`.

Typical expectations:
- The group name in SRDF matches the key in `kinematics.yaml`
- The plugin exports a class via pluginlib
- Joint ordering matches the solver assumptions

## Quick verification
Inside the container, after sourcing:
```bash
source /opt/ros/humble/setup.bash
source /root/ws_moveit/install/setup.bash

ros2 param get /move_group robot_description_kinematics
```

If MoveIt falls back to KDL, the plugin likely did not load.

### Converting URDF to IKFast MoveIt2 Plugin

#### Step 1: Convert URDF to Collada
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

#### Step 2: Generate IKFast C++ File

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


   #### Step 3: Create IKFast Plugin for MoveIt2

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