FROM moveit/moveit2:humble-humble-tutorial-source

ENV ROS2_WORKSPACE=/root/ws_moveit
ARG ROS_DISTRO=humble

# Install GPU and graphics libraries first
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # --- GPU and Graphics Libraries ---
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglapi-mesa \
    libglu1-mesa \
    mesa-utils \
    mesa-utils-extra \
    libegl1-mesa \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libxi6 \
    libxtst6 \
    # --- X11 and Display ---
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    xvfb \
    # --- NVIDIA Container Runtime (if needed) ---
    libnvidia-gl-470 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # --- Gazebo Sim (ros_gz) bridge & simulator ---
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gz-ros2-control \
    # --- MoveIt 2 + RViz plugin ---
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    # --- Octomap & Point Cloud Processing ---
    ros-${ROS_DISTRO}-octomap-server \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-point-cloud-transport \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    # --- RViz 2 ---
    ros-${ROS_DISTRO}-rviz2 \
    # --- Ros 2 Control ---
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    # --- Other Dependencies ---
    build-essential curl gnupg lsb-release git \
    python3-dev python3-distutils python3-pip \
    python3-rosdep python3-colcon-common-extensions \
    python3-colcon-mixin python3-vcstool \
    iputils-ping && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro=$ROS_DISTRO && apt dist-upgrade

# Set up GPU and graphics environment
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Copy Moveit2 workspace files to the correct location
COPY workspaces/moveit2_code $ROS2_WORKSPACE
COPY interfaces/robot_interfaces $ROS2_WORKSPACE/src/robot_interfaces

WORKDIR $ROS2_WORKSPACE

# STEP 2: build the rest after sourcing the install setup
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    . install/setup.bash && \
    colcon build --packages-select fanuc_arm_config fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin"

# Ensure the workspace is sourced in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source $ROS2_WORKSPACE/install/setup.bash" >> /root/.bashrc

# Ensure the entrypoint script sources the workspace
RUN sed --in-place --expression \
        '$isource "$ROS2_WORKSPACE/install/setup.bash"' \
        /ros_entrypoint.sh
