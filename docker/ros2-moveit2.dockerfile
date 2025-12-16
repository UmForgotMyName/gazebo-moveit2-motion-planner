FROM moveit/moveit2:humble-humble-tutorial-source

ENV ROS2_WORKSPACE=/root/ws_moveit
ARG ROS_DISTRO=humble

# =============================================================================
# WSL2 GPU FIX: Install Mesa 25 with d3d12 Gallium driver for OpenGL 4.6 support
# This fixes Ogre2 rendering issues in WSL2/WSLg environments
# Reference: https://github.com/gazebosim/gz-sim/issues/2502
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:kisak/turtle && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # --- GPU and Graphics Libraries (GLVND + Mesa utils) ---
    libglvnd0 \
    libgl1 \
    libglx0 \
    libxext6 \
    libx11-6 \
    libxkbcommon-x11-0 \
    libgl1-mesa-dri \
    libegl1 \
    libegl1-mesa \
    libglu1-mesa \
    mesa-utils \
    mesa-utils-extra \
    # --- Your original extras (safe to keep) ---
    libgl1-mesa-glx \
    libglapi-mesa \
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
    # --- Topic tools (for relay node) ---
    ros-${ROS_DISTRO}-topic-tools \
    # --- Other Dependencies ---
    build-essential curl gnupg lsb-release git \
    python3-dev python3-distutils python3-pip \
    python3-rosdep python3-colcon-common-extensions \
    python3-colcon-mixin python3-vcstool \
    iputils-ping && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Keep rosdep metadata fresh; avoid dist-upgrade to prevent driver/lib drift
RUN rosdep update --rosdistro=$ROS_DISTRO

# Env defaults (runtime will pass DISPLAY and GPU caps; keeping these is harmless)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Copy Moveit2 workspace files to the correct location
COPY workspaces/moveit2_code $ROS2_WORKSPACE
COPY interfaces/robot_interfaces $ROS2_WORKSPACE/src/robot_interfaces

WORKDIR $ROS2_WORKSPACE

# Build after sourcing ROS; guard install/setup.bash if it doesn't exist yet
RUN /bin/bash -lc ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -f install/setup.bash ]; then . install/setup.bash; fi && \
    colcon build --packages-select fanuc_arm_config fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin"

# Ensure the workspace is sourced in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "if [ -f \"$ROS2_WORKSPACE/install/setup.bash\" ]; then source \"$ROS2_WORKSPACE/install/setup.bash\"; fi" >> /root/.bashrc

# Ensure the entrypoint script sources the workspace if present
RUN sed --in-place --expression \
        '$isource "$ROS2_WORKSPACE/install/setup.bash"' \
        /ros_entrypoint.sh
