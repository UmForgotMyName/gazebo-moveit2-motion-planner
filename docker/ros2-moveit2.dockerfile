FROM moveit/moveit2:humble-humble-tutorial-source

ENV ROS2_WORKSPACE=/root/ws_moveit
ARG ROS_DISTRO=humble

# =============================================================================
# GAZEBO HARMONIC MIGRATION
# =============================================================================
# WHY HARMONIC (gz-sim 8.x)?
# - Has the ogre-next fix backported that resolves Mesa d3d12 rendering crashes
# - Reference: https://github.com/gazebosim/gz-sim/issues/2502
# - Harmonic is the newest LTS (supported until 2028)
# - Includes improved sensor rendering stability
#
# ARCHITECTURE:
# 1. Remove ros-humble-ros-gz (which installs Gazebo Fortress)
# 2. Install Gazebo Harmonic from OSRF packages
# 3. Build ros_gz from source (for Humble + Harmonic compatibility)
# =============================================================================

# =============================================================================
# STAGE 1: Mesa 25 with d3d12 Gallium driver for WSL2/Docker Desktop GPU support
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:kisak/turtle && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 1b: Remove any pre-installed Gazebo Fortress packages from base image
# The base MoveIt2 image may have ros-gz (Fortress) - we need Harmonic instead
# =============================================================================
RUN apt-get update && \
    apt-get remove -y --purge \
    ros-${ROS_DISTRO}-ros-gz* \
    ros-${ROS_DISTRO}-gz-* \
    gz-fortress* \
    libgz-* \
    || true && \
    apt-get autoremove -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 2: Install base ROS2/MoveIt2 packages (WITHOUT ros-gz)
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # --- MoveIt 2 ---
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    # --- Octomap & Point Cloud Processing ---
    ros-${ROS_DISTRO}-octomap-server \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-point-cloud-transport \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    # --- RViz 2 ---
    ros-${ROS_DISTRO}-rviz2 \
    # --- ROS 2 Control ---
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    # --- Topic tools (for relay node in octomap pipeline) ---
    ros-${ROS_DISTRO}-topic-tools \
    # --- Build/Dev Dependencies ---
    build-essential curl gnupg lsb-release git wget \
    python3-dev python3-distutils python3-pip \
    python3-rosdep python3-colcon-common-extensions \
    python3-colcon-mixin python3-vcstool \
    iputils-ping \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 3: Install Gazebo Harmonic from OSRF packages
# Reference: https://gazebosim.org/docs/harmonic/install_ubuntu
# =============================================================================
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gz-harmonic \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 4: Graphics/Display Libraries
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # --- GPU and Graphics Libraries (GLVND + Mesa) ---
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
    libgl1-mesa-glx \
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
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 5: Upgrade Mesa to version 25.x from kisak PPA for d3d12 support
# =============================================================================
RUN apt-get update && \
    apt-get remove -y --allow-change-held-packages \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    mesa-vdpau-drivers \
    mesa-vulkan-drivers \
    libglu1-mesa-dev \
    || true

RUN apt-mark unhold libglapi-mesa || true && \
    apt-get remove -y libglapi-mesa || true

RUN printf 'Package: *mesa* *libgl* *libegl* *libgbm* *libglapi*\nPin: release o=LP-PPA-kisak-turtle\nPin-Priority: 1001\n' > /etc/apt/preferences.d/mesa-kisak

RUN apt-get update && \
    apt-get install -y --allow-downgrades \
    mesa-libgallium=25.0.7~kisak3~j \
    libglx-mesa0=25.0.7~kisak3~j \
    libgl1-mesa-dri=25.0.7~kisak3~j \
    libegl-mesa0=25.0.7~kisak3~j \
    libgbm1=25.0.7~kisak3~j \
    mesa-va-drivers=25.0.7~kisak3~j \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Verify d3d12 driver exists
RUN ls -la /usr/lib/x86_64-linux-gnu/dri/d3d12* && echo "d3d12 driver found!" || echo "Warning: d3d12 driver not found"

# =============================================================================
# STAGE 6: Build ros_gz for ROS2 Humble + Gazebo Harmonic from source
# Reference: https://github.com/gazebosim/ros_gz/tree/humble
# =============================================================================
ENV GZ_VERSION=harmonic

# Create ros_gz workspace
RUN mkdir -p /root/ros_gz_ws/src

WORKDIR /root/ros_gz_ws/src

# Clone ros_gz for Humble (supports Harmonic via GZ_VERSION)
RUN git clone https://github.com/gazebosim/ros_gz.git -b humble

# Install dependencies
WORKDIR /root/ros_gz_ws
RUN apt-get update && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    rosdep install -r --from-paths src -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys="gz-cmake3 gz-common5 gz-fuel-tools9 gz-gui8 gz-math7 gz-msgs10 gz-physics7 gz-plugin2 gz-rendering8 gz-sensors8 gz-sim8 gz-tools2 gz-transport13 gz-utils2" \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Build ros_gz
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    export GZ_VERSION=harmonic && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# =============================================================================
# STAGE 7: Build gz_ros2_control for Harmonic from source
# =============================================================================
RUN mkdir -p /root/gz_ros2_control_ws/src

WORKDIR /root/gz_ros2_control_ws/src

# Clone gz_ros2_control (humble branch supports Harmonic)
RUN git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

WORKDIR /root/gz_ros2_control_ws

# Install dependencies and build
RUN apt-get update && \
    rosdep install -r --from-paths src -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys="gz-cmake3 gz-common5 gz-fuel-tools9 gz-gui8 gz-math7 gz-msgs10 gz-physics7 gz-plugin2 gz-rendering8 gz-sensors8 gz-sim8 gz-tools2 gz-transport13 gz-utils2" \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    . /root/ros_gz_ws/install/setup.bash && \
    export GZ_VERSION=harmonic && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# =============================================================================
# STAGE 8: Workspace Setup
# =============================================================================
RUN rosdep update --rosdistro=$ROS_DISTRO

# Environment defaults
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1
ENV GZ_VERSION=harmonic

# Copy workspace files
COPY workspaces/moveit2_code $ROS2_WORKSPACE
COPY interfaces/robot_interfaces $ROS2_WORKSPACE/src/robot_interfaces

WORKDIR $ROS2_WORKSPACE

# Build workspace packages
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    . /root/ros_gz_ws/install/setup.bash && \
    . /root/gz_ros2_control_ws/install/setup.bash && \
    if [ -f install/setup.bash ]; then . install/setup.bash; fi && \
    colcon build --packages-select fanuc_arm_config fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin"

# Source all workspaces in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros_gz_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/gz_ros2_control_ws/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f \"$ROS2_WORKSPACE/install/setup.bash\" ]; then source \"$ROS2_WORKSPACE/install/setup.bash\"; fi" >> /root/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> /root/.bashrc

# Update entrypoint to source all workspaces
RUN sed --in-place --expression \
        '$isource "/root/ros_gz_ws/install/setup.bash"\nsource "/root/gz_ros2_control_ws/install/setup.bash"\nsource "$ROS2_WORKSPACE/install/setup.bash"\nexport GZ_VERSION=harmonic' \
        /ros_entrypoint.sh
