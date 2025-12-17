FROM moveit/moveit2:humble-humble-tutorial-source

ENV ROS2_WORKSPACE=/root/ws_moveit
ARG ROS_DISTRO=humble

# =============================================================================
# STAGE 1: Mesa 25 with d3d12 Gallium driver for WSL2/Docker Desktop GPU support
# 
# WHY THIS IS NEEDED (from GitHub issue =#2502):
# - Gazebo Fortress + Ogre2 + WSLg has known rendering issues
# - The ogre-next fix was backported to Garden/Harmonic, NOT Fortress
# - Workaround: Update Mesa to 25.x which has full OpenGL 4.6 on d3d12
# - This avoids needing the ogre-next patch
#
# References:
#   https://github.com/gazebosim/gz-sim/issues/2502
#   https://github.com/gazebosim/gz-sim/issues/2595
#   Mesa 24.0+ achieved OpenGL 4.6 on d3d12: https://docs.mesa3d.org/relnotes/24.0.0.html
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:kisak/turtle && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 2: Install all ROS2/Gazebo/MoveIt2 packages
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # --- Gazebo Sim (ros_gz) - COMPLETE PACKAGE SET ---
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-gz-ros2-control \
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
    build-essential curl gnupg lsb-release git \
    python3-dev python3-distutils python3-pip \
    python3-rosdep python3-colcon-common-extensions \
    python3-colcon-mixin python3-vcstool \
    iputils-ping \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 3: Graphics/Display Libraries
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
# STAGE 4: Upgrade Mesa to version 25.x from kisak PPA for d3d12 support
# This MUST happen AFTER all other package installs
# =============================================================================
# Remove packages that pin old Mesa versions
RUN apt-get update && \
    apt-get remove -y --allow-change-held-packages \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    mesa-vdpau-drivers \
    mesa-vulkan-drivers \
    libglu1-mesa-dev \
    || true

# Remove old libglapi-mesa
RUN apt-mark unhold libglapi-mesa || true && \
    apt-get remove -y libglapi-mesa || true

# Pin kisak PPA for Mesa packages
RUN printf 'Package: *mesa* *libgl* *libegl* *libgbm* *libglapi*\nPin: release o=LP-PPA-kisak-turtle\nPin-Priority: 1001\n' > /etc/apt/preferences.d/mesa-kisak

# Install Mesa 25.x (libglapi-mesa is virtual, provided by mesa-libgallium)
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
# STAGE 4b: Reinstall ros-gz packages AFTER Mesa upgrade
# This ensures the Gazebo rendering libraries link against the new Mesa
# =============================================================================
RUN apt-get update && \
    apt-get install -y --reinstall --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-gz-ros2-control \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 5: Workspace Setup
# =============================================================================
RUN rosdep update --rosdistro=$ROS_DISTRO

# Environment defaults
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# =============================================================================
# Mesa d3d12 + Ogre2 Workarounds for WSL2
# These help stabilize Ogre2 rendering on Mesa d3d12
# =============================================================================
# Use threaded GL for better stability
ENV mesa_glthread=false
# Disable problematic features
ENV MESA_NO_ERROR=1
# Force specific GL version for Ogre2 compatibility
ENV MESA_GL_VERSION_OVERRIDE=4.5
ENV MESA_GLSL_VERSION_OVERRIDE=450

# Copy workspace files
COPY workspaces/moveit2_code $ROS2_WORKSPACE
COPY interfaces/robot_interfaces $ROS2_WORKSPACE/src/robot_interfaces

WORKDIR $ROS2_WORKSPACE

# Build workspace packages
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -f install/setup.bash ]; then . install/setup.bash; fi && \
    colcon build --packages-select fanuc_arm_config fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin"

# Source workspace in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "if [ -f \"$ROS2_WORKSPACE/install/setup.bash\" ]; then source \"$ROS2_WORKSPACE/install/setup.bash\"; fi" >> /root/.bashrc

# Source workspace in entrypoint
RUN sed --in-place --expression \
        '$isource "$ROS2_WORKSPACE/install/setup.bash"' \
        /ros_entrypoint.sh
