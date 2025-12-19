FROM ros:humble-ros-base

ENV ROS2_WORKSPACE=/root/ws_moveit
ARG ROS_DISTRO=humble

# =============================================================================
# ROS2 Humble + MoveIt2 + Gazebo Harmonic
# =============================================================================

# =============================================================================
# STAGE 1: Add repositories
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common wget lsb-release gnupg && \
    add-apt-repository -y ppa:kisak/turtle && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 2: Graphics libraries + Mesa upgrade FIRST (before Qt5-dependent packages)
# =============================================================================
# This avoids the package conflict where Mesa upgrade removes Qt5 packages

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libglvnd0 libgl1 libglx0 libxext6 libx11-6 libxkbcommon-x11-0 \
    libegl1 libglu1-mesa mesa-utils mesa-utils-extra \
    libxrandr2 libxss1 libxcursor1 libxcomposite1 libasound2 libxi6 libxtst6 \
    x11-apps x11-utils x11-xserver-utils xauth \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Pin Mesa packages to kisak-turtle PPA for WSL2 d3d12 compatibility
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

RUN ls -la /usr/lib/x86_64-linux-gnu/dri/d3d12* && echo "d3d12 driver installed"

# =============================================================================
# STAGE 3: Install Gazebo Harmonic (now installed AFTER Mesa upgrade)
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gz-harmonic \
    libgz-sim8-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STAGE 4: Install MoveIt2, RViz2, and ROS2 packages (AFTER Mesa upgrade)
# =============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-octomap-server \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-point-cloud-transport \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-topic-tools \
    qtwayland5 \
    build-essential git python3-colcon-common-extensions iputils-ping \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

## No LIBGL_DRI3_DISABLE is set; DRI3 is enabled by default for best compatibility with WSLg/Wayland.
# =============================================================================
# STAGE 5: Build ros_gz AND gz_ros2_control for Harmonic
# =============================================================================
ENV GZ_VERSION=harmonic

RUN mkdir -p /root/gz_ros2_control_ws/src && \
    cd /root/gz_ros2_control_ws/src && \
    git clone https://github.com/gazebosim/ros_gz.git -b humble && \
    git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

WORKDIR /root/gz_ros2_control_ws
RUN apt-get update && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    rosdep install -r --from-paths src -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys="gz-cmake3 gz-common5 gz-fuel-tools9 gz-gui8 gz-math7 gz-msgs10 gz-physics7 gz-plugin2 gz-rendering8 gz-sensors8 gz-sim8 gz-tools2 gz-transport13 gz-utils2 ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-image ros_gz ros_gz_bridge ros_gz_sim ros_gz_image" \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    export GZ_VERSION=harmonic && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# =============================================================================
# STAGE 6: Workspace setup
# =============================================================================
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

COPY workspaces/moveit2_code $ROS2_WORKSPACE
COPY interfaces/robot_interfaces $ROS2_WORKSPACE/src/robot_interfaces

WORKDIR $ROS2_WORKSPACE

RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    . /root/gz_ros2_control_ws/install/setup.bash && \
    colcon build --packages-select fanuc_arm_config fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/gz_ros2_control_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source $ROS2_WORKSPACE/install/setup.bash" >> /root/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/container_entrypoint.sh"]
CMD ["bash"]
