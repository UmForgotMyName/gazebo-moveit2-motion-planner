# 🤖 Fanuc LR Mate 200iC/5L Motion Planning Sandbox (ROS 2 Humble + MoveIt 2 + Gazebo Harmonic)

<div align="center">

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge) ![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Motion_Planning-6A4C93?style=for-the-badge) ![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-4B8BBE?style=for-the-badge) ![Docker](https://img.shields.io/badge/Docker-Containerized-0DB7ED?style=for-the-badge&logo=docker&logoColor=white) ![WSL2](https://img.shields.io/badge/WSL2-Ready-0A5E9A?style=for-the-badge)

**A ready-to-use manipulation platform that simulates a Fanuc LR Mate 200iC/5L with a custom gripper, featuring MoveIt 2 planning (OMPL), IKFast kinematics, and an optional D405 pointcloud to Octomap pipeline for collision-aware planning.**

</div>

---

## 📌 Table of contents
- [Overview](#-overview)
- [Key features](#-key-features)
- [System architecture](#-system-architecture)
- [Requirements](#-requirements)
- [Quick start](#-quick-start)
- [Launch options](#-launch-options)
- [Verify your setup](#-verify-your-setup)
- [Repo layout](#-repo-layout)
- [Key configuration files](#-key-configuration-files)
- [Guides](#-guides)
- [Roadmap](#-roadmap)
- [References](#-references)

---

## 🎯 Overview
This repository provides a manipulation sandbox built around:
- **ROS 2 Humble**
- **MoveIt 2** (RViz MotionPlanning panel + OMPL planning pipelines)
- **Gazebo Harmonic** simulation of the Fanuc LR Mate 200iC/5L + SG2
- Optional **D405 RGB-D perception** feeding **Octomap** for dynamic collision avoidance

Primary goals:
- A clean baseline for **motion planning + execution** in simulation
- A practical starting point for **perception-driven planning** (pointcloud to Octomap)
- A Docker workflow that behaves predictably under **WSL2 + WSLg**

---

## ✨ Key features
✅ Gazebo Harmonic simulation (stable LTS direction)  
✅ MoveIt 2 planning with OMPL + interactive RViz workflow  
✅ IKFast kinematics plugin for fast analytical IK (`fanuc_arm` group)  
✅ ros2_control JointTrajectoryController for execution  
✅ Optional D405 pointcloud to MoveIt Octomap (`PointCloudOctomapUpdater`)  
✅ Dockerized environment designed for WSL2 + WSLg graphics + GPU acceleration  

---


## 🧩 Components

| Component | Purpose |
|---|---|
| Fanuc LR Mate 200iC/5L (sim) | 6-DOF industrial arm model used for planning and execution |
| SG2 gripper (sim) | End-effector model for collision geometry and planning scene correctness |
| Gazebo Harmonic | Physics + rendering for simulation bringup |
| MoveIt 2 | PlanningScene, OMPL planning, execution via ros2_control |
| IKFast plugin | Analytical IK for fast planning and repeatable solutions |
| D405 RGB-D (sim, optional) | Publishes PointCloud2 for Octomap-based collision avoidance |
| Docker + WSL2 | Reproducible dev environment and WSLg GUI routing |

## 🧭 Typical workflow

1. Launch Gazebo + MoveIt + RViz
2. Use RViz interactive markers to set a goal pose
3. Plan with OMPL (IK via IKFast)
4. Execute through ros2_control to move the simulated robot
5. If Octomap is enabled, obstacles from the D405 pointcloud are inserted into the planning scene

## 🧱 System architecture
```text
┌─────────────────────────────┐
│         Gazebo (gz-sim)     │
│  Fanuc + SG2 + D405 sensor  │
└───────────────┬─────────────┘
                │ (gz msgs)
                ▼
┌─────────────────────────────┐
│        ros_gz_bridge         │
│  bridges images, depth, PC2  │
└───────────────┬─────────────┘
                │ (ROS 2 topics)
                ▼
┌────────────────────────────────────────────────────────────┐
│                         ROS 2                              │
│  ┌──────────────────────┐    ┌──────────────────────────┐  │
│  │      ros2_control    │    │        MoveIt 2          │  │
│  │  JointTrajectory Ctrl│    │  move_group + RViz UI    │  │
│  └──────────┬───────────┘    │  OMPL + IKFast           │  │
│             │                │  (optional) Octomap      │  │
│             │                └──────────┬───────────────┘  │
│             │                           │ (planning scene) │
│             ▼                           ▼                  │
│    Simulated execution        Collision-aware planning     │
└────────────────────────────────────────────────────────────┘
```

---

## 🧰 Requirements

### Supported environments
- Windows 11 + WSL2 + Docker Desktop (WSL integration enabled)
- Ubuntu 22.04 inside WSL2 (recommended)
- Native Ubuntu should also work, but the compose defaults are tuned for WSLg

### Hardware and GPU (recommended)
- NVIDIA GPU with WSL2 GPU support (`/dev/dxg` present)
- Enough RAM to run Gazebo + RViz smoothly (8 GB minimum recommended)

---

## 🚀 Quick start

### 1) Clone and start the container
```bash
git clone <repository-url>
cd gazebo-moveit2-motion-planner

# From WSL Ubuntu (not PowerShell)
xhost +local:docker

docker compose up --build -d
```

### 2) Enter the container
```bash
docker exec -it ros2-moveit2 bash
```

### 3) Launch with D405 + Octomap
```bash
ros2 launch fanuc_arm_config gazebo_moveit_octomap.launch.py
```

---

## 🧭 Launch options
- Gazebo + MoveIt + RViz (baseline): `gazebo_moveit_integration.launch.py`
- Gazebo + MoveIt + RViz + Octomap: `gazebo_moveit_octomap.launch.py`
- RViz only (visualize + plan): `moveit-rviz.launch.py`

---

## ✅ Verify your setup

### Confirm joint states
```bash
ros2 topic echo /joint_states --once
```

### Confirm controllers
```bash
ros2 control list_controllers
```

### Confirm actions
```bash
ros2 action list | sort
```

### GPU check (WSL2)
```bash
glxinfo -B
```

If GLX is failing, test software rendering once to confirm it is a driver path issue:
```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

---

## 🗂 Repo layout
```text
gazebo-moveit2-motion-planner/
├─ docker-compose.yml
├─ README.MD
├─ docker/
│  └─ ros2-moveit2.dockerfile
├─ docs/
│  ├─ README.md
│  ├─ troubleshooting.md
│  ├─ wslg-gpu.md
│  ├─ octomap-perception.md
│  └─ ikfast.md
├─ interfaces/
│  └─ robot_interfaces/
└─ workspaces/
   └─ moveit2_code/src/
      ├─ fanuc_arm_config/
      └─ fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin/
```

---

## 🧩 Key configuration files
- URDF: `workspaces/moveit2_code/src/fanuc_arm_config/urdf/fanuc200ic5l_w_sg2.urdf`
- SRDF: `workspaces/moveit2_code/src/fanuc_arm_config/config/lrmate200ic5l_with_sg2.srdf`
- ros2_control: `workspaces/moveit2_code/src/fanuc_arm_config/config/ros2_controllers.yaml`
- MoveIt controller mapping: `workspaces/moveit2_code/src/fanuc_arm_config/config/moveit_controllers.yaml`
- MoveIt sensors (Octomap): `workspaces/moveit2_code/src/fanuc_arm_config/config/sensors_3d.yaml`

---

## 📚 Guides
Deep-dive and operational docs live in `docs/`:
- `docs/README.md` - guide index
- `docs/troubleshooting.md` - the "something broke" checklist
- `docs/wslg-gpu.md` - WSLg display, Wayland-first setup, GPU checks
- `docs/octomap-perception.md` - D405 pointcloud to MoveIt Octomap, TF requirements, tuning
- `docs/ikfast.md` - how the IKFast plugin is wired in, and how to regenerate

---

## 🛠 Roadmap
High priority:
- [ ] `/compute_ik` service wrapper using the IKFast plugin
- [ ] Path planning service using MoveGroupInterface (plan + execute)

Medium priority:
- [ ] Real-time obstacle detection using RGB-D data
- [ ] Pointcloud preprocessing (filters and ROI cropping) before Octomap
- [ ] Pick-and-place demo

Low priority:
- [ ] Force/torque control experiments
- [ ] Alternative planning pipelines and benchmarking

---
## 🔗 References
- Gazebo Harmonic ogre-next fix: https://github.com/gazebosim/gz-sim/issues/2502
- WSLg GUI apps guidance: https://learn.microsoft.com/windows/wsl/tutorials/gui-apps
