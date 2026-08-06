# IGVC Navigation Stack — IVDC Autonomy Division

Navigation and motion-planning software for the Intelligent Ground Vehicle Competition (IGVC) autonomous platform, built on **ROS 2 Humble**, **Nav2**, and **Gazebo**.

This repository contains the complete Navigation sub-team software stack developed for the **IVDC Autonomy Division**, including simulation, SLAM, planning, control, behavior trees, lifecycle management, and interfaces for perception-based goal generation.

---

#  System Overview

The navigation stack is designed for autonomous outdoor navigation using online SLAM and the ROS2 Navigation2 framework.

### Features

-  **Simulation**
  - Gazebo simulation with textured grass terrain
  - Double white lane markings
  - Traffic cones
  - Barrels
  - Textured tyre obstacles

-  **Online Mapping**
  - Cartographer SLAM
  - Simultaneous localization and mapping
  - No pre-built map required

-  **Navigation**
  - NavfnPlanner (A*)
  - Regulated Pure Pursuit Controller
  - Dynamic obstacle avoidance through embedded costmaps

-  **Behavior Trees**
  - Spin recovery
  - Wait recovery
  - Backup recovery

- ⚙ **Lifecycle Management**
  - Automatic lifecycle transitions
  - Managed startup and shutdown

-  **Goal Interface**
  - Supports hardcoded goals
  - Ready for perception-generated goals

-  **Current Robot**
  - TurtleBot3 Waffle
  - Temporary platform before deployment to the real IGVC vehicle

---

#  Navigation Architecture

```text
   ┌─────────────────────────────────────────────────────────┐
   │                  Sensors & Perception                   │
   │           (LiDAR, Camera, Odom, /camera/image_raw)      │
   └────────────────────────────┬────────────────────────────┘
                                │
                                ▼
   ┌─────────────────────────────────────────────────────────┐
   │                Cartographer (Live SLAM)                 │
   │               Publishes: /map, map ──► odom             │
   └────────────────────────────┬────────────────────────────┘
                                │
                                ▼
   ┌─────────────────────────────────────────────────────────┐
   │       Embedded Costmaps (planner & controller)          │
   │                   Consumes: /scan                       │
   └────────────────────────────┬────────────────────────────┘
                                │
                                ▼
   ┌─────────────────────────────────────────────────────────┐
   │        Global Planner (A*) ──► Local Controller         │
   │   (nav2_navfn_planner)      (Regulated Pure Pursuit)    │
   └──────────────┬──────────────────────────┬───────────────┘
                  │                          │
                  ▼                          ▼
   ┌─────────────────────────────────────────────────────────┐
   │              bt_navigator + behavior_server             │
   │            (Recovery: Spin / Wait / Backup)             │
   └────────────────────────────┬────────────────────────────┘
                                │
                                ▼
   ┌─────────────────────────────────────────────────────────┐
   │            Lifecycle Manager & Goal Logic               │
   │     Manages Nav2 nodes and receives navigation goals    │
   └────────────────────────────┴────────────────────────────┘
```

---

#  Design Decisions

## Cartographer Instead of AMCL

AMCL requires a static pre-built occupancy map before localization can begin.

IGVC is a mapping challenge where the environment is unknown beforehand, making AMCL unsuitable.

Instead, **Cartographer** performs **Simultaneous Localization and Mapping (SLAM)**, continuously generating the map while estimating the robot pose.

---

## Embedded Costmaps

Instead of launching independent costmap nodes, both the planner server and controller server instantiate their own internal costmaps directly from their YAML configuration files.

This reduces unnecessary nodes and follows the standard Nav2 architecture.

---

## Corner-Clipping Fix

During testing, the robot occasionally cut tight corners.

This was corrected by

- decreasing

```yaml
lookahead_dist
```

- increasing

```yaml
regulated_linear_scaling_min_radius
```

allowing the controller to slow down earlier before sharp turns.

---

## TF Extrapolation Fix

High CPU load occasionally caused transform lookup failures.

Adding

```yaml
transform_tolerance: 0.5
```

provided sufficient tolerance for delayed transforms.

---

#  Repository Structure

```text
igvc_ws/
└── src/
    ├── igvc_sim/
    │   ├── Gazebo worlds
    │   ├── Robot description
    │   └── Spawn scripts
    │
    ├── igvc_mapping/
    │   ├── Cartographer configuration
    │   └── SLAM launch files
    │
    ├── igvc_global_planner/
    │   ├── NavfnPlanner parameters
    │   └── Planner launch files
    │
    ├── igvc_local_planner/
    │   ├── Regulated Pure Pursuit parameters
    │   └── Controller launch files
    │
    ├── igvc_bringup/
    │   ├── Navigation launch
    │   ├── bt_navigator
    │   ├── behavior_server
    │   └── lifecycle_manager
    │
    ├── igvc_goal_logic/
    │   ├── Test goal publisher
    │   └── Perception goal interface
    │
    └── igvc_teleop/
        └── Custom keyboard teleoperation
```

---

# ⚙ Requirements

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- Nav2
- Cartographer
- TurtleBot3 packages

---

#  Installation

Update package lists

```bash
sudo apt update
```

Install ROS packages

```bash
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-teleop-twist-keyboard \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-cv-bridge \
    git-lfs
```

Install Python dependency

```bash
pip install opencv-python --break-system-packages
```

Set TurtleBot3 model

```bash
export TURTLEBOT3_MODEL=waffle
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
```

Build the workspace

```bash
cd ~/igvc_ws

colcon build --symlink-install

source install/setup.bash
```

---

#  Running the Navigation Stack

Launch the system using four terminals.

---

## Terminal 1 — Gazebo Simulation

```bash
ros2 launch igvc_sim sim_launch.py
```

---

## Terminal 2 — Cartographer SLAM

```bash
ros2 launch igvc_mapping cartographer_launch.py
```

---

## Terminal 3 — Manual Mapping

Drive around once to allow Cartographer to build an initial map.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Terminal 4 — Navigation Stack

```bash
ros2 launch igvc_bringup bringup_launch.py
```

---

#  Navigation Pipeline

```
Camera / LiDAR
        │
        ▼
 Cartographer SLAM
        │
        ▼
 Occupancy Grid
        │
        ▼
 Embedded Costmaps
        │
        ▼
 Global Planner (A*)
        │
        ▼
 Local Controller
(Regulated Pure Pursuit)
        │
        ▼
 Behavior Tree
        │
        ▼
 Robot Motion
```

---

# 🔧 Navigation Components

| Component | Package |
|-----------|---------|
| SLAM | Cartographer |
| Global Planner | NavfnPlanner (A*) |
| Local Controller | Regulated Pure Pursuit |
| Costmaps | Nav2 Embedded Costmaps |
| Behavior Tree | bt_navigator |
| Recovery Behaviors | behavior_server |
| Lifecycle | lifecycle_manager |
| Simulation | Gazebo Classic |
| Robot | TurtleBot3 Waffle |

---

---
- Cartographer
- Gazebo Classic
- TurtleBot3
