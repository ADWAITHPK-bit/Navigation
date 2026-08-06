IGVC Navigation Stack — IVDC Autonomy DivisionNavigation and motion-planning software for the Intelligent Ground Vehicle Competition (IGVC) autonomous platform, built on ROS 2 Humble, Nav2, and Gazebo.This repository covers the Navigation sub-team's stack: simulation, SLAM, global/local costmaps, route planning, behavior tree recovery, and perception integration interfaces. System Status & OverviewSimulation: Gazebo world with textured grass ground, painted double white lines, cones, barrels, and textured cylinders (tyres).SLAM: Cartographer running live for simultaneous mapping and localization.Costmaps: Embedded global and local costmap layers running directly inside the planner and controller servers.Planning & Control: $A^*$ (NavfnPlanner) for global path search paired with Regulated Pure Pursuit (RegulatedPurePursuitController) for local trajectory following.Behavior & Lifecycle: bt_navigator managing recovery actions (spin, wait, backup) with automated lifecycle state transitions.Current Platform: TurtleBot3 Waffle (used as a temporary stand-in chassis for its front camera until ported to the real competition vehicle). System Architecture   ┌─────────────────────────────────────────────────────────┐
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
   │        Global Planner (A*) ──► Local Controller          │
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
   │                Lifecycle Manager & Goal                 │
   │       Manages nodes & receives goals via igvc_goal_logic│
   └────────────────────────────┴────────────────────────────┘
 Key Technical DetailsCartographer vs. AMCL: AMCL was dropped because it requires a pre-built map. IGVC requires mapping on the fly, so Cartographer runs continuously to build and localize within the map as the robot moves.Embedded Costmaps: Standalone costmap nodes were eliminated—planner and controller servers each instantiate their own internal costmaps directly from YAML configs.Corner-Clipping Fix: Adjusted Pure Pursuit settings by decreasing lookahead_dist and increasing regulated_linear_scaling_min_radius to force earlier slowdowns on tight turns.TF Extrapolation Fix: Added transform_tolerance: 0.5 to handle high CPU load drops during testing.🛠 Repository Structureigvc_ws/src/
├── igvc_sim/          # Gazebo world files, spawn scripts, and robot descriptions
├── igvc_mapping/      # Cartographer configuration and SLAM launch files
├── igvc_global_planner# NavfnPlanner (A*) configuration and launch files
├── igvc_local_planner # Regulated Pure Pursuit controller parameters and launch
├── igvc_bringup/      # Master launch, bt_navigator, behavior_server, lifecycle
├── igvc_goal_logic/   # Hardcoded test goals and perception target generator
└── igvc_teleop/       # Custom WASD teleoperation scripts
World
<img width="1281" height="614" alt="image" src="https://github.com/user-attachments/assets/02e5f767-d93e-4a27-b4fa-a05eeaa95bdf" />
 Environment SetupInstall required dependencies and set up the model environment:Bash# Install dependencies
sudo apt update && sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description \
  ros-humble-cv-bridge \
  git-lfs

pip install opencv-python --break-system-packages

# Set default model
export TURTLEBOT3_MODEL=waffle
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# Build workspace
cd ~/igvc_ws
colcon build --symlink-install
source install/setup.bash
🚀 How to RunLaunch the stack using four terminals in this order:Bash# Terminal 1: Launch simulation
ros2 launch igvc_sim sim_launch.py

# Terminal 2: Launch SLAM
ros2 launch igvc_mapping cartographer_launch.py

# Terminal 3: Drive once manually to map the track area
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Launch Navigation stack
ros2 launch igvc_bringup bringup_launch.py
