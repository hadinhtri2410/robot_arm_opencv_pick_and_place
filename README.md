# UR3e Pick-and-Place with Neural Motion Planning Data Collection

A ROS 2 Humble project that simulates a UR3e robot arm with a Robotiq Hand-E gripper performing pick-and-place tasks in Gazebo. Includes an automated data collection pipeline that randomizes scene layouts and records MoveIt RRTConnect trajectories as JSON files for training a neural motion planner.

## Prerequisites

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble Hawksbill
- **Simulator**: Gazebo Classic (gazebo11)

## 1. Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repo
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes Gazebo, RViz)
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 2. Install System Dependencies

```bash
# ROS 2 build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# MoveIt 2
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-planners \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-ros-warehouse

# Gazebo and ROS 2 control
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui

# UR robot packages
sudo apt install -y \
  ros-humble-ur-controllers

# TF and geometry
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf-transformations

# Perception
sudo apt install -y \
  ros-humble-cv-bridge \
  python3-opencv

# Other dependencies
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-warehouse-ros-mongo
```

## 3. Clone the Repository

```bash
mkdir -p ~/ros2/src
cd ~/ros2
git clone <this-repo-url> .
```

## 4. Clone External Dependencies

The following repositories must be cloned into the `src/` directory:

```bash
cd ~/ros2/src

# Universal Robots description (URDF/meshes)
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

# Universal Robots Gazebo simulation
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git

# Robotiq gripper (Hand-E support)
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git

# Link attacher plugin for Gazebo (grasp simulation)
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
```

## 5. Initialize rosdep and Install Remaining Dependencies

```bash
cd ~/ros2

# Initialize rosdep (only needed once)
sudo rosdep init  # skip if already initialized
rosdep update

# Install any missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## 6. Build the Workspace

```bash
cd ~/ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> **Tip**: Add `source ~/ros2/install/setup.bash` to your `~/.bashrc` so it loads automatically.

## 7. Running the Simulation

### Standard Pick-and-Place (single run)

```bash
source ~/ros2/install/setup.bash
ros2 launch robot_arm spawn_ur3e_camera_moveit.launch.py
```

This launches:
- Gazebo with the pick-and-place world (table, bins, blocks)
- Robot state publisher and controllers (arm + gripper)
- MoveIt move_group with OMPL RRTConnect planner
- RRT-Connect planning node
- Camera perception node
- RViz for visualization
- Pick-and-place execution node

### Data Collection Mode (multiple iterations)

To collect trajectory data for neural network training, set `num_iterations` to the desired count:

```bash
ros2 launch robot_arm spawn_ur3e_camera_moveit.launch.py num_iterations:=1000
```

Each iteration:
1. Returns the arm to the home position
2. Randomizes block and bin positions on the table
3. Executes the full pick-and-place sequence
4. Saves trajectory JSON files to the `trajectory_data/` directory

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `target_color` | `red` | Color to pick: `red`, `blue`, or `yellow` |
| `run_pick_and_place` | `true` | Whether to launch the pick-and-place node |
| `num_iterations` | `1` | Number of pick-and-place iterations |
| `output_dir` | `trajectory_data` | Directory for saved trajectory data |
| `pre_pick_offset` | `0.20` | Height (m) above block before descending |
| `pick_height_offset` | `0.145` | Height offset (m) from block z for grasp |
| `place_height_offset` | `0.18` | Height offset (m) from bin z for place |
| `gripper_close_target` | `0.02` | Gripper finger distance (m) when closing |

Example with custom parameters:

```bash
ros2 launch robot_arm spawn_ur3e_camera_moveit.launch.py \
  target_color:=blue \
  num_iterations:=500 \
  output_dir:=my_dataset
```

### Launch Without the Pick-and-Place Node

To only bring up the simulation and controllers (useful for manual testing via RViz):

```bash
ros2 launch robot_arm spawn_ur3e_camera_moveit.launch.py run_pick_and_place:=false
```

## 8. Output Data Format

Trajectory data is saved to `trajectory_data/` (or the directory specified by `output_dir`).

### Trajectory JSON Files

Each planning segment produces a JSON file named `traj_<ITER>_<LABEL>.json`:

| Label | Description |
|---|---|
| `pre_pick` | Joint-space move from home to above the block |
| `pick_descend` | Cartesian descent to grasp height |
| `place_lift` | Lift after grasping |
| `place_above_bin` | Joint-space move to above the bin |
| `place_retreat` | Retreat upward after releasing |

Each JSON contains:

```json
{
  "iteration": 0,
  "label": "pre_pick",
  "planning_time_ms": 51.95,
  "scene": {
    "block_x": 0.302, "block_y": 0.372,
    "bin_x": 0.407, "bin_y": 0.255
  },
  "joint_names": ["shoulder_pan_joint", "..."],
  "start_joints": [0.0, -1.57, "..."],
  "goal_joints": [1.2, -2.1, "..."],
  "num_waypoints": 84,
  "trajectory": [[0.0, -1.57, "..."], ["..."]]
}
```

### Iteration Summary CSV

A summary file `iteration_summary.csv` is generated with per-iteration statistics:

```
trial,planning_success,planning_time_s,num_waypoints
1,Yes,0.18,22
2,Yes,0.21,25
3,No,0.05,84
```

## Project Structure

```
ros2/
├── src/
│   ├── robot_arm/                  # Main package: launch files, URDF, models, C++ nodes
│   │   ├── src/
│   │   │   ├── pick_and_place_moveit.cpp   # Pick-and-place with data collection
│   │   │   └── pick_and_place.cpp          # Original pick-and-place (no MoveIt)
│   │   ├── launch/
│   │   │   └── spawn_ur3e_camera_moveit.launch.py
│   │   ├── urdf/                   # Robot URDF/xacro files
│   │   ├── models/                 # Gazebo SDF models (table, bins, blocks)
│   │   └── worlds/                 # Gazebo world files
│   ├── my_robot_interfaces/        # Custom ROS 2 service definitions
│   ├── my_arm_planning_py/         # Python RRT-Connect planning node
│   ├── opencv_perception/          # OpenCV-based perception nodes
│   ├── ur3e_camera_moveit_config/  # MoveIt configuration for UR3e + camera
│   ├── ros2_robotiq_gripper/       # Robotiq gripper description and controllers
│   ├── IFRA_LinkAttacher/          # Gazebo link attacher plugin
│   ├── Universal_Robots_ROS2_Description/      # UR robot URDF
│   └── Universal_Robots_ROS2_Gazebo_Simulation/ # UR Gazebo simulation
└── trajectory_data/                # Output directory for collected trajectories
```
