# Drift Robot — Home Tidying Simulation

A complete ROS 2 + Gazebo Classic simulation of a home-tidying robot that
navigates a two-room environment and performs pick-and-place of small objects.

---

## Requirements

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Gazebo | Classic 11 |
| Python | 3.10+ |

---

## Installation

### 1. Install ROS 2 Humble (if not already installed)

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Install Gazebo Classic 11 + ROS bridge

```bash
sudo apt install -y \
  gazebo11 \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions
```

### 3. (Optional) Link-attacher plugin for pick-and-place

```bash
sudo apt install -y ros-humble-gazebo-ros-link-attacher
```

> If unavailable, the pick-and-place node gracefully degrades to a simulated
> attach (object is not physically moved but the sequence is logged).

### 4. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/drift_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running

Single launch command — spawns Gazebo, the robot, navigator, and pick-place:

```bash
source /opt/ros/humble/setup.bash
source ~/drift_ws/install/setup.bash
ros2 launch drift_robot simulation.launch.py
```

The simulation completes its navigation loop in under 5 minutes of sim time.

### Verify sensor output

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/drift_ws/install/setup.bash

# Camera feed
ros2 topic hz /drift_robot/camera/image_raw

# LiDAR
ros2 topic hz /drift_robot/scan

# Odometry
ros2 topic echo /drift_robot/odom
```

---

## Repository Layout

```
drift_robot/
├── urdf/
│   └── drift_robot.urdf.xacro   # Full robot model
├── worlds/
│   └── home.world                # Two-room SDF world
├── launch/
│   └── simulation.launch.py      # Single-command launch
├── scripts/
│   ├── navigator.py              # Waypoint navigator + LiDAR avoidance
│   └── pick_place.py             # Pick-and-place state machine
├── config/                       # (reserved for future controller YAML)
├── docs/
│   └── APPROACH.md
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Demo Video

<!-- Replace with your link -->
[Simulation demo — YouTube/Loom link here]

---

## Known Issues / Troubleshooting

**Robot falls through ground on spawn**
→ Check that `z_pos` arg is ≥ 0.10. Wheels have radius 0.09 m.

**`xacro` parse error**
→ Ensure `ros-humble-xacro` is installed: `sudo apt install ros-humble-xacro`

**Gazebo window doesn't open**
→ Gazebo client and server are separate; if running headless just ignore the
  gzclient error and monitor via `ros2 topic echo`.

**Navigator not moving**
→ Confirm the robot spawned: `ros2 topic echo /drift_robot/odom` should show
  non-zero stamp after ~5 s.