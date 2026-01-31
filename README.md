# ROS2 Husky Navigation with Nav2

A complete ROS2 navigation stack for the Clearpath Husky robot, featuring autonomous navigation, SLAM mapping, and teleoperation capabilities using Nav2 and slam_toolbox.

## 🎯 Features

- **Autonomous Navigation** - Nav2-based path planning and obstacle avoidance
- **SLAM Mapping** - Real-time environment mapping with slam_toolbox
- **Gazebo Simulation** - Pre-configured simulation environments
- **Teleoperation** - Joystick and keyboard control support
- **Custom Controllers** - Tuned PID controllers for smooth motion
- **RViz Visualization** - Pre-configured dashboards for monitoring and debugging

## 📋 Prerequisites

- **ROS2** (Humble/Iron recommended)
- **Gazebo Classic** or **Gazebo Fortress**
- **Nav2** navigation stack
- **slam_toolbox**
- **VS Code** (optional, for devcontainer support)

## 🚀 Quick Start

### Using Dev Container (Recommended)

1. **Open in VS Code**
   ```bash
   code .
   ```

2. **Reopen in Container**
   - Press `F1` or `Ctrl+Shift+P`
   - Select "Dev Containers: Reopen in Container"
   - Wait for container to build (first time only)

3. **Build the Workspace**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### Manual Installation

1. **Clone the Repository**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   cd ~/ros2_ws
   ```

2. **Install Dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🎮 Usage

### 1. Gazebo Simulation

Launch Husky in a TurtleBot3 world:

```bash
ros2 launch husky_navigation husky_in_tb3_world.launch.py
```

### 2. SLAM Mapping

Create a map of your environment:

```bash
ros2 launch husky_navigation husky_slam.launch.py
```

**Controls:**
- Use teleoperation (see below) to drive the robot
- The map builds in real-time in RViz
- Save the map when complete:
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
  ```

### 3. Autonomous Navigation

Navigate using a pre-built map:

```bash
ros2 launch husky_navigation husky_navigation.launch.py map:=~/maps/my_map.yaml
```

**In RViz:**
- Click "2D Pose Estimate" to set initial position
- Click "Nav2 Goal" to send navigation goals

### 4. Teleoperation

Manual control with keyboard or joystick:

```bash
ros2 launch husky_navigation husky_teleop.launch.py
```

**Keyboard Controls:**
- `i` - Forward
- `k` - Stop
- `,` - Backward
- `j` - Turn left
- `l` - Turn right

## 📁 Repository Structure

```
.
├── .devcontainer/          # VS Code dev container configuration
│   ├── devcontainer.json   # Container specifications
│   └── Dockerfile          # ROS2 environment setup
│
├── config/                 # Configuration files
│   ├── husky_controllers.yaml      # Robot controller parameters
│   ├── nav2_params.yaml            # Nav2 stack configuration
│   └── slam_toolbox_config.yaml    # SLAM parameters
│
├── launch/                 # ROS2 launch files
│   ├── husky_control.launch.py     # Basic robot bringup
│   ├── husky_teleop.launch.py      # Teleoperation mode
│   ├── husky_slam.launch.py        # SLAM mapping
│   ├── husky_navigation.launch.py  # Autonomous navigation
│   ├── custom_nav2.launch.py       # Custom Nav2 configuration
│   └── husky_in_tb3_world.launch.py # Gazebo simulation
│
├── maps/                   # Map files
│   ├── example_map.pgm     # Sample occupancy grid
│   └── example_map.yaml    # Map metadata
│
├── meshes/                 # 3D models
│   ├── base_link.stl       # Husky chassis
│   ├── wheel.stl           # Wheel geometry
│   ├── bumper.stl          # Bumper mesh
│   └── accessories/        # Sensor mounts
│
├── rviz/                   # RViz configurations
│   ├── husky.rviz          # Robot monitoring dashboard
│   └── slam.rviz           # SLAM visualization
│
├── urdf/                   # Robot description
│   └── husky_ros2.urdf.xacro # Modular robot URDF
│
├── src/                    # Custom nodes (future)
├── slam.mkv                # SLAM mapping demo video
├── nav.mkv                 # Navigation demo video
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
└── README.md               # This file
```

## 🔄 Workflow Pipeline

```
┌─────────────────────────────────────────────────────────┐
│  1. Simulation (Testing)                                │
│     └─ husky_in_tb3_world.launch.py                     │
│        └─ Test robot behavior in Gazebo                 │
├─────────────────────────────────────────────────────────┤
│  2. Mapping (Environment Scanning)                      │
│     └─ husky_slam.launch.py                             │
│        └─ Create occupancy grid map                     │
├─────────────────────────────────────────────────────────┤
│  3. Navigation (Autonomous Operation)                   │
│     └─ husky_navigation.launch.py                       │
│        └─ Navigate using pre-built map                  │
├─────────────────────────────────────────────────────────┤
│  4. Teleoperation (Manual Control)                      │
│     └─ husky_teleop.launch.py                           │
│        └─ Manual control for testing/recovery           │
└─────────────────────────────────────────────────────────┘
```

## ⚙️ Configuration

### Controller Tuning

Edit PID gains in `config/husky_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz
    
diff_drive_controller:
  ros__parameters:
    linear:
      x:
        max_velocity: 1.0
        min_velocity: -1.0
```

### Navigation Parameters

Modify planner behavior in `config/nav2_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
```

### SLAM Tuning

Adjust mapping quality in `config/slam_toolbox_config.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    resolution: 0.05
    max_laser_range: 20.0
    minimum_travel_distance: 0.5
```

## 📹 Demo Videos

Watch these demonstrations to see the system in action:

### SLAM Mapping Demo
![SLAM Demo](slam.mp4)

Real-time map building as the robot explores the environment using slam_toolbox.

### Autonomous Navigation Demo
![Navigation Demo](nav.mp4)

Nav2 path planning and execution with dynamic obstacle avoidance.

> **Note**: Video files are located in the root directory of the repository. Download them to view the full demonstrations.

## 🔧 Troubleshooting

### Robot not moving in simulation

Check controller status:
```bash
ros2 control list_controllers
```

### SLAM not building map

Verify lidar data:
```bash
ros2 topic echo /scan
```

### Navigation failing

Check costmap updates:
```bash
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

### Transform errors

Verify TF tree:
```bash
ros2 run tf2_tools view_frames
```

## 🚧 Future Work

- [ ] **Dual UR5 Arm Integration** - Add two UR5 robotic arms to Husky platform
- [ ] **MoveIt2 Integration** - Arm manipulation and motion planning
- [ ] **Advanced Perception** - 3D mapping with depth cameras
- [ ] **Multi-Robot Coordination** - Fleet management capabilities
- [ ] **Custom Behaviors** - Application-specific behavior trees
- [ ] **Real Hardware Support** - Launch files for physical Husky robot

## 📚 Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Clearpath Husky](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)

## 📄 License

[Add your license here]

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 👥 Authors

[Add your name/contact here]

## 🙏 Acknowledgments

- Clearpath Robotics for the Husky platform
- Nav2 team for the navigation stack
- slam_toolbox developers
- ROS2 community

---

**Note**: This project is under active development. Features and documentation may change.
