# ROS2 Husky Navigation with Nav2

## Features
- Husky robot navigation using ROS2 Nav2
- SLAM mapping with slam_toolbox
- Teleoperation capabilities
- RViz configurations for visualization

## Quick Start
Use the devcontainer folder to create a container in vscode 

## Videos
1. **Navigation Demo**: [slam.mkv]
2. **SLAM Mapping Demo**: [nav.mkv]

## Future Work
- Integration of UR5 robotic arm (dual arm support)
- MoveIt2 integration for arm manipulation
- ...

```markdown
# Repository Structure

## 📁 `.devcontainer/`
Development container configuration for reproducible environment setup
- `devcontainer.json` → VS Code development container specifications
- `Dockerfile` → Container image definition with ROS2 and dependencies

## 📁 `config/`
Robot and navigation parameter configurations
- `husky_controllers.yaml` → PID controller gains and joint limits for Husky
- `nav2_params.yaml` → Navigation parameters (planners, controllers, recovery behaviors)
- `slam_toolbox_config.yaml` → SLAM configuration (scan matching, optimization)

## 📁 `launch/`
ROS2 launch files for different operational modes
- `husky_control.launch.py` → Basic robot bringup with controllers
- `husky_teleop.launch.py` → Joystick/keyboard teleoperation
- `husky_slam.launch.py` → SLAM mapping with real-time visualization
- `husky_navigation.launch.py` → Autonomous navigation with pre-built map
- `custom_nav2.launch.py` → Customized Nav2 stack with specific configurations
- `husky_in_tb3_world.launch.py` → Gazebo simulation in TurtleBot3 world

## 📁 `maps/`
Pre-built and example environment maps
- `example_map.pgm` → Sample occupancy grid map (PNG format)
- `example_map.yaml` → Map metadata (resolution, origin, thresholds)

## 📁 `meshes/`
3D model files for robot visualization
- `base_link.stl` → Husky chassis mesh
- `wheel.stl` → Wheel geometry
- `bumper.stl` → Bumper collision mesh
- `accessories/` → Additional sensor mounts and accessories

## 📁 `rviz/`
RViz2 visualization configurations
- `husky.rviz` → Robot monitoring dashboard with sensor displays
- `slam.rviz` → SLAM-specific visualization with map, scans, and pose

## 📁 `urdf/`
Robot description files
- `husky_ros2.urdf.xacro` → Modular URDF with macros for sensors and extensions

## 📁 `src/`
Custom ROS2 nodes and packages (if any)
- *(Placeholder for future custom controllers, planners, or utilities)*

## 📄 Root Files
- `CMakeLists.txt` → Build system configuration
- `package.xml` → ROS2 package metadata and dependencies
- `README.md` → Project documentation (this file)
- `.gitignore` → Git exclusion patterns
- `.gitattributes` → Git LFS configuration (if using large mesh files)

## Key File Relationships:
```
┌─────────────────────────────────────────────────────────┐
│   URDF (husky_ros2.urdf.xacro)                          │
│   └── Defines robot geometry & joints                   │
├─────────────────────────────────────────────────────────┤
│   Launch Files                                          │
│   ├── husky_slam.launch.py → Uses:                      │
│   │   ├── config/slam_toolbox_config.yaml               │
│   │   └── rviz/slam.rviz                                │
│   ├── husky_navigation.launch.py → Uses:                │
│   │   ├── config/nav2_params.yaml                       │
│   │   ├── maps/example_map.yaml                         │
│   │   └── rviz/husky.rviz                               │
└─────────────────────────────────────────────────────────┘
```

## Workflow Pipeline:
1. **Simulation**: `husky_in_tb3_world.launch.py` → Test in Gazebo
2. **Mapping**: `husky_slam.launch.py` → Create environment map
3. **Navigation**: `husky_navigation.launch.py` → Autonomous navigation
4. **Teleoperation**: `husky_teleop.launch.py` → Manual control for testing
```
```

