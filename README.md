# Vision-Based Target Tracking Robot 🤖

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**Autonomous TurtleBot3 robot using computer vision and LIDAR sensor fusion to detect and track colored targets in simulation.**

![Robot Demo](https://img.shields.io/badge/Status-Working-brightgreen)

## 🎯 Overview

This project demonstrates autonomous target tracking using:
- **🎥 Computer Vision**: RGB camera with HSV color filtering to detect cyan/teal targets
- **📡 LIDAR Sensing**: 360° laser scanner for precise distance measurement and collision avoidance
- **🧠 Intelligent Control**: Multi-priority decision system for smooth navigation
- **🔄 Real-time Tracking**: Follows moving targets and searches when lost

## ✨ Key Features

- ✅ **Color-Based Detection**: Uses HSV color space to identify cyan targets
- ✅ **Sensor Fusion**: Combines camera vision with LIDAR distance data
- ✅ **Edge-Based Distance Control**: Intelligent approach system that maximizes proximity while keeping target fully in frame
- ✅ **Fast & Responsive**: Optimized speeds (up to 0.30 m/s approach) for quick target acquisition
- ✅ **Dynamic Tracking**: Actively follows moving targets with real-time centering
- ✅ **Search Behavior**: Automatically scans environment when target lost
- ✅ **User-Friendly HUD**: Plain English status updates with visual indicators - no technical jargon!
- ✅ **Stable Positioning**: Two-tier safety margin system prevents oscillation

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Fortress

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/TUCTUC222/AI-Robotics-Group-12-Project.git
   cd AI-Robotics-Group-12-Project
   ```

2. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
                    ros-humble-cv-bridge libopencv-dev
   ```

3. **Build the project:**
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```

### Running the Simulation

**Recommended - Run with Camera Viewer (All-in-One):**
```bash
./run_with_camera_viewer.sh
```
This automatically launches both the Gazebo simulation and camera viewer together!

**Alternative - Manual Launch:**
```bash
ros2 launch lidar_target_follower obstacle_course.launch.py

# In a new terminal to view camera
python3 view_camera.py
```

**What happens:**
1. Gazebo opens with the TurtleBot3 and a cyan target cylinder
2. Robot displays yellow glowing lines showing its 60° camera field of view
3. Camera viewer window shows live feed with detection overlays
4. Robot's camera detects the cyan color
5. Robot turns to center the target in its view
6. Robot maintains ~90cm optimal tracking distance
7. Try moving the target - the robot will actively follow it!

**The camera viewer displays:**
- 🎥 Live camera feed with target detection
- 🎯 Green crosshair at image center
- 🔵 Cyan contours around detected targets
- 📊 Real-time info (area, offset, turn direction)

## 📂 Project Structure

```
AI-Robotics-Group-12-Project/
├── src/
│   └── vision_target_follower_node.cpp  # Main control node (Vision + LIDAR)
├── launch/
│   └── obstacle_course.launch.py        # Simulation launcher
├── models/
│   ├── turtlebot/                       # TurtleBot3 with camera & LIDAR
│   ├── cyan_target/                     # Target cylinder model
│   └── turtlebot3_common/               # Mesh files
├── worlds/
│   └── obstacle_course.world            # Gazebo simulation environment
├── CMakeLists.txt                       # Build configuration
├── package.xml                          # ROS2 package metadata
├── DOCUMENTATION.md                     # Detailed technical documentation
└── README.md                            # This file
```

## 🔧 How It Works

### Simple Explanation
1. **Camera** detects cyan/teal colored targets using HSV color filtering
2. **LIDAR** measures exact distance to objects for collision avoidance
3. **Control System** makes decisions 20 times per second:
   - If target visible → Center it horizontally while approaching
   - If target edges near frame border → Stop approaching (hold position)
   - If target edges clipped → Back up to keep fully visible
   - If target lost → Spin in place to search

### Technical Details

**Vision Pipeline:**
- Convert RGB to HSV color space (better for color detection)
- Threshold for cyan (Hue: 80-100, Saturation: 50-255)
- Apply morphological operations (closing & opening) to reduce noise
- Find contours and calculate bounding box
- Detect edge positions and visibility status
- Output normalized X-position for steering control

**Edge-Based Distance Control:**
The robot uses a sophisticated two-tier margin system:
- **Safety Margin (15px)**: Stop approaching when target edges are within 15 pixels of frame border
- **Clip Margin (2px)**: Back up if target edges are actually clipped (within 2 pixels)
- **Hold Zone**: Between margins, robot holds position without oscillating
- **Approach Logic**: Only approach when edges are safely within the safety margin AND target fills <60% of FOV

This prevents the oscillation problem of percentage-based systems while maximizing proximity to the target.

**Control Logic (Priority System):**
1. **Priority 1**: Emergency (object <30cm) → EMERGENCY STOP
2. **Priority 2**: Target edges clipped → BACK UP (-0.20 m/s)
3. **Priority 3**: Target too small (<20% FOV) → APPROACH FAST (0.30 m/s)
4. **Priority 4**: Target edges safe + small (<60% FOV) → APPROACH MEDIUM (0.15 m/s)
5. **Priority 5**: Target edges safe + large (≥60% FOV) → HOLD POSITION
6. **Priority 6**: Target edges in danger zone → HOLD POSITION
7. **Priority 7**: Target off-center → TURN + APPROACH MEDIUM (0.15 m/s)
8. **Priority 8**: No target → SEARCH (rotate 0.4 rad/s)

**Sensors:**
- RGB Camera: 640×480 @ 30Hz
- GPU LIDAR: 360° @ 5Hz, 3.5m range

## 📊 Technical Specifications

| Component | Specification |
|-----------|--------------|
| **Robot Platform** | TurtleBot3 Burger (Differential Drive) |
| **Camera** | 640×480 pixels, 30 Hz, 60° FOV |
| **LIDAR** | 360° scan, 5 Hz, 0.12-3.5m range |
| **Control Rate** | 20 Hz |
| **Target Detection** | HSV color filtering (Cyan: H=80-100) |
| **Edge Detection** | 15px safety margin, 2px clip margin |
| **Distance Control** | Edge-based approach with stability zones |
| **Approach Speed** | Fast: 0.30 m/s, Medium: 0.15 m/s, Backup: -0.20 m/s |
| **Turn Control** | Proportional with size-based damping |
| **Emergency Stop** | <0.30m distance threshold |

## 🎮 Interactive Commands

**View the camera feed with user-friendly HUD:**
```bash
python3 view_camera.py
```
Shows live camera view with modern interface:
- **Modern Rounded Panels**: Clean dark backgrounds with elegant rounded corners
- **Robot Status**: What the robot is doing in plain English (e.g., "Getting closer to target")
- **Visual Speed Bar**: Shows movement intensity at a glance
- **Target Info**: Distance, size, and alignment status
- **Visual Indicators**: Arrows for turn direction, circle for centered [OK]
- **Color-Coded Status**: Green (good), Cyan (moving), Orange (searching), Red (warning)
- **Minimalist Design**: No borders or clutter - thin typography, subtle separators
- **Dual Windows**: Main camera view + cyan detection mask

**Monitor LIDAR data:**
```bash
ros2 topic echo /scan
```

**Check velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Move the target:** Use Gazebo's "Translate" tool to drag the cyan cylinder

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot spins in circles | Target not visible. Ensure cyan cylinder is in the world |
| Robot doesn't move | Check if camera/LIDAR topics are publishing: `ros2 topic list` |
| Build fails | Install dependencies: `sudo apt install libopencv-dev ros-humble-cv-bridge` |
| Gazebo crashes | Kill processes: `pkill -f "ign gazebo"` and restart |

## 📚 Documentation

For detailed technical documentation, see [DOCUMENTATION.md](DOCUMENTATION.md)

## 🤝 Contributing

This is an academic project for AI Robotics coursework. Feel free to fork and experiment!

## 📝 License

This project is available for educational purposes.

## 👥 Authors

- **Group 12** - AI Robotics Course Project

## 🙏 Acknowledgments

- TurtleBot3 models from [ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3)
- ROS2 and Gazebo communities
- OpenCV library for computer vision

## Running the Simulation

### Option 1: Using the Launch File (Recommended)
```bash
source install/setup.bash
ros2 launch lidar_target_follower target_follower.launch.py
```

This will:
1. Start Gazebo with the custom world
2. Spawn the robot with LIDAR sensor
3. Start the target follower node
4. Bridge data between Gazebo and ROS2

### Option 2: Manual Step-by-Step Launch

**Terminal 1 - Start Gazebo:**
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models
gz sim -r -s worlds/target_follower.world
```

**Terminal 2 - Start Gazebo GUI:**
```bash
gz sim -g
```

**Terminal 3 - Spawn Robot:**
```bash
gz service -s /world/target_follower_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "models/follower_robot/model.sdf", name: "follower_robot"'
```

**Terminal 4 - Bridge LIDAR:**
```bash
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

**Terminal 5 - Bridge cmd_vel:**
```bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

**Terminal 6 - Run Follower Node:**
```bash
source install/setup.bash
ros2 run lidar_target_follower target_follower_node
```

## Monitoring and Debugging

### View LIDAR Data
```bash
ros2 topic echo /lidar
```

### View Velocity Commands
```bash
ros2 topic echo /cmd_vel
```

### Visualize LIDAR in RViz2
```bash
ros2 run rviz2 rviz2
```
Add a LaserScan display and set the topic to `/lidar`.

### Check Node Info
```bash
ros2 node info /target_follower_node
```

### Adjust Parameters at Runtime
```bash
ros2 param set /target_follower_node linear_speed 0.8
ros2 param set /target_follower_node angular_speed 1.0
```

## How It Works

1. **Detection Phase**: 
   - The LIDAR sensor scans 360° around the robot
   - The node identifies the closest valid reading within range
   - This closest point is assumed to be the target object

2. **Tracking Phase**:
   - The angle to the target is calculated from the LIDAR index
   - The robot rotates to face the target
   - Once aligned (within ~17°), it moves forward

3. **Distance Control**:
   - **Too far (>2.5m)**: Move at full speed
   - **Good range (0.8-2.5m)**: Proportional speed based on distance
   - **Too close (<0.8m)**: Stop or back up slightly

4. **Search Mode**:
   - If no target is detected, the robot slowly rotates
   - This helps it find the target if it's lost

## Customization

### Modify Robot Behavior
Edit `src/target_follower_node.cpp` and adjust:
- Detection algorithm (e.g., filter by angle range)
- Control gains for smoother/faster movement
- Distance thresholds for following behavior

### Change Robot Model
Edit `models/follower_robot/model.sdf`:
- Adjust dimensions and mass for different dynamics
- Modify LIDAR parameters (range, samples, update rate)
- Add additional sensors (camera, IMU, etc.)

### Create New Worlds
Edit or create new `.world` files in `worlds/`:
- Add more obstacles
- Change target movement pattern
- Modify lighting and environment

## Troubleshooting

### Gazebo doesn't find the robot model
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models
```

### Robot doesn't move
- Check that bridges are running: `ros2 topic list | grep cmd_vel`
- Verify target follower node is publishing: `ros2 topic echo /cmd_vel`
- Check Gazebo GUI to ensure robot spawned correctly

### LIDAR data not available
- Verify bridge is running: `ros2 topic list | grep lidar`
- Check Gazebo sensor manager plugin is loaded in world file
- Try visualizing LIDAR in Gazebo GUI (enable sensor visualization)

### Build errors
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --cmake-clean-cache
```

## License

MIT License

## Contributors

Group 12 - AI Robotics Project
