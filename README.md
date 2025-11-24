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
- ✅ **Bug2 Pathfinding**: Intelligent wall-following algorithm to navigate around obstacles blocking the target
- ✅ **Vision-Assisted Navigation**: Camera stays oriented towards walls during obstacle avoidance
- ✅ **Edge-Based Distance Control**: Intelligent approach system that maximizes proximity while keeping target fully in frame
- ✅ **Fast & Responsive**: Optimized speeds (up to 0.30 m/s approach) for quick target acquisition
- ✅ **Dynamic Tracking**: Actively follows moving targets with real-time centering
- ✅ **Search Behavior**: Automatically scans environment when target lost (turns in place)
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
2. Camera viewer window shows live feed with detection overlays
3. Robot's camera detects the cyan color
4. Robot turns to center the target in its view
5. Robot approaches until target edges are near frame borders (maximizing proximity)
6. Try moving the target - the robot will actively follow it!

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
2. **LIDAR** measures exact distance to objects for simple obstacle avoidance
3. **Control System** makes decisions 20 times per second:
   - If obstacle too close (<0.5m) → STOP (won't hit anything!)
   - If obstacle ahead (0.5-0.8m) → Slow down significantly
   - If target visible → Center it horizontally while approaching
   - If target edges near frame border → Stop approaching (hold position)
   - If target edges clipped → Back up to keep fully visible
   - If target lost → Turn in place to search
   
**Note:** Robot uses reactive obstacle avoidance - it stops before hitting things but doesn't navigate around obstacles to reach blocked targets.

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
1. **Priority 1**: Obstacle too close (<0.5m) → STOP COMPLETELY
2. **Priority 2**: Target edges clipped → BACK UP (-0.20 m/s)
3. **Priority 3**: Obstacle ahead (0.5-0.8m) → SLOW DOWN (0.05 m/s)
4. **Priority 4**: Target too small (<20% FOV) → APPROACH FAST (0.30 m/s)
5. **Priority 5**: Target edges safe + small (<60% FOV) → APPROACH MEDIUM (0.15 m/s)
6. **Priority 6**: Target edges safe + large (≥60% FOV) → HOLD POSITION
7. **Priority 7**: Target edges in danger zone → HOLD POSITION
8. **Priority 8**: Target off-center → TURN + APPROACH MEDIUM (0.15 m/s)
9. **Priority 9**: No target → SEARCH (rotate in place 0.4 rad/s)

**Sensors:**
- RGB Camera: 640×480 @ 30Hz
- GPU LIDAR: 360° @ 5Hz, 3.5m range

## 📊 Technical Specifications

| Component | Specification |
|-----------|--------------|
| **Robot Platform** | TurtleBot3 Burger (Differential Drive) |
| **Camera** | 640×480 pixels, 30 Hz, 90° FOV |
| **LIDAR** | 360° scan, 5 Hz, 0.12-3.5m range |
| **Control Rate** | 20 Hz |
| **Target Detection** | HSV color filtering (Cyan: H=80-100) |
| **Edge Detection** | 15px safety margin, 2px clip margin |
| **Distance Control** | Edge-based approach with stability zones |
| **Obstacle Avoidance** | Stop: <0.5m, Slow: 0.5-0.8m, Clear: >0.8m |
| **Approach Speed** | Fast: 0.30 m/s, Medium: 0.15 m/s, Slow: 0.05 m/s, Backup: -0.20 m/s |
| **Turn Control** | Proportional with size-based damping (gain: 2.5) |
| **Search Rotation** | 0.4 rad/s when target lost |

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
