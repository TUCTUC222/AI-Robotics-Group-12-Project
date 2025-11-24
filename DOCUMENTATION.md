# Vision-Based Target Tracking Robot

**Autonomous TurtleBot3 that uses computer vision and LIDAR to find and follow targets.**

---

## 🚀 How It Works (Simple Explanation)

This robot operates like a simple autonomous creature:

1.  **Seeing (Vision)**: It uses a front-mounted camera to look for **Cyan/Teal** objects. It ignores everything else (like walls or other robots).
2.  **Sensing (LIDAR)**: It uses a laser scanner to measure exactly how far away objects are, ensuring it doesn't crash.
3.  **Thinking (Control)**:
    *   **If it sees the target**: It turns to face it and drives forward.
    *   **If it loses the target**: It spins in place to search for it.
    *   **If it gets close (30cm)**: It stops automatically.

It combines these senses to smoothly track moving targets and stop safely.

---

## ⚡ Quick Start

**1. Build the Project**
```bash
cd ~/AI-Robotics-Group-12-Project
colcon build
source install/setup.bash
```

**2. Run the Simulation**
```bash
ros2 launch lidar_target_follower obstacle_course.launch.py
```

**3. Interact**
*   **Move the Target**: In Gazebo, use the "Translate" tool to drag the cyan cylinder. The robot will follow.
*   **View Camera**: Run `rqt_image_view` in a new terminal to see what the robot sees.

---

## 🔧 Technical Architecture

### Hardware & Sensors
*   **Robot Platform**: TurtleBot3 Burger (Differential Drive)
*   **Vision Sensor**: RGB Camera (640x480 @ 30Hz)
*   **Distance Sensor**: GPU LIDAR (360° scan @ 5Hz, 3.5m range)

### Software Stack
*   **OS**: Ubuntu 22.04 LTS
*   **Middleware**: ROS2 Humble
*   **Simulation**: Ignition Gazebo Fortress
*   **Vision Library**: OpenCV 4.5.4 (via `cv_bridge`)

### Control Logic (Priority System)
The robot makes decisions 20 times per second (20Hz) based on this hierarchy:

1.  **STOP (Priority 1)**: If target distance ≤ 0.3m (Target Reached).
2.  **EMERGENCY (Priority 2)**: If *any* object is < 0.25m (Collision Avoidance).
3.  **TRACK (Priority 3)**: If target detected in camera:
    *   Calculate error from image center.
    *   Apply proportional control (`angular_vel = -error * 1.5`).
    *   Adjust speed based on distance (0.3 m/s far, 0.1 m/s near).
4.  **SEARCH (Priority 4)**: If no target visible, rotate at 0.4 rad/s.

### Vision Pipeline
1.  **Input**: Raw RGB image from camera.
2.  **Preprocessing**: Convert to HSV color space.
3.  **Filtering**: Threshold for Cyan (Hue: 80-100).
4.  **Cleanup**: Apply Morphological Open/Close to remove noise.
5.  **Detection**: Find largest contour -> Calculate Centroid -> Output X-coordinate.

---

## 📂 Project Structure

```text
AI-Robotics-Group-12-Project/
├── src/
│   └── vision_target_follower_node.cpp  # Main C++ control node
├── launch/
│   └── obstacle_course.launch.py        # Launches Gazebo, Robot, and Nodes
├── models/                              # SDF Models
│   ├── turtlebot/                       # Robot with Camera + LIDAR
│   └── cyan_target/                     # The target object
├── worlds/
│   └── obstacle_course.world            # Simulation environment
└── CMakeLists.txt                       # Build configuration
```

---

## 🛠️ Installation & Setup

**Prerequisites**: Ubuntu 22.04, ROS2 Humble, Gazebo Fortress.

**1. Install Dependencies**
```bash
sudo apt update
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
                 ros-humble-cv-bridge libopencv-dev
```

**2. Setup Workspace**
```bash
mkdir -p ~/AI-Robotics-Group-12-Project/src
# Clone or copy files here
```

**3. Build**
```bash
colcon build --symlink-install
```

---

## ❓ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot spins in circles** | It cannot see the target. Ensure the Cyan Cylinder is in the world. |
| **Robot hits the wall** | LIDAR might be blocked. Check if `scan` topic is publishing. |
| **Build fails** | Ensure `cv_bridge` and `OpenCV` are installed (`sudo apt install libopencv-dev`). |
| **Gazebo crashes** | Try killing processes: `pkill -f "ign gazebo"`. |
