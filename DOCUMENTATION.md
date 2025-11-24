# Vision-Based Target Tracking Robot

**Autonomous TurtleBot3 that uses computer vision and LIDAR to find and follow targets.**

---

## 🚀 How It Works (Simple Explanation)

This robot operates like a simple autonomous creature:

1.  **Seeing (Vision)**: It uses a front-mounted camera (90° FOV) to look for **Cyan/Teal** objects. It ignores everything else (like walls or other robots).
2.  **Sensing (LIDAR)**: It uses a laser scanner to measure exactly how far away objects are for simple obstacle avoidance.
3.  **Thinking (Control)**:
    *   **If obstacle too close (<0.5m)**: STOPS completely (won't hit anything!)
    *   **If obstacle ahead (0.5-0.8m)**: Slows down to be careful
    *   **If it sees the target**: It turns to center it horizontally while approaching.
    *   **Edge-based distance control**: Instead of using fixed distances, it approaches until the target edges are near the frame border, then stops.
    *   **If target edges clip**: Backs up to keep the full target visible.
    *   **If it loses the target**: Turns in place to search for it (3-phase search behavior).

**Bug2 Wall Following:**
When an obstacle blocks the path to the target, the robot uses the Bug2 algorithm:
- **Detect Obstacle**: When the robot gets too close to an obstacle while searching for the target (<0.5m), it enters Bug2 mode
- **Rotate to Wall**: Turns 60° (left or right, depending on which side has more space) to orient the camera towards the wall
- **Follow Wall**: Moves forward while maintaining a fixed distance from the wall (0.4m) using LIDAR feedback
- **Camera Bias**: Applies a rotational bias to keep the camera angled towards the wall for vision-assisted navigation
- **Exit Condition**: When the target becomes visible again and the path is clear, returns to normal tracking mode

This allows the robot to intelligently navigate around obstacles to reach targets that would otherwise be unreachable.

**Smart Distance Control:**
The robot uses a two-tier margin system to get as close as possible without the target leaving the camera frame:
- **15px Safety Margin**: Stops approaching when target edges are within 15 pixels of frame border
- **2px Clip Margin**: Backs up if target edges actually reach the frame border (clipped)
- **Hold Zone**: Between these margins, the robot holds steady without oscillating

This intelligent approach maximizes proximity while ensuring the target stays fully visible at all times.

---

## ⚡ Quick Start

**1. Build the Project**
```bash
cd ~/AI-Robotics-Group-12-Project
colcon build
source install/setup.bash
```

**2. Run the Simulation (Recommended)**
```bash
./run_with_camera_viewer.sh
```
This single command launches both Gazebo simulation and the camera viewer!

**Alternative - Manual Launch:**
```bash
# Terminal 1: Launch simulation
ros2 launch lidar_target_follower obstacle_course.launch.py

# Terminal 2: View camera feed
python3 view_camera.py
```

**3. Interact**
*   **Move the Target**: In Gazebo, use the "Translate" tool to drag the cyan cylinder. The robot will actively follow and maintain distance.
*   **Camera Viewer**: Shows live feed with modern HUD displaying target detection, contours, crosshair, and tracking info in plain English.

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

### Control Logic (Bug2 State Machine)
The robot operates in different states based on what it perceives:

**State 1: GO_TO_GOAL** (Normal Vision Tracking)
1.  **OBSTACLE TOO CLOSE & TARGET VISIBLE**: If obstacle < 0.5m while tracking → SWITCH TO BUG2 MODE.
2.  **OBSTACLE TOO CLOSE**: If *any* object is < 0.5m → STOP COMPLETELY (prevents collisions).
3.  **EDGES CLIPPED**: If target edges are within 2px of frame border → BACK UP at -0.20 m/s.
4.  **OBSTACLE AHEAD**: If object < 0.8m → SLOW DOWN to 0.05 m/s (cautious approach).
5.  **TARGET TOO FAR**: If target fills < 20% of FOV → APPROACH FAST at 0.30 m/s.
6.  **TARGET SAFE & SMALL**: If edges safe (>15px from border) AND fills < 60% FOV → APPROACH MEDIUM at 0.15 m/s.
7.  **TARGET SAFE & LARGE**: If edges safe AND fills ≥ 60% FOV → HOLD POSITION (stabilize).
8.  **EDGES IN DANGER ZONE**: If edges between 2-15px from border → HOLD POSITION (prevent oscillation).
9.  **TARGET OFF-CENTER**: If target not centered → TURN + APPROACH MEDIUM at 0.15 m/s with proportional control.
10. **NO TARGET & OBSTACLE AHEAD**: If searching and obstacle < 0.6m → SWITCH TO BUG2 MODE.
11. **NO TARGET**: If nothing detected → SEARCH by rotating in place at 0.4 rad/s.

**State 2: ROTATE_TO_WALL** (Preparing for Wall Follow)
- Robot rotates 60° (towards the side with more space) to face the wall
- Uses odometry to track rotation progress
- Transitions to WALL_FOLLOW when rotation complete

**State 3: WALL_FOLLOW** (Navigating Around Obstacle)
- Maintains 0.4m distance from wall using LIDAR feedback and P-controller
- Applies rotational bias to keep camera angled towards wall
- Detects corners and adjusts turn rate accordingly
- **Exit Condition**: When target becomes visible AND path is clear (>0.8m) → Return to GO_TO_GOAL
- **Reactive Only**: Robot stops before hitting obstacles but doesn't navigate around them (no pathfinding)

**Turn Control:**
- Proportional control: `angular_vel = -horizontal_error * 2.5` (increased gain for faster centering)
- Size-based damping: Reduces sensitivity when target is close (prevents over-correction)
- Turn deadzone: ±0.03 (tighter tolerance for better centering)

**Edge Detection System:**
- **Safety Margin (15px)**: Creates a "keep-out zone" near frame borders
- **Clip Margin (2px)**: Detects when target is actually clipping the frame
- **Visibility Status**: Tracks whether all 4 edges (left, right, top, bottom) are safely visible
- **Prevents Oscillation**: Robot won't flip-flop between approach/backup at the boundary

### Vision Pipeline
1.  **Input**: Raw RGB image from camera (640×480).
2.  **Preprocessing**: Convert BGR to HSV color space (better for color detection).
3.  **Filtering**: Threshold for Cyan (Hue: 80-100, Saturation: 50-255, Value: 50-255).
4.  **Cleanup**: Apply morphological operations (5×5 ellipse kernel):
    - Close: Fill small holes inside target
    - Open: Remove small noise outside target
5.  **Detection**: Find contours using `cv::findContours()`.
6.  **Selection**: Choose largest contour (most likely the target).
7.  **Bounding Box**: Calculate `cv::boundingRect()` for edge positions.
8.  **Edge Analysis**: Check if each edge (left, right, top, bottom) is:
    - Within safety margin (15px from border)
    - Clipped (within 2px from border)
9.  **Output**: 
    - Normalized X-coordinate for steering (-1 = left, +1 = right)
    - Width/Height ratios (target size as fraction of frame)
    - Edge visibility status (safe, danger zone, or clipped)

### Modern HUD Camera Viewer
The `view_camera.py` script displays real-time telemetry with a user-friendly interface designed for non-technical users:

**Features:**
- **"What is the Robot Doing?" Panel** (top):
  - Simple status icons: MOVING, REACHED, SEARCHING, BACKING UP, NAVIGATING, IDLE
  - Plain English explanations (e.g., "Getting closer to target", "Going around obstacle", "Too close! Stopping.")
  - Visual speed bar showing movement intensity
  - Color-coded status (Green=Good, Cyan=Moving, Orange=Searching, Red=Warning, Magenta=Backing, Yellow=Navigating)
  
- **Target Information Panel** (bottom):
  - "TARGET FOUND" or "NO TARGET DETECTED" status
  - Distance to target in meters
  - Size in view (% of frame coverage)
  - Alignment status with visual indicators:
    - Arrows showing turn direction when off-center
    - Circle indicator [OK] when centered
  
- **Visual Elements**:
  - Center crosshair for alignment reference
  - Target bounding box with corner brackets
  - Clean minimalist design without frame borders

**Design:**
- Modern rounded panels with dark semi-transparent backgrounds
- Thin, elegant typography for readability
- Colored header text with subtle separator lines
- Color-coded status for quick understanding
- No technical jargon - friendly for all users
- Visual indicators (bars, arrows, shapes) instead of complex numbers
- Left-side consolidated layout keeps view unobstructed

---

## 📂 Project Structure

```text
AI-Robotics-Group-12-Project/
├── src/
│   └── vision_target_follower_node.cpp  # Main C++ control node with Bug2 pathfinding
├── launch/
│   └── obstacle_course.launch.py        # Launches Gazebo, Robot, Nodes, and Odometry bridge
├── models/                              # SDF Models
│   ├── turtlebot/                       # Robot with Camera + LIDAR
│   └── cyan_target/                     # The target cylinder
├── worlds/
│   └── obstacle_course.world            # Simulation environment
├── view_camera.py                       # Modern HUD camera viewer
├── run_with_camera_viewer.sh            # One-command launch script
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
