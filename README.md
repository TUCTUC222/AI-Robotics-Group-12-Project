# Obstacle Course with LIDAR Robot

A ROS2/Gazebo simulation featuring a differential drive robot that uses LIDAR to navigate an obstacle course and find a target.

## Overview

This project demonstrates:
- **LIDAR Navigation**: A TurtleBot-like robot equipped with a LIDAR sensor.
- **Target Finding**: The robot autonomously moves towards the closest object (the target).
- **Obstacle Course**: A complex environment with walls, slalom cones, and a maze.

## Project Structure

```
AI-Robotics-Group-12-Project/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # ROS2 package manifest
├── README.md                      # This file
├── launch/
│   └── obstacle_course.launch.py  # Launch file to start simulation
├── models/
│   ├── turtlebot/                 # TurtleBot model with LIDAR
│   └── cyan_target/               # Cyan cylinder target
├── src/
│   └── target_follower_node.cpp   # C++ node for navigation logic
└── worlds/
    └── obstacle_course.world      # Gazebo world with obstacles
```

## Usage

1. Build the package:
   ```bash
   colcon build
   source install/setup.bash
   ```

2. Launch the simulation:
   ```bash
   ros2 launch lidar_target_follower obstacle_course.launch.py
   ```

## Features

- **TurtleBot**: A differential drive robot modeled after the TurtleBot 3 Burger.
- **Cyan Target**: A distinct cyan cylinder that the robot will attempt to approach.
- **Autonomous Logic**: The `target_follower_node` processes LIDAR scans to steer the robot towards the nearest object.

   - **Angular velocity**: Turn towards target
   - **Linear velocity**: Move forward/backward based on distance
   - **Search mode**: Rotate slowly if no target detected
5. Publish velocity commands (`/cmd_vel`)

#### Control Parameters
- `linear_speed`: Maximum forward speed (default: 0.5 m/s)
- `angular_speed`: Maximum turning speed (default: 0.8 rad/s)
- `min_distance`: Minimum distance to maintain (default: 0.8 m)
- `max_distance`: Distance to approach at full speed (default: 2.5 m)
- `detection_threshold`: Maximum LIDAR detection range (default: 5.0 m)

## Requirements

### System Dependencies
- Ubuntu 22.04 (recommended)
- ROS2 Humble or later
- Gazebo Harmonic (gz-sim)
- C++ compiler with C++17 support

### ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs
```

## Building the Project

1. **Navigate to the project directory:**
   ```bash
   cd ~/AI-Robotics-Group-12-Project
   ```

2. **Source ROS2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the package:**
   ```bash
   colcon build
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

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

## Future Enhancements

- [ ] Add obstacle avoidance while following
- [ ] Implement PID controller for smoother movement
- [ ] Add camera-based target identification
- [ ] Support multiple targets and target selection
- [ ] Add path planning for complex environments
- [ ] Implement Kalman filter for target state estimation
- [ ] Add RViz2 visualization launch configuration

## License

MIT License

## Contributors

Group 12 - AI Robotics Project
