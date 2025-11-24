#!/bin/bash

# Script to run Gazebo simulation with camera viewer
# This allows you to see both the FOV visualization in Gazebo 
# and what the camera actually sees in real-time

echo "Starting Gazebo simulation with camera viewer..."
echo "================================================"

# Kill any existing Gazebo/ROS processes
echo "Cleaning up any existing processes..."
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "vision_target_follower" 2>/dev/null
pkill -9 -f "view_camera.py" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
sleep 2

# Build the project
echo "Building project..."
colcon build
if [ $? -ne 0 ]; then
    echo "Build failed! Exiting."
    exit 1
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the simulation in the background
echo "Launching simulation..."
ros2 launch lidar_target_follower obstacle_course.launch.py &
SIM_PID=$!

# Wait for simulation to initialize
echo "Waiting for simulation to start (10 seconds)..."
sleep 10

# Launch the camera viewer
echo "Starting camera viewer..."
python3 view_camera.py

# When camera viewer is closed, kill the simulation
echo "Camera viewer closed. Stopping simulation..."
kill $SIM_PID
wait $SIM_PID 2>/dev/null

echo "Done!"
