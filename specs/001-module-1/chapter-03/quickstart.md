# Quickstart: Chapter 3 - Building Your First ROS 2 Nodes

## Prerequisites
- Complete Chapter 1 (Physical AI concepts) and Chapter 2 (ROS 2 architecture)
- ROS 2 Humble installed on Ubuntu 22.04
- Python 3.10+ with pip
- Basic terminal/command-line skills
- Understanding of pub-sub pattern from Chapter 2

## Setup Your Workspace
```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package for this chapter
ros2 pkg create --build-type ament_python my_robot_pkg
```

## Verify ROS 2 Installation
```bash
# Check that ROS 2 is properly installed
source /opt/ros/humble/setup.bash
ros2 --version

# Check that Python can import rclpy
python3 -c "import rclpy; print('rclpy available')"
```

## Create Basic Node Structure
```bash
# Navigate to your package
cd ~/ros2_ws/src/my_robot_pkg

# Create the nodes directory if it doesn't exist
mkdir -p my_robot_pkg
```

## Essential Commands for Testing
```bash
# Build your workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash

# Run a publisher node
ros2 run my_robot_pkg simple_publisher

# In another terminal, run a subscriber node
ros2 run my_robot_pkg simple_subscriber
```

## Common Verification Steps
1. **Check nodes are running**: `ros2 node list`
2. **Check topics**: `ros2 topic list`
3. **Echo topic data**: `ros2 topic echo /topic_name std_msgs/msg/String`
4. **Check node info**: `ros2 node info /node_name`

## Expected Output Format
- Publisher should output messages with timestamps
- Subscriber should receive and log the same messages
- Both nodes should handle shutdown gracefully

## Troubleshooting Quick Fixes
- **"Command not found"**: Make sure to source the workspace (`source install/setup.bash`)
- **"No executable found"**: Check that the executable is properly declared in setup.py
- **"No messages received"**: Verify topic names match exactly between publisher and subscriber

## Next Steps After Completion
1. Complete the hands-on lab exercise in the chapter
2. Experiment with different message types
3. Try creating multiple publishers/subscribers
4. Move to Chapter 4: URDF Robot Descriptions