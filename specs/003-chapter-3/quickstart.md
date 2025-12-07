# Quickstart: Chapter 3 - Building Your First ROS 2 Nodes

## Prerequisites
- Complete Chapter 1 (Intro to Physical AI) and Chapter 2 (ROS 2 Architecture)
- ROS 2 Humble installed on Ubuntu 22.04
- Python 3.10+ with pip
- Basic command-line skills

## Setup Your Workspace
```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package for this chapter
ros2 pkg create --build-type ament_python my_robot_pkg
```

## Create Your First Publisher
1. Navigate to your package directory:
```bash
cd ~/ros2_ws/src/my_robot_pkg
```

2. Create the publisher node file:
```bash
touch my_robot_pkg/publisher_node.py
```

3. Add the publisher code from the chapter

4. Make it executable:
```bash
chmod +x my_robot_pkg/publisher_node.py
```

## Build and Run
```bash
# Build the workspace
cd ~/ros2_ws
colcon build

# Source the setup file
source install/setup.bash

# Run the publisher
ros2 run my_robot_pkg publisher_node
```

## What You'll Learn
- How to create publisher and subscriber nodes
- How to define and use custom message types
- How to use services for request-response communication
- How to configure nodes with parameters
- How to orchestrate multiple nodes with launch files
- How to bridge AI agents to ROS controllers

## Next Steps
After completing this chapter, you'll be ready to learn about URDF (Unified Robot Description Format) in Chapter 4, where you'll describe the physical structure of humanoid robots.