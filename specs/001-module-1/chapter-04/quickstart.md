# Quickstart: Chapter 4 - URDF Robot Descriptions

## Prerequisites
- Complete Chapter 1 (Physical AI concepts), Chapter 2 (ROS 2 architecture), and Chapter 3 (ROS 2 nodes)
- ROS 2 Humble installed on Ubuntu 22.04
- Basic understanding of XML syntax
- Familiarity with ROS 2 workspace structure from Chapter 3
- Understanding of coordinate frames and transforms (covered in Chapter 2)

## Setup Your URDF Environment
```bash
# Verify ROS 2 Humble is sourced
source /opt/ros/humble/setup.bash

# Check URDF packages are available
dpkg -l | grep urdf
dpkg -l | grep xacro
```

## Essential URDF Tools
```bash
# Check URDF parser availability
ros2 pkg list | grep urdf_parser

# Check robot_state_publisher availability
ros2 pkg list | grep robot_state_publisher

# Check joint_state_publisher availability
ros2 pkg list | grep joint_state_publisher
```

## Create URDF Package Structure
```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Create a URDF package for this chapter
ros2 pkg create --build-type ament_python urdf_tutorial
cd urdf_tutorial
mkdir urdf
mkdir meshes  # For 3D mesh files
mkdir launch  # For launch files
```

## Basic URDF File Structure
```bash
# Create a basic URDF file
touch urdf/simple_robot.urdf
```

## Essential Commands for Testing URDF
```bash
# Check URDF file syntax
check_urdf urdf/simple_robot.urdf

# Visualize URDF in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/simple_robot.urdf

# Convert Xacro to URDF
ros2 run xacro xacro urdf/robot.xacro > urdf/robot.urdf
```

## Common Visualization Commands
```bash
# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat urdf/simple_robot.urdf)

# Launch joint state publisher for visualization
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Expected URDF Structure
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Troubleshooting Quick Fixes
- **"Invalid URDF"**: Check XML syntax and proper closing tags
- **"No robot shown in RViz"**: Verify robot_description parameter and TF publishers
- **"Joints not moving"**: Check joint_state_publisher is running and publishing
- **"Xacro not found"**: Make sure xacro package is installed: `sudo apt install ros-humble-xacro`

## Next Steps After Completion
1. Complete the humanoid robot model exercise
2. Experiment with different joint types and limits
3. Try adding sensors to your URDF model
4. Move to Module 2: The Digital Twin (Gazebo & Unity)