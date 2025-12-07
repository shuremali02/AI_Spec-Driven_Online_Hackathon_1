---
id: chapter-04-urdf
title: "Chapter 4: URDF Robot Descriptions"
sidebar_position: 4
sidebar_label: "Chapter 4: URDF Robot Descriptions"
description: "Learn to create URDF (Unified Robot Description Format) robot descriptions focusing on humanoid robots, including links, joints, transforms, and Xacro macros"
keywords: [URDF, robot description, ROS 2, humanoid robotics, XML, robot modeling, Xacro]
---

# Chapter 4: URDF Robot Descriptions

## Section 1: Introduction to URDF

### Hook: Describe Your Robot Before You Control It

Before you can command a robot to move, walk, or perform complex tasks, you need to tell the system exactly what the robot looks like. In the world of robotics, this is accomplished through URDF (Unified Robot Description Format), which serves as the robot's digital blueprint. Think of URDF as the robot's DNA—containing all the essential information about its physical structure, from the shape of its body parts to how its joints connect and move.

Just as architects create detailed blueprints before constructing a building, roboticists use URDF to define every aspect of a robot's physical form. This digital description is crucial for everything from visualization and simulation to motion planning and control. Without a proper URDF, a robot controller would be like trying to navigate an unfamiliar city without a map.

### What is URDF and Why It Matters for Robotics

URDF (Unified Robot Description Format) is an XML-based file format that describes robots in terms of their links, joints, and other physical properties. It's the standard way to represent robot models in the ROS ecosystem and serves as the foundation for:

- **Visualization**: Seeing your robot in tools like RViz
- **Simulation**: Running your robot in Gazebo or other simulators
- **Kinematics**: Understanding how robot parts move relative to each other
- **Motion Planning**: Planning paths around obstacles with awareness of robot shape
- **Control**: Mapping commands to physical actuators based on the robot's structure

In the context of humanoid robotics, URDF becomes even more critical. Humanoid robots have complex kinematic structures with multiple degrees of freedom, requiring precise descriptions of limbs, joints, and their relationships. A well-crafted URDF for a humanoid robot must capture not just the physical geometry but also the anthropomorphic aspects that make these robots unique.

### URDF in the Context of Humanoid Robots

Humanoid robots present unique challenges in URDF modeling. Unlike simpler robots with just a few links and joints, humanoid robots have complex structures that mirror human anatomy:

- **Multiple limbs**: Arms, legs, and a torso that must work together
- **Complex joints**: Shoulders with multiple degrees of freedom, wrists, fingers
- **Balance considerations**: Center of mass and stability requirements
- **Anthropomorphic proportions**: Following human-like dimensions and capabilities

Creating a URDF for a humanoid robot requires understanding not just the mechanical aspects but also how human-like movement patterns should be represented in the robot's description. This makes URDF both an engineering tool and an art form.

### Chapter Roadmap and Learning Objectives

In this chapter, you'll progress from simple single-link robots to complex humanoid models with multiple limbs. Here's what you'll learn:

**Learning Objectives:**
1. Understand URDF XML syntax and structure for robot descriptions
2. Create links and joints to build robot models
3. Model humanoid robots with appropriate joint constraints
4. Visualize URDF models in RViz for verification
5. Use Xacro to create parameterized and reusable robot components

The journey begins with basic single-link robots, progresses through multi-link systems, and culminates in complete humanoid models using Xacro macros for efficient design. By the end of this chapter, you'll have the skills to describe virtually any robot in URDF format.

## Section 2: URDF Fundamentals

### URDF XML Structure and Basic Syntax

URDF is an XML-based format that follows a hierarchical structure. Every URDF file begins with an XML declaration and contains a single `<robot>` element as its root. The basic structure looks like this:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <!-- Visual properties for display -->
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <!-- Collision properties for physics -->
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <!-- Mass and inertial properties -->
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links together -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
</robot>
```

:::info
**URDF Structure Diagram**
*This diagram shows the hierarchical relationship between robot, links, and joints in URDF.*
:::

The fundamental elements of URDF are:

- **`<robot>`**: The root element containing the entire robot description
- **`<link>`**: Represents a rigid body or part of the robot
- **`<joint>`**: Connects two links together with specific kinematic properties
- **`<visual>`**: Defines how the link appears visually in RViz
- **`<collision>`**: Defines the collision boundaries for physics simulation
- **`<inertial>`**: Defines mass and inertial properties for dynamics

### Understanding Links: Visual, Collision, and Inertial Properties

A link in URDF represents a rigid body part of the robot, such as a wheel, arm segment, or torso. Each link can have three main properties that define different aspects of the physical body:

**Visual Properties**:
The `<visual>` element defines how the link appears when visualized in tools like RViz. This includes the shape, size, and material properties:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

**Collision Properties**:
The `<collision>` element defines the collision boundaries for physics simulation. This can be different from the visual representation to optimize simulation performance:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

**Inertial Properties**:
The `<inertial>` element defines mass and inertial properties for dynamic simulation. This is critical for realistic physics:

```xml
<inertial>
  <mass value="0.1"/>
  <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
</inertial>
```

### Understanding Joints: Types and Properties

Joints in URDF connect links together and define the kinematic relationship between them. URDF supports several joint types:

**Fixed Joints**: These connect two links without allowing any relative motion. Used for permanently attached parts:
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

**Revolute Joints**: These allow rotation around a single axis with limited range:
```xml
<joint name="hinge_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

**Continuous Joints**: These allow unlimited rotation around a single axis:
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
</joint>
```

**Prismatic Joints**: These allow linear sliding motion along an axis:
```xml
<joint name="slider_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.1" effort="10" velocity="1"/>
</joint>
```

### Coordinate Frames and Transforms

URDF uses a right-handed coordinate system where:
- **X-axis**: Points forward (or to the right, depending on convention)
- **Y-axis**: Points left (or forward, depending on convention)
- **Z-axis**: Points upward

Transforms between links are specified using the `<origin>` element, which contains both position (xyz) and orientation (rpy - roll, pitch, yaw):

```xml
<origin xyz="1.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
```

This places the child link 1 meter forward from the parent link, with no rotation. The transform defines the pose of the child link's frame relative to the parent link's frame.

:::info
**Coordinate Frame System**
*This diagram shows the right-handed coordinate system used in URDF with X, Y, and Z axes and how transforms work between links.*
:::

### Materials and Colors in URDF

Materials define the visual appearance of links and can be defined globally and reused across multiple links:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<visual>
  <material name="red"/>
  <!-- geometry definition -->
</visual>
```

Colors are specified using RGBA values where each component ranges from 0.0 to 1.0.

## Section 3: Creating Simple Robot Models

### Basic Single-Link Robot

Let's start with the simplest possible robot: a single link. This example creates a box-shaped robot that can serve as a foundation for more complex models:

```xml
<?xml version="1.0"?>
<robot name="simple_box_robot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0292" ixy="0.0" ixz="0.0" iyy="0.0292" iyz="0.0" izz="0.0417"/>
    </inertial>
  </link>
</robot>
```

This simple robot has a base link shaped like a box with dimensions 0.5m × 0.5m × 0.2m. The inertial properties are calculated based on the box's geometry and mass. To test this URDF:

1. Save the code as `simple_box_robot.urdf` in your URDF directory
2. Validate the URDF with: `check_urdf simple_box_robot.urdf`
3. Visualize in RViz using the Robot Model display

### Adding Joints to Create Multi-Link Systems

Now let's extend our simple robot to include multiple links connected by joints. This example creates a 2-link arm:

```xml
<?xml version="1.0"?>
<robot name="two_link_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0067" ixy="0.0" ixz="0.0" iyy="0.0067" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- First arm link -->
  <link name="arm_link_1">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0104" ixy="0.0" ixz="0.0" iyy="0.0104" iyz="0.0" izz="0.0006"/>
    </inertial>
  </link>

  <!-- Second arm link -->
  <link name="arm_link_2">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0104" ixy="0.0" ixz="0.0" iyy="0.0104" iyz="0.0" izz="0.0006"/>
    </inertial>
  </link>

  <!-- Joint connecting base to first arm link -->
  <joint name="base_to_arm_1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Joint connecting first arm link to second arm link -->
  <joint name="arm_1_to_arm_2" type="revolute">
    <parent link="arm_link_1"/>
    <child link="arm_link_2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

This robot has three links connected by two revolute joints. The first joint allows rotation around the Z-axis (like a rotating base), and the second joint allows rotation around the Y-axis (like an elbow joint).

### Setting Joint Limits and Properties

Joint limits are crucial for realistic robot simulation and control. The `<limit>` element specifies:

- `lower` and `upper`: The range of motion in radians (for revolute joints) or meters (for prismatic joints)
- `effort`: The maximum effort (torque or force) the joint can apply
- `velocity`: The maximum velocity of the joint

For humanoid robots, realistic joint limits are essential for human-like movement. For example, human elbow joints have a limited range of motion, which should be reflected in the URDF.

### Complete URDF Example with Explanation

Here's a complete robot example with a more complex structure:

```xml
<?xml version="1.0"?>
<robot name="mobile_base">
  <!-- Base footprint for 2D navigation -->
  <link name="base_footprint">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </visual>
  </link>

  <!-- Main robot body -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.8 0.4 0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.8 0.4 0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.175" ixy="0.0" ixz="0.0" iyy="0.417" iyz="0.0" izz="0.533"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <link name="wheel_left">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0025" ixy="0.0" ixz="0.0" iyy="0.0025" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <link name="wheel_right">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0025" ixy="0.0" ixz="0.0" iyy="0.0025" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Fixed joints connecting components -->
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0.2 -0.2 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0.2 0.2 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

This mobile robot example demonstrates several important concepts:

1. **Base footprint**: A virtual link for 2D navigation that sits on the ground plane
2. **Complex geometry**: Different shapes (boxes and cylinders) for different parts
3. **Joint positioning**: Joints placed at specific locations relative to parent links
4. **Material reuse**: Color definitions used across multiple links

### Visualizing in RViz

To visualize any URDF in RViz:

1. Make sure the `robot_state_publisher` node is running to publish the robot's joint states
2. In RViz, add a RobotModel display
3. Set the Robot Description parameter to the parameter name where your URDF is stored
4. The robot should appear in the 3D view

You can also launch a quick visualization with:
```bash
ros2 launch urdf_tutorial display.launch.py model:=path_to_your_robot.urdf
```

## Section 4: Humanoid Robot Anatomy

### Humanoid Robot Structure: Torso, Head, Arms, Legs

Humanoid robots are designed to mimic human anatomy and movement patterns. When modeling humanoid robots in URDF, it's important to follow anthropomorphic principles to ensure the robot behaves in expected ways. A typical humanoid robot structure includes:

**Torso**: The central body that connects all other parts
- Should be appropriately sized to support arms and head
- Often contains the main computer, power systems, and sensors
- May have multiple segments for added flexibility

**Head**: Contains perception sensors and potentially neck mobility
- Typically houses cameras, microphones, and other sensors
- May include neck joints for looking around
- Should be properly balanced with the rest of the body

**Arms**: Upper and lower segments with hands/actuators
- Should have shoulder, elbow, and wrist joints
- Often modeled with 7 degrees of freedom per arm
- End effectors (hands/grippers) at the distal ends

**Legs**: Support structure with mobility for locomotion
- Include hip, knee, and ankle joints
- Often have 6 degrees of freedom per leg
- Feet provide stability and balance

:::info
**Humanoid Robot Anatomy**
*This diagram shows the key anatomical components of a humanoid robot: torso, head, arms, and legs with joint locations.*
:::

### Joint Configuration for Human-like Movement

Humanoid robots require careful joint configuration to achieve human-like movement. Here's how human joints typically map to robot joints:

**Shoulder Complex**: Humans have a complex shoulder girdle, but robot shoulders typically have:
- Roll (internal/external rotation)
- Pitch (flexion/extension)
- Yaw (abduction/adduction)

**Elbow Joint**: Similar to humans, with flexion/extension motion only
- Limited range of motion (typically 0° to 170°)

**Wrist Joints**: Often include:
- Pitch and yaw for orientation
- Roll for rotation of the end effector

**Hip Complex**: Multi-axis joints similar to shoulders
- Roll, pitch, and yaw for full mobility

**Knee Joint**: Primarily flexion/extension like human knees
- Limited or no backward bending

**Ankle Joints**: Pitch and roll for balance and stepping

### Degrees of Freedom in Humanoid Robots

Degrees of Freedom (DOF) refer to the number of independent movements a robot can make. Humanoid robots typically have:

**Low-DOF Humanoids (20-30 DOF)**:
- Simplified design focusing on essential movements
- More robust and easier to control
- Good for basic interaction tasks

**High-DOF Humanoids (40+ DOF)**:
- More human-like with finger articulation
- Greater flexibility and capability
- More complex to control and maintain

The most common humanoid configuration has:
- 6 DOF per arm (shoulder: 3, elbow: 1, wrist: 2)
- 6 DOF per leg (hip: 3, knee: 1, ankle: 2)
- 3 DOF for the torso (if segmented)
- 3 DOF for the head (neck: 3)
- Total: 27 DOF for basic configuration

### Link and Joint Naming Conventions

Consistent naming conventions are crucial for humanoid robots to ensure compatibility with ROS tools and other robots. Common conventions include:

**For arms**:
- Left arm: `l_shoulder_pan`, `l_shoulder_lift`, `l_elbow_flex`, `l_wrist_flex`, `l_wrist_roll`
- Right arm: `r_shoulder_pan`, `r_shoulder_lift`, `r_elbow_flex`, `r_wrist_flex`, `r_wrist_roll`

**For legs**:
- Left leg: `l_hip_yaw`, `l_hip_roll`, `l_hip_pitch`, `l_knee_flex`, `l_ankle_pitch`, `l_ankle_roll`
- Right leg: `r_hip_yaw`, `r_hip_roll`, `r_hip_pitch`, `r_knee_flex`, `r_ankle_pitch`, `r_ankle_roll`

**For torso and head**:
- Torso: `torso_lift`, `torso_roll`, `torso_pitch`
- Head: `head_pan`, `head_tilt`

### Kinematic Chains in Humanoid Robots

A kinematic chain is a series of links connected by joints that can move relative to each other. In humanoid robots, we have several important kinematic chains:

**Arm Chains**: Each arm forms a kinematic chain from shoulder to hand
- Critical for manipulation tasks
- Need to consider end effector pose
- Often require inverse kinematics for control

**Leg Chains**: Each leg forms a chain from hip to foot
- Essential for locomotion and balance
- Closed-chain kinematics when feet are on ground
- Critical for walking and standing

**Trunk Chain**: Torso and head movement chain
- Affects overall balance and orientation
- Impacts arm and leg kinematics
- Important for gaze control and interaction

:::info
**Kinematic Chain Diagram**
*This diagram illustrates how links and joints form kinematic chains in humanoid robots, showing the connection from hip to foot and shoulder to hand.*
:::

## Section 5: Building a Complete Humanoid URDF

### Step-by-Step Humanoid Robot Construction

Now let's build a complete humanoid robot URDF. We'll start with the torso and then add limbs systematically. This example will create a simplified humanoid with basic anthropomorphic structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Materials -->
  <material name="red">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1.0"/>
  </material>
  <material name="white">
    <color rgba="0.9 0.9 0.9 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

  <!-- Root link: base of the torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0125" ixy="0.0" ixz="0.0" iyy="0.0125" iyz="0.0" izz="0.025"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.09"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0045" ixy="0.0" ixz="0.0" iyy="0.0045" iyz="0.0" izz="0.0045"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <link name="l_upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0028125" ixy="0.0" ixz="0.0" iyy="0.0028125" iyz="0.0" izz="0.000075"/>
    </inertial>
  </link>

  <link name="l_lower_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.00225" ixy="0.0" ixz="0.0" iyy="0.00225" iyz="0.0" izz="0.0000533"/>
    </inertial>
  </link>

  <link name="l_hand">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.000167" ixy="0.0" ixz="0.0" iyy="0.000167" iyz="0.0" izz="0.000167"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <link name="r_upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0028125" ixy="0.0" ixz="0.0" iyy="0.0028125" iyz="0.0" izz="0.000075"/>
    </inertial>
  </link>

  <link name="r_lower_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.00225" ixy="0.0" ixz="0.0" iyy="0.00225" iyz="0.0" izz="0.0000533"/>
    </inertial>
  </link>

  <link name="r_hand">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.000167" ixy="0.0" ixz="0.0" iyy="0.000167" iyz="0.0" izz="0.000167"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="l_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01667" ixy="0.0" ixz="0.0" iyy="0.01667" iyz="0.0" izz="0.00144"/>
    </inertial>
  </link>

  <link name="l_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.0125" ixy="0.0" ixz="0.0" iyy="0.0125" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <link name="l_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.000833" ixy="0.0" ixz="0.0" iyy="0.001167" iyz="0.0" izz="0.001167"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <link name="r_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01667" ixy="0.0" ixz="0.0" iyy="0.01667" iyz="0.0" izz="0.00144"/>
    </inertial>
  </link>

  <link name="r_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.0125" ixy="0.0" ixz="0.0" iyy="0.0125" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <link name="r_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.000833" ixy="0.0" ixz="0.0" iyy="0.001167" iyz="0.0" izz="0.001167"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm Joints -->
  <joint name="torso_to_l_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="l_upper_arm"/>
    <origin xyz="0.15 0.1 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="l_upper_arm_to_l_lower_arm" type="revolute">
    <parent link="l_upper_arm"/>
    <child link="l_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="l_lower_arm_to_l_hand" type="fixed">
    <parent link="l_lower_arm"/>
    <child link="l_hand"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Right Arm Joints -->
  <joint name="torso_to_r_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="r_upper_arm"/>
    <origin xyz="0.15 -0.1 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="r_upper_arm_to_r_lower_arm" type="revolute">
    <parent link="r_upper_arm"/>
    <child link="r_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="r_lower_arm_to_r_hand" type="fixed">
    <parent link="r_lower_arm"/>
    <child link="r_hand"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Left Leg Joints -->
  <joint name="base_to_l_upper_leg" type="revolute">
    <parent link="base_link"/>
    <child link="l_upper_leg"/>
    <origin xyz="-0.075 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <joint name="l_upper_leg_to_l_lower_leg" type="revolute">
    <parent link="l_upper_leg"/>
    <child link="l_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="0" effort="15" velocity="1"/>
  </joint>

  <joint name="l_lower_leg_to_l_foot" type="fixed">
    <parent link="l_lower_leg"/>
    <child link="l_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  </joint>

  <!-- Right Leg Joints -->
  <joint name="base_to_r_upper_leg" type="revolute">
    <parent link="base_link"/>
    <child link="r_upper_leg"/>
    <origin xyz="-0.075 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <joint name="r_upper_leg_to_r_lower_leg" type="revolute">
    <parent link="r_upper_leg"/>
    <child link="r_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="0" effort="15" velocity="1"/>
  </joint>

  <joint name="r_lower_leg_to_r_foot" type="fixed">
    <parent link="r_lower_leg"/>
    <child link="r_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  </joint>
</robot>
```

This humanoid URDF demonstrates several important principles:

1. **Anthropomorphic Proportions**: Links are sized to approximate human proportions
2. **Color Coding**: Left and right limbs use different colors for easier identification
3. **Proper Joint Placement**: Joints are positioned at anatomically correct locations
4. **Appropriate Joint Limits**: Limits reflect realistic human-like ranges of motion
5. **Realistic Inertial Properties**: Mass and inertia values are appropriate for the link sizes

### Creating Torso, Head, and Limbs

When creating a humanoid URDF, it's helpful to think of it in anatomical segments:

**The Torso**: The central hub connecting all other parts. It should be substantial enough to house computing elements and provide stability for the limbs. The torso is typically the heaviest part of the robot.

**The Head**: Positioned at the top of the torso, the head contains sensors for perception. Even if the head doesn't move, it's often connected via joints to allow for future expansion.

**The Limbs**: Arms and legs follow similar patterns but serve different purposes. Arms typically have more DOF for manipulation, while legs are optimized for stability and locomotion.

### Joint Constraints for Realistic Movement

Realistic joint constraints are critical for humanoid robots. In the example above:

- **Knee joints** only bend in one direction (negative limit) to mimic human knees
- **Shoulder joints** have appropriate ranges to allow for natural arm movement
- **Hip joints** allow for full leg mobility while maintaining structural integrity
- **Neck joints** have limited range to protect delicate head actuators

### Complete Humanoid URDF Example

The example above represents a complete humanoid with:
- 8 moving joints (excluding fixed connections)
- Proper inertial properties for each link
- Realistic anthropomorphic proportions
- Color coding for identification
- Proper coordinate frame relationships

### Adding Sensors and Other Components

To make a humanoid robot more functional, you can add various sensors and components:

```xml
<!-- Camera on the head -->
<gazebo reference="head">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>

<!-- IMU in the torso -->
<gazebo reference="torso">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
  </sensor>
</gazebo>
```

## Section 6: URDF Visualization and Tools

### Using RViz to Visualize URDF Models

RViz is the primary visualization tool for URDF models in ROS 2. To visualize your URDF:

1. **Launch the Robot State Publisher**: This node publishes the robot's joint states and transforms:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat path/to/your/robot.urdf)
   ```

2. **Start RViz**:
   ```bash
   rviz2
   ```

3. **Configure RViz**: Add a RobotModel display and set the Robot Description parameter to the same parameter name used in robot_state_publisher

4. **Control Joints**: Use joint_state_publisher to move the robot's joints:
   ```bash
   ros2 run joint_state_publisher joint_state_publisher
   ```

For a complete visualization setup, you can create a launch file:

```xml
<launch>
  <arg name="model" default=""/>
  <param name="robot_description" value="$(find-pkg-share my_robot_description)/urdf/robot.urdf"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher" exec="joint_state_publisher"/>
  <node name="rviz2" pkg="rviz2" exec="rviz2" args="-d $(find-pkg-share my_robot_description)/rviz/robot.rviz"/>
</launch>
```

:::info
**RViz Visualization Workflow**
*This diagram shows the workflow from URDF to robot_state_publisher to RViz for visualization.*
:::

### Robot State Publisher for TF Transforms

The `robot_state_publisher` package is crucial for URDF visualization. It reads the robot's URDF from the parameter server and subscribes to joint state messages. It then calculates the forward kinematics of the robot and publishes the resulting transforms to the tf2 tree.

The robot_state_publisher automatically:
- Parses the URDF model
- Creates tf frames for each link
- Calculates transforms based on joint states
- Publishes transforms to the tf tree

This allows other ROS packages to know the pose of each part of the robot relative to other parts.

### Checking URDF Validity with check_urdf Command

Before visualizing or simulating your robot, always validate the URDF:

```bash
check_urdf path/to/your/robot.urdf
```

This command will:
- Validate the XML syntax
- Check for missing or duplicate names
- Verify that all joints have properly connected parent and child links
- Report the kinematic tree structure
- Show any errors or warnings

### Common URDF Errors and Debugging

Here are common URDF errors and how to fix them:

**1. Invalid XML Syntax**:
- Check for unclosed tags
- Ensure proper escaping of special characters
- Verify XML declaration is at the beginning

**2. Missing Joint Connections**:
- Every joint must have both a parent and child link
- The parent and child links must exist in the robot
- Joint names must be unique

**3. Inconsistent Mass/Inertia**:
- Inertial elements must have positive mass values
- Inertia matrix must be positive definite
- Units should be consistent (meters, kilograms, seconds)

**4. Coordinate Frame Issues**:
- Transforms must be physically possible
- Parent-child relationships must form a valid tree structure (no loops)

:::tip
**Troubleshooting Tips**
<details>
<summary>Click here for common troubleshooting solutions</summary>

- **"No transform found" errors**: Make sure robot_state_publisher is running and publishing transforms
- **Parts of robot missing in RViz**: Check that all links have proper visual elements defined
- **Robot appears distorted**: Verify that all geometry dimensions are in meters
- **Joints don't move**: Ensure joint_state_publisher is running and publishing joint states
- **"URDF Parse Error"**: Use `check_urdf` command to validate your URDF file

</details>
:::

## Section 7: Xacro for Complex URDFs

### Introduction to Xacro (XML Macros)

Xacro (XML Macros) is a macro language for XML that extends URDF with several useful features:

- **Macros**: Reusable robot components
- **Properties**: Parameter substitution
- **Mathematical Expressions**: Calculations within the XML
- **File Inclusion**: Combining multiple files

Xacro files typically use the `.xacro` extension and must be processed before they can be used as URDF.

### Creating Reusable Components with Xacro

Here's a simple example of a wheel macro in Xacro:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="wheel" params="prefix parent *origin radius:=0.1 width:=0.05 mass:=0.5">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${width}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${0.25 * mass * radius * radius}"
                 ixy="0"
                 ixz="0"
                 iyy="${0.25 * mass * radius * radius}"
                 iyz="0"
                 izz="${0.5 * mass * radius * radius}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.8 0.4 0.3"/>
      </geometry>
    </visual>
  </link>

  <xacro:wheel prefix="left" parent="base_link">
    <origin xyz="0.2 -0.2 0.1" rpy="0 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="right" parent="base_link">
    <origin xyz="0.2 0.2 0.1" rpy="0 0 0"/>
  </xacro:wheel>
</robot>
```

### Parameterizing Robot Models

Xacro's power comes from its ability to parameterize robot models. Here's a more complex example with a configurable robot arm:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="arm_radius" value="0.05"/>
  <xacro:property name="arm_density" value="1000"/>

  <!-- Arm link macro -->
  <xacro:macro name="arm_link" params="name length mass radius *origin">
    <link name="${name}">
      <visual>
        <xacro:insert_block name="origin"/>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1"/>
        </material>
      </visual>

      <collision>
        <xacro:insert_block name="origin"/>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="${mass}"/>
        <xacro:insert_block name="origin"/>
        <inertia
          ixx="${mass * (3 * radius * radius + length * length) / 12}"
          ixy="0"
          ixz="0"
          iyy="${mass * (3 * radius * radius + length * length) / 12}"
          iyz="0"
          izz="${mass * radius * radius / 2}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Revolute joint macro -->
  <xacro:macro name="revolute_joint" params="name parent child origin_xyz origin_rpy axis lower upper">
    <joint name="${name}" type="revolute">
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
      <parent link="${parent}"/>
      <child link="${child}"/>
      <axis xyz="${axis}"/>
      <limit lower="${lower}" upper="${upper}" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Robot body -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Arm construction using macros -->
  <xacro:arm_link name="upper_arm" length="0.4" mass="0.8" radius="${arm_radius}">
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </xacro:arm_link>

  <xacro:arm_link name="lower_arm" length="0.3" mass="0.5" radius="${arm_radius}">
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </xacro:arm_link>

  <!-- Joints connecting the arm -->
  <joint name="shoulder_joint" type="revolute">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <joint name="elbow_joint" type="revolute">
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>
</robot>
```

### Example: Modular Humanoid with Xacro

Here's how to create a modular humanoid using Xacro to demonstrate its advantages for complex robots:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="modular_humanoid">
  <!-- Include other Xacro files if needed -->
  <xacro:include filename="$(find-pkg-share my_robot_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find-pkg-share my_robot_description)/urdf/common_properties.xacro"/>

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="torso_height" value="0.6"/>
  <xacro:property name="torso_width" value="0.3"/>
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="leg_length" value="0.5"/>

  <!-- Macro for a humanoid limb -->
  <xacro:macro name="limb" params="name parent side length radius mass joint_type:=revolute">
    <link name="${side}_${name}_link">
      <visual>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${side}_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <inertia
          ixx="${mass * (3 * radius * radius + length * length) / 12}"
          ixy="0"
          ixz="0"
          iyy="${mass * (3 * radius * radius + length * length) / 12}"
          iyz="0"
          izz="${mass * radius * radius / 2}"/>
      </inertial>
    </link>

    <joint name="${side}_${name}_joint" type="${joint_type}">
      <parent link="${parent}"/>
      <child link="${side}_${name}_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Robot torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="${torso_width} ${torso_width} ${torso_height}"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="${torso_width} ${torso_width} ${torso_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.09"/>
    </inertial>
  </link>

  <!-- Create limbs using the macro -->
  <xacro:limb name="upper_arm" parent="torso" side="l" length="${arm_length}" radius="0.05" mass="0.5"/>
  <xacro:limb name="lower_arm" parent="l_upper_arm_link" side="l" length="${arm_length*0.8}" radius="0.04" mass="0.3"/>
  <xacro:limb name="upper_leg" parent="torso" side="l" length="${leg_length}" radius="0.06" mass="0.8"/>
  <xacro:limb name="lower_leg" parent="l_upper_leg_link" side="l" length="${leg_length*0.9}" radius="0.05" mass="0.6"/>

  <xacro:limb name="upper_arm" parent="torso" side="r" length="${arm_length}" radius="0.05" mass="0.5"/>
  <xacro:limb name="lower_arm" parent="r_upper_arm_link" side="r" length="${arm_length*0.8}" radius="0.04" mass="0.3"/>
  <xacro:limb name="upper_leg" parent="torso" side="r" length="${leg_length}" radius="0.06" mass="0.8"/>
  <xacro:limb name="lower_leg" parent="r_upper_leg_link" side="r" length="${leg_length*0.9}" radius="0.05" mass="0.6"/>
</robot>
```

:::info
**Xacro Component Reuse**
*This diagram shows how Xacro macros enable component reuse in robot design, making it easier to create complex robots with modular parts.*
:::

Xacro's modular approach offers several advantages:
- **Reusability**: Common components can be defined once and used multiple times
- **Parameterization**: Robot dimensions and properties can be easily adjusted
- **Maintainability**: Changes to common components automatically propagate
- **Readability**: Complex robots can be described more concisely

## Advanced URDF Features: Transmissions

Transmissions are an important but often overlooked aspect of URDF that define how actuators (motors) connect to joints. While joints define the kinematic relationships between links, transmissions define the physical connection between actuators and joints, including gear ratios, mechanical interfaces, and control properties.

A transmission element typically includes:

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

There are several transmission types available:
- **SimpleTransmission**: Single joint connected to single actuator
- **DifferentialTransmission**: Connects two joints to two actuators with differential relationship
- **FourBarLinkageTransmission**: For four-bar linkage mechanisms

:::info
**Transmission Elements**
*This diagram shows how transmission elements connect joints to actuators in URDF, defining the mechanical interface between the two.*
:::

The key parameters for transmissions include:
- **Joint**: The joint that the actuator controls
- **Actuator**: The physical motor or actuator
- **Hardware Interface**: The type of control interface (position, velocity, effort)
- **Mechanical Reduction**: Gear ratios and mechanical advantage
- **Safety Controllers**: Limit switches and safety parameters

## Section 8: Chapter Summary and Next Steps

### Key Takeaways from URDF Modeling

In this chapter, you've learned the fundamentals of URDF (Unified Robot Description Format) for describing robots, with a focus on humanoid models. Here are the key concepts you should remember:

1. **URDF Structure**: The fundamental elements of URDF - `<robot>`, `<link>`, `<joint>`, `<visual>`, `<collision>`, and `<inertial>` - form the building blocks of any robot description.

2. **Physical Properties**: Each link requires visual, collision, and inertial properties to properly represent the robot in visualization, physics simulation, and control systems.

3. **Joint Types**: Understanding the different joint types (fixed, revolute, continuous, prismatic) and their appropriate use cases is crucial for creating functional robot models.

4. **Humanoid Considerations**: When modeling humanoid robots, anthropomorphic proportions, appropriate joint constraints, and realistic degrees of freedom are essential for creating believable and functional models.

5. **Visualization and Tools**: The robot_state_publisher, joint_state_publisher, and RViz form the core toolchain for visualizing and debugging URDF models.

6. **Xacro Advantages**: Xacro macros provide powerful parameterization and reusability for complex robot models, making it easier to create and maintain sophisticated humanoid robots.

### Next Steps and Additional Resources

With your foundation in URDF modeling, you're now ready to explore more advanced topics in robotics simulation and control. Here are suggested next steps:

1. **Gazebo Simulation**: Move to Module 2 to learn how to simulate your URDF models in Gazebo, where you can test robot behaviors in realistic physics environments.

2. **Motion Planning**: Explore how URDF models are used in motion planning algorithms to navigate around obstacles and perform complex tasks.

3. **Robot Control**: Learn how to control your URDF robots using ROS 2 controllers and command interfaces.

4. **Advanced Xacro**: Delve deeper into Xacro's capabilities, including conditional statements, mathematical functions, and file inclusion for creating highly modular robot descriptions.

### Additional Resources for URDF Learning

- **Official URDF Tutorials**: The ROS wiki provides comprehensive tutorials on URDF creation and best practices
- **Xacro Documentation**: Detailed documentation on Xacro macros and advanced features
- **Robot Mesh Resources**: Sites like GrabCAD and Thingiverse provide 3D models that can be integrated into URDF files
- **ROS Community**: The ROS Discourse forums and answers.ros.org are excellent places to get help with URDF challenges

By mastering URDF, you've gained a fundamental skill that will serve you throughout your robotics journey. Whether you're creating simple wheeled robots or complex humanoid systems, the principles you've learned in this chapter form the foundation of robot description and modeling in the ROS ecosystem.