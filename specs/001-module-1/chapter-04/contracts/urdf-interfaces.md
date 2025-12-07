# URDF Interface Contracts: Chapter 4

## Core URDF Robot Element

### Required Attributes
- `name`: String identifier for the robot

### Child Elements Required
- At least one `<link>` element
- At least one `<joint>` element (if multiple links exist)

### Expected Behavior
- Robot name must be unique within namespace
- All links and joints must have unique names within robot
- XML must be well-formed and valid

## Link Element Interface Contract

### Required Attributes
- `name`: String identifier for the link

### Optional Child Elements
- `<visual>`: Visual properties for rendering
- `<collision>`: Collision properties for physics
- `<inertial>`: Mass and inertial properties for physics

### Expected Behavior
- Link name must be unique within robot
- Visual and collision elements can share geometry but may differ
- Inertial properties are required for dynamic simulation

## Joint Element Interface Contract

### Required Attributes
- `name`: String identifier for the joint
- `type`: Joint type (revolute, continuous, prismatic, fixed, floating, planar)
- `parent`: Name of parent link
- `child`: Name of child link

### Required Child Elements
- `<parent>`: Reference to parent link
- `<child>`: Reference to child link

### Optional Child Elements
- `<origin>`: Transform from parent to child
- `<axis>`: Joint axis for rotational/linear motion
- `<limit>`: Motion limits for revolute/prismatic joints

### Expected Behavior
- Joint connects exactly two links (parent and child)
- Joint type determines degrees of freedom
- Limits must be specified for revolute and prismatic joints

## Visual Element Contract

### Required Child Elements
- `<geometry>`: Shape definition

### Optional Child Elements
- `<material>`: Color and texture properties
- `<origin>`: Transform within the link

### Expected Behavior
- Defines how link appears in visualization
- Does not affect physics simulation
- Geometry must be valid shape (box, cylinder, sphere, mesh)

## Collision Element Contract

### Required Child Elements
- `<geometry>`: Shape definition

### Optional Child Elements
- `<origin>`: Transform within the link

### Expected Behavior
- Defines collision boundaries for physics
- Can differ from visual geometry
- Must be convex shapes or meshes for physics engines

## Geometry Element Contract

### Supported Shapes
- `<box size="x y z">`: Rectangular cuboid
- `<cylinder radius="r" length="l">`: Cylindrical shape
- `<sphere radius="r">`: Spherical shape
- `<mesh filename="path" scale="x y z">`: Mesh file

### Expected Behavior
- Size/dimensions must be positive values
- Mesh files must exist at specified path
- Units are typically meters for consistency

## Material Element Contract

### Required or Optional Child Elements
- `<color rgba="r g b a">`: Color specification
- OR `<texture filename="path">`: Texture file

### Expected Behavior
- Colors specified as RGBA values (0.0 to 1.0)
- Textures loaded from valid file paths
- Materials can be reused across multiple visuals

## Inertial Element Contract

### Required Child Elements
- `<mass value="m">`: Mass in kilograms
- `<inertia ixx="ixx" ixy="ixy" ixz="ixz" iyy="iyy" iyz="iyz" izz="izz">`: Inertia matrix

### Expected Behavior
- Mass must be positive
- Inertia values must form positive-definite matrix
- Values significantly affect simulation behavior

## Xacro Macro Interface Contract

### Required Syntax
- `<?xml version="1.0"?>` declaration
- `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">` namespace

### Macro Definition
- `<xacro:macro name="macro_name" params="param1 param2">`: Macro definition
- Macro parameters can have default values

### Expected Behavior
- Xacro files must be processed before use as URDF
- Macros expand to valid URDF elements
- Parameter substitution occurs during processing

## Coordinate Frame Convention

### Standard Convention
- X: Forward/Right
- Y: Left/Forward (depends on convention)
- Z: Up

### Transform Specification
- `<origin xyz="x y z" rpy="roll pitch yaw">`: Position and orientation
- Units in meters for position, radians for rotation

### Expected Behavior
- Consistent frame usage across robot
- Proper parent-child relationships between frames
- Transforms are relative from parent to child