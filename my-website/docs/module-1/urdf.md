---
sidebar_position: 7
---

# URDF: Unified Robot Description Format

## Introduction to URDF

Unified Robot Description Format (URDF) is an XML-based format used to describe robots in ROS. It provides a complete specification of a robot's physical structure, including kinematic and dynamic properties, visual appearance, and collision geometry. URDF is fundamental to robot simulation, visualization, and motion planning in the ROS ecosystem.

## URDF Architecture

URDF defines robots using a tree structure of rigid bodies connected by joints:

- **Links**: Rigid bodies representing physical parts of the robot
- **Joints**: Constraints that define the relationship between links
- **Materials**: Visual appearance properties
- **Transmissions**: Actuator interface definitions
- **Gazebo plugins**: Simulation-specific extensions

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.167"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Link Elements

Links represent rigid bodies in the robot structure:

### Visual Properties
The `<visual>` element defines how the link appears in visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Choose one geometry type -->
    <box size="0.1 0.2 0.3"/>
    <!-- OR -->
    <cylinder radius="0.1" length="0.2"/>
    <!-- OR -->
    <sphere radius="0.1"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/link.stl"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Properties
The `<collision>` element defines collision geometry:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.2 0.3"/>
  </geometry>
</collision>
```

### Inertial Properties
The `<inertial>` element defines mass and inertial properties:

```xml
<inertial>
  <mass value="0.1"/>
  <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
</inertial>
```

## Joint Elements

Joints define the relationship between links:

### Joint Types
- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits
- **prismatic**: Linear joint with limits
- **fixed**: No movement allowed
- **floating**: 6 DOF movement
- **planar**: Movement in a plane

### Joint Definition Example
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Advanced URDF Features

### Materials
Define reusable materials:

```xml
<material name="silver">
  <color rgba="0.7 0.7 0.7 1"/>
  <texture filename="path/to/texture.png"/>
</material>
```

### Gazebo Extensions
Add simulation-specific properties:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>1.0</kd>
</gazebo>
```

### Transmission Elements
Define how joints connect to actuators:

```xml
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Xacro: XML Macros for URDF

Xacro extends URDF with macros, properties, and mathematical expressions:

### Basic Xacro Example
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Macros -->
  <xacro:macro name="wheel" params="prefix parent *origin">
    <joint name="${prefix}_joint" type="continuous">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_link"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_link">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- Using the macro -->
  <xacro:wheel prefix="front_left" parent="base_link">
    <origin xyz="0.2 0.1 0"/>
  </xacro:wheel>
</robot>
```

### Mathematical Expressions
```xml
<xacro:property name="pi" value="3.14159"/>
<xacro:property name="half_pi" value="${pi/2}"/>

<joint name="example_joint" type="revolute">
  <origin xyz="0 0 ${wheel_radius}" rpy="0 ${half_pi} 0"/>
  <limit lower="${-pi/4}" upper="${pi/4}" effort="10" velocity="1"/>
</joint>
```

## URDF Tools and Validation

### Checking URDF Files
```bash
# Validate URDF syntax
check_urdf my_robot.urdf

# Display URDF information
urdf_to_graphiz my_robot.urdf
```

### Robot State Publishing
```xml
<!-- Add to launch files -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
  <param name="robot_description" command="$(find xacro)/xacro $(find my_robot)/urdf/my_robot.xacro"/>
</node>
```

## Best Practices

### Naming Conventions
- Use descriptive names for links and joints
- Follow consistent naming patterns
- Use underscores to separate words
- Include semantic meaning in names

### Structure Organization
- Start with a base link
- Create a tree structure (no loops)
- Use meaningful parent-child relationships
- Consider kinematic chains

### Performance Considerations
- Simplify collision geometry for real-time applications
- Use appropriate mesh resolution
- Minimize the number of links when possible
- Optimize visual geometry for rendering

### Maintainability
- Use Xacro for complex robots
- Break large URDF files into includes
- Document non-obvious design decisions
- Use consistent formatting

## Common URDF Patterns

### Mobile Base
```xml
<link name="base_link">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/base.stl"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 0.15"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.25"/>
  </inertial>
</link>
```

### 6-DOF Arm
```xml
<!-- Shoulder -->
<joint name="shoulder_pan_joint" type="revolute">
  <parent link="base_link"/>
  <child link="shoulder_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
</joint>

<!-- Elbow -->
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="forearm_link"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Integration with ROS Tools

### Robot State Publisher
Publishes transforms for the robot structure:

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

class RobotModelNode(Node):
    def __init__(self):
        super().__init__('robot_model_node')
        # Robot state publisher handles URDF transforms
```

### TF2 and Transformations
URDF defines the static transforms between links, which TF2 uses for coordinate transformations throughout the system.

## Troubleshooting Common Issues

### Invalid URDF
- Check XML syntax and structure
- Ensure all referenced files exist
- Verify joint limits and kinematic constraints

### Visualization Problems
- Check that materials are properly defined
- Verify mesh file paths and formats
- Ensure collision and visual origins align

### Kinematic Issues
- Verify joint types and limits
- Check for kinematic loops
- Validate axis orientations

## Summary

URDF provides the essential framework for describing robot structures in ROS. Understanding its elements, syntax, and best practices is crucial for creating robots that can be properly simulated, visualized, and controlled. With URDF, rclpy, and the other ROS 2 components we've covered, you have the foundation for building complex robotic systems. In the next module, we'll explore simulation environments with Gazebo.