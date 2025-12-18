# URDF and SDF in Simulation: Loading Your Robot Model

In this section, you'll learn how to take the URDF robot model you created in Module 1 and load it into Gazebo for physics simulation. We'll cover the conversion process from URDF to SDF format, proper configuration for simulation, and troubleshooting common issues.

## Understanding URDF vs SDF

**URDF (Unified Robot Description Format)** is primarily used in ROS for describing robot kinematics and visual properties. It's excellent for ROS-based applications but has limitations when it comes to complex physics simulation.

**SDF (Simulation Description Format)** is designed specifically for Gazebo and other simulation environments. It extends URDF capabilities with physics-specific properties like collision geometries, friction coefficients, and material properties optimized for simulation.

## Converting URDF to SDF for Gazebo

Gazebo can automatically convert URDF files to SDF when loading, but it's often better to explicitly convert and configure for simulation. Here's how:

### Method 1: Direct Loading (Automatic Conversion)

You can load a URDF file directly into Gazebo using a ROS 2 launch file:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn your robot model
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'my_robot'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
```

### Method 2: Manual SDF Conversion

For more control, you can manually convert your URDF to SDF format. Create a file called `my_robot.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="my_robot">
    <!-- Include your URDF as a model -->
    <include>
      <uri>model://my_robot_description</uri>
      <name>my_robot</name>
    </include>

    <!-- Add Gazebo-specific plugins for simulation -->
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/my_robot</robotNamespace>
    </plugin>

    <!-- Physics properties -->
    <static>false</static>
    <self_collide>false</self_collide>
    <enable_wind>false</enable_wind>
    <kinematic>false</kinematic>
  </model>
</sdf>
```

## Configuring Your Robot for Simulation

To ensure your robot works properly in Gazebo, you need to add simulation-specific configurations to your URDF:

### 1. Add Gazebo Plugins

Include Gazebo plugins in your URDF file:

```xml
<!-- Add this to your URDF file -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

### 2. Define Material Properties

Add material properties for better visualization:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>1.0</kd>
</gazebo>
```

### 3. Configure Inertial Properties

Ensure your robot has proper inertial properties:

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/link_name.dae"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/link_name_collision.stl"/>
    </geometry>
  </collision>
</link>
```

## Launching Your Robot in Gazebo

Create a launch file to bring up your robot in Gazebo:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_xacro_file = LaunchConfiguration('robot_xacro_file')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(robot_xacro_file.get_substitution()).read()
        }]
    )

    # Spawn Entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_xacro_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'my_robot.urdf.xacro'
            ]),
            description='Full path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node
    ])
```

## Troubleshooting Common Issues

### 1. Robot Falls Through Ground
- Check that collision geometries are properly defined
- Verify that all links have proper inertial properties
- Ensure gravity is enabled in the world file

### 2. Robot Joints Don't Move Properly
- Verify joint limits and types are correctly specified
- Check that joint controllers are properly configured
- Ensure proper plugin configuration

### 3. Visual Mesh Not Loading
- Verify file paths are correct and meshes exist
- Check that mesh files are in the correct format (DAE, STL, OBJ)
- Ensure proper package paths are used

## Best Practices

1. **Use Xacro**: Use Xacro to make your URDF files more maintainable and reusable
2. **Separate Collision and Visual**: Use different meshes for collision (simpler) and visual (detailed)
3. **Proper Inertial Values**: Use realistic inertial values for stable simulation
4. **Test Incrementally**: Start with a simple model and add complexity gradually

## Exercise

Take your Module 1 URDF robot model and load it into Gazebo. Observe how it behaves under gravity, and experiment with different joint configurations. Try to make your robot stand upright by adjusting the initial joint positions.

In the next section, we'll explore how to simulate various sensors on your robot model.