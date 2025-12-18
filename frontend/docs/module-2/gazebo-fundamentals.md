# Gazebo Fundamentals: Setting Up Physics Simulation

Gazebo is a powerful open-source robotics simulator that provides accurate physics simulation, realistic rendering, and convenient programmatic interfaces. In this section, you'll learn how to install and configure Gazebo, set up basic physics simulations, and understand the core concepts of physics-based robot simulation.

## Installing Gazebo

Gazebo is typically installed as part of a ROS 2 distribution. If you have ROS 2 installed, you can install Gazebo with the following command:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

For standalone installation:

```bash
sudo apt install gazebo
```

## Launching Gazebo

To launch Gazebo with a basic empty world:

```bash
gazebo
```

Or launch with a specific world file:

```bash
gazebo worlds/empty.world
```

## Understanding Physics Simulation

Gazebo's physics engine simulates real-world physics including gravity, collisions, and material properties. The simulation is governed by several key concepts:

### Gravity

By default, Gazebo simulates Earth's gravity (9.8 m/s²) pulling objects downward. You can modify gravity in world files:

```xml
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### Collision Detection

Gazebo uses collision shapes to detect when objects interact. Common collision shapes include:
- Box: Rectangular prisms
- Sphere: Perfect spheres
- Cylinder: Cylindrical shapes
- Mesh: Complex custom shapes

### Material Properties

Objects have properties that affect their behavior:
- Mass: How much matter the object contains
- Inertia: Resistance to rotational motion
- Friction: Resistance to sliding motion
- Bounce: Elasticity of collisions

## Creating Your First World

Create a simple world file called `simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Include a default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a default light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box object -->
    <model name="box">
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

Launch your custom world:

```bash
gazebo simple_world.world
```

## Physics Engines

Gazebo supports multiple physics engines:
- **ODE (Open Dynamics Engine)**: Default engine, good balance of speed and accuracy
- **Bullet**: Good for complex collision detection
- **DART**: Advanced dynamics with constraint-based simulation

## Controlling Simulation

You can control the simulation through:
- GUI: Pause, step, reset simulation
- ROS 2 topics: `/clock`, `/gazebo/set_physics_properties`
- Services: `/gazebo/reset_simulation`, `/gazebo/pause_physics`

## Best Practices

1. **Start Simple**: Begin with basic shapes and simple physics before moving to complex models
2. **Tune Parameters**: Adjust physics parameters to match real-world behavior
3. **Validate Results**: Compare simulation results with real-world tests when possible
4. **Performance**: Balance visual quality with simulation speed

## Exercise

Create a world file with multiple objects of different shapes and observe how they interact under gravity. Try changing gravity values and observe the effects on object motion.

In the next section, we'll explore how to load your URDF robot model into Gazebo for simulation.