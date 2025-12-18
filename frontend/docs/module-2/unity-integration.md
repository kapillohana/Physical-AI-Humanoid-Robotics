# Integration with Unity: High-Fidelity Rendering and Human-Robot Interaction

In this section, you'll learn how to connect your Gazebo physics simulation with Unity for high-fidelity rendering and human-robot interaction. This integration allows you to leverage Unity's powerful graphics capabilities while maintaining Gazebo's accurate physics simulation.

## Overview of Gazebo-Unity Integration

The Gazebo-Unity integration enables you to:
- Visualize your robot and environment with photorealistic quality
- Create immersive human-robot interaction interfaces
- Develop AR/VR applications for robot teleoperation
- Test perception algorithms with realistic rendering

## Unity Robotics Setup

### Installing Unity

1. Download Unity Hub from the Unity website
2. Install Unity Editor (2021.3 LTS or later recommended)
3. Create a Unity project for your robotics application

### Unity Robotics Hub Package

The Unity Robotics Hub provides essential tools for ROS integration:

1. Open your Unity project
2. Go to Window → Package Manager
3. Click the + button → Add package from git URL
4. Add: `com.unity.robotics.ros-tcp-connector` (for ROS communication)
5. Add: `com.unity.robotics.urdf-importer` (for importing robot models)

## Setting Up ROS-TCP-Connector

The ROS-TCP-Connector enables communication between ROS 2 and Unity:

### In Unity:

1. Create a new GameObject and add the ROSConnection component
2. Configure the IP address and port (typically localhost:10000)
3. Create publisher and subscriber scripts as needed

Example Unity script for receiving robot joint states:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class JointStateSubscriber : MonoBehaviour
{
    [SerializeField] private string topicName = "/my_robot/joint_states";
    [SerializeField] private GameObject[] joints; // Array of joint objects in Unity

    private ROSConnection ros;
    private float[] jointPositions;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<sensor_msgs_JointState>(topicName, JointStateCallback);
    }

    void JointStateCallback(sensor_msgs_JointState jointState)
    {
        if (jointPositions == null || jointPositions.Length != jointState.position.Count)
        {
            jointPositions = new float[jointState.position.Count];
        }

        for (int i = 0; i < jointState.position.Count && i < joints.Length; i++)
        {
            jointPositions[i] = (float)jointState.position[i];
            joints[i].transform.localEulerAngles = new Vector3(0, 0, jointPositions[i] * Mathf.Rad2Deg);
        }
    }
}
```

### In ROS 2:

Create a bridge node to relay data between Gazebo and Unity:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from unity_robotics_demo_msgs.msg import UnityHand, UnityArm

class GazeboUnityBridge(Node):
    def __init__(self):
        super().__init__('gazebo_unity_bridge')

        # Subscribe to Gazebo joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/my_robot/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for Unity visualization
        self.unity_joint_pub = self.create_publisher(
            JointState,
            '/unity/joint_states',
            10
        )

        # Publisher for Unity sensor data
        self.unity_sensor_pub = self.create_publisher(
            Range,
            '/unity/laser_scan',
            10
        )

    def joint_state_callback(self, msg):
        # Relay joint states to Unity
        self.unity_joint_pub.publish(msg)

        # Process and send additional data to Unity as needed
        self.process_for_unity(msg)

    def process_for_unity(self, joint_state_msg):
        # Process data specifically for Unity visualization
        # This might include custom messages or transformations
        pass
```

## Unity Robotics Simulation Framework

### URDF Importer

The Unity URDF Importer allows you to import your robot model directly:

1. Go to GameObject → URDF Importer → Load URDF
2. Select your URDF file or Xacro file
3. Configure import settings:
   - Import visuals and collisions
   - Set joint types and limits
   - Configure materials and textures

### Creating the Robot in Unity

After importing your URDF, you'll have a robot model in Unity that can be controlled via ROS messages. The imported model will maintain the kinematic structure of your original robot.

## Synchronization Between Gazebo and Unity

### Physics Synchronization

To maintain synchronization between Gazebo physics and Unity visualization:

1. **State Publishing**: Gazebo publishes robot states (joint positions, velocities, efforts) to ROS topics
2. **State Subscribing**: Unity subscribes to these topics and updates the visual representation
3. **Command Publishing**: Unity can publish commands back to Gazebo to control the physical simulation

### Time Synchronization

Ensure both systems use the same time reference:

```csharp
// In Unity, handle ROS time messages
public class TimeSynchronizer : MonoBehaviour
{
    [SerializeField] private bool useSimulationTime = true;

    void OnEnable()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<rosgraph_msgs_Clock>(
            "/clock",
            ClockCallback
        );
    }

    void ClockCallback(rosgraph_msgs_Clock clock)
    {
        // Use simulation time instead of real time
        if (useSimulationTime)
        {
            Time.timeScale = 1.0f; // Or adjust as needed
        }
    }
}
```

## Human-Robot Interaction in Unity

### Teleoperation Interface

Create a Unity interface for human-robot interaction:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class TeleoperationController : MonoBehaviour
{
    [SerializeField] private string cmdVelTopic = "/my_robot/cmd_vel";
    [SerializeField] private float moveSpeed = 1.0f;
    [SerializeField] private float turnSpeed = 1.0f;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    void Update()
    {
        // Get input from user
        float moveInput = Input.GetAxis("Vertical");
        float turnInput = Input.GetAxis("Horizontal");

        // Create twist message
        geometry_msgs_Twist cmdVel = new geometry_msgs_Twist
        {
            linear = new geometry_msgs_Vector3
            {
                x = moveInput * moveSpeed,
                y = 0,
                z = 0
            },
            angular = new geometry_msgs_Vector3
            {
                x = 0,
                y = 0,
                z = turnInput * turnSpeed
            }
        };

        // Publish command
        ros.Publish(cmdVelTopic, cmdVel);
    }
}
```

### VR/AR Integration

For immersive interaction, integrate with VR/AR platforms:

```csharp
// Example for VR hand tracking
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;

public class VRHandController : MonoBehaviour
{
    [SerializeField] private string handPoseTopic = "/my_robot/hand_pose";

    void Update()
    {
        // Get VR controller input
        InputDevice device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

        // Publish hand pose to ROS
        if (device.isValid)
        {
            Vector3 position = transform.position;
            Quaternion rotation = transform.rotation;

            // Publish to ROS topic for robot manipulation
        }
    }
}
```

## Advanced Visualization Techniques

### Custom Shaders for Robotics

Create specialized shaders for robotics visualization:

```hlsl
Shader "Robotics/DepthVisualization"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _DepthRange ("Depth Range", Range(0, 100)) = 10.0
    }
    SubShader
    {
        // Visualization shader for depth camera data
    }
}
```

### Point Cloud Visualization

Display point cloud data from simulated LiDAR:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class PointCloudVisualizer : MonoBehaviour
{
    [SerializeField] private string pointCloudTopic = "/my_robot/points";
    [SerializeField] private GameObject pointPrefab;
    [SerializeField] private Transform pointCloudContainer;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<sensor_msgs_PointCloud2>(
            pointCloudTopic,
            PointCloudCallback
        );
    }

    void PointCloudCallback(sensor_msgs_PointCloud2 pointCloud)
    {
        // Parse and visualize point cloud data
        // This is a simplified example - actual implementation would be more complex
    }
}
```

## Performance Optimization

### Level of Detail (LOD)

Implement LOD systems for complex environments:

```csharp
public class RobotLOD : MonoBehaviour
{
    [SerializeField] private Renderer[] lodRenderers;
    [SerializeField] private float[] lodDistances;

    private Camera mainCamera;

    void Start()
    {
        mainCamera = Camera.main;
    }

    void Update()
    {
        float distance = Vector3.Distance(mainCamera.transform.position, transform.position);

        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distance < lodDistances[i])
            {
                SetLOD(i);
                return;
            }
        }
    }

    void SetLOD(int lodLevel)
    {
        for (int i = 0; i < lodRenderers.Length; i++)
        {
            lodRenderers[i].enabled = (i == lodLevel);
        }
    }
}
```

## Launching the Integrated System

Create a launch file to start both Gazebo and Unity:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Launch ROS-Unity bridge
    ros_unity_bridge = Node(
        package='my_robot_bringup',
        executable='gazebo_unity_bridge',
        name='gazebo_unity_bridge',
        output='screen'
    )

    # Launch Unity application (if it has a ROS interface)
    unity_app = ExecuteProcess(
        cmd=['/path/to/unity/robotics/app'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        ros_unity_bridge,
        unity_app
    ])
```

## Best Practices for Unity Integration

1. **Performance**: Optimize Unity scenes for real-time rendering while maintaining quality
2. **Synchronization**: Ensure proper timing between Gazebo physics and Unity visualization
3. **Data Efficiency**: Minimize network traffic between ROS and Unity
4. **Error Handling**: Implement robust error handling for connection failures
5. **Scalability**: Design systems that can handle multiple robots and sensors

## Exercise

Set up a basic Gazebo-Unity integration for your robot model. Create a Unity scene that visualizes your robot's movement based on joint states from Gazebo. Add a simple teleoperation interface that allows you to control the robot through Unity and see the effects in both Gazebo and Unity.

This concludes Module 2! You now have the tools to create sophisticated digital twins of your robots with physics-accurate simulation and high-fidelity visualization.