# Sensor Simulation: LiDAR, Depth Cameras, and IMUs

In this section, you'll learn how to simulate various sensors on your robot model in Gazebo. Realistic sensor simulation is crucial for developing perception algorithms and navigation systems that can later be deployed on physical robots. We'll cover LiDAR, depth cameras (like RealSense), and IMUs.

## Overview of Sensor Simulation

Sensor simulation in Gazebo works by creating virtual sensors that generate realistic data streams similar to their physical counterparts. These simulated sensors publish data to ROS 2 topics just like real sensors, allowing you to develop and test perception algorithms without physical hardware.

## LiDAR Simulation

LiDAR (Light Detection and Ranging) sensors are essential for navigation and mapping. In Gazebo, you can simulate various types of LiDAR sensors.

### Adding a 2D LiDAR to Your Robot

Add a 2D LiDAR sensor to your URDF file:

```xml
<link name="laser_link">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.02" length="0.04"/>
    </geometry>
  </collision>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.02" length="0.04"/>
    </geometry>
    <material name="black"/>
  </visual>

  <inertial>
    <mass value="0.1" />
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
  </inertial>
</link>

<joint name="laser_joint" type="fixed">
  <origin xyz="0.1 0.0 0.1" rpy="0 0 0" />
  <parent link="base_link"/>
  <child link="laser_link" />
</joint>

<gazebo reference="laser_link">
  <sensor type="ray" name="laser">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### Adding a 3D LiDAR (HDL-32E Style)

For 3D mapping and perception:

```xml
<gazebo reference="laser_link">
  <sensor type="ray" name="hdl32e">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1024</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>32</samples>
          <resolution>1</resolution>
          <min_angle>-0.523599</min_angle>
          <max_angle>0.523599</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="hdl32e_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera Simulation (RealSense-style)

Depth cameras provide both RGB and depth information, essential for 3D perception and object recognition.

### Adding an RGB-D Camera

```xml
<link name="camera_link">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.05 0.1 0.04"/>
    </geometry>
  </collision>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.05 0.1 0.04"/>
    </geometry>
    <material name="red"/>
  </visual>

  <inertial>
    <mass value="0.01" />
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <origin xyz="0.1 0.0 0.1" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="camera_link"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="depth" name="camera">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.089</horizontal_fov>
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>/my_robot/camera/image_raw</imageTopicName>
      <depthImageTopicName>/my_robot/camera/depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>/my_robot/camera/depth/points</pointCloudTopicName>
      <cameraInfoTopicName>/my_robot/camera/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>/my_robot/camera/depth/camera_info</depthImageCameraInfoTopicName>
      <frameName>camera_depth_optical_frame</frameName>
      <pointCloudCutoff>0.1</pointCloudCutoff>
      <pointCloudCutoffMax>3.5</pointCloudCutoffMax>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <CxPrime>0.0</CxPrime>
      <Cx>320.0</Cx>
      <Cy>240.0</Cy>
      <focalLength>320.0</focalLength>
      <hackBaseline>0.07</hackBaseline>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Simulation

IMUs (Inertial Measurement Units) provide orientation and acceleration data, crucial for robot stabilization and navigation.

### Adding an IMU to Your Robot

```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <topic>__default_topic__</topic>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## Visualizing Sensor Data

Once your sensors are configured, you can visualize the data using ROS 2 tools:

### Using RViz2

Launch RViz2 to visualize sensor data:

```bash
ros2 run rviz2 rviz2
```

In RViz2, add displays for:
- LaserScan for LiDAR data
- Image for camera feeds
- PointCloud2 for 3D point clouds
- Imu for orientation data

### Command Line Visualization

Monitor sensor topics from the command line:

```bash
# View laser scan data
ros2 topic echo /my_robot/scan

# View camera image info
ros2 topic echo /my_robot/camera/image_raw

# View IMU data
ros2 topic echo /my_robot/imu
```

## Sensor Fusion Example

Combine data from multiple sensors for enhanced perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create subscribers for all sensors
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/my_robot/scan',
            self.scan_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/my_robot/imu',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/my_robot/camera/image_raw',
            self.camera_callback,
            10
        )

        self.cv_bridge = CvBridge()
        self.latest_scan = None
        self.latest_imu = None
        self.latest_image = None

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.process_sensor_data()

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.process_sensor_data()

    def camera_callback(self, msg):
        self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_sensor_data()

    def process_sensor_data(self):
        # Combine sensor data for perception tasks
        if self.latest_scan and self.latest_imu:
            # Example: Use IMU data to correct LiDAR for robot tilt
            # Use camera data for object detection in LiDAR points
            pass
```

## Best Practices for Sensor Simulation

1. **Match Real Sensor Specs**: Configure simulation parameters to match real sensor specifications
2. **Add Noise**: Include realistic noise models to make simulation more challenging
3. **Validate Data**: Verify that sensor data looks realistic and matches expectations
4. **Performance**: Balance sensor quality with simulation performance
5. **Topic Names**: Use consistent and descriptive topic names across all sensors

## Exercise

Add LiDAR, depth camera, and IMU sensors to your robot model from Module 1. Launch the simulation and visualize the sensor data in RViz2. Experiment with different sensor configurations and observe how the data changes in different environments.

In the next section, we'll explore how to integrate Gazebo with Unity for high-fidelity visualization and human-robot interaction.