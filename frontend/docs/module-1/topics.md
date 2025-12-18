---
sidebar_position: 4
---

# ROS 2 Topics: Asynchronous Communication

## Introduction to Topics

Topics are the primary mechanism for asynchronous, many-to-many communication in ROS 2. They enable nodes to publish data that can be received by multiple subscriber nodes simultaneously, creating a decoupled communication architecture that is fundamental to distributed robotic systems.

## Topic Architecture

Topics operate on a publish-subscribe model:

- **Publishers**: Nodes that send data to a topic
- **Subscribers**: Nodes that receive data from a topic
- **Message Types**: Strictly defined data structures that ensure compatibility

## Topic Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Decoupled**: Publishers and subscribers are unaware of each other
- **Many-to-many**: Multiple publishers can send to a topic, multiple subscribers can receive from it
- **Typed**: All messages on a topic must conform to a specific message type

## Creating Publishers and Subscribers

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Quality of Service (QoS) Settings

ROS 2 provides QoS policies to control communication behavior:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (replay last message to new subscribers)
- **History**: Keep all messages vs. keep last N messages
- **Depth**: Size of the message queue

### QoS Example

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

## Message Types

ROS 2 provides standard message types in packages like:
- `std_msgs`: Basic data types (String, Int32, Float64, etc.)
- `sensor_msgs`: Sensor data (LaserScan, Image, JointState, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `nav_msgs`: Navigation messages (Odometry, Path, OccupancyGrid, etc.)

## Topic Commands

Useful command-line tools for working with topics:

- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 topic info <topic_name>`: Get information about a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish to a topic

## Best Practices

- Use descriptive topic names following ROS conventions
- Choose appropriate QoS settings for your application
- Implement proper message validation
- Consider bandwidth and frequency requirements
- Use appropriate message types or define custom ones when needed

## Advanced Topic Patterns

### Latching (Transient Local Durability)

For critical information that new subscribers should receive immediately:

```python
from rclpy.qos import QoSDurabilityPolicy

qos_latched = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
```

### Throttling

Limit the rate of message processing:

```python
from rclpy.time import Time

def __init__(self):
    # ... other initialization
    self.last_msg_time = Time()

def listener_callback(self, msg):
    current_time = self.get_clock().now()
    if (current_time - self.last_msg_time).nanoseconds > 100000000:  # 100ms
        # Process message
        self.last_msg_time = current_time
```

## Summary

Topics provide the asynchronous communication backbone for ROS 2 systems. Understanding their proper use is crucial for building responsive and efficient robotic applications. Next, we'll explore Services for synchronous communication patterns.