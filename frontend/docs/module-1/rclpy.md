---
sidebar_position: 6
---

# rclpy: Python Client Library for ROS 2

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing Python developers with access to all ROS 2 features including nodes, topics, services, actions, parameters, and lifecycle management. It serves as the bridge between Python applications and the ROS 2 ecosystem, enabling seamless integration of Python-based algorithms and tools into robotic systems.

## Architecture Overview

rclpy is built on top of the ROS Client Library (rcl) and the underlying DDS (Data Distribution Service) implementation. This layered architecture provides:

- **Abstraction**: High-level Python interfaces to ROS 2 concepts
- **Performance**: Direct access to the optimized C-based rcl
- **Compatibility**: Consistent API across different DDS implementations
- **Flexibility**: Support for both simple and complex robotic applications

## Core Components

### Node Management

The fundamental building block of rclpy is the Node class:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node-specific initialization

def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy
    node = MyNode()

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Clean up
        rclpy.shutdown()     # Shutdown rclpy

if __name__ == '__main__':
    main()
```

### Topic Communication

rclpy provides publisher and subscriber interfaces for topic-based communication:

```python
# Publisher
publisher = node.create_publisher(String, 'topic_name', qos_profile)

# Subscriber
subscription = node.create_subscription(
    String,           # Message type
    'topic_name',     # Topic name
    callback_function,# Callback function
    qos_profile       # QoS profile
)
```

### Service Communication

For service-based communication:

```python
# Service Server
service = node.create_service(
    AddTwoInts,           # Service type
    'service_name',       # Service name
    callback_function     # Callback function
)

# Service Client
client = node.create_client(
    AddTwoInts,           # Service type
    'service_name'        # Service name
)
```

## Advanced rclpy Features

### Parameters

rclpy provides a comprehensive parameter system:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('threshold', 0.5)

        # Get parameter values
        self.my_param = self.get_parameter('my_param').value
        self.threshold = self.get_parameter('threshold').value

        # Callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'threshold' and param.type_ == Parameter.Type.DOUBLE:
                self.threshold = param.value
        return SetParametersResult(successful=True)
```

### Timers

Execute code at regular intervals:

```python
def __init__(self):
    # ... other initialization
    self.timer_period = 0.1  # seconds
    self.timer = self.create_timer(
        self.timer_period,
        self.timer_callback
    )

def timer_callback(self):
    # Code executed at regular intervals
    msg = String()
    msg.data = f'Hello at {self.get_clock().now()}'
    self.publisher.publish(msg)
```

### Actions

For goal-oriented communication with feedback:

```python
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        # Accept or reject a goal
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept or reject a cancel request
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Execute the action
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Quality of Service (QoS) in rclpy

rclpy provides extensive QoS configuration options:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# Custom QoS profile
custom_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Built-in QoS profiles
from rclpy.qos import qos_profile_sensor_data, qos_profile_parameters

publisher = node.create_publisher(String, 'topic', qos_profile_sensor_data)
```

## Threading and Concurrency

rclpy provides several execution models for handling concurrent operations:

### Single-threaded Executor

```python
executor = SingleThreadedExecutor()
executor.add_node(node)
try:
    executor.spin()
except KeyboardInterrupt:
    pass
finally:
    executor.shutdown()
    node.destroy_node()
```

### Multi-threaded Executor

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
```

## Error Handling and Logging

Proper error handling is crucial in robotic applications:

```python
import rclpy
from rclpy.exceptions import ParameterNotDeclaredException

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        # Use logging throughout your node
        self.get_logger().info('Node initialized successfully')

    def safe_parameter_access(self, param_name, default_value):
        try:
            if self.has_parameter(param_name):
                return self.get_parameter(param_name).value
            else:
                self.declare_parameter(param_name, default_value)
                return default_value
        except ParameterNotDeclaredException:
            self.get_logger().error(f'Parameter {param_name} not declared')
            return default_value
```

## Common Patterns and Best Practices

### Node Composition

For complex applications, consider node composition:

```python
class CompositeNode(Node):
    def __init__(self):
        super().__init__('composite_node')

        # Initialize multiple components
        self.sensor_processor = SensorProcessor(self)
        self.motion_controller = MotionController(self)
        self.behavior_manager = BehaviorManager(self)
```

### Resource Management

Always implement proper cleanup:

```python
def destroy_node(self):
    # Clean up resources before destroying node
    if hasattr(self, 'publisher'):
        self.publisher.destroy()
    if hasattr(self, 'subscription'):
        self.subscription.destroy()
    if hasattr(self, 'timer'):
        self.timer.destroy()

    # Call parent destroy method
    super().destroy_node()
```

### Testing with rclpy

rclpy includes testing utilities:

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor

class TestMyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_creation(self):
        node = MyNode()
        self.assertIsNotNone(node)
        node.destroy_node()
```

## Performance Considerations

### Memory Management

- Use appropriate QoS settings to minimize memory usage
- Consider message size and frequency
- Implement message pooling for high-frequency applications

### CPU Usage

- Use timers efficiently
- Consider using callbacks vs. busy-waiting
- Profile applications for optimization

## Integration with Python Ecosystem

rclpy integrates well with Python libraries:

```python
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        # Integrate with OpenCV, NumPy, etc.
```

## Troubleshooting Common Issues

### Node Not Found

```python
# Ensure rclpy is initialized before creating nodes
rclpy.init(args=args)
node = MyNode()
```

### Topic/Service Not Connecting

- Check that topic/service names match exactly
- Verify QoS profiles are compatible
- Ensure nodes are in the same ROS domain

### Memory Leaks

- Always destroy publishers, subscribers, and timers
- Use context managers when possible
- Monitor memory usage during development

## Summary

rclpy provides the essential Python interface to ROS 2, enabling Python developers to fully participate in the ROS 2 ecosystem. Understanding its capabilities and best practices is crucial for building robust robotic applications that leverage Python's rich ecosystem. Next, we'll explore URDF (Unified Robot Description Format) for describing robot structures.