---
sidebar_position: 3
---

# ROS 2 Nodes: The Building Blocks

## Introduction to ROS 2 Nodes

A ROS 2 Node is the fundamental execution environment in the Robot Operating System. Think of nodes as individual processes that perform specific functions within a robotic system. Each node typically handles a particular task such as sensor data processing, actuator control, or decision-making algorithms.

## Node Architecture

In ROS 2, nodes are designed to be:
- **Modular**: Each node performs a specific function
- **Communicative**: Nodes communicate through topics, services, and actions
- **Independent**: Nodes can be started and stopped independently
- **Reusable**: Nodes can be used across different robotic applications

## Creating a Node

A basic ROS 2 node consists of:

1. **Initialization**: Setting up the node with a unique name
2. **Communication interfaces**: Publishers, subscribers, services, or actions
3. **Execution loop**: Processing data and performing tasks
4. **Cleanup**: Properly shutting down resources

### Basic Node Structure in Python

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Communication Patterns

Nodes communicate using three primary patterns:

- **Topics**: Asynchronous, many-to-many communication
- **Services**: Synchronous, request-response communication
- **Actions**: Asynchronous, goal-oriented communication with feedback

## Node Lifecycle

ROS 2 nodes follow a specific lifecycle:

1. **Unconfigured**: Node created but not configured
2. **Inactive**: Node configured but not active
3. **Active**: Node running and processing data
4. **Finalized**: Node shut down and cleaned up

## Best Practices

- Use descriptive node names that reflect their function
- Implement proper error handling and logging
- Follow the single responsibility principle
- Use parameters for configuration
- Implement proper cleanup in destructors

## Practical Example: Sensor Node

A typical sensor node might subscribe to hardware interfaces and publish processed sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.publisher = self.create_publisher(LaserScan, 'processed_scan', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'raw_scan',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Process the raw scan data
        processed_msg = self.process_scan(msg)
        self.publisher.publish(processed_msg)

    def process_scan(self, scan_msg):
        # Implement your processing logic here
        return scan_msg
```

## Summary

Nodes form the foundation of ROS 2 architecture. Understanding how to create, configure, and manage nodes is essential for building complex robotic systems. In the next section, we'll explore how nodes communicate through Topics.