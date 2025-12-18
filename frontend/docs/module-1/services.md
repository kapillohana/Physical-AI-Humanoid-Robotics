---
sidebar_position: 5
---

# ROS 2 Services: Synchronous Communication

## Introduction to Services

Services provide synchronous, request-response communication in ROS 2. Unlike topics which are asynchronous and many-to-many, services follow a client-server pattern where a client sends a request and waits for a response from a server. This pattern is ideal for operations that require immediate feedback or completion confirmation.

## Service Architecture

Services operate on a request-response model:

- **Service Server**: Node that provides a service and processes requests
- **Service Client**: Node that calls the service and waits for a response
- **Service Types**: Defined interfaces with request and response message structures

## Service Characteristics

- **Synchronous**: Client waits for the server to process the request
- **Request-Response**: One request generates one response
- **Reliable**: Requests are guaranteed to reach the server
- **Typed**: Both request and response messages have strict type definitions

## Creating Services

### Service Definition

First, define a service interface in an `.srv` file (e.g., `AddTwoInts.srv`):

```
int64 a
int64 b
---
int64 sum
```

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Asynchronous Service Clients

For non-blocking service calls:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AsyncClient(Node):
    def __init__(self):
        super().__init__('async_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_async_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.cli.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
```

## Standard Services

ROS 2 provides common service types in packages like:
- `std_srvs`: Basic services (Empty, Trigger, SetBool, etc.)
- `example_interfaces`: Example service definitions
- `nav_msgs`: Navigation services
- `tf2_msgs`: Transform services

## Service Commands

Useful command-line tools for working with services:

- `ros2 service list`: List all active services
- `ros2 service info <service_name>`: Get information about a service
- `ros2 service call <service_name> <service_type> <args>`: Call a service
- `ros2 service type <service_name>`: Get the type of a service

## Service vs Topic Decision Matrix

Use services when:
- You need immediate response
- Operation should be completed before continuing
- Request-response pattern is natural for the task
- You need to confirm success/failure

Use topics when:
- Data flows continuously
- Asynchronous processing is acceptable
- Multiple subscribers need the same data
- Real-time performance is critical

## Error Handling

Service calls can fail for various reasons. Always implement proper error handling:

```python
def send_request_with_error_handling(self, a, b):
    request = AddTwoInts.Request()
    request.a = a
    request.b = b

    future = self.cli.call_async(request)

    try:
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            return response
        else:
            self.get_logger().error('Service call returned None')
            return None
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
        return None
```

## Best Practices

- Use services for operations that require immediate feedback
- Implement proper timeout handling for service calls
- Design service interfaces to be idempotent when possible
- Use descriptive service names that indicate their function
- Consider using actions instead of services for long-running operations
- Always check for service availability before making calls

## Advanced Service Patterns

### Batch Operations

For operations that process multiple items:

```python
# Service definition for batch processing
# ProcessMultiple.srv
string[] inputs
---
string[] outputs
bool success
string error_message
```

### Status Services

Services that return system status:

```python
def get_system_status_callback(self, request, response):
    response.status = "OPERATIONAL"
    response.timestamp = self.get_clock().now().to_msg()
    response.components = ["sensor", "actuator", "controller"]
    response.health_status = [True, True, True]
    return response
```

## Summary

Services provide synchronous communication for operations requiring immediate responses. They complement topics by offering request-response patterns essential for many robotic operations. In the next section, we'll explore rclpy, the Python client library for ROS 2.