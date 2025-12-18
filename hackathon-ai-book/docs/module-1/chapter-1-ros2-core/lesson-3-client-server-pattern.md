---
sidebar_label: 'Lesson 3: Client-Server Pattern (Services)'
sidebar_position: 3
---

# Lesson 3: Client-Server Pattern (Services)

## Overview
In this lesson, you'll learn about the client-server communication pattern in ROS 2, which enables synchronous communication between nodes. You'll create services and clients and understand when to use services versus topics.

## Learning Objectives
- Understand synchronous communication in ROS 2
- Create service servers that respond to requests
- Create service clients that make requests
- Handle service requests and responses
- Understand when to use services vs topics

## Introduction to Synchronous Communication

The client-server pattern is a messaging pattern where clients send requests to services and wait for responses. This enables synchronous communication where the client blocks until it receives a response.

For humanoid robotics, this pattern is essential for:
- Action execution requests (move to position, grasp object)
- Configuration changes (set parameters, change modes)
- Synchronous data requests (get current state, sensor calibration)

## Creating a Service Server

Let's create a service server that adds two integers:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Now let's create a client that calls the service:

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Service Requests and Responses

Service callbacks receive both a request object and a response object. The callback processes the request and returns the response:

```python
def service_callback(self, request, response):
    # Process the request
    result = perform_calculation(request.input)

    # Set the response
    response.output = result

    # Log the operation
    self.get_logger().info(f'Processed request: {request.input} -> {response.output}')

    # Return the response
    return response
```

## When to Use Services vs Topics

### Use Services When:
- You need a response to a specific request
- Communication is request-response based
- You need guaranteed delivery of the response
- The operation has a clear beginning and end
- You're performing an action that returns a result

### Use Topics When:
- You need to broadcast information continuously
- Communication is one-to-many or many-to-many
- You don't need guaranteed delivery
- Data is streaming or periodic
- You're sharing sensor data or status updates

## Practical Example: Robot Control Service

Let's create a practical example for humanoid robot control:

**Robot Control Service Server:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger  # Built-in service for simple triggers

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        self.srv = self.create_service(
            Trigger,
            'robot_move_to_home',
            self.move_to_home_callback
        )
        self.get_logger().info('Robot control service started')

    def move_to_home_callback(self, request, response):
        # Simulate moving robot to home position
        self.get_logger().info('Moving robot to home position...')

        # In a real implementation, this would control the actual robot
        # For simulation, we'll just wait a bit
        import time
        time.sleep(2)

        # Set response
        response.success = True
        response.message = 'Robot successfully moved to home position'

        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    robot_service = RobotControlService()
    rclpy.spin(robot_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Robot Control Service Client:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class RobotControlClient(Node):
    def __init__(self):
        super().__init__('robot_control_client')
        self.cli = self.create_client(Trigger, 'robot_move_to_home')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Trigger.Request()

    def send_move_home_request(self):
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    client = RobotControlClient()

    future = client.send_move_home_request()

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        result = future.result()
        if result.success:
            client.get_logger().info(f'Success: {result.message}')
        else:
            client.get_logger().error(f'Failed: {result.message}')
    else:
        client.get_logger().error('Service call failed')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Combining Topics and Services

In real applications, you'll often combine both patterns. For example, a robot might:
- Use services to request specific actions
- Use topics to broadcast sensor data
- Use services to get current robot state
- Use topics to stream camera feeds

## Hands-On Exercise

Create a service that accepts robot joint angles and returns whether those angles are valid (within joint limits). Create a client that calls this service with different angle values.

## Summary

In this lesson, you learned about the client-server pattern in ROS 2, which enables synchronous communication between nodes. You created both services and clients and learned when to use services versus topics.

## Review Questions

1. What is the difference between synchronous and asynchronous communication?
2. When would you use services instead of topics?
3. How do service callbacks work in ROS 2?

## Chapter Conclusion

In this chapter, you've learned the three fundamental communication patterns in ROS 2:
1. Nodes - the basic computational units
2. Topics - for asynchronous communication
3. Services - for synchronous communication

These patterns form the foundation of ROS 2 and are essential for building complex robotic systems, especially humanoid robots with multiple subsystems that need to communicate effectively.

## Next Chapter

Continue to Chapter 2: Python Bridging with rclpy to learn how to connect high-level AI agents with ROS controllers.