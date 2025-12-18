# Chapter 1 Specification: ROS 2 Core - Nodes, Topics, Services

## Overview
This chapter introduces readers to the fundamental concepts of ROS 2, focusing on the core communication patterns that form the backbone of any robotic system. The chapter is designed for robotics engineers and advanced CS students who need practical knowledge of ROS 2 implementation.

## Learning Objectives
- Understand the ROS 2 architecture and node-based design
- Implement basic ROS 2 nodes in Python
- Master the publisher-subscriber communication pattern (Topics)
- Master the client-server communication pattern (Services)
- Execute essential ROS 2 CLI commands for system management

## Content Structure

### Section 1.1: Introduction to ROS 2 Architecture
- What is ROS 2 and why it's important for humanoid robotics
- The node-based architecture concept
- ROS 2 vs ROS 1: Key differences relevant to this module
- Setting up the development environment

### Section 1.2: Creating ROS 2 Nodes in Python
- Understanding the node lifecycle
- Creating a basic ROS 2 node with rclpy
- Node initialization and spinning
- Error handling in nodes

### Section 1.3: Publisher-Subscriber Pattern (Topics)
- Understanding asynchronous communication
- Creating publisher nodes
- Creating subscriber nodes
- Message types and custom messages
- Quality of Service (QoS) settings

### Section 1.4: Client-Server Pattern (Services)
- Understanding synchronous communication
- Creating service servers
- Creating service clients
- Handling service requests and responses
- When to use services vs topics

### Section 1.5: Essential ROS 2 CLI Commands
- ros2 node: list, info, kill
- ros2 topic: list, echo, pub, info
- ros2 service: list, call, find
- ros2 param: get, set, list
- ros2 launch: running launch files

## Code Examples

### Example 1.1: Basic Publisher Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 1.2: Basic Subscriber Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 1.3: Service Server
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

### Example 1.4: Service Client
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

## Essential CLI Commands Reference

### Node Commands
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info <node_name>

# Kill a specific node (if needed for debugging)
# Note: Nodes should typically be stopped gracefully
```

### Topic Commands
```bash
# List all active topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Publish a message to a topic directly from command line
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello World'"

# Get information about a topic
ros2 topic info /topic_name
```

### Service Commands
```bash
# List all available services
ros2 service list

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Find all services of a specific type
ros2 service find example_interfaces/srv/AddTwoInts
```

## Hands-on Exercises

### Exercise 1.1: Create a Temperature Publisher
Create a publisher that publishes temperature readings (using std_msgs/Float64) every 2 seconds.

### Exercise 1.2: Create a Temperature Subscriber
Create a subscriber that listens to temperature readings and logs them with timestamps.

### Exercise 1.3: Create a Math Operations Service
Create a service that accepts two numbers and an operation type (add, subtract, multiply, divide) and returns the result.

## Assessment Criteria
- Students can create and run basic ROS 2 nodes
- Students can implement publisher-subscriber pairs that communicate successfully
- Students can implement service client-server pairs that communicate successfully
- Students can use essential CLI commands to monitor and manage ROS 2 systems
- Students understand when to use topics vs services for different communication needs