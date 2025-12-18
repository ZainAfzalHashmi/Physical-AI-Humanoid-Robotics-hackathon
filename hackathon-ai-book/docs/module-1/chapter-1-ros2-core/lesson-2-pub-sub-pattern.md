---
sidebar_label: 'Lesson 2: Publisher-Subscriber Pattern (Topics)'
sidebar_position: 2
---

# Lesson 2: Publisher-Subscriber Pattern (Topics)

## Overview
In this lesson, you'll learn about the publisher-subscriber communication pattern in ROS 2, which enables asynchronous communication between nodes. You'll create publishers and subscribers and understand Quality of Service (QoS) settings.

## Learning Objectives
- Understand asynchronous communication in ROS 2
- Create publisher nodes that broadcast messages
- Create subscriber nodes that receive messages
- Understand message types and custom messages
- Configure Quality of Service (QoS) settings

## Introduction to Asynchronous Communication

The publisher-subscriber pattern is a messaging pattern where publishers send messages to topics without knowing which subscribers (if any) will receive them. This enables loose coupling between nodes and allows for multiple subscribers to receive the same data stream.

For humanoid robotics, this pattern is essential for:
- Sensor data distribution (camera feeds, IMU data, etc.)
- Control commands broadcast
- Status updates across different subsystems

## Creating a Publisher Node

Let's create a publisher that sends string messages:

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

## Creating a Subscriber Node

Now let's create a subscriber that receives the messages:

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

## Message Types and Custom Messages

ROS 2 provides several built-in message types:
- `std_msgs`: Basic data types (String, Int32, Float64, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `sensor_msgs`: Sensor data (Image, LaserScan, etc.)
- `nav_msgs`: Navigation messages (Odometry, Path, etc.)

You can also create custom messages for your specific use cases.

## Quality of Service (QoS) Settings

QoS settings control how messages are delivered between publishers and subscribers:

```python
from rclpy.qos import QoSProfile

# Example with custom QoS settings
qos_profile = QoSProfile(depth=10)
self.publisher_ = self.create_publisher(String, 'topic', qos_profile)

# Reliability and durability options
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

# Reliable delivery
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Practical Example: Temperature Publisher and Subscriber

Let's create a more practical example with temperature data:

**Temperature Publisher:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float64, 'temperature', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        # Simulate temperature reading between 15-30 degrees
        msg.data = 15.0 + random.random() * 15.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    temp_publisher = TemperaturePublisher()
    rclpy.spin(temp_publisher)
    temp_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Temperature Subscriber:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'temperature',
            self.temperature_callback,
            10)
        self.subscription  # prevent unused variable warning

    def temperature_callback(self, msg):
        self.get_logger().info(f'Received temperature: {msg.data:.2f}°C')
        # Add logic based on temperature reading
        if msg.data > 25.0:
            self.get_logger().info('Temperature is high!')

def main(args=None):
    rclpy.init(args=args)
    temp_subscriber = TemperatureSubscriber()
    rclpy.spin(temp_subscriber)
    temp_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercise

Create a publisher that publishes velocity commands (geometry_msgs/Twist) and a subscriber that logs these commands. Use appropriate QoS settings for real-time control applications.

## Summary

In this lesson, you learned about the publisher-subscriber pattern in ROS 2, which enables asynchronous communication between nodes. You created both publishers and subscribers and learned about QoS settings.

## Review Questions

1. What is the difference between synchronous and asynchronous communication?
2. What is the purpose of QoS settings in ROS 2?
3. When would you use the publisher-subscriber pattern instead of services?

## Next Lesson

Continue to Lesson 3: Client-Server Pattern (Services) to learn about synchronous communication in ROS 2.