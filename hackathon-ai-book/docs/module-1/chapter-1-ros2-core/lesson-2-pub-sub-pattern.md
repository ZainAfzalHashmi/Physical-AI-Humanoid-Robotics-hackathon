# Lesson 2: Publisher-Subscriber Pattern (Topics)

## Learning Objectives
By the end of this lesson, you will be able to:
- Implement publishers and subscribers in ROS 2
- Create and use custom message types
- Understand Quality of Service (QoS) settings for topics
- Debug topic communication issues
- Use command-line tools for topic management

## Introduction to Topics

Topics in ROS 2 implement a one-to-many, asynchronous communication pattern where publishers send messages to a topic without knowing which subscribers will receive them. This decouples the publisher from the subscriber, allowing for flexible system design.

### Key Concepts:
- **Publisher**: Node that sends messages to a topic
- **Subscriber**: Node that receives messages from a topic
- **Message**: Data structure sent between nodes
- **Topic**: Named channel through which messages are sent

## Creating a Publisher Node

Let's create a publisher that sends temperature readings:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Temperature Publisher has started')

    def timer_callback(self):
        msg = Float32()
        msg.data = random.uniform(18.0, 30.0)  # Random temperature between 18-30°C
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}°C')


def main(args=None):
    rclpy.init(args=args)
    temperature_publisher = TemperaturePublisher()
    
    try:
        rclpy.spin(temperature_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

Now let's create a subscriber that receives temperature messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received temperature: {msg.data}°C')
        
        # Simple logic to trigger alerts if temperature is too high or low
        if msg.data > 25.0:
            self.get_logger().warn(f'High temperature alert: {msg.data}°C')
        elif msg.data < 20.0:
            self.get_logger().warn(f'Low temperature alert: {msg.data}°C')


def main(args=None):
    rclpy.init(args=args)
    temperature_subscriber = TemperatureSubscriber()
    
    try:
        rclpy.spin(temperature_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

QoS settings allow you to control the behavior of your publishers and subscribers in terms of reliability, durability, and other aspects. Here are the most common QoS profiles:

### Reliability Policy
- **RELIABLE**: All messages are delivered (with retries)
- **BEST_EFFORT**: Messages may be lost, but lower latency

### Durability Policy
- **TRANSIENT_LOCAL**: Publisher keeps message history for late-joining subscribers
- **VOLATILE**: No message history kept

### Example with QoS:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        # Create a QoS profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.publisher_ = self.create_publisher(Float32, 'qos_topic', qos_profile)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = 42.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        
        # Use the same QoS profile for the subscriber
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.subscription = self.create_subscription(
            Float32,
            'qos_topic', 
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## Creating Custom Message Types

While standard messages like `Float32` are useful, you'll often need custom message types for more complex data.

### Step 1: Create the .msg file
In your package's `msg` directory, create `RobotStatus.msg`:

```
string robot_name
float32 battery_level
bool is_moving
int32[] sensors_status
```

### Step 2: Configure package.xml
Add the following dependencies to your `package.xml`:

```xml
<depend>builtin_interfaces</depend>
<depend>std_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 3: Update CMakeLists.txt
Add the following lines:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
```

### Step 4: Using Custom Messages
```python
from py_nodes_tutorial.msg import RobotStatus  # Your custom message

class CustomMessagePublisher(Node):
    def __init__(self):
        super().__init__('custom_msg_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = RobotStatus()
        msg.robot_name = 'RoverBot_01'
        msg.battery_level = 85.5
        msg.is_moving = True
        msg.sensors_status = [1, 1, 0, 1]  # 1 for active, 0 for inactive
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published robot status for {msg.robot_name}')
```

## Essential Topic Commands

### List all topics:
```bash
ros2 topic list
```

### Get info about a specific topic:
```bash
ros2 topic info /temperature
```

### Echo messages from a topic:
```bash
ros2 topic echo /temperature
```

### Publish a single message to a topic:
```bash
ros2 topic pub /temperature std_msgs/Float32 "{data: 25.0}"
```

### Find the type of a topic:
```bash
ros2 topic type /temperature
```

### Monitor topic statistics:
```bash
ros2 topic hz /temperature
```

## Common Topic Patterns

### 1. Sensor Data Pattern
Publish sensor readings at regular intervals:

```python
class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)
        
        # Use a timer to publish at a fixed frequency
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
    def publish_sensor_data(self):
        msg = Float32()
        # Simulate sensor reading
        msg.data = self.read_sensor()  # Replace with actual sensor reading
        self.publisher_.publish(msg)
```

### 2. Control Command Pattern
Subscribe to commands and control actuators:

```python
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Float32,
            'motor_speed_cmd',
            self.command_callback,
            10)
            
    def command_callback(self, msg):
        # Send command to actual motor
        self.control_motor(msg.data)
        self.get_logger().info(f'Setting motor speed to: {msg.data}')
```

### 3. Transformation Pattern
Subscribe to one topic and publish to another after processing:

```python
class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.subscriber = self.create_subscription(
            Float32,
            'raw_data',
            self.process_callback,
            10)
        self.publisher = self.create_publisher(Float32, 'processed_data', 10)
        
    def process_callback(self, msg):
        # Process the incoming data
        processed_value = msg.data * 2.0  # Example processing
        
        # Publish the processed data
        processed_msg = Float32()
        processed_msg.data = processed_value
        self.publisher.publish(processed_msg)
```

## Debugging Topic Communication

### Check if nodes are communicating:
```bash
# In one terminal
ros2 run demo_nodes_cpp talker

# In another terminal
ros2 run demo_nodes_cpp listener
```

### Monitor bandwidth usage:
```bash
ros2 topic bw /temperature
```

### Check for latched connections:
```bash
ros2 topic info /parameter_events
```

## Advanced Topic Concepts

### Latching (Transient Local Durability)
Latching ensures that the last published message is saved on the topic and sent to any new subscribers that connect:

```python
from rclpy.qos import QoSProfile, DurabilityPolicy

latched_qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

latched_publisher = node.create_publisher(MyMessage, 'latched_topic', latched_qos)
```

### Publisher/Subscriber Events
Monitor connection and disconnection events:

```python
def subscription_match_callback(event):
    print(f'New subscription matched for publisher: {event}')

publisher = node.create_publisher(MyMessage, 'topic', 10)
publisher.matched_callback = subscription_match_callback
```

## Best Practices for Topics

1. **Topic Naming**: Use descriptive, consistent naming conventions (e.g., `/robot_name/sensor_name`)
2. **Message Design**: Keep messages lightweight and include only necessary data
3. **QoS Matching**: Ensure publisher and subscriber QoS settings are compatible
4. **Frequency**: Don't publish more frequently than necessary to avoid network congestion
5. **Data Types**: Use appropriate data types for your application requirements
6. **Rate Limiting**: Implement rate limiting to prevent overwhelming slower nodes

## Lesson Summary

In this lesson, you've learned about the publisher-subscriber pattern in ROS 2:
- How to create publishers and subscribers
- QoS settings and their impact on communication
- How to create and use custom message types
- Essential command-line tools for topic management
- Common design patterns for topic-based communication
- Debugging techniques for topic communication

## Exercises

1. Create a publisher that publishes robot pose information (x, y, theta) using a custom message type.
2. Implement a subscriber that subscribes to sensor data and publishes commands based on the received data.
3. Experiment with different QoS settings and observe how they affect communication.

## Next Steps

In the next lesson, we'll explore the service-based communication pattern that enables synchronous request/response interactions between nodes.