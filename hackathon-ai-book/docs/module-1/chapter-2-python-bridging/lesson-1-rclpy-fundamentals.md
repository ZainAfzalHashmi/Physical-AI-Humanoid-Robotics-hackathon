# Lesson 1: Python Client Library (rclpy) Fundamentals

## Learning Objectives
By the end of this lesson, you will be able to:
- Understand the architecture of rclpy and its relationship to ROS 2
- Create and manage nodes using rclpy
- Implement publishers, subscribers, services, and clients in Python
- Work with parameters and callbacks in rclpy
- Debug and profile Python ROS 2 nodes

## Introduction to rclpy

`rclpy` is the Python Client Library for ROS 2. It provides a Python API that allows users to implement ROS 2 concepts like nodes, publishers, subscribers, services, and parameters in Python. The library is a Python wrapper around the ROS Client Library (rcl), which provides the underlying functionality for ROS 2 client libraries.

### Key Features of rclpy:
- Direct access to ROS 2 functionality from Python
- Support for all ROS 2 communication patterns (topics, services, actions)
- Integration with Python's asyncio for asynchronous programming
- Parameter system for configuration
- Lifecycle management for nodes
- Logging and introspection capabilities

## Installing and Setting Up rclpy

`rclpy` comes pre-installed with ROS 2 distributions, but you can also install it separately if needed:

```bash
# When using a full ROS 2 installation, rclpy is included
# To install separately (not recommended for standard use):
pip install rclpy
```

## Basic Node Structure in rclpy

Here's the fundamental structure of a ROS 2 node implemented in Python:

```python
import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    def __init__(self):
        # Initialize the Node with a name
        super().__init__('minimal_node')
        self.get_logger().info('MinimalNode has been started')


def main(args=None):
    # Initialize the ROS 2 communication
    rclpy.init(args=args)
    
    # Create an instance of the node
    minimal_node = MinimalNode()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        minimal_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Publishers and Subscribers with rclpy

Let's look at how to implement the publisher-subscriber pattern using rclpy:

### Publisher Example:
```python
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

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Example:
```python
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

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Services and Clients with rclpy

Here's how to implement the service-client pattern:

### Service Server Example:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client Example:
```python
import sys
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


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    minimal_client.get_logger().info(
        f'Result of {sys.argv[1]} + {sys.argv[2]} = {response.sum}'
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Working with Parameters in rclpy

Parameters allow nodes to be configured without recompilation:

```python
import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_string_param', 'default_value')
        self.declare_parameter('my_int_param', 42)
        self.declare_parameter('my_double_param', 3.14)
        self.declare_parameter('my_bool_param', True)
        
        # Get parameter values
        my_string = self.get_parameter('my_string_param').value
        my_int = self.get_parameter('my_int_param').value
        my_double = self.get_parameter('my_double_param').value
        my_bool = self.get_parameter('my_bool_param').value
        
        self.get_logger().info(f'String param: {my_string}')
        self.get_logger().info(f'Int param: {my_int}')
        self.get_logger().info(f'Double param: {my_double}')
        self.get_logger().info(f'Bool param: {my_bool}')
        
        # Set a parameter callback to handle dynamic parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    
    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    from rclpy.parameter import Parameter
    from rclpy.exceptions import ParameterNotDeclaredException
    from rcl_interfaces.msg import SetParametersResult
    
    main()
```

## Advanced rclpy Concepts

### Working with Timers
Timers are used to execute callbacks at regular intervals:

```python
import rclpy
from rclpy.node import Node


class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        
        # Create a timer that calls a callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Create a single-shot timer
        self.one_shot_timer = self.create_timer(5.0, self.one_shot_callback)
        self.one_shot_timer.cancel()  # Cancel it for now
        
        self.count = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executed {self.count} times')
        self.count += 1
        
        # Activate the one-shot timer after 10 regular callbacks
        if self.count == 10:
            self.get_logger().info('Starting one-shot timer')
            self.one_shot_timer.reset()

    def one_shot_callback(self):
        self.get_logger().info('One-shot timer executed')
        self.one_shot_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    timer_node = TimerNode()
    
    try:
        rclpy.spin(timer_node)
    except KeyboardInterrupt:
        pass
    finally:
        timer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Using Callback Groups
Callback groups allow you to control the execution of callbacks:

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class CallbackGroupNode(Node):
    def __init__(self):
        super().__init__('callback_group_node')
        
        # Create callback groups
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()  # Can be called in parallel
        
        # Create a publisher
        self.pub = self.create_publisher(String, 'callback_test', 10)
        
        # Create timers with different callback groups
        self.timer1 = self.create_timer(1.0, self.timer1_callback, callback_group=self.group1)
        self.timer2 = self.create_timer(1.5, self.timer2_callback, callback_group=self.group2)
        self.timer3 = self.create_timer(2.0, self.timer3_callback, callback_group=self.group3)
        
    def timer1_callback(self):
        self.get_logger().info('Timer 1 callback (Group 1)')
        msg = String()
        msg.data = 'Message from Timer 1'
        self.pub.publish(msg)

    def timer2_callback(self):
        self.get_logger().info('Timer 2 callback (Group 2)')
        msg = String()
        msg.data = 'Message from Timer 2'
        self.pub.publish(msg)

    def timer3_callback(self):
        self.get_logger().info('Timer 3 callback (Group 3)')
        msg = String()
        msg.data = 'Message from Timer 3'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CallbackGroupNode()
    
    # Use a multi-threaded executor to handle different callback groups properly
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    from std_msgs.msg import String
    main()
```

## Error Handling and Logging in rclpy

Proper error handling is crucial for robust ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.publisher = self.create_publisher(String, 'robust_topic', 10)
        
        # Create a timer with error handling
        self.timer = self.create_timer(1.0, self.safe_timer_callback)
        self.error_count = 0

    def safe_timer_callback(self):
        try:
            # Simulate potential failure
            if self.error_count < 3:
                # Simulate an error every now and then
                if self.error_count % 2 == 0:
                    raise RuntimeError(f"Simulated error #{self.error_count}")
                else:
                    msg = String()
                    msg.data = f'Normal message #{self.error_count}'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
            self.error_count += 1
        finally:
            self.error_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = RobustNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    except Exception as e:
        node.get_logger().fatal(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Debugging rclpy Nodes

### Using ROS 2 Command Line Tools for Debugging:
- `ros2 node list` - List all active nodes
- `ros2 node info <node_name>` - Get details about a specific node
- `ros2 topic list` - List all active topics
- `ros2 service list` - List all active services
- `ros2 param list <node_name>` - List parameters of a node
- `ros2 lifecycle list <node_name>` - List lifecycle states

### Adding Debug Information:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        
        # Enable different log levels
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Subscribe to a topic
        self.subscription = self.create_subscription(
            String,
            'debug_topic',
            self.debug_callback,
            10)
        
        # Create a publisher
        self.publisher = self.create_publisher(String, 'debug_output', 10)
        
        self.get_logger().debug('Debug node initialized')
        self.get_logger().info('Debug node running')

    def debug_callback(self, msg):
        self.get_logger().debug(f'Received message: {msg.data}')
        
        # Process message
        processed_msg = String()
        processed_msg.data = f'Processed: {msg.data}'
        
        self.publisher.publish(processed_msg)
        self.get_logger().info(f'Published processed message: {processed_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Debug node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Performance Considerations in rclpy

### 1. Avoid Blocking Operations
```python
# BAD: This will block the node
import time
def blocking_callback(self, msg):
    time.sleep(5)  # Blocks the entire node!
    self.publisher.publish(msg)

# GOOD: Use non-blocking alternatives
import asyncio
async def async_callback(self, msg):
    await asyncio.sleep(5)  # Non-blocking
    self.publisher.publish(msg)
```

### 2. Efficient Message Handling
```python
# GOOD: Efficient message processing
def efficient_callback(self, msg):
    # Process message quickly
    self.internal_state.update(msg.data)
    # Publish result if needed
    result_msg = String()
    result_msg.data = self.process_data(msg.data)
    self.publisher.publish(result_msg)
```

## Best Practices for rclpy Development

1. **Always clean up resources** in `destroy_node()` or `finally` blocks
2. **Use appropriate log levels** for different types of messages
3. **Handle exceptions gracefully** to maintain system stability
4. **Declare parameters** before using them
5. **Use callback groups** appropriately for complex nodes
6. **Test parameter limits** to prevent runtime errors
7. **Implement proper error recovery** mechanisms

## Lesson Summary

In this lesson, you've learned the fundamentals of rclpy, the Python Client Library for ROS 2:
- Basic node structure and lifecycle
- Implementation of publishers, subscribers, services, and clients
- Parameter handling and callbacks
- Advanced concepts like timers and callback groups
- Error handling and debugging techniques
- Performance considerations for Python nodes

## Exercises

1. Create a node that publishes sensor data at a fixed frequency using a timer.
2. Implement a service that processes incoming requests and returns calculated results.
3. Build a node that uses parameters to configure its behavior at runtime.
4. Create a complex node using different callback groups to manage multiple concurrent operations.

## Next Steps

In the next lesson, we'll explore creating Python packages and implementing more complex nodes.