# Lesson 3: Client-Server Pattern (Services)

## Learning Objectives
By the end of this lesson, you will be able to:
- Implement ROS 2 services for request/response communication
- Create custom service types
- Build both service servers and clients
- Compare service-based communication with topic-based communication
- Use command-line tools for service management

## Introduction to Services

Services in ROS 2 implement a synchronous, one-to-one communication pattern where a client sends a request to a server and waits for a response. This is fundamentally different from the asynchronous, one-to-many communication provided by topics.

### Key Concepts:
- **Service Server**: Node that provides a specific service
- **Service Client**: Node that requests the service
- **Service Type**: Defines the request and response message structure
- **Service**: Named channel for request/response communication

### When to Use Services vs. Topics:
Use services when you need:
- Synchronous communication with guaranteed response
- Request/response pattern
- Immediate acknowledgment of completion
- Simple operations that complete quickly

Use topics when you need:
- Asynchronous communication
- One-to-many broadcasting
- Continuous data streams
- Decoupled communication

## Creating a Service Server

Let's create a service that performs simple arithmetic operations:

First, we need to define our service type. Create a file `srv/Arithmetic.srv` in your package:

```
# Request
float64 a
float64 b
string operation # +, -, *, /

---
# Response
float64 result
bool success
string message
```

Now create the service server:

```python
import rclpy
from rclpy.node import Node
from your_package.srv import Arithmetic  # Replace with your package name


class ArithmeticService(Node):
    def __init__(self):
        super().__init__('arithmetic_service')
        self.srv = self.create_service(Arithmetic, 'arithmetic', self.arithmetic_callback)

    def arithmetic_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.a} {request.operation} {request.b}')
        
        if request.operation == '+':
            response.result = request.a + request.b
        elif request.operation == '-':
            response.result = request.a - request.b
        elif request.operation == '*':
            response.result = request.a * request.b
        elif request.operation == '/':
            if request.b != 0.0:
                response.result = request.a / request.b
            else:
                response.success = False
                response.message = 'Division by zero'
                return response
        else:
            response.success = False
            response.message = f'Unknown operation: {request.operation}'
            return response
        
        response.success = True
        response.message = 'Calculation successful'
        return response


def main(args=None):
    rclpy.init(args=args)
    arithmetic_service = ArithmeticService()
    
    try:
        rclpy.spin(arithmetic_service)
    except KeyboardInterrupt:
        pass
    finally:
        arithmetic_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Service Client

Now let's create a client that uses the arithmetic service:

```python
import rclpy
from rclpy.node import Node
from your_package.srv import Arithmetic  # Replace with your package name
import sys


class ArithmeticClient(Node):
    def __init__(self):
        super().__init__('arithmetic_client')
        self.cli = self.create_client(Arithmetic, 'arithmetic')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Arithmetic.Request()

    def send_request(self, a, b, operation):
        self.req.a = a
        self.req.b = b
        self.req.operation = operation

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().info(
                f'Result of {a} {operation} {b} = {response.result} '
                f'(Success: {response.success}, Message: {response.message})'
            )
            return response
        else:
            self.get_logger().error('Exception occurred while calling service')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        print('Usage: ros2 run your_package arithmetic_client <a> <b> <op>')
        return
    
    a = float(sys.argv[1])
    b = float(sys.argv[2])
    operation = sys.argv[3]
    
    arithmetic_client = ArithmeticClient()
    response = arithmetic_client.send_request(a, b, operation)
    
    arithmetic_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating Custom Service Types

### Step 1: Service Definition File
Create a `.srv` file in your package's `srv` directory. The format is:

```
# Request fields
field_type field_name
field_type field_name

---
# Response fields
field_type field_name
field_type field_name
```

Example: `srv/MoveRobot.srv`:

```
# Request
float32 x
float32 y
float32 theta

---
# Response
bool success
string message
float32 final_x
float32 final_y
```

### Step 2: Package Configuration
Add to your `package.xml`:

```xml
<depend>builtin_interfaces</depend>
<depend>std_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 3: CMake Configuration
In `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRobot.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)
```

## Essential Service Commands

### List all services:
```bash
ros2 service list
```

### Get info about a specific service:
```bash
ros2 service info /arithmetic
```

### Call a service from command line:
```bash
ros2 service call /arithmetic your_package/Arithmetic "{a: 10.0, b: 5.0, operation: '+'}"
```

### Find the type of a service:
```bash
ros2 service type /arithmetic
```

## Service Server Best Practices

### 1. Non-blocking Service Execution
Perform long-running operations without blocking the service:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading


class NonBlockingService(Node):
    def __init__(self):
        super().__init__('non_blocking_service')
        self.srv = self.create_service(
            YourServiceType, 
            'your_service', 
            self.service_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.executor = MultiThreadedExecutor()

    def service_callback(self, request, response):
        # Perform the operation in a separate thread to avoid blocking
        def long_running_operation():
            # Do the actual work here
            result = self.perform_calculation(request)
            response.result = result
            return response
            
        response = long_running_operation()
        return response
```

### 2. Error Handling in Services
Implement proper error handling in your services:

```python
class SafeService(Node):
    def __init__(self):
        super().__init__('safe_service')
        self.srv = self.create_service(
            YourServiceType, 
            'safe_service', 
            self.service_callback
        )

    def service_callback(self, request, response):
        try:
            # Validate input
            if not self.validate_request(request):
                response.success = False
                response.message = "Invalid request parameters"
                return response
            
            # Perform operation
            result = self.perform_operation(request)
            
            # Set response
            response.result = result
            response.success = True
            response.message = "Operation completed successfully"
            
        except ValueError as e:
            response.success = False
            response.message = f"Value error: {str(e)}"
        except Exception as e:
            self.get_logger().error(f'Service exception: {str(e)}')
            response.success = False
            response.message = f"Internal error: {str(e)}"
        
        return response
```

### 3. Parameterized Services
Use parameters to configure service behavior:

```python
class ConfigurableService(Node):
    def __init__(self):
        super().__init__('configurable_service')
        
        # Declare parameters
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('max_retries', 3)
        
        # Create service
        self.srv = self.create_service(
            YourServiceType,
            'configurable_service',
            self.service_callback
        )

    def service_callback(self, request, response):
        timeout = self.get_parameter('timeout').value
        max_retries = self.get_parameter('max_retries').value
        
        # Use parameters in your service logic
        for attempt in range(max_retries):
            try:
                result = self.perform_operation_with_timeout(request, timeout)
                response.result = result
                response.success = True
                break
            except TimeoutError:
                if attempt == max_retries - 1:
                    response.success = False
                    response.message = f"Operation timed out after {max_retries} attempts"
                continue
        
        return response
```

## Service Client Best Practices

### 1. Asynchronous Service Calls
Avoid blocking when making service calls:

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class AsyncServiceClient(Node):
    def __init__(self):
        super().__init__('async_service_client')
        self.cli = self.create_client(
            YourServiceType, 
            'your_service',
            callback_group=ReentrantCallbackGroup()
        )
        
        # Wait for service with timeout
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available')
            raise RuntimeError('Service not available')

    def async_call(self, request_data):
        request = YourServiceType.Request()
        # Set request data
        request.data = request_data
        
        # Make asynchronous call
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)
        
        return future

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    client = AsyncServiceClient()
    
    # Create executor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    
    # Make service call
    client.async_call("test_data")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()
```

### 2. Timeout Handling
Implement proper timeout handling for service calls:

```python
import rclpy
from rclpy.node import Node
from rclpy.task import Future


class TimeoutServiceClient(Node):
    def __init__(self, timeout=5.0):
        super().__init__('timeout_service_client')
        self.cli = self.create_client(YourServiceType, 'your_service')
        self.timeout = timeout

    def call_with_timeout(self, request):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available')
            return None

        future = self.cli.call_async(request)
        
        # Wait with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        
        if future.done():
            return future.result()
        else:
            self.get_logger().warning('Service call timed out')
            return None
```

## Service vs. Topic Comparison

| Aspect | Services | Topics |
|--------|----------|--------|
| Communication Type | Synchronous (Request/Response) | Asynchronous (Publish/Subscribe) |
| Coupling | Tight (client must know server) | Loose (publisher/subscriber independent) |
| Response | Guaranteed (blocking until response) | No direct response |
| Connection | Point-to-point | One-to-many |
| Performance | Higher latency, lower throughput | Lower latency, higher throughput |
| Use Case | Actions that require immediate response | Continuous data streaming |

## Real-World Service Examples

### 1. Navigation Service
A service for requesting robot navigation:

```python
# srv/NavigationRequest.srv
float64 target_x
float64 target_y
float64 target_theta

---
bool success
string message
float64 time_to_completion
```

### 2. Parameter Update Service
A service for updating parameters dynamically:

```python
# srv/UpdateParameter.srv
string parameter_name
string parameter_value

---
bool success
string message
```

### 3. Sensor Calibration Service
A service for calibrating sensors:

```python
# srv/CalibrateSensor.srv
string sensor_id
int32 calibration_type

---
bool success
float64[] calibration_values
string message
```

## Service Implementation Patterns

### 1. State Service Pattern
A service that maintains and reports state:

```python
class StateService(Node):
    def __init__(self):
        super().__init__('state_service')
        self.current_state = "idle"
        self.srv = self.create_service(
            GetState, 
            'get_state', 
            self.get_state_callback
        )
        self.update_srv = self.create_service(
            SetState,
            'set_state',
            self.set_state_callback
        )

    def get_state_callback(self, request, response):
        response.state = self.current_state
        return response

    def set_state_callback(self, request, response):
        self.current_state = request.state
        response.success = True
        response.message = f"State set to {request.state}"
        return response
```

### 2. Batch Operation Pattern
A service that processes multiple requests in a batch:

```python
# srv/BatchProcess.srv
YourRequestType[] requests

---
YourResponseType[] responses
bool[] success
string[] messages
```

## Common Service Pitfalls and Solutions

### 1. Blocking Service Server
**Problem**: Long-running operations in service callback blocking other requests
**Solution**: Use threading or separate the operation from the service call

### 2. Timeout Issues
**Problem**: Service calls timing out due to slow processing
**Solution**: Implement proper timeout handling and consider if service is the right communication pattern

### 3. Resource Management
**Problem**: Service operations consuming excessive resources
**Solution**: Implement rate limiting or queuing mechanisms

## Lesson Summary

In this lesson, you've learned about the client-server pattern in ROS 2:
- How to create service servers and clients
- The differences between services and topics
- How to create and use custom service types
- Essential command-line tools for service management
- Best practices for service implementation
- Real-world service examples and patterns

## Exercises

1. Create a parameter update service that allows clients to change node parameters dynamically.
2. Implement a sensor calibration service that performs calibration and returns results.
3. Build a navigation service that plans and executes robot paths.

## Next Steps

Now that you've completed the three lessons in Chapter 1, you have a comprehensive understanding of ROS 2's core communication patterns. In the next chapter, we'll explore Python bridging and more advanced topics.