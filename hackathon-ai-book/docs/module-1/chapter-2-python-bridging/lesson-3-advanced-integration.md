# Lesson 3: Advanced Python Patterns and Integration

## Learning Objectives
By the end of this lesson, you will be able to:
- Create and use custom message and service types in Python
- Implement action servers and clients for goal-oriented tasks
- Integrate external Python libraries with ROS 2 nodes
- Use advanced data structures and algorithms in ROS 2 contexts
- Implement advanced debugging and profiling techniques

## Custom Message and Service Types

### Creating Custom Messages

Custom messages allow you to define complex data structures specific to your application. Let's create a custom message for robot status:

**msg/RobotStatus.msg**:
```
# Custom message for robot status information
string robot_name
float32 battery_level
bool is_charging
int32[] diagnostic_codes
geometry_msgs/Pose pose
builtin_interfaces/Time last_update
```

### Creating Custom Services

Custom services allow you to define specific request/response patterns:

**srv/NavigationCommand.srv**:
```
# Request
float32 target_x
float32 target_y
float32 target_theta

---
# Response
bool success
string message
builtin_interfaces/Time completion_time
```

### Using Custom Messages in Python

First, make sure your `package.xml` includes the necessary dependencies:

```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>builtin_interfaces</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

And update your `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/NavigationCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
```

### Example Node Using Custom Messages

```python
import rclpy
from rclpy.node import Node
from my_robot_pkg.msg import RobotStatus  # Your custom message
from my_robot_pkg.srv import NavigationCommand  # Your custom service
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time


class AdvancedRobotNode(Node):
    def __init__(self):
        super().__init__('advanced_robot_node')
        
        # Publisher for custom message
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        
        # Service server for custom service
        self.nav_service = self.create_service(
            NavigationCommand, 
            'navigation_command', 
            self.navigation_command_callback
        )
        
        # Timer to publish status periodically
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Simulated robot state
        self.robot_battery = 85.0
        self.robot_pose = Pose()
        self.robot_pose.position.x = 0.0
        self.robot_pose.position.y = 0.0
        self.robot_pose.position.z = 0.0
        self.robot_pose.orientation.w = 1.0  # No rotation

    def publish_status(self):
        """Publish robot status using custom message"""
        msg = RobotStatus()
        msg.robot_name = 'AdvancedRobot'
        msg.battery_level = self.robot_battery
        msg.is_charging = False
        msg.diagnostic_codes = [0, 0, 0, 1]  # Example: some diagnostics
        msg.pose = self.robot_pose
        msg.last_update = self.get_clock().now().to_msg()
        
        self.status_pub.publish(msg)
        self.get_logger().info(f'Published status: Battery {msg.battery_level}%')

    def navigation_command_callback(self, request, response):
        """Handle navigation command requests"""
        self.get_logger().info(
            f'Received navigation command: ({request.target_x}, {request.target_y}, {request.target_theta})'
        )
        
        # Simulate navigation execution
        # In a real implementation, this would call navigation system
        try:
            # Update robot pose (simulation)
            self.robot_pose.position.x = request.target_x
            self.robot_pose.position.y = request.target_y
            
            # Decrease battery based on distance traveled
            distance = ((request.target_x ** 2 + request.target_y ** 2) ** 0.5)
            battery_decrease = distance * 0.5  # 0.5% per meter
            self.robot_battery = max(0.0, self.robot_battery - battery_decrease)
            
            response.success = True
            response.message = f'Navigated to ({request.target_x}, {request.target_y})'
            response.completion_time = self.get_clock().now().to_msg()
            
        except Exception as e:
            response.success = False
            response.message = f'Navigation failed: {str(e)}'
            response.completion_time = self.get_clock().now().to_msg()
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions in ROS 2

Actions are used for goal-oriented tasks that take time to complete and can be canceled. They provide feedback during execution.

### Creating Custom Actions

**action/MoveRobot.action**:
```
# Goal
float32 target_x
float32 target_y
float32 target_theta

---
# Result
bool success
string message
float32 final_x
float32 final_y

---
# Feedback
float32 current_x
float32 current_y
float32 distance_remaining
string status
```

### Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from my_robot_pkg.action import MoveRobot

from geometry_msgs.msg import Twist
import time


class MoveRobotActionServer(Node):
    def __init__(self):
        super().__init__('move_robot_action_server')
        
        # Create action server
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self._goal_handle = None
        self._cancel_requested = False

    def goal_callback(self, goal_request):
        """Accept or reject a goal"""
        self.get_logger().info(f'Received goal request: ({goal_request.target_x}, {goal_request.target_y})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info('Executing goal...')
        
        # Store goal handle
        self._goal_handle = goal_handle
        self._cancel_requested = False
        
        # Get goal parameters
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        
        # Calculate distance to target
        distance = ((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2) ** 0.5
        
        # Movement parameters
        speed = 0.2  # m/s
        dt = 0.1  # seconds
        step_size = speed * dt
        
        feedback_msg = MoveRobot.Feedback()
        
        try:
            # Move toward target
            while not self._cancel_requested:
                # Calculate direction vector
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dist_to_target = (dx ** 2 + dy ** 2) ** 0.5
                
                # Check if we've reached the target
                if dist_to_target < 0.1:  # Within 10cm tolerance
                    break
                
                # Normalize direction and scale to step size
                if dist_to_target > 0:
                    dx_norm = dx / dist_to_target
                    dy_norm = dy / dist_to_target
                else:
                    dx_norm = 0
                    dy_norm = 0
                
                # Update position
                self.current_x += dx_norm * step_size
                self.current_y += dy_norm * step_size
                
                # Publish velocity command
                cmd_msg = Twist()
                cmd_msg.linear.x = speed
                cmd_msg.angular.z = 0.0  # Simplified: no rotation for now
                self.cmd_vel_pub.publish(cmd_msg)
                
                # Publish feedback
                feedback_msg.current_x = self.current_x
                feedback_msg.current_y = self.current_y
                feedback_msg.distance_remaining = dist_to_target
                feedback_msg.status = f'Moving: {dist_to_target:.2f}m remaining'
                
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(feedback_msg.status)
                
                # Sleep for dt seconds
                time.sleep(dt)
        
        except Exception as e:
            self.get_logger().error(f'Error during action execution: {str(e)}')
            goal_handle.abort()
            
            result = MoveRobot.Result()
            result.success = False
            result.message = f'Action failed: {str(e)}'
            result.final_x = self.current_x
            result.final_y = self.current_y
            return result
        
        # Check if goal was canceled
        if self._cancel_requested:
            goal_handle.canceled()
            result = MoveRobot.Result()
            result.success = False
            result.message = 'Goal canceled'
            result.final_x = self.current_x
            result.final_y = self.current_y
        else:
            # Goal completed successfully
            goal_handle.succeed()
            result = MoveRobot.Result()
            result.success = True
            result.message = f'Reached target ({target_x}, {target_y})'
            result.final_x = self.current_x
            result.final_y = self.current_y
        
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = MoveRobotActionServer()
    
    # Use a multi-threaded executor to handle callbacks properly
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_pkg.action import MoveRobot


class MoveRobotActionClient(Node):
    def __init__(self):
        super().__init__('move_robot_action_client')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self, x, y, theta):
        """Send a goal to the action server"""
        goal_msg = MoveRobot.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_theta = theta

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: ({x}, {y}, {theta})')

        # Send the goal and get a future
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callbacks for goal response
        send_goal_future.add_done_callback(self.goal_response_callback)

        return send_goal_future

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result future
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        self.get_logger().info(f'Final position: ({result.final_x}, {result.final_y})')

    def feedback_callback(self, feedback_msg):
        """Handle feedback during execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.status}'
        )


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveRobotActionClient()
    
    # Send a goal
    action_client.send_goal(1.0, 1.0, 0.0)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Integration with External Python Libraries

### 1. NumPy and Scientific Computing

```python
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        
        # Subscriber for point cloud data
        self.pc_sub = self.create_subscription(
            PointCloud2,
            'pointcloud_raw',
            self.pc_callback,
            10
        )
        
        # Publisher for processed data
        self.pc_pub = self.create_publisher(
            PointCloud2,
            'pointcloud_filtered',
            10
        )
        
    def pc_callback(self, msg):
        """Process point cloud data using NumPy"""
        # Convert PointCloud2 message to NumPy array
        points = self.pointcloud2_to_array(msg)
        
        # Apply filtering using NumPy
        filtered_points = self.filter_ground_plane(points)
        
        # Publish filtered data
        filtered_msg = self.array_to_pointcloud2(filtered_points, msg.header)
        self.pc_pub.publish(filtered_msg)

    def pointcloud2_to_array(self, msg):
        """Convert PointCloud2 message to NumPy array"""
        # Get the size of each field in bytes
        field_sizes = {
            PointField.INT8: 1,
            PointField.UINT8: 1,
            PointField.INT16: 2,
            PointField.UINT16: 2,
            PointField.INT32: 4,
            PointField.UINT32: 4,
            PointField.FLOAT32: 4,
            PointField.FLOAT64: 8
        }
        
        # Calculate the offset of each field
        field_offsets = {}
        offset = 0
        for field in msg.fields:
            field_offsets[field.name] = offset
            offset += field_sizes[field.datatype]
        
        # Unpack the data
        points = []
        for i in range(msg.width * msg.height):
            start_idx = i * msg.point_step
            x = struct.unpack('f', msg.data[start_idx + field_offsets['x']: start_idx + field_offsets['x'] + 4])[0]
            y = struct.unpack('f', msg.data[start_idx + field_offsets['y']: start_idx + field_offsets['y'] + 4])[0]
            z = struct.unpack('f', msg.data[start_idx + field_offsets['z']: start_idx + field_offsets['z'] + 4])[0]
            points.append([x, y, z])
        
        return np.array(points)

    def filter_ground_plane(self, points):
        """Filter out ground plane using RANSAC algorithm"""
        # Simple ground plane filter - in practice you'd use RANSAC
        # For now, just remove points with negative Z values
        return points[points[:, 2] > 0.1]  # Only keep points above 10cm

    def array_to_pointcloud2(self, points, header):
        """Convert NumPy array back to PointCloud2"""
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        
        # Set up fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes each
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        
        # Pack data
        data = b''
        for point in points:
            data += struct.pack('fff', point[0], point[1], point[2])
        msg.data = data
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    processor = PointCloudProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. TensorFlow/PyTorch Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Import your ML framework (only if available)
try:
    import tensorflow as tf
    ML_AVAILABLE = True
except ImportError:
    ML_AVAILABLE = False
    print("TensorFlow not available, ML features will be disabled")


class MLVisionNode(Node):
    def __init__(self):
        super().__init__('ml_vision_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(
            String,  # You would create a custom message for detection results
            'detection_results',
            10
        )
        
        # Initialize ML model if available
        if ML_AVAILABLE:
            self.load_model()
        else:
            self.model = None
            self.get_logger().warn('ML framework not available, using mock processing')

    def load_model(self):
        """Load the ML model"""
        try:
            # Load a pre-trained model
            self.model = tf.keras.applications.MobileNetV2(
                weights='imagenet',
                include_top=True
            )
            self.get_logger().info('ML model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            self.model = None

    def image_callback(self, msg):
        """Process image with ML model"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Preprocess image for model
            if self.model is not None:
                # Resize image to model input size
                input_image = cv2.resize(cv_image, (224, 224))
                input_image = tf.keras.applications.mobilenet_v2.preprocess_input(input_image)
                input_image = np.expand_dims(input_image, axis=0)
                
                # Run inference
                predictions = self.model.predict(input_image)
                
                # Get top prediction
                top_prediction = tf.keras.applications.imagenet_utils.decode_predictions(
                    predictions, top=1
                )[0][0]
                
                class_name = top_prediction[1]
                confidence = float(top_prediction[2])
                
                # Publish results
                result_msg = String()
                result_msg.data = f'{class_name}: {confidence:.2f}'
                self.detection_pub.publish(result_msg)
                
                self.get_logger().info(f'Detected: {class_name} ({confidence:.2f})')
            else:
                # Mock processing if ML not available
                result_msg = String()
                result_msg.data = 'Mock detection: object, 0.85'
                self.detection_pub.publish(result_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = MLVisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Data Structures and Algorithms

### 1. Efficient Data Handling

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from collections import deque
import numpy as np


class EfficientDataProcessor(Node):
    def __init__(self):
        super().__init__('efficient_data_processor')
        
        # Use deque for efficient data buffering
        self.scan_buffer = deque(maxlen=10)  # Keep last 10 scans
        
        # Subscribe to laser scans
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Timer for processing buffered data
        self.process_timer = self.create_timer(0.5, self.process_buffered_data)

    def scan_callback(self, msg):
        """Add scan to buffer"""
        # Convert to more efficient format if needed
        processed_scan = {
            'header': msg.header,
            'ranges': np.array(msg.ranges, dtype=np.float32),
            'intensities': np.array(msg.intensities, dtype=np.float32) if len(msg.intensities) > 0 else None
        }
        
        self.scan_buffer.append(processed_scan)

    def process_buffered_data(self):
        """Process buffered scan data"""
        if len(self.scan_buffer) == 0:
            return
            
        # Example: Calculate average of last few scans to reduce noise
        if len(self.scan_buffer) >= 3:
            # Get the last 3 scans
            recent_scans = list(self.scan_buffer)[-3:]
            
            # Calculate average ranges (excluding invalid values)
            all_ranges = [scan['ranges'] for scan in recent_scans]
            stacked_ranges = np.stack(all_ranges)
            
            # Replace inf values with a large number for averaging
            masked_ranges = np.where(stacked_ranges == float('inf'), 100.0, stacked_ranges)
            
            # Calculate mean, but keep original inf values where all were inf
            avg_ranges = np.mean(masked_ranges, axis=0)
            # Reset values to inf where original data had inf for all scans
            all_inf_mask = np.all(stacked_ranges == float('inf'), axis=0)
            avg_ranges[all_inf_mask] = float('inf')
            
            self.get_logger().info(f'Processed {len(recent_scans)} scans, avg range shape: {avg_ranges.shape}')
```

### 2. State Machines

```python
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String


class RobotState(Enum):
    IDLE = 'idle'
    NAVIGATING = 'navigating'
    MANIPULATING = 'manipulating'
    EMERGENCY_STOP = 'emergency_stop'


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        # Initialize state
        self.current_state = RobotState.IDLE
        
        # Subscriber for commands
        self.command_sub = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )
        
        # Publisher for state updates
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        
        # Timer for state-specific behavior
        self.state_timer = self.create_timer(0.1, self.state_behavior)
        
        self.get_logger().info(f'Initial state: {self.current_state.value}')

    def command_callback(self, msg):
        """Handle incoming commands that may change state"""
        command = msg.data.lower()
        
        # State transitions based on commands
        if command == 'start_navigation' and self.current_state == RobotState.IDLE:
            self.set_state(RobotState.NAVIGATING)
        elif command == 'stop_navigation' and self.current_state == RobotState.NAVIGATING:
            self.set_state(RobotState.IDLE)
        elif command == 'start_manipulation' and self.current_state == RobotState.IDLE:
            self.set_state(RobotState.MANIPULATING)
        elif command == 'stop_manipulation' and self.current_state == RobotState.MANIPULATING:
            self.set_state(RobotState.IDLE)
        elif command == 'emergency_stop':
            self.set_state(RobotState.EMERGENCY_STOP)
        elif command == 'clear_emergency' and self.current_state == RobotState.EMERGENCY_STOP:
            self.set_state(RobotState.IDLE)

    def set_state(self, new_state):
        """Safely transition to a new state"""
        old_state = self.current_state
        self.current_state = new_state
        
        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')
        
        # Perform state entry actions
        self.on_state_enter(new_state, old_state)
        
        # Publish state change
        state_msg = String()
        state_msg.data = new_state.value
        self.state_pub.publish(state_msg)

    def on_state_enter(self, new_state, old_state):
        """Handle actions when entering a new state"""
        if new_state == RobotState.NAVIGATING:
            self.start_navigation()
        elif new_state == RobotState.MANIPULATING:
            self.start_manipulation()
        elif new_state == RobotState.IDLE:
            self.stop_all_actions()
        elif new_state == RobotState.EMERGENCY_STOP:
            self.emergency_stop_actions()

    def state_behavior(self):
        """Execute behavior based on current state"""
        if self.current_state == RobotState.NAVIGATING:
            self.navigating_behavior()
        elif self.current_state == RobotState.MANIPULATING:
            self.manipulation_behavior()
        elif self.current_state == RobotState.EMERGENCY_STOP:
            self.emergency_behavior()

    def start_navigation(self):
        """Initialize navigation-specific actions"""
        self.get_logger().info('Starting navigation system')
        # Publish navigation command, etc.

    def start_manipulation(self):
        """Initialize manipulation-specific actions"""
        self.get_logger().info('Starting manipulation system')
        # Send commands to manipulator, etc.

    def stop_all_actions(self):
        """Stop all robot actions"""
        self.get_logger().info('Stopping all robot actions')
        # Send stop commands to all systems

    def emergency_stop_actions(self):
        """Execute emergency stop procedures"""
        self.get_logger().warn('EMERGENCY STOP - All systems halted')
        # Send emergency stop to all subsystems

    def navigating_behavior(self):
        """Behavior executed while in NAVIGATING state"""
        # Check for obstacles, update path, etc.
        self.get_logger().debug('Executing navigation behavior')

    def manipulation_behavior(self):
        """Behavior executed while in MANIPULATING state"""
        # Update manipulator position, check for completion, etc.
        self.get_logger().debug('Executing manipulation behavior')

    def emergency_behavior(self):
        """Behavior executed while in EMERGENCY_STOP state"""
        # Monitor for clear command, keep systems stopped
        self.get_logger().warn('Robot in emergency stop state')


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Debugging and Profiling Techniques

### 1. Advanced Logging

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import psutil
import os


class DebuggingNode(Node):
    def __init__(self):
        super().__init__('debugging_node')
        
        # Set custom logging configuration
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Create QoS profiles for different types of data
        self.reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publisher with custom QoS
        self.debug_pub = self.create_publisher(String, 'debug_info', self.reliable_qos)
        
        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.system_monitor)
        
        self.start_time = time.time()
        self.message_count = 0

    def system_monitor(self):
        """Monitor system resources and performance"""
        # Get system information
        process = psutil.Process(os.getpid())
        cpu_percent = psutil.cpu_percent()
        memory_info = process.memory_info()
        memory_mb = memory_info.rss / 1024 / 1024  # Convert to MB
        uptime = time.time() - self.start_time
        
        # Log system info
        self.get_logger().info(
            f'CPU: {cpu_percent:.1f}%, Memory: {memory_mb:.1f}MB, '
            f'Uptime: {uptime:.1f}s, Messages: {self.message_count}'
        )
        
        # Publish debug information
        debug_msg = String()
        debug_msg.data = f'CPU:{cpu_percent:.1f}%,MEM:{memory_mb:.1f}MB,UPTIME:{uptime:.1f}s'
        self.debug_pub.publish(debug_msg)
        
        self.message_count += 1
        
        # Add performance warnings
        if memory_mb > 100:  # 100MB threshold
            self.get_logger().warn(f'High memory usage: {memory_mb:.1f}MB')
        if cpu_percent > 80:  # 80% CPU threshold
            self.get_logger().warn(f'High CPU usage: {cpu_percent:.1f}%' %)


def main(args=None):
    rclpy.init(args=args)
    node = DebuggingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Enable detailed logging from command line if needed
    import sys
    if '--debug' in sys.argv:
        rclpy.logging.set_logger_level('debugging_node', rclpy.logging.LoggingSeverity.DEBUG)
    
    main()
```

### 2. Performance Profiling

```python
import rclpy
from rclpy.node import Node
import cProfile
import pstats
from io import StringIO
import functools
import time


def profile_function(func):
    """Decorator to profile function execution"""
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        
        exec_time = (end_time - start_time) * 1000  # Convert to milliseconds
        args[0].get_logger().debug(f'{func.__name__} executed in {exec_time:.2f}ms')
        
        return result
    return wrapper


class ProfilingNode(Node):
    def __init__(self):
        super().__init__('profiling_node')
        
        # Initialize profiler
        self.profiler = cProfile.Profile()
        
        # Timer to trigger profiling
        self.profile_timer = self.create_timer(5.0, self.run_profiled_function)
        
        self.iteration = 0

    @profile_function
    def run_profiled_function(self):
        """Function to be profiled"""
        self.get_logger().info(f'Running profiled function iteration {self.iteration}')
        
        # Simulate some work
        data = []
        for i in range(1000):
            data.append(i ** 2)
        
        # Process the data
        result = sum(data) / len(data)
        
        self.iteration += 1
        
        if self.iteration == 1:  # Profile first iteration
            self.profiler.enable()
        elif self.iteration == 6:  # Disable profiler after 5 iterations
            self.profiler.disable()
            self.print_profile_stats()

    def print_profile_stats(self):
        """Print profiling statistics"""
        s = StringIO()
        ps = pstats.Stats(self.profiler, stream=s)
        ps.sort_stats('cumulative')
        ps.print_stats(10)  # Print top 10 functions
        
        profile_output = s.getvalue()
        self.get_logger().info('Profiling Results:\n' + profile_output)


def main(args=None):
    rclpy.init(args=args)
    node = ProfilingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Lesson Summary

In this lesson, you've learned advanced Python patterns for ROS 2:
- Creating and using custom message and service types
- Implementing action servers and clients for complex tasks
- Integrating external Python libraries like NumPy, TensorFlow
- Using advanced data structures and algorithms in ROS 2
- Implementing state machines for complex robot behaviors
- Advanced debugging and profiling techniques

## Exercises

1. Create a custom message for robot arm joint states and implement a node that publishes joint positions.
2. Build an action server for a robot arm that moves to specific joint configurations.
3. Integrate OpenCV with a ROS 2 node to perform basic image processing.
4. Implement a finite state machine for a robot that performs different actions based on its state.

## Next Steps

In the final lesson of this chapter, we'll explore performance optimization and best practices for deploying Python-based ROS 2 applications in production environments.