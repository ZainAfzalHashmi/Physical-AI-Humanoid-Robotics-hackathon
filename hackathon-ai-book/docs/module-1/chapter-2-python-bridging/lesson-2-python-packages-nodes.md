# Lesson 2: Creating Python Packages and Nodes

## Learning Objectives
By the end of this lesson, you will be able to:
- Create ROS 2 Python packages following best practices
- Structure Python code for modular and reusable nodes
- Implement complex node architectures with multiple components
- Use parameter and configuration files effectively
- Package and distribute Python-based ROS 2 packages

## Creating a Python Package for ROS 2

### Using the Command Line Tool

ROS 2 provides a convenient command to create Python packages:

```bash
# In your workspace's src directory
ros2 pkg create --build-type ament_python my_robot_pkg
```

This creates a basic Python package structure with the following files:
- `setup.py` - Package configuration for Python
- `setup.cfg` - Installation configuration
- `my_robot_pkg/__init__.py` - Python package file
- `package.xml` - Package metadata
- `my_robot_pkg/my_robot_pkg/` - Main package directory

### Package Structure

A well-structured Python package for ROS 2 typically has this layout:

```
my_robot_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── my_robot_pkg/
│   ├── __init__.py
│   ├── robot_controller.py
│   ├── sensor_processor.py
│   └── utils/
│       ├── __init__.py
│       └── helpers.py
├── launch/
│   └── robot_launch.py
├── config/
│   └── params.yaml
├── test/
│   └── test_my_robot_pkg.py
└── README.md
```

## Detailed Package Configuration

### 1. package.xml Configuration

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>Package for controlling my robot</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 2. setup.py Configuration

The `setup.py` file configures how your Python package is built and installed:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for controlling my robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_pkg.robot_controller:main',
            'sensor_processor = my_robot_pkg.sensor_processor:main',
        ],
    },
)
```

## Creating Complex Node Architectures

### 1. Modular Node Design

Let's create a more complex node that demonstrates modular design:

**robot_controller.py**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from my_robot_pkg.utils.helpers import MovementHelper, SafetyChecker


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize helper components
        self.movement_helper = MovementHelper(self)
        self.safety_checker = SafetyChecker(self)
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 0.5),
                ('max_angular_speed', 1.0),
                ('safety_distance', 0.5),
            ]
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )
        
        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot Controller initialized')

    def laser_callback(self, msg):
        """Handle laser scan data for obstacle detection"""
        self.safety_checker.update_scan_data(msg)
        
    def command_callback(self, msg):
        """Handle high-level commands"""
        self.get_logger().info(f'Received command: {msg.data}')
        # Process command and update internal state
        self.process_command(msg.data)

    def process_command(self, command_str):
        """Process high-level commands"""
        if command_str == 'forward':
            self.movement_helper.set_target_movement(0.5, 0.0)  # linear, angular
        elif command_str == 'turn_left':
            self.movement_helper.set_target_movement(0.0, 0.5)
        elif command_str == 'stop':
            self.movement_helper.set_target_movement(0.0, 0.0)

    def control_loop(self):
        """Main control loop executed at regular intervals"""
        # Check safety conditions
        if self.safety_checker.is_safe_to_move():
            # Get target movement
            linear_vel, angular_vel = self.movement_helper.get_current_movement()
            
            # Apply parameter limits
            max_linear = self.get_parameter('max_linear_speed').value
            max_angular = self.get_parameter('max_angular_speed').value
            
            linear_vel = max(min(linear_vel, max_linear), -max_linear)
            angular_vel = max(min(angular_vel, max_angular), -max_angular)
            
            # Create and publish velocity command
            cmd_msg = Twist()
            cmd_msg.linear.x = linear_vel
            cmd_msg.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd_msg)
        else:
            # Stop if not safe
            cmd_msg = Twist()
            self.cmd_vel_pub.publish(cmd_msg)
            self.get_logger().warn('Movement stopped for safety')


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Helper Components

**utils/helpers.py**:
```python
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MovementHelper:
    """Helper class for movement-related functionality"""
    
    def __init__(self, node: Node):
        self.node = node
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.max_acceleration = 0.5  # m/s^2
        self.max_deceleration = 1.0  # m/s^2
        
    def set_target_movement(self, linear: float, angular: float):
        """Set target movement values"""
        self.target_linear = linear
        self.target_angular = angular
        
    def get_current_movement(self):
        """Get current movement with smooth interpolation"""
        # Simplified smoothing algorithm
        dt = 0.1  # Assuming 10Hz control loop
        
        # Linear acceleration
        if self.current_linear < self.target_linear:
            self.current_linear = min(
                self.current_linear + self.max_acceleration * dt,
                self.target_linear
            )
        elif self.current_linear > self.target_linear:
            self.current_linear = max(
                self.current_linear - self.max_deceleration * dt,
                self.target_linear
            )
            
        # Angular acceleration
        if self.current_angular < self.target_angular:
            self.current_angular = min(
                self.current_angular + self.max_acceleration * dt,
                self.target_angular
            )
        elif self.current_angular > self.target_angular:
            self.current_angular = max(
                self.current_angular - self.max_deceleration * dt,
                self.target_angular
            )
                
        return self.current_linear, self.current_angular


class SafetyChecker:
    """Helper class for safety-related checks"""
    
    def __init__(self, node: Node):
        self.node = node
        self.laser_data = None
        self.safe_distance = node.get_parameter('safety_distance').value \
            if node.has_parameter('safety_distance') else 0.5
        
    def update_scan_data(self, scan_msg: LaserScan):
        """Update laser scan data for safety checks"""
        self.laser_data = scan_msg
        
    def is_safe_to_move(self):
        """Check if it's safe to move forward"""
        if self.laser_data is None:
            self.node.get_logger().warn('No laser data available')
            return False
            
        # Check distances in front of the robot (simplified)
        middle_idx = len(self.laser_data.ranges) // 2
        front_distances = self.laser_data.ranges[middle_idx-10:middle_idx+10]
        
        # Filter out invalid readings
        valid_distances = [d for d in front_distances if d != float('inf') and not np.isnan(d)]
        
        if not valid_distances:
            self.node.get_logger().warn('No valid distance readings')
            return False
            
        min_distance = min(valid_distances)
        
        if min_distance < self.safe_distance:
            self.node.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m (safe distance: {self.safe_distance:.2f}m)')
            return False
            
        return True
```

### 3. Launch Files

**launch/robot_launch.py**:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Configuration files
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value='/path/to/my_robot_pkg/config/params.yaml',
        description='Path to parameters file'
    )
    
    return LaunchDescription([
        use_sim_time,
        params_file,
        
        # Robot controller node
        Node(
            package='my_robot_pkg',
            executable='robot_controller',
            name='robot_controller',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            output='screen'
        ),
        
        # Sensor processor node (if needed)
        Node(
            package='my_robot_pkg',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )
    ])
```

### 4. Parameter Configuration

**config/params.yaml**:
```yaml
/**:  # Applies to all nodes
  ros__parameters:
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    safety_distance: 0.5
    control_frequency: 10.0

robot_controller:  # Applies only to robot_controller node
  ros__parameters:
    max_linear_speed: 0.7
    obstacle_threshold: 0.6

sensor_processor:  # Applies only to sensor_processor node
  ros__parameters:
    scan_frequency: 10.0
    filter_window_size: 5
```

## Advanced Python Patterns in ROS 2

### 1. Class-Based Node Architecture

```python
import rclpy
from rclpy.node import Node
from threading import Lock


class AdvancedRobotNode(Node):
    """Advanced node with internal state management"""
    
    def __init__(self):
        super().__init__('advanced_robot_node')
        
        # Internal state with thread safety
        self._state_lock = Lock()
        self._robot_state = {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],  # quaternion
            'velocity': [0.0, 0.0, 0.0],
            'status': 'idle'
        }
        
        # Create publishers and subscribers
        self.position_pub = self.create_publisher(
            geometry_msgs.msg.Point, 
            'robot_position', 
            10
        )
        
        # Timer for state publishing
        self.publish_timer = self.create_timer(0.1, self.publish_state)
        
    def update_position(self, x, y, z):
        """Thread-safe position update"""
        with self._state_lock:
            self._robot_state['position'] = [x, y, z]
            
    def get_position(self):
        """Thread-safe position retrieval"""
        with self._state_lock:
            return self._robot_state['position'].copy()
            
    def publish_state(self):
        """Publish current robot state"""
        pos = self.get_position()
        msg = geometry_msgs.msg.Point()
        msg.x = pos[0]
        msg.y = pos[1]
        msg.z = pos[2]
        self.position_pub.publish(msg)
```

### 2. Asynchronous Programming with rclpy

```python
import rclpy
from rclpy.node import Node
import asyncio
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future


class AsyncRobotNode(Node):
    """Node demonstrating async programming patterns"""
    
    def __init__(self):
        super().__init__('async_robot_node')
        
        # Create clients for services
        self.navigation_client = self.create_client(
            NavigateToPose, 
            'navigate_to_pose'
        )
        
    async def async_navigate_to_pose(self, x, y, theta):
        """Asynchronously navigate to a pose"""
        # Wait for service to be available
        while not self.navigation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Navigation service not available, waiting...')
            
        # Create request
        request = NavigateToPose.Request()
        request.pose = self.create_pose(x, y, theta)
        
        # Call service asynchronously
        future = self.navigation_client.call_async(request)
        
        # Wait for response
        await future
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Navigation result: {response}')
            return response
        else:
            self.get_logger().error('Navigation service call failed')
            return None

    def create_pose(self, x, y, theta):
        """Create a PoseStamped message"""
        from geometry_msgs.msg import PoseStamped
        from builtin_interfaces.msg import Time
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import math
        from tf_transformations import quaternion_from_euler
        
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = AsyncRobotNode()
    
    async def run():
        # Perform async operations
        await node.async_navigate_to_pose(1.0, 2.0, 0.0)
        
    # Note: This is a simplified example; proper async integration
    # requires more complex executor management
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Testing Python Nodes

### 1. Unit Tests

**test/test_robot_controller.py**:
```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from my_robot_pkg.robot_controller import RobotController


class TestRobotController(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        
    def setUp(self):
        self.node = RobotController()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
    def tearDown(self):
        self.node.destroy_node()
        
    def test_node_initialization(self):
        """Test that node initializes correctly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'robot_controller')
        
    def test_parameter_declaration(self):
        """Test that required parameters are declared"""
        parameters = self.node.get_parameters(['max_linear_speed', 'max_angular_speed', 'safety_distance'])
        for param in parameters:
            self.assertIsNotNone(param)


if __name__ == '__main__':
    unittest.main()
```

## Packaging and Distribution

### 1. Creating a Debian Package

To distribute your Python package, you can create a Debian package:

```bash
# Build the package
colcon build --packages-select my_robot_pkg

# Create Debian package (requires additional configuration)
# This requires creating a CPack configuration
```

### 2. PyPI Distribution (for standalone Python modules)

For standalone Python modules that can be used outside of ROS 2:

```python
# setup.py for PyPI distribution
from setuptools import setup, find_packages

setup(
    name='my_robot_utils',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        # Regular Python dependencies, not ROS-specific
    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='Robot utilities package',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/your-repo',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)
```

## Best Practices for Python Package Development

### 1. Code Organization
- Use meaningful package and module names
- Group related functionality into modules
- Use consistent naming conventions
- Document your code with docstrings

### 2. Error Handling
- Implement proper exception handling
- Use ROS logging appropriately 
- Handle parameter and configuration errors gracefully

### 3. Performance Considerations
- Minimize blocking operations in callbacks
- Use appropriate QoS settings
- Consider memory usage in long-running nodes

### 4. Testing
- Write comprehensive unit tests
- Test parameter changes during runtime
- Test node lifecycle management
- Use mock objects for external dependencies

## Lesson Summary

In this lesson, you've learned how to create well-structured Python packages for ROS 2:
- Package creation using command-line tools
- Proper configuration with setup.py and package.xml
- Modular node architecture with helper classes
- Launch file and parameter configuration
- Testing strategies for Python nodes
- Best practices for Python development in ROS 2

## Exercises

1. Create a Python package for a simple sensor fusion node that combines data from multiple sensors.
2. Implement a parameterized controller node with multiple operational modes.
3. Create a launch file that starts multiple nodes with different configurations.
4. Write unit tests for a simple publisher/subscriber pair.

## Next Steps

In the next lesson, we'll explore advanced Python patterns and integration techniques in ROS 2, including working with custom message types and advanced communication patterns.