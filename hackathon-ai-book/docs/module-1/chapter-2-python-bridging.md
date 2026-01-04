---
sidebar_label: 'Chapter 2: Python Bridging - Connecting Agents to ROS Controllers'
sidebar_position: 3
---

# Chapter 2: Python Bridging - Connecting Agents to ROS Controllers

## Overview
This chapter focuses on creating bridges between high-level Python agents and ROS 2 controllers using rclpy. Readers will learn how to integrate AI decision-making components with low-level robot control systems, which is essential for practical humanoid robotics applications.

## Learning Objectives
- Understand the rclpy library and its role in Python-ROS integration
- Implement a functional bridge between Python agents and ROS controllers
- Use both topic-based and service-based communication patterns in a single application
- Design Python agents that can control ROS systems effectively
- Handle errors and edge cases in the communication bridge

## Content Structure

### Section 2.1: Introduction to rclpy
- Understanding the rclpy architecture
- How rclpy connects Python to the ROS 2 execution model
- Setting up rclpy in your Python environment
- Key differences between rclpy and rospy

### Section 2.2: Python Agent Architecture
- Designing Python agents for robotics applications
- State management in Python agents
- Decision-making patterns for robot control
- Integrating sensor data processing

### Section 2.3: Topic-Based Control Implementation
- Using rclpy publishers to send commands to ROS controllers
- Using rclpy subscribers to receive sensor data from ROS systems
- Managing asynchronous communication in Python agents
- Quality of Service (QoS) considerations for control applications

### Section 2.4: Service-Based Control Implementation
- Using rclpy service clients to request specific actions
- Using rclpy service servers to provide agent capabilities
- Managing synchronous communication in real-time systems
- Error handling in service-based interactions

### Section 2.5: Combining Topic and Service Patterns
- Architectural patterns for hybrid communication
- When to use topics vs services in agent design
- Managing multiple communication channels
- Performance considerations and best practices

## Code Examples

### Example 2.1: Basic Python Agent with rclpy Integration
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from example_interfaces.srv import SetBool

class PythonRobotAgent(Node):
    def __init__(self):
        super().__init__('python_robot_agent')

        # Publishers for sending commands to ROS controllers
        self.joint_cmd_publisher = self.create_publisher(
            Float64, '/joint_position_controller/commands', 10
        )

        # Subscribers for receiving sensor data
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Service client for requesting specific actions
        self.calibrate_service_client = self.create_client(
            SetBool, '/calibrate_robot'
        )

        # Service server for providing agent capabilities
        self.execute_plan_service = self.create_service(
            SetBool, 'execute_robot_plan', self.execute_plan_callback
        )

        # Internal state
        self.current_joint_positions = {}
        self.is_calibrated = False

        # Timer for decision-making loop
        self.decision_timer = self.create_timer(0.1, self.decision_callback)

    def joint_state_callback(self, msg):
        """Process joint state messages from ROS"""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]
        self.get_logger().info(f'Updated joint positions: {self.current_joint_positions}')

    def decision_callback(self):
        """Main decision-making loop"""
        if not self.is_calibrated:
            self.calibrate_robot()
        else:
            # Implement agent logic here
            self.execute_basic_movement()

    def calibrate_robot(self):
        """Calibrate the robot using service call"""
        if not self.calibrate_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Calibration service not available')
            return

        request = SetBool.Request()
        request.data = True
        future = self.calibrate_service_client.call_async(request)
        future.add_done_callback(self.calibration_response_callback)

    def calibration_response_callback(self, future):
        """Handle calibration service response"""
        try:
            response = future.result()
            if response.success:
                self.is_calibrated = True
                self.get_logger().info('Robot calibrated successfully')
            else:
                self.get_logger().error(f'Calibration failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Calibration service call failed: {e}')

    def execute_basic_movement(self):
        """Execute a basic movement pattern"""
        # Send a command to move a joint
        cmd_msg = Float64()
        cmd_msg.data = 1.5  # radians
        self.joint_cmd_publisher.publish(cmd_msg)

    def execute_plan_callback(self, request, response):
        """Handle external request to execute a plan"""
        try:
            if request.data:
                self.get_logger().info('External request to execute plan received')
                # Execute the plan logic
                # This could involve complex decision making
                response.success = True
                response.message = 'Plan execution started'
            else:
                response.success = False
                response.message = 'Plan execution cancelled'
        except Exception as e:
            response.success = False
            response.message = f'Error executing plan: {str(e)}'

        return response

def main(args=None):
    rclpy.init(args=args)
    agent = PythonRobotAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted, shutting down')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.2: Advanced Python Agent with State Management
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from example_interfaces.srv import Trigger
import time
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    CALIBRATING = 2
    MOVING = 3
    ERROR = 4

class AdvancedPythonAgent(Node):
    def __init__(self):
        super().__init__('advanced_python_agent')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(Float64, '/joint_position_controller/command', 10)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.feedback_subscriber = self.create_subscription(
            String, '/robot_feedback', self.feedback_callback, 10
        )

        # Services
        self.emergency_stop_client = self.create_client(Trigger, '/emergency_stop')
        self.reset_system_client = self.create_client(Trigger, '/reset_system')

        # Internal state
        self.current_state = RobotState.IDLE
        self.joint_positions = {}
        self.last_feedback = ""
        self.start_time = time.time()

        # Timer for state machine
        self.state_machine_timer = self.create_timer(0.05, self.state_machine_callback)

    def joint_state_callback(self, msg):
        """Update internal joint position tracking"""
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]

    def feedback_callback(self, msg):
        """Handle feedback from the robot"""
        self.last_feedback = msg.data
        self.get_logger().info(f'Robot feedback: {msg.data}')

    def state_machine_callback(self):
        """Main state machine for the agent"""
        if self.current_state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.current_state == RobotState.CALIBRATING:
            self.handle_calibration_state()
        elif self.current_state == RobotState.MOVING:
            self.handle_moving_state()
        elif self.current_state == RobotState.ERROR:
            self.handle_error_state()

    def handle_idle_state(self):
        """Handle the idle state"""
        # Check if we should start calibration
        if time.time() - self.start_time > 2.0:  # Wait 2 seconds before starting
            self.current_state = RobotState.CALIBRATING
            self.get_logger().info('Transitioning to calibration state')

    def handle_calibration_state(self):
        """Handle the calibration state"""
        # In a real implementation, you might call a calibration service here
        self.get_logger().info('Calibration in progress...')
        # After some time, move to moving state
        if time.time() - self.start_time > 5.0:  # Calibration takes 5 seconds
            self.current_state = RobotState.MOVING
            self.get_logger().info('Calibration complete, transitioning to moving state')

    def handle_moving_state(self):
        """Handle the moving state"""
        # Send movement commands
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        cmd.angular.z = 0.2  # Slight turn
        self.cmd_vel_publisher.publish(cmd)

        # Check if we should stop based on feedback
        if 'obstacle' in self.last_feedback.lower():
            self.current_state = RobotState.IDLE
            self.get_logger().info('Obstacle detected, stopping')

    def handle_error_state(self):
        """Handle the error state"""
        # Try to reset the system
        if self.reset_system_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            self.reset_system_client.call_async(request)
            self.current_state = RobotState.IDLE
        else:
            self.get_logger().error('Reset service not available, staying in error state')

def main(args=None):
    rclpy.init(args=args)
    agent = AdvancedPythonAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted, shutting down')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.3: Bridge Pattern with Error Handling
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from example_interfaces.srv import SetBool
import threading
import time

class RobustBridge(Node):
    def __init__(self):
        super().__init__('robust_bridge')

        # Bridge configuration
        self.agent_commands = []
        self.ros_feedback = {}
        self.bridge_active = True

        # Publishers and subscribers for ROS side
        self.ros_command_publisher = self.create_publisher(
            Float64MultiArray, '/robot_commands', 10
        )
        self.ros_feedback_subscriber = self.create_subscription(
            JointState, '/joint_states', self.ros_feedback_callback, 10
        )

        # Service for bridge control
        self.bridge_control_service = self.create_service(
            SetBool, 'bridge_control', self.bridge_control_callback
        )

        # Timer for bridge operations
        self.bridge_timer = self.create_timer(0.02, self.bridge_callback)

        # Threading for Python agent integration
        self.agent_thread = threading.Thread(target=self.python_agent_loop)
        self.agent_thread.daemon = True
        self.agent_thread.start()

    def ros_feedback_callback(self, msg):
        """Handle feedback from ROS system"""
        for i, name in enumerate(msg.name):
            self.ros_feedback[name] = {
                'position': msg.position[i],
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0
            }

    def bridge_callback(self):
        """Main bridge operation loop"""
        if not self.bridge_active:
            return

        # Process any commands from Python agent
        if self.agent_commands:
            cmd = self.agent_commands.pop(0)
            self.publish_to_ros(cmd)

    def python_agent_loop(self):
        """Simulated Python agent that sends commands to the bridge"""
        while rclpy.ok() and self.bridge_active:
            try:
                # Simulate agent decision making
                command = self.make_decision()
                if command is not None:
                    self.agent_commands.append(command)
                time.sleep(0.05)  # Agent decision frequency
            except Exception as e:
                self.get_logger().error(f'Error in Python agent loop: {e}')
                time.sleep(0.1)

    def make_decision(self):
        """Simulated decision making function"""
        # This would be replaced with actual AI logic
        import random
        if random.random() > 0.8:  # 20% chance of sending a command
            cmd = Float64MultiArray()
            cmd.data = [random.uniform(-1.0, 1.0) for _ in range(6)]  # 6 joint commands
            return cmd
        return None

    def publish_to_ros(self, command):
        """Publish command to ROS system"""
        try:
            self.ros_command_publisher.publish(command)
            self.get_logger().info(f'Published command: {command.data}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish command: {e}')

    def bridge_control_callback(self, request, response):
        """Handle bridge enable/disable requests"""
        try:
            self.bridge_active = request.data
            response.success = True
            response.message = f'Bridge {"enabled" if self.bridge_active else "disabled"}'
            self.get_logger().info(f'Bridge control: {response.message}')
        except Exception as e:
            response.success = False
            response.message = f'Error controlling bridge: {str(e)}'

        return response

def main(args=None):
    rclpy.init(args=args)
    bridge = RobustBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('Interrupted, shutting down')
    finally:
        bridge.bridge_active = False  # Stop the agent thread
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 2.1: Create a Simple Agent
Create a Python agent that monitors joint positions and sends corrective commands when positions deviate from desired values.

### Exercise 2.2: Implement a State Machine Agent
Create an agent with a state machine that transitions between different behaviors based on sensor feedback.

### Exercise 2.3: Create a Bridge with Error Recovery
Implement a bridge that can detect communication failures and attempt to recover automatically.

## Assessment Criteria
- Students can create Python agents that communicate with ROS systems using rclpy
- Students can implement both topic and service communication patterns in a single application
- Students understand when to use different communication patterns
- Students can handle errors and edge cases in the Python-ROS bridge
- Students can design agents with appropriate state management