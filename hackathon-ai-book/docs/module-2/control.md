---
sidebar_label: 'Control'
sidebar_position: 3
---

# Control

## Overview
This chapter covers control theory applied to robotics, including feedback control and system stability for humanoid robots. Control is the final component of the perception-planning-control loop that enables humanoid robots to execute desired behaviors and maintain stability.

## Learning Objectives
- Understand fundamental control theory concepts
- Implement PID controllers for robot systems
- Design feedback control systems for stability
- Apply advanced control techniques to humanoid robots
- Integrate control with perception and planning systems

## Content Structure

### Section 2.1: Control Theory Fundamentals
- Open-loop vs closed-loop control
- Transfer functions and system modeling
- Stability analysis and Routh-Hurwitz criterion
- Time and frequency domain analysis

### Section 2.2: PID Control for Robotics
- Proportional, integral, and derivative terms
- PID tuning methods (Ziegler-Nichols, trial-and-error)
- Implementation considerations for robotic systems
- Cascade control structures

### Section 2.3: Advanced Control Techniques
- State-space representation
- Linear quadratic regulators (LQR)
- Model predictive control (MPC)
- Adaptive and robust control

### Section 2.4: Control for Humanoid Stability
- Center of mass control
- Zero moment point (ZMP) control
- Balance control algorithms
- Walking pattern generation

### Section 2.5: Control System Integration
- Sensor feedback integration
- Actuator command generation
- Safety considerations and limits
- Performance optimization

## Code Examples

### Example 2.1: PID Controller Implementation
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Create subscriber for desired position
        self.desired_subscription = self.create_subscription(
            Float64,
            '/desired_position',
            self.desired_callback,
            10
        )

        # Create subscriber for current position
        self.current_subscription = self.create_subscription(
            Float64,
            '/current_position',
            self.current_callback,
            10
        )

        # Create publisher for control output
        self.control_publisher = self.create_publisher(
            Float64,
            '/control_output',
            10
        )

        # PID parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain

        # PID internal variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # Desired and current values
        self.desired_position = 0.0
        self.current_position = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.02, self.control_callback)  # 50 Hz

    def desired_callback(self, msg):
        """Handle desired position updates"""
        self.desired_position = msg.data

    def current_callback(self, msg):
        """Handle current position updates"""
        self.current_position = msg.data

    def control_callback(self):
        """Main PID control loop"""
        # Calculate error
        error = self.desired_position - self.current_position

        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        # Update last time
        self.last_time = current_time

        # Avoid division by zero
        if dt <= 0:
            return

        # Calculate derivative
        derivative = (error - self.previous_error) / dt

        # Calculate integral (with anti-windup)
        self.integral += error * dt
        # Limit integral to prevent windup
        integral_limit = 10.0
        self.integral = max(-integral_limit, min(integral_limit, self.integral))

        # Calculate PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Update previous error
        self.previous_error = error

        # Publish control output
        output_msg = Float64()
        output_msg.data = float(output)
        self.control_publisher.publish(output_msg)

        self.get_logger().debug(f'PID: e={error:.3f}, out={output:.3f}')

def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode()

    try:
        rclpy.spin(pid_controller_node)
    except KeyboardInterrupt:
        pid_controller_node.get_logger().info('Interrupted, shutting down')
    finally:
        pid_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.2: Joint Position Controller for Robot Arm
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class JointPositionControllerNode(Node):
    def __init__(self):
        super().__init__('joint_position_controller_node')

        # Create subscriber for desired joint positions
        self.desired_subscription = self.create_subscription(
            Float64MultiArray,
            '/desired_joint_positions',
            self.desired_callback,
            10
        )

        # Create subscriber for current joint states
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )

        # PID parameters for each joint
        self.kp = [10.0, 10.0, 8.0, 8.0, 5.0, 5.0]  # Proportional gains
        self.ki = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]    # Integral gains
        self.kd = [0.5, 0.5, 0.5, 0.5, 0.2, 0.2]    # Derivative gains

        # PID internal variables for each joint
        self.previous_errors = [0.0] * 6
        self.integrals = [0.0] * 6

        # Joint names and desired/current positions
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.desired_positions = [0.0] * 6
        self.current_positions = [0.0] * 6
        self.current_velocities = [0.0] * 6

        # Control timer
        self.control_timer = self.create_timer(0.02, self.control_callback)  # 50 Hz

    def desired_callback(self, msg):
        """Handle desired joint position updates"""
        if len(msg.data) == len(self.desired_positions):
            self.desired_positions = list(msg.data)
        else:
            self.get_logger().warn(f'Expected {len(self.desired_positions)} joint positions, got {len(msg.data)}')

    def joint_state_callback(self, msg):
        """Handle current joint state updates"""
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.current_velocities[i] = msg.velocity[idx]
            except ValueError:
                # Joint name not found in message
                pass

    def control_callback(self):
        """Main joint position control loop"""
        # Calculate control outputs for each joint
        control_outputs = []

        for i in range(len(self.joint_names)):
            # Calculate error
            error = self.desired_positions[i] - self.current_positions[i]

            # Calculate derivative (using velocity feedback if available)
            derivative = -self.current_velocities[i]  # Negative because we want position error derivative

            # Calculate integral (with anti-windup)
            self.integrals[i] += error * 0.02  # dt = 0.02s
            # Limit integral to prevent windup
            integral_limit = 5.0
            self.integrals[i] = max(-integral_limit, min(integral_limit, self.integrals[i]))

            # Calculate PID output
            output = (self.kp[i] * error) + (self.ki[i] * self.integrals[i]) + (self.kd[i] * derivative)

            # Update previous error
            self.previous_errors[i] = error

            control_outputs.append(output)

        # Publish joint commands
        command_msg = Float64MultiArray()
        command_msg.data = [float(out) for out in control_outputs]
        self.joint_command_publisher.publish(command_msg)

        self.get_logger().debug(f'Joint control outputs: {control_outputs}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller_node = JointPositionControllerNode()

    try:
        rclpy.spin(joint_controller_node)
    except KeyboardInterrupt:
        joint_controller_node.get_logger().info('Interrupted, shutting down')
    finally:
        joint_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.3: Balance Controller for Humanoid Robot
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Float64
import numpy as np

class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__('balance_controller_node')

        # Create subscriber for IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for center of pressure adjustment
        self.cop_publisher = self.create_publisher(
            Point,
            '/center_of_pressure_adjustment',
            10
        )

        # Create publisher for ankle torque commands
        self.ankle_torque_publisher = self.create_publisher(
            Vector3,
            '/ankle_torque_commands',
            10
        )

        # Balance control parameters
        self.kp_roll = 15.0   # Proportional gain for roll control
        self.ki_roll = 1.0    # Integral gain for roll control
        self.kd_roll = 2.0    # Derivative gain for roll control

        self.kp_pitch = 12.0  # Proportional gain for pitch control
        self.ki_pitch = 0.8   # Integral gain for pitch control
        self.kd_pitch = 1.5   # Derivative gain for pitch control

        # Internal variables for PID controllers
        self.roll_error_integral = 0.0
        self.roll_error_previous = 0.0

        self.pitch_error_integral = 0.0
        self.pitch_error_previous = 0.0

        # Robot parameters
        self.robot_height = 0.8  # Height of the robot's center of mass (m)
        self.gravity = 9.81      # Gravity constant (m/s^2)

        # Control timer
        self.control_timer = self.create_timer(0.01, self.balance_control_callback)  # 100 Hz

        # Store latest IMU data
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_roll_velocity = 0.0
        self.latest_pitch_velocity = 0.0

        self.get_logger().info('Balance controller initialized')

    def imu_callback(self, msg):
        """Handle IMU data"""
        # Extract roll and pitch from quaternion
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        # Convert quaternion to roll, pitch, yaw
        # Note: This is a simplified conversion that works for small angles
        self.latest_roll = np.arctan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
        self.latest_pitch = np.arcsin(2.0 * (w * y - z * x))

        # Extract angular velocities
        self.latest_roll_velocity = msg.angular_velocity.x
        self.latest_pitch_velocity = msg.angular_velocity.y

    def balance_control_callback(self):
        """Main balance control loop"""
        # Desired angles (should be zero for upright position)
        desired_roll = 0.0
        desired_pitch = 0.0

        # Calculate errors
        roll_error = desired_roll - self.latest_roll
        pitch_error = desired_pitch - self.latest_pitch

        # Roll PID controller
        self.roll_error_integral += roll_error * 0.01  # dt = 0.01s
        roll_error_derivative = (roll_error - self.roll_error_previous) / 0.01

        # Limit integral to prevent windup
        integral_limit = 0.5
        self.roll_error_integral = max(-integral_limit, min(integral_limit, self.roll_error_integral))

        roll_control_output = (self.kp_roll * roll_error +
                              self.ki_roll * self.roll_error_integral +
                              self.kd_roll * roll_error_derivative)

        self.roll_error_previous = roll_error

        # Pitch PID controller
        self.pitch_error_integral += pitch_error * 0.01  # dt = 0.01s
        pitch_error_derivative = (pitch_error - self.pitch_error_previous) / 0.01

        # Limit integral to prevent windup
        self.pitch_error_integral = max(-integral_limit, min(integral_limit, self.pitch_error_integral))

        pitch_control_output = (self.kp_pitch * pitch_error +
                               self.ki_pitch * self.pitch_error_integral +
                               self.kd_pitch * pitch_error_derivative)

        self.pitch_error_previous = pitch_error

        # Publish ankle torque commands
        torque_msg = Vector3()
        torque_msg.x = float(pitch_control_output)  # Pitch torque (around x-axis)
        torque_msg.y = float(-roll_control_output)  # Roll torque (around y-axis) - negative for correct direction
        torque_msg.z = 0.0  # No yaw control in this simple example
        self.ankle_torque_publisher.publish(torque_msg)

        # Publish center of pressure adjustment (simplified)
        cop_msg = Point()
        # Adjust CoP based on lean direction (simplified model)
        cop_msg.x = -pitch_control_output * 0.01  # Proportional to pitch correction
        cop_msg.y = roll_control_output * 0.01    # Proportional to roll correction
        cop_msg.z = 0.0
        self.cop_publisher.publish(cop_msg)

        self.get_logger().debug(f'Balance: roll_err={roll_error:.3f}, pitch_err={pitch_error:.3f}, '
                               f'torques=({torque_msg.x:.3f}, {torque_msg.y:.3f})')

def main(args=None):
    rclpy.init(args=args)
    balance_controller_node = BalanceControllerNode()

    try:
        rclpy.spin(balance_controller_node)
    except KeyboardInterrupt:
        balance_controller_node.get_logger().info('Interrupted, shutting down')
    finally:
        balance_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 2.1: Implement a Cascade PID Controller
Create a controller that uses two PID controllers in cascade - one for position control and one for velocity control.

### Exercise 2.2: Implement a LQR Controller
Implement a Linear Quadratic Regulator controller for a simple robotic system and compare its performance with PID.

### Exercise 2.3: Create a Walking Pattern Generator
Implement a controller that generates stable walking patterns for a humanoid robot using ZMP-based control.

## Assessment Criteria
- Students can implement PID controllers for robotic systems
- Students understand advanced control techniques and can apply them to robotics
- Students can design controllers for humanoid stability and balance
- Students can integrate control systems with perception and planning
- Students understand safety considerations in control system design