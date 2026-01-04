---
sidebar_label: 'Balance'
sidebar_position: 3
---

# Balance

## Overview
This chapter covers maintaining stability for humanoid robots, including balance control and fall prevention strategies. Balance is fundamental to humanoid robotics as it enables robots to maintain an upright posture while performing tasks and moving through the environment.

## Learning Objectives
- Understand principles of humanoid balance and stability
- Implement balance control algorithms
- Design controllers for static and dynamic balance
- Apply Zero Moment Point (ZMP) and Capture Point concepts
- Integrate balance with locomotion and manipulation

## Content Structure

### Section 3.1: Principles of Humanoid Balance
- Center of mass and base of support
- Static vs dynamic balance
- Stability margins and balance metrics
- Human balance strategies as inspiration

### Section 3.2: Balance Control Algorithms
- Inverted pendulum models
- Linear inverted pendulum model (LIPM)
- Capture point theory
- Feedback control for balance

### Section 3.3: ZMP-Based Balance Control
- Zero moment point concept and calculation
- ZMP reference generation
- ZMP feedback control
- Stability criteria

### Section 3.4: Balance Strategies
- Ankle strategy
- Hip strategy
- Stepping strategy
- Upper body compensation

### Section 3.5: Advanced Balance Techniques
- Disturbance detection and rejection
- Fall prevention and recovery
- Multi-contact balance
- Balance during manipulation

## Code Examples

### Example 3.1: Inverted Pendulum Balance Controller
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Float64
import numpy as np

class InvertedPendulumBalanceNode(Node):
    def __init__(self):
        super().__init__('inverted_pendulum_balance_node')

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

        # Create publisher for CoM adjustment commands
        self.com_publisher = self.create_publisher(
            Point,
            '/com_adjustment',
            10
        )

        # Balance control parameters
        self.pendulum_height = 0.8  # Height of the pendulum (CoM) in meters
        self.gravity = 9.81         # Gravity constant (m/s^2)

        # Control gains for inverted pendulum
        self.kp = 15.0  # Proportional gain
        self.kd = 5.0   # Derivative gain

        # Internal variables for balance control
        self.roll_error_integral = 0.0
        self.roll_error_previous = 0.0
        self.pitch_error_integral = 0.0
        self.pitch_error_previous = 0.0

        # Robot state
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_roll_velocity = 0.0
        self.current_pitch_velocity = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.01, self.balance_control_callback)  # 100 Hz

        self.get_logger().info('Inverted pendulum balance controller initialized')

    def imu_callback(self, msg):
        """Handle IMU data"""
        # Extract roll and pitch from quaternion
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        # Convert quaternion to roll, pitch, yaw
        self.current_roll = np.arctan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
        self.current_pitch = np.arcsin(2.0 * (w * y - z * x))

        # Extract angular velocities
        self.current_roll_velocity = msg.angular_velocity.x
        self.current_pitch_velocity = msg.angular_velocity.y

    def balance_control_callback(self):
        """Main balance control loop using inverted pendulum model"""
        # Desired angles (should be zero for upright position)
        desired_roll = 0.0
        desired_pitch = 0.0

        # Calculate errors
        roll_error = desired_roll - self.current_roll
        pitch_error = desired_pitch - self.current_pitch

        # Calculate desired CoP based on inverted pendulum model
        # For a linear inverted pendulum: CoP = CoM - (h/g) * CoM_ddot
        # Where h is the pendulum height and g is gravity
        # We'll use a simplified approach with PD control

        # Roll control
        roll_control_output = (self.kp * roll_error -
                              self.kd * self.current_roll_velocity)

        # Pitch control
        pitch_control_output = (self.kp * pitch_error -
                               self.kd * self.current_pitch_velocity)

        # Calculate CoP adjustment (simplified model)
        cop_msg = Point()
        cop_msg.x = float(-pitch_control_output * 0.02)  # Proportional to pitch correction
        cop_msg.y = float(roll_control_output * 0.02)    # Proportional to roll correction
        cop_msg.z = 0.0
        self.cop_publisher.publish(cop_msg)

        # Calculate CoM adjustment (for visualization or higher-level control)
        com_msg = Point()
        com_msg.x = float(-pitch_control_output * 0.005)  # Smaller adjustment for CoM
        com_msg.y = float(roll_control_output * 0.005)
        com_msg.z = 0.0
        self.com_publisher.publish(com_msg)

        self.get_logger().debug(f'Balance: roll={self.current_roll:.3f}, pitch={self.current_pitch:.3f}, '
                               f'CoP=({cop_msg.x:.3f}, {cop_msg.y:.3f})')

def main(args=None):
    rclpy.init(args=args)
    balance_controller = InvertedPendulumBalanceNode()

    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        balance_controller.get_logger().info('Interrupted, shutting down')
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3.2: ZMP-Based Balance Controller
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

class ZMPBalanceControllerNode(Node):
    def __init__(self):
        super().__init__('zmp_balance_controller_node')

        # Create subscriber for ZMP measurement
        self.zmp_subscription = self.create_subscription(
            Point,
            '/measured_zmp',
            self.zmp_callback,
            10
        )

        # Create subscriber for desired ZMP
        self.desired_zmp_subscription = self.create_subscription(
            Point,
            '/desired_zmp',
            self.desired_zmp_callback,
            10
        )

        # Create publisher for foot placement adjustment
        self.foot_adjustment_publisher = self.create_publisher(
            Twist,
            '/foot_adjustment',
            10
        )

        # Create publisher for CoM adjustment
        self.com_adjustment_publisher = self.create_publisher(
            Point,
            '/com_adjustment',
            10
        )

        # Robot parameters
        self.com_height = 0.8  # Center of mass height (m)
        self.gravity = 9.81    # Gravity (m/s^2)

        # ZMP control parameters
        self.zmp_kp = 20.0  # Proportional gain for ZMP control
        self.zmp_ki = 2.0   # Integral gain for ZMP control
        self.zmp_kd = 5.0   # Derivative gain for ZMP control

        # Internal variables
        self.measured_zmp = Point(x=0.0, y=0.0, z=0.0)
        self.desired_zmp = Point(x=0.0, y=0.0, z=0.0)
        self.zmp_error_integral_x = 0.0
        self.zmp_error_integral_y = 0.0
        self.previous_zmp_error_x = 0.0
        self.previous_zmp_error_y = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.02, self.zmp_control_callback)  # 50 Hz

        self.get_logger().info('ZMP balance controller initialized')

    def zmp_callback(self, msg):
        """Handle measured ZMP updates"""
        self.measured_zmp = msg

    def desired_zmp_callback(self, msg):
        """Handle desired ZMP updates"""
        self.desired_zmp = msg

    def zmp_control_callback(self):
        """Main ZMP control loop"""
        # Calculate ZMP errors
        zmp_error_x = self.desired_zmp.x - self.measured_zmp.x
        zmp_error_y = self.desired_zmp.y - self.measured_zmp.y

        # Update integral terms with anti-windup
        self.zmp_error_integral_x += zmp_error_x * 0.02  # dt = 0.02s
        self.zmp_error_integral_y += zmp_error_y * 0.02

        # Limit integral to prevent windup
        integral_limit = 0.1
        self.zmp_error_integral_x = max(-integral_limit, min(integral_limit, self.zmp_error_integral_x))
        self.zmp_error_integral_y = max(-integral_limit, min(integral_limit, self.zmp_error_integral_y))

        # Calculate derivative terms
        zmp_error_derivative_x = (zmp_error_x - self.previous_zmp_error_x) / 0.02
        zmp_error_derivative_y = (zmp_error_y - self.previous_zmp_error_y) / 0.02

        # Update previous errors
        self.previous_zmp_error_x = zmp_error_x
        self.previous_zmp_error_y = zmp_error_y

        # Calculate control outputs
        control_output_x = (self.zmp_kp * zmp_error_x +
                           self.zmp_ki * self.zmp_error_integral_x +
                           self.zmp_kd * zmp_error_derivative_x)

        control_output_y = (self.zmp_kp * zmp_error_y +
                           self.zmp_ki * self.zmp_error_integral_y +
                           self.zmp_kd * zmp_error_derivative_y)

        # Publish foot adjustment commands
        foot_adjustment_msg = Twist()
        foot_adjustment_msg.linear.x = float(control_output_x * 0.01)  # Scale appropriately
        foot_adjustment_msg.linear.y = float(control_output_y * 0.01)
        foot_adjustment_msg.linear.z = 0.0
        foot_adjustment_msg.angular.x = 0.0
        foot_adjustment_msg.angular.y = 0.0
        foot_adjustment_msg.angular.z = 0.0
        self.foot_adjustment_publisher.publish(foot_adjustment_msg)

        # Publish CoM adjustment commands
        com_adjustment_msg = Point()
        com_adjustment_msg.x = float(control_output_x * 0.005)  # Smaller adjustment for CoM
        com_adjustment_msg.y = float(control_output_y * 0.005)
        com_adjustment_msg.z = 0.0
        self.com_adjustment_publisher.publish(com_adjustment_msg)

        self.get_logger().debug(f'ZMP: measured=({self.measured_zmp.x:.3f}, {self.measured_zmp.y:.3f}), '
                               f'desired=({self.desired_zmp.x:.3f}, {self.desired_zmp.y:.3f}), '
                               f'control=({control_output_x:.3f}, {control_output_y:.3f})')

def main(args=None):
    rclpy.init(args=args)
    zmp_balance_controller = ZMPBalanceControllerNode()

    try:
        rclpy.spin(zmp_balance_controller)
    except KeyboardInterrupt:
        zmp_balance_controller.get_logger().info('Interrupted, shutting down')
    finally:
        zmp_balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3.3: Capture Point Balance Controller
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
import numpy as np

class CapturePointBalanceNode(Node):
    def __init__(self):
        super().__init__('capture_point_balance_node')

        # Create subscriber for IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for step adjustment commands
        self.step_adjustment_publisher = self.create_publisher(
            Twist,
            '/step_adjustment',
            10
        )

        # Create publisher for CoM adjustment
        self.com_adjustment_publisher = self.create_publisher(
            Point,
            '/com_adjustment',
            10
        )

        # Robot parameters
        self.com_height = 0.8  # Center of mass height (m)
        self.gravity = 9.81    # Gravity (m/s^2)

        # Capture point control parameters
        self.cp_kp = 10.0  # Proportional gain for capture point control
        self.cp_kd = 3.0   # Derivative gain for capture point control

        # Internal variables
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_roll_velocity = 0.0
        self.current_pitch_velocity = 0.0
        self.previous_capture_point_x = 0.0
        self.previous_capture_point_y = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.02, self.capture_point_control_callback)  # 50 Hz

        self.get_logger().info('Capture point balance controller initialized')

    def imu_callback(self, msg):
        """Handle IMU data"""
        # Extract roll and pitch from quaternion
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        # Convert quaternion to roll, pitch, yaw
        self.current_roll = np.arctan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
        self.current_pitch = np.arcsin(2.0 * (w * y - z * x))

        # Extract angular velocities
        self.current_roll_velocity = msg.angular_velocity.x
        self.current_pitch_velocity = msg.angular_velocity.y

    def capture_point_control_callback(self):
        """Main capture point control loop"""
        # Calculate current CoM velocity (approximated from angular velocity and CoM height)
        com_vel_x = self.com_height * self.current_pitch_velocity
        com_vel_y = self.com_height * self.current_roll_velocity

        # Calculate capture point (where to step to stop the robot)
        # Capture point = CoM position + (CoM velocity / sqrt(g / height))
        # For small angles, CoM position can be approximated from angle * height
        cp_x = self.com_height * self.current_pitch + com_vel_x / np.sqrt(self.gravity / self.com_height)
        cp_y = self.com_height * self.current_roll + com_vel_y / np.sqrt(self.gravity / self.com_height)

        # Calculate desired capture point (should be within the support polygon)
        # For simplicity, we want the capture point to be at the center of the support polygon
        desired_cp_x = 0.0
        desired_cp_y = 0.0

        # Calculate capture point errors
        cp_error_x = desired_cp_x - cp_x
        cp_error_y = desired_cp_y - cp_y

        # Calculate capture point velocity
        dt = 0.02
        cp_vel_x = (cp_x - self.previous_capture_point_x) / dt
        cp_vel_y = (cp_y - self.previous_capture_point_y) / dt

        # Update previous capture point
        self.previous_capture_point_x = cp_x
        self.previous_capture_point_y = cp_y

        # Calculate control outputs using PD control on capture point
        control_output_x = self.cp_kp * cp_error_x + self.cp_kd * (0 - cp_vel_x)
        control_output_y = self.cp_kp * cp_error_y + self.cp_kd * (0 - cp_vel_y)

        # Determine if we need to step (if capture point is outside support polygon)
        support_polygon_radius = 0.1  # Approximate radius of support polygon (m)
        should_step = np.sqrt(cp_x**2 + cp_y**2) > support_polygon_radius

        # Publish step adjustment commands
        step_adjustment_msg = Twist()
        if should_step:
            # If we need to step, command a step toward the capture point
            step_adjustment_msg.linear.x = float(cp_x * 0.5)  # Scale appropriately
            step_adjustment_msg.linear.y = float(cp_y * 0.5)
            step_adjustment_msg.linear.z = 0.0  # Not used in this context
        else:
            # If balance is OK, just make small adjustments
            step_adjustment_msg.linear.x = float(control_output_x * 0.01)
            step_adjustment_msg.linear.y = float(control_output_y * 0.01)

        step_adjustment_msg.angular.x = 0.0
        step_adjustment_msg.angular.y = 0.0
        step_adjustment_msg.angular.z = 0.0
        self.step_adjustment_publisher.publish(step_adjustment_msg)

        # Publish CoM adjustment commands
        com_adjustment_msg = Point()
        com_adjustment_msg.x = float(control_output_x * 0.005)  # Smaller adjustment for CoM
        com_adjustment_msg.y = float(control_output_y * 0.005)
        com_adjustment_msg.z = 0.0
        self.com_adjustment_publisher.publish(com_adjustment_msg)

        self.get_logger().debug(f'Capture Point: pos=({cp_x:.3f}, {cp_y:.3f}), '
                               f'error=({cp_error_x:.3f}, {cp_error_y:.3f}), '
                               f'step_needed={should_step}')

def main(args=None):
    rclpy.init(args=args)
    capture_point_balance = CapturePointBalanceNode()

    try:
        rclpy.spin(capture_point_balance)
    except KeyboardInterrupt:
        capture_point_balance.get_logger().info('Interrupted, shutting down')
    finally:
        capture_point_balance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 3.1: Implement a Multi-Strategy Balance Controller
Create a controller that switches between ankle, hip, and stepping strategies based on the magnitude of perturbation.

### Exercise 3.2: Create a Balance Controller with Disturbance Detection
Implement a balance controller that can detect external disturbances and adjust its control strategy accordingly.

### Exercise 3.3: Implement Balance During Manipulation
Extend the balance controller to maintain stability while the robot is performing manipulation tasks with its arms.

## Assessment Criteria
- Students can implement balance control algorithms using inverted pendulum models
- Students understand ZMP and capture point concepts and can implement controllers based on them
- Students can design controllers for static and dynamic balance
- Students can integrate balance with locomotion and manipulation systems
- Students understand safety considerations in balance control