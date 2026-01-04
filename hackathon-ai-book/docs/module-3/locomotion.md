---
sidebar_label: 'Locomotion'
sidebar_position: 1
---

# Locomotion

## Overview
This chapter covers walking and movement patterns for humanoid robots, including gait generation and locomotion control. Locomotion is a complex challenge in humanoid robotics that requires coordination of multiple systems including perception, planning, control, and mechanical design.

## Learning Objectives
- Understand fundamental principles of humanoid locomotion
- Implement basic walking patterns and gaits
- Design controllers for stable locomotion
- Generate dynamic walking patterns
- Integrate locomotion with balance and planning systems

## Content Structure

### Section 3.1: Principles of Humanoid Locomotion
- Differences between wheeled and legged locomotion
- Static vs dynamic walking
- Zero moment point (ZMP) and capture point concepts
- Energy efficiency in locomotion

### Section 3.2: Gait Generation
- Walk cycle phases (stance, swing, double support)
- Trajectory planning for feet
- Joint angle generation for walking
- Adaptive gait patterns

### Section 3.3: Walking Controllers
- Inverse kinematics for leg control
- Central pattern generators (CPGs)
- Feedback control for walking stability
- Disturbance rejection techniques

### Section 3.4: Dynamic Locomotion
- Running and other dynamic movements
- Control strategies for dynamic locomotion
- Transition between different gaits
- Handling uneven terrain

### Section 3.5: Locomotion Integration
- Coordination with balance control
- Integration with perception for terrain adaptation
- Planning for locomotion in complex environments
- Safety considerations in locomotion

## Code Examples

### Example 3.1: Simple Inverse Kinematics for Leg Control
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class LegIKControllerNode(Node):
    def __init__(self):
        super().__init__('leg_ik_controller_node')

        # Create subscriber for desired foot position
        self.foot_position_subscription = self.create_subscription(
            Point,
            '/desired_foot_position',
            self.foot_position_callback,
            10
        )

        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/leg_controller/commands',
            10
        )

        # Robot leg parameters (simplified 3-DOF leg: hip, knee, ankle)
        self.upper_leg_length = 0.35  # meters
        self.lower_leg_length = 0.35  # meters

        # Current desired position
        self.desired_foot_position = Point(x=0.0, y=0.0, z=-0.7)  # Default standing position

    def foot_position_callback(self, msg):
        """Handle desired foot position updates"""
        self.desired_foot_position = msg

    def calculate_ik(self, x, y, z):
        """
        Calculate inverse kinematics for a 3-DOF leg.
        x: forward/backward
        y: left/right
        z: up/down
        """
        # Calculate hip joint angle (yaw) based on lateral position
        hip_yaw = np.arctan2(y, x)

        # Project the desired position onto the sagittal plane
        r = np.sqrt(x**2 + y**2)

        # Calculate the distance from hip to foot in the sagittal plane
        d = np.sqrt(r**2 + z**2)

        # Check if the desired position is reachable
        if d > (self.upper_leg_length + self.lower_leg_length):
            # Position is out of reach, extend leg fully forward
            hip_pitch = 0.0
            knee_pitch = 0.0
            ankle_pitch = 0.0
            return hip_yaw, hip_pitch, knee_pitch, ankle_pitch

        # Calculate knee angle using law of cosines
        cos_knee = (self.upper_leg_length**2 + self.lower_leg_length**2 - d**2) / \
                   (2 * self.upper_leg_length * self.lower_leg_length)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)  # Clamp to valid range
        knee_angle = np.pi - np.arccos(cos_knee)

        # Calculate hip angle
        cos_hip = (self.upper_leg_length**2 + d**2 - self.lower_leg_length**2) / \
                  (2 * self.upper_leg_length * d)
        cos_hip = np.clip(cos_hip, -1.0, 1.0)  # Clamp to valid range
        alpha = np.arccos(cos_hip)
        beta = np.arctan2(-z, r)  # Angle from hip to foot
        hip_angle = alpha + beta

        # Calculate ankle angle to maintain foot orientation
        ankle_angle = -(hip_angle + (np.pi - knee_angle))

        return hip_yaw, hip_angle, knee_angle, ankle_angle

    def publish_joint_commands(self):
        """Calculate and publish joint commands based on desired foot position"""
        # Calculate inverse kinematics
        hip_yaw, hip_pitch, knee_pitch, ankle_pitch = self.calculate_ik(
            self.desired_foot_position.x,
            self.desired_foot_position.y,
            self.desired_foot_position.z
        )

        # Create and publish joint command message
        command_msg = Float64MultiArray()
        command_msg.data = [
            float(hip_yaw),    # Hip yaw
            float(hip_pitch),  # Hip pitch
            float(knee_pitch), # Knee pitch
            float(ankle_pitch) # Ankle pitch
        ]

        self.joint_command_publisher.publish(command_msg)

        self.get_logger().debug(f'Foot pos: ({self.desired_foot_position.x:.2f}, '
                               f'{self.desired_foot_position.y:.2f}, '
                               f'{self.desired_foot_position.z:.2f}), '
                               f'Joints: ({hip_yaw:.2f}, {hip_pitch:.2f}, '
                               f'{knee_pitch:.2f}, {ankle_pitch:.2f})')

def main(args=None):
    rclpy.init(args=args)
    leg_ik_controller = LegIKControllerNode()

    # Create a timer to periodically publish joint commands
    timer = leg_ik_controller.create_timer(0.02, leg_ik_controller.publish_joint_commands)  # 50 Hz

    try:
        rclpy.spin(leg_ik_controller)
    except KeyboardInterrupt:
        leg_ik_controller.get_logger().info('Interrupted, shutting down')
    finally:
        leg_ik_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3.2: Central Pattern Generator for Walking
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class CPGWalkingControllerNode(Node):
    def __init__(self):
        super().__init__('cpg_walking_controller_node')

        # Create subscriber for walking commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/walking_controller/commands',
            10
        )

        # CPG parameters
        self.step_frequency = 1.0  # Hz
        self.step_height = 0.05    # meters
        self.step_length = 0.2     # meters

        # Walking state variables
        self.left_leg_phase = 0.0
        self.right_leg_phase = np.pi  # Start 180 degrees out of phase
        self.is_walking = False
        self.walking_speed = 0.0
        self.turning_speed = 0.0

        # Control timer for CPG
        self.cpg_timer = self.create_timer(0.02, self.cpg_callback)  # 50 Hz

    def cmd_vel_callback(self, msg):
        """Handle walking command updates"""
        self.walking_speed = msg.linear.x
        self.turning_speed = msg.angular.z

        # Determine if we should be walking
        self.is_walking = abs(self.walking_speed) > 0.01 or abs(self.turning_speed) > 0.01

    def cpg_callback(self):
        """Central Pattern Generator callback"""
        if not self.is_walking:
            # Stop walking - return to neutral position
            neutral_msg = Float64MultiArray()
            neutral_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All joints to neutral
            self.joint_command_publisher.publish(neutral_msg)
            return

        # Update phases based on walking speed
        dt = 0.02  # Timer period
        phase_increment = 2 * np.pi * self.step_frequency * dt

        # Adjust phase increment based on walking speed
        adjusted_phase_increment = phase_increment * (1 + abs(self.walking_speed) * 0.5)

        self.left_leg_phase += adjusted_phase_increment
        self.right_leg_phase += adjusted_phase_increment

        # Keep phases within 0-2π range
        self.left_leg_phase = self.left_leg_phase % (2 * np.pi)
        self.right_leg_phase = self.right_leg_phase % (2 * np.pi)

        # Calculate joint angles based on CPG phases
        left_hip = self.walking_speed * 0.2 * np.sin(self.left_leg_phase)
        left_knee = self.step_height * np.sin(self.left_leg_phase * 2)  # Double frequency for knee
        left_ankle = -left_hip * 0.5  # Compensate for hip movement

        right_hip = self.walking_speed * 0.2 * np.sin(self.right_leg_phase)
        right_knee = self.step_height * np.sin(self.right_leg_phase * 2)  # Double frequency for knee
        right_ankle = -right_hip * 0.5  # Compensate for hip movement

        # For turning, add differential to legs
        if self.turning_speed > 0:  # Turn right
            right_hip += self.turning_speed * 0.1
            left_hip -= self.turning_speed * 0.1
        elif self.turning_speed < 0:  # Turn left
            right_hip -= abs(self.turning_speed) * 0.1
            left_hip += abs(self.turning_speed) * 0.1

        # Create and publish joint command message
        command_msg = Float64MultiArray()
        command_msg.data = [
            float(left_hip),    # Left hip
            float(left_knee),   # Left knee
            float(left_ankle),  # Left ankle
            float(right_hip),   # Right hip
            float(right_knee),  # Right knee
            float(right_ankle)  # Right ankle
        ]

        self.joint_command_publisher.publish(command_msg)

        self.get_logger().debug(f'Walking: speed={self.walking_speed:.2f}, '
                               f'phase=({self.left_leg_phase:.2f}, {self.right_leg_phase:.2f}), '
                               f'joints={command_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    cpg_walking_controller = CPGWalkingControllerNode()

    try:
        rclpy.spin(cpg_walking_controller)
    except KeyboardInterrupt:
        cpg_walking_controller.get_logger().info('Interrupted, shutting down')
    finally:
        cpg_walking_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3.3: ZMP-based Walking Pattern Generator
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class ZMPWalkingControllerNode(Node):
    def __init__(self):
        super().__init__('zmp_walking_controller_node')

        # Create subscriber for walking commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for footstep positions
        self.footstep_publisher = self.create_publisher(
            Point,
            '/desired_foot_position',
            10
        )

        # Create publisher for ZMP adjustment
        self.zmp_publisher = self.create_publisher(
            Point,
            '/zmp_reference',
            10
        )

        # Walking parameters
        self.step_width = 0.2    # Distance between feet (m)
        self.step_length = 0.3   # Step length (m)
        self.step_height = 0.05  # Clearance height (m)
        self.step_duration = 1.0 # Duration of each step (s)
        self.com_height = 0.8    # Center of mass height (m)

        # Walking state
        self.walking_speed = 0.0
        self.turning_speed = 0.0
        self.left_support = True  # Which foot is currently supporting
        self.step_phase = 0.0     # Phase of current step (0.0 to 1.0)
        self.foot_x = 0.0         # Current foot position
        self.foot_y = self.step_width / 2  # Current foot position
        self.com_x = 0.0          # Center of mass position
        self.com_y = 0.0

        # Control timer
        self.walk_timer = self.create_timer(0.02, self.walk_callback)  # 50 Hz

    def cmd_vel_callback(self, msg):
        """Handle walking command updates"""
        self.walking_speed = msg.linear.x
        self.turning_speed = msg.angular.z

    def walk_callback(self):
        """Main walking control loop"""
        # Update step phase
        dt = 0.02
        if abs(self.walking_speed) > 0.01 or abs(self.turning_speed) > 0.01:
            # Calculate phase increment based on speed
            phase_increment = dt / self.step_duration
            self.step_phase += phase_increment

            # If we've completed a step, switch support foot
            if self.step_phase >= 1.0:
                self.step_phase = 0.0
                self.left_support = not self.left_support

                # Update CoM position based on step
                self.com_x += self.step_length * np.cos(self.turning_speed) * self.walking_speed
                self.com_y += self.step_length * np.sin(self.turning_speed) * self.walking_speed

                # Update stance foot position
                if self.left_support:
                    self.foot_x = self.com_x - self.step_length / 2
                    self.foot_y = self.step_width / 2
                else:
                    self.foot_x = self.com_x - self.step_length / 2
                    self.foot_y = -self.step_width / 2
        else:
            # Stop walking, reset phase
            self.step_phase = 0.0

        # Calculate foot position based on step phase
        if self.left_support != self.left_support:  # This condition is always false, so we're always calculating swing foot
            # Calculate swing foot position
            if self.left_support:
                # Right foot is swinging
                swing_x = self.com_x + self.step_length * self.step_phase
                swing_y = self.step_width / 2 if self.step_phase < 0.5 else -self.step_width / 2
                # Add step height in the middle of the step
                swing_z = 0.0
                if 0.2 < self.step_phase < 0.8:
                    # Calculate parabolic trajectory for step height
                    phase_in_swing = (self.step_phase - 0.2) / 0.6  # Normalize to 0-1 for swing phase
                    swing_z = self.step_height * np.sin(np.pi * phase_in_swing)
            else:
                # Left foot is swinging
                swing_x = self.com_x + self.step_length * self.step_phase
                swing_y = -self.step_width / 2 if self.step_phase < 0.5 else self.step_width / 2
                # Add step height in the middle of the step
                swing_z = 0.0
                if 0.2 < self.step_phase < 0.8:
                    # Calculate parabolic trajectory for step height
                    phase_in_swing = (self.step_phase - 0.2) / 0.6  # Normalize to 0-1 for swing phase
                    swing_z = self.step_height * np.sin(np.pi * phase_in_swing)

            # Publish desired foot position
            foot_msg = Point()
            foot_msg.x = float(swing_x)
            foot_msg.y = float(swing_y)
            foot_msg.z = float(swing_z)
            self.footstep_publisher.publish(foot_msg)

        # Calculate and publish ZMP reference
        # For simplicity, ZMP is just under the supporting foot
        zmp_msg = Point()
        if self.left_support:
            zmp_msg.x = float(self.com_x - self.step_length / 4)  # Slightly in front of stance foot
            zmp_msg.y = float(self.step_width / 2)
        else:
            zmp_msg.x = float(self.com_x - self.step_length / 4)  # Slightly in front of stance foot
            zmp_msg.y = float(-self.step_width / 2)
        zmp_msg.z = 0.0  # On the ground
        self.zmp_publisher.publish(zmp_msg)

        self.get_logger().debug(f'Walking: phase={self.step_phase:.2f}, '
                               f'support={"left" if self.left_support else "right"}, '
                               f'ZMP=({zmp_msg.x:.2f}, {zmp_msg.y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    zmp_walking_controller = ZMPWalkingControllerNode()

    try:
        rclpy.spin(zmp_walking_controller)
    except KeyboardInterrupt:
        zmp_walking_controller.get_logger().info('Interrupted, shutting down')
    finally:
        zmp_walking_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 3.1: Implement a Bipedal Walking Controller
Create a controller that coordinates two legs to achieve stable bipedal walking using inverse kinematics.

### Exercise 3.2: Create an Adaptive Gait Generator
Implement a gait generator that adjusts step parameters based on terrain information from perception systems.

### Exercise 3.3: Implement Capture Point Control
Extend the ZMP controller to use capture point concepts for more robust balance during walking.

## Assessment Criteria
- Students can implement basic walking patterns for humanoid robots
- Students understand the principles of legged locomotion
- Students can design controllers for stable walking
- Students can integrate locomotion with balance and planning systems
- Students understand safety considerations in locomotion control