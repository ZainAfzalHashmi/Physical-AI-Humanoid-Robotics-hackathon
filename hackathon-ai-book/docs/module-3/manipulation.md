---
sidebar_label: 'Manipulation'
sidebar_position: 2
---

# Manipulation

## Overview
This chapter covers arm and hand control for humanoid robots, including grasping and object manipulation techniques. Manipulation is a critical capability for humanoid robots to interact with their environment and perform useful tasks.

## Learning Objectives
- Understand kinematics for robotic arms
- Implement grasping and manipulation techniques
- Design controllers for precise manipulation
- Plan manipulation trajectories
- Integrate manipulation with perception and planning systems

## Content Structure

### Section 3.1: Robotic Arm Kinematics
- Forward and inverse kinematics
- Jacobian matrices and their applications
- Workspace analysis
- Redundancy resolution

### Section 3.2: Grasping Techniques
- Types of grasps (power, precision)
- Grasp planning algorithms
- Contact mechanics and grasp stability
- Multi-fingered hand control

### Section 3.3: Manipulation Control
- Cartesian and joint space control
- Impedance control for compliant manipulation
- Force control techniques
- Visual servoing

### Section 3.4: Manipulation Planning
- Trajectory planning for manipulation tasks
- Collision avoidance in manipulation
- Task space planning
- Integration with locomotion

### Section 3.5: Advanced Manipulation
- Bimanual manipulation
- Tool use and manipulation
- Learning from demonstration
- Human-robot collaboration in manipulation

## Code Examples

### Example 3.1: Inverse Kinematics for Robotic Arm
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class ArmIKControllerNode(Node):
    def __init__(self):
        super().__init__('arm_ik_controller_node')

        # Create subscriber for desired end-effector pose
        self.pose_subscription = self.create_subscription(
            Pose,
            '/desired_end_effector_pose',
            self.pose_callback,
            10
        )

        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # Robot arm parameters (simplified 3-DOF planar arm)
        self.link1_length = 0.3  # meters
        self.link2_length = 0.25 # meters
        self.link3_length = 0.2  # meters

        # Current desired pose
        self.desired_pose = Pose()
        self.desired_pose.position.x = 0.5
        self.desired_pose.position.y = 0.0
        self.desired_pose.position.z = 0.1

    def pose_callback(self, msg):
        """Handle desired end-effector pose updates"""
        self.desired_pose = msg

    def calculate_ik(self, x, y, z):
        """
        Calculate inverse kinematics for a 3-DOF arm.
        This is a simplified planar example for the x-z plane.
        """
        # Project to 2D (x-z plane) for this example
        r = np.sqrt(x**2 + z**2)

        # Check if position is reachable
        if r > (self.link1_length + self.link2_length + self.link3_length):
            # Position is out of reach, extend arm fully forward
            theta1 = 0.0
            theta2 = 0.0
            theta3 = 0.0
            return theta1, theta2, theta3

        # Calculate theta2 using law of cosines
        # For a 2-link manipulator, we calculate the angle of the second joint
        # First, we need to account for the third link
        effector_x = r - self.link3_length * np.cos(0)  # Assuming fixed wrist angle for simplicity
        effector_z = z - self.link3_length * np.sin(0)  # Assuming fixed wrist angle for simplicity

        # Calculate distance from base to target (excluding first link)
        d = np.sqrt(effector_x**2 + effector_z**2)

        # Calculate angle between first and second link
        cos_theta2 = (self.link1_length**2 + self.link2_length**2 - d**2) / \
                     (2 * self.link1_length * self.link2_length)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Clamp to valid range
        theta2 = np.pi - np.arccos(cos_theta2)

        # Calculate angle of first link
        alpha = np.arctan2(effector_z, effector_x)
        beta = np.arccos((self.link1_length**2 + d**2 - self.link2_length**2) / \
                         (2 * self.link1_length * d))
        theta1 = alpha + beta

        # For this example, theta3 is used for wrist orientation
        theta3 = 0.0  # Fixed for this example

        # Calculate base rotation (around y-axis)
        base_rotation = np.arctan2(y, x) if np.sqrt(x**2 + y**2) > 0.01 else 0.0

        return base_rotation, theta1, theta2, theta3

    def publish_joint_commands(self):
        """Calculate and publish joint commands based on desired pose"""
        # Calculate inverse kinematics
        base_rot, shoulder, elbow, wrist = self.calculate_ik(
            self.desired_pose.position.x,
            self.desired_pose.position.y,
            self.desired_pose.position.z
        )

        # Create and publish joint command message
        command_msg = Float64MultiArray()
        command_msg.data = [
            float(base_rot),  # Base rotation (around y-axis)
            float(shoulder),  # Shoulder joint
            float(elbow),     # Elbow joint
            float(wrist),     # Wrist joint
            0.0,              # Gripper (not implemented in IK)
            0.0               # Gripper (not implemented in IK)
        ]

        self.joint_command_publisher.publish(command_msg)

        self.get_logger().debug(f'End-effector pos: ({self.desired_pose.position.x:.2f}, '
                               f'{self.desired_pose.position.y:.2f}, '
                               f'{self.desired_pose.position.z:.2f}), '
                               f'Joints: ({base_rot:.2f}, {shoulder:.2f}, '
                               f'{elbow:.2f}, {wrist:.2f})')

def main(args=None):
    rclpy.init(args=args)
    arm_ik_controller = ArmIKControllerNode()

    # Create a timer to periodically publish joint commands
    timer = arm_ik_controller.create_timer(0.02, arm_ik_controller.publish_joint_commands)  # 50 Hz

    try:
        rclpy.spin(arm_ik_controller)
    except KeyboardInterrupt:
        arm_ik_controller.get_logger().info('Interrupted, shutting down')
    finally:
        arm_ik_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3.2: Grasp Planning Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import String
import numpy as np

class GraspPlannerNode(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')

        # Create subscriber for object information
        self.object_subscription = self.create_subscription(
            Pose,
            '/object_pose',
            self.object_callback,
            10
        )

        # Create publisher for grasp poses
        self.grasp_publisher = self.create_publisher(
            Pose,
            '/grasp_pose',
            10
        )

        # Create publisher for grasp type
        self.grasp_type_publisher = self.create_publisher(
            String,
            '/grasp_type',
            10
        )

        # Robot hand parameters
        self.finger_span = 0.1  # Maximum distance between fingers (m)
        self.hand_depth = 0.08  # Depth of the hand (m)

        # Current object information
        self.current_object_pose = None
        self.object_size = Vector3(x=0.05, y=0.05, z=0.05)  # Default size

    def object_callback(self, msg):
        """Handle object pose updates"""
        self.current_object_pose = msg
        self.plan_grasp()

    def plan_grasp(self):
        """Plan an appropriate grasp for the current object"""
        if self.current_object_pose is None:
            return

        # Determine object size (in a real implementation, this would come from perception)
        object_width = self.object_size.x
        object_height = self.object_size.y
        object_depth = self.object_size.z

        # Determine grasp type based on object dimensions
        grasp_type_msg = String()
        if object_width < 0.03 and object_height < 0.03:
            # Small object - precision grasp
            grasp_type_msg.data = "precision"
        elif object_width > 0.1 or object_height > 0.1 or object_depth > 0.1:
            # Large object - power grasp
            grasp_type_msg.data = "power"
        else:
            # Medium object - intermediate grasp
            grasp_type_msg.data = "intermediate"

        # Plan grasp pose
        grasp_pose = Pose()
        grasp_pose.position = self.current_object_pose.position

        # Adjust position to approach from the side
        grasp_pose.position.x -= 0.1  # Approach from the front

        # Set orientation based on grasp type
        if grasp_type_msg.data == "precision":
            # For precision grasp, align with object's orientation
            grasp_pose.orientation = self.current_object_pose.orientation
        else:
            # For power grasp, align hand perpendicular to object
            # This is a simplified example
            grasp_pose.orientation.w = 1.0
            grasp_pose.orientation.x = 0.0
            grasp_pose.orientation.y = 0.0
            grasp_pose.orientation.z = 0.0

        # Publish results
        self.grasp_publisher.publish(grasp_pose)
        self.grasp_type_publisher.publish(grasp_type_msg)

        self.get_logger().info(f'Planned {grasp_type_msg.data} grasp for object at '
                              f'({grasp_pose.position.x:.2f}, {grasp_pose.position.y:.2f}, '
                              f'{grasp_pose.position.z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    grasp_planner = GraspPlannerNode()

    try:
        rclpy.spin(grasp_planner)
    except KeyboardInterrupt:
        grasp_planner.get_logger().info('Interrupted, shutting down')
    finally:
        grasp_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3.3: Impedance Controller for Compliant Manipulation
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class ImpedanceControllerNode(Node):
    def __init__(self):
        super().__init__('impedance_controller_node')

        # Create subscriber for desired pose
        self.desired_pose_subscription = self.create_subscription(
            PoseStamped,
            '/desired_manipulator_pose',
            self.desired_pose_callback,
            10
        )

        # Create subscriber for current pose (from forward kinematics)
        self.current_pose_subscription = self.create_subscription(
            PoseStamped,
            '/current_manipulator_pose',
            self.current_pose_callback,
            10
        )

        # Create subscriber for force/torque sensor
        self.wrench_subscription = self.create_subscription(
            WrenchStamped,
            '/wrench_sensor',
            self.wrench_callback,
            10
        )

        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/impedance_controller/commands',
            10
        )

        # Impedance control parameters
        self.mass = np.diag([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])  # Mass matrix (x, y, z, roll, pitch, yaw)
        self.damping = np.diag([20.0, 20.0, 20.0, 10.0, 10.0, 10.0])  # Damping matrix
        self.stiffness = np.diag([500.0, 500.0, 500.0, 200.0, 200.0, 200.0])  # Stiffness matrix

        # Robot state
        self.desired_pose = None
        self.current_pose = None
        self.current_wrench = None
        self.previous_error = np.zeros(6)
        self.integrated_error = np.zeros(6)

        # Control timer
        self.control_timer = self.create_timer(0.005, self.impedance_control_callback)  # 200 Hz

    def desired_pose_callback(self, msg):
        """Handle desired pose updates"""
        self.desired_pose = msg

    def current_pose_callback(self, msg):
        """Handle current pose updates"""
        self.current_pose = msg

    def wrench_callback(self, msg):
        """Handle wrench sensor updates"""
        self.current_wrench = msg

    def impedance_control_callback(self):
        """Main impedance control loop"""
        if self.desired_pose is None or self.current_pose is None:
            return

        # Calculate position and orientation errors
        pos_error = np.array([
            self.desired_pose.pose.position.x - self.current_pose.pose.position.x,
            self.desired_pose.pose.position.y - self.current_pose.pose.position.y,
            self.desired_pose.pose.position.z - self.current_pose.pose.position.z
        ])

        # Simplified orientation error (in a real implementation, use quaternion error)
        orient_error = np.array([
            self.desired_pose.pose.orientation.x - self.current_pose.pose.orientation.x,
            self.desired_pose.pose.orientation.y - self.current_pose.pose.orientation.y,
            self.desired_pose.pose.orientation.z - self.current_pose.pose.orientation.z
        ])

        # Combine position and orientation errors
        total_error = np.concatenate([pos_error, orient_error])

        # Calculate desired acceleration using impedance model
        # M*a + D*v + K*x = F
        # a = M^(-1) * (F - D*v - K*x)

        # For this example, we'll use a simplified approach
        # Calculate velocity (approximated from position changes)
        velocity = (total_error - self.previous_error) / 0.005  # dt = 0.005s

        # Calculate impedance force
        acceleration = np.linalg.inv(self.mass).dot(
            -self.damping.dot(velocity) -
            self.stiffness.dot(total_error)
        )

        # Add external force if available
        if self.current_wrench is not None:
            external_force = np.array([
                self.current_wrench.wrench.force.x,
                self.current_wrench.wrench.force.y,
                self.current_wrench.wrench.force.z,
                self.current_wrench.wrench.torque.x,
                self.current_wrench.wrench.torque.y,
                self.current_wrench.wrench.torque.z
            ])
            acceleration += np.linalg.inv(self.mass).dot(external_force)

        # Update previous error
        self.previous_error = total_error

        # Integrate to get velocity and position adjustments
        dt = 0.005
        self.integrated_error += acceleration * dt

        # Create and publish joint command (simplified - in reality, this would involve
        # inverse kinematics and dynamics calculations)
        command_msg = Float64MultiArray()
        command_msg.data = [float(val) for val in self.integrated_error[:6]]  # Use first 6 DOF

        self.joint_command_publisher.publish(command_msg)

        self.get_logger().debug(f'Impedance control: pos_error=({pos_error[0]:.3f}, {pos_error[1]:.3f}, {pos_error[2]:.3f}), '
                               f'acceleration=({acceleration[0]:.3f}, {acceleration[1]:.3f}, {acceleration[2]:.3f})')

def main(args=None):
    rclpy.init(args=args)
    impedance_controller = ImpedanceControllerNode()

    try:
        rclpy.spin(impedance_controller)
    except KeyboardInterrupt:
        impedance_controller.get_logger().info('Interrupted, shutting down')
    finally:
        impedance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 3.1: Implement a Cartesian Controller
Create a controller that allows direct Cartesian control of the end-effector while maintaining joint limits.

### Exercise 3.2: Create a Grasp Stability Evaluator
Implement a node that evaluates the stability of a planned grasp based on contact points and object properties.

### Exercise 3.3: Implement Visual Servoing
Create a controller that uses visual feedback to precisely position the end-effector relative to a target.

## Assessment Criteria
- Students can implement inverse kinematics for robotic arms
- Students understand grasping techniques and can plan appropriate grasps
- Students can design controllers for compliant manipulation
- Students can plan manipulation trajectories with collision avoidance
- Students can integrate manipulation with perception and planning systems