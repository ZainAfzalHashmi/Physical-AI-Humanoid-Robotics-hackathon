---
sidebar_label: 'Perception'
sidebar_position: 1
---

# Perception

## Overview
This chapter covers sensing and understanding the environment for humanoid robots, including sensor integration and data processing techniques. Perception is fundamental to Physical AI as it provides the sensory input necessary for intelligent decision-making and interaction with the environment.

## Learning Objectives
- Understand various sensor types used in humanoid robotics
- Implement sensor fusion techniques for robust perception
- Process sensor data for environment understanding
- Apply computer vision techniques for humanoid robots
- Integrate perception with planning and control systems

## Content Structure

### Section 2.1: Sensor Types and Integration
- Overview of sensors for humanoid robots (cameras, LIDAR, IMU, force/torque sensors)
- Sensor placement and mounting considerations
- Sensor calibration techniques
- ROS 2 sensor interfaces and message types

### Section 2.2: Computer Vision for Humanoid Robots
- Image processing fundamentals
- Object detection and recognition
- Visual SLAM for environment mapping
- Human detection and tracking
- Depth perception and 3D reconstruction

### Section 2.3: Sensor Fusion Techniques
- Combining data from multiple sensors
- Kalman filters for state estimation
- Particle filters for complex environments
- Handling sensor noise and uncertainty

### Section 2.4: Environment Understanding
- Mapping and localization (SLAM)
- Object recognition and scene understanding
- Dynamic obstacle detection
- Human-robot interaction through perception

### Section 2.5: Real-time Processing Considerations
- Computational efficiency in perception systems
- Real-time constraints for humanoid control
- Edge computing for perception tasks
- Performance optimization techniques

## Code Examples

### Example 2.1: Basic Camera Image Processing Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Create subscriber for camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_processed',
            10
        )

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Timer for processing loop
        self.process_timer = self.create_timer(0.1, self.process_callback)

        # Internal state
        self.latest_image = None
        self.processed_image = None

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store the latest image for processing
            self.latest_image = cv_image.copy()

            # Log that we received an image
            self.get_logger().info(f'Received image: {cv_image.shape}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_callback(self):
        """Process the latest image"""
        if self.latest_image is not None:
            # Apply some basic processing (e.g., edge detection)
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Convert back to color for visualization
            self.processed_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            # Publish the processed image
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(self.processed_image, encoding='bgr8')
                processed_msg.header = self.latest_image.header if hasattr(self.latest_image, 'header') else msg.header
                self.publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing processed image: {e}')

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Interrupted, shutting down')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.2: Object Detection with OpenCV
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Create subscriber for camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for object detection results
        self.detection_publisher = self.create_publisher(
            String,
            '/object_detection/results',
            10
        )

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Load pre-trained model (Haar cascade as example)
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # Internal state
        self.latest_image = None

    def image_callback(self, msg):
        """Process incoming camera image for object detection"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform face detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )

            # Draw rectangles around detected faces
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Publish detection results
            if len(faces) > 0:
                result_msg = String()
                result_msg.data = f"Detected {len(faces)} face(s)"
                self.detection_publisher.publish(result_msg)
                self.get_logger().info(f'Detected {len(faces)} face(s)')

        except Exception as e:
            self.get_logger().error(f'Error in object detection: {e}')

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        object_detection_node.get_logger().info('Interrupted, shutting down')
    finally:
        object_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.3: Sensor Fusion with Kalman Filter
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import PointStamped
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create subscribers for different sensors
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.range_subscription = self.create_subscription(
            Range,
            '/range/sensor',
            self.range_callback,
            10
        )

        # Create publisher for fused position estimate
        self.position_publisher = self.create_publisher(
            PointStamped,
            '/fused_position',
            10
        )

        # Initialize Kalman filter parameters
        # State: [position, velocity]
        self.state = np.array([0.0, 0.0])  # [pos, vel]
        self.covariance = np.array([[1.0, 0.0], [0.0, 1.0]])  # P

        # Process noise
        self.Q = np.array([[0.1, 0.0], [0.0, 0.1]])  # Process noise covariance

        # Measurement noise for different sensors
        self.R_imu = np.array([[0.01]])  # IMU acceleration noise
        self.R_range = np.array([[0.05]])  # Range sensor noise

        # Time step
        self.dt = 0.05  # 20 Hz
        self.last_time = self.get_clock().now()

        # Sensor data storage
        self.latest_imu = None
        self.latest_range = None

    def imu_callback(self, msg):
        """Handle IMU data"""
        # Extract linear acceleration (assuming z-axis is up)
        acceleration = msg.linear_acceleration.z

        # Perform prediction step of Kalman filter
        self.predict(acceleration)

        # Update with IMU measurement if available
        if self.latest_range is not None:
            self.update_with_range(self.latest_range)

        # Publish current estimate
        self.publish_estimate()

    def range_callback(self, msg):
        """Handle range sensor data"""
        # Store the latest range measurement
        self.latest_range = msg.range

    def predict(self, acceleration):
        """Prediction step of Kalman filter"""
        # State transition matrix
        F = np.array([[1.0, self.dt], [0.0, 1.0]])

        # Control input matrix (acceleration affects velocity)
        B = np.array([0.5 * self.dt**2, self.dt])

        # Observation matrix (we can observe position)
        H = np.array([1.0, 0.0])

        # Predict state
        self.state = F @ self.state + B * acceleration

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update_with_range(self, range_measurement):
        """Update step with range measurement"""
        # Observation matrix (we observe position)
        H = np.array([[1.0, 0.0]])

        # Innovation
        z = np.array([range_measurement])  # Measurement
        h = H @ self.state  # Expected measurement
        y = z - h  # Innovation

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_range

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance

        # Reset range measurement after using it
        self.latest_range = None

    def publish_estimate(self):
        """Publish the current position estimate"""
        try:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'base_link'
            point_msg.point.x = 0.0  # We're tracking 1D position in z
            point_msg.point.y = 0.0
            point_msg.point.z = float(self.state[0])  # Position estimate

            self.position_publisher.publish(point_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing position estimate: {e}')

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        sensor_fusion_node.get_logger().info('Interrupted, shutting down')
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 2.1: Implement Color-Based Object Detection
Create a perception node that detects objects of a specific color (e.g., red) in camera images using color space conversion and thresholding.

### Exercise 2.2: Create a Simple SLAM Node
Implement a basic SLAM system that combines odometry and sensor data to build a map of the environment.

### Exercise 2.3: Sensor Fusion for Position Estimation
Extend the Kalman filter example to fuse data from multiple sensors (IMU, range, odometry) for more accurate position estimation.

## Assessment Criteria
- Students can implement basic computer vision algorithms for humanoid robots
- Students understand sensor fusion techniques and can implement simple filters
- Students can integrate perception outputs with planning and control systems
- Students can optimize perception algorithms for real-time performance
- Students understand the limitations and uncertainties of perception systems