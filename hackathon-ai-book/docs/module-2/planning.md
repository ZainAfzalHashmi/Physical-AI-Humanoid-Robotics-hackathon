---
sidebar_label: 'Planning'
sidebar_position: 2
---

# Planning

## Overview
This chapter covers path planning and motion planning for humanoid robots, including trajectory generation and obstacle avoidance. Planning is a critical component of Physical AI that enables robots to navigate complex environments and execute tasks efficiently.

## Learning Objectives
- Understand fundamental path planning algorithms
- Implement motion planning for humanoid robots
- Generate smooth trajectories for robot movement
- Handle dynamic obstacles and replanning
- Integrate planning with perception and control systems

## Content Structure

### Section 2.1: Path Planning Fundamentals
- Configuration space and workspace concepts
- Graph-based planning algorithms (A*, Dijkstra)
- Sampling-based methods (RRT, RRT*)
- Grid-based planning approaches

### Section 2.2: Motion Planning for Humanoid Robots
- Kinematic constraints in humanoid planning
- Whole-body motion planning
- Inverse kinematics integration
- Planning for multi-degree-of-freedom systems

### Section 2.3: Trajectory Generation
- Polynomial trajectory generation
- Velocity and acceleration profiles
- Time-optimal trajectories
- Smoothing techniques for natural movement

### Section 2.4: Dynamic Environment Planning
- Handling moving obstacles
- Real-time replanning strategies
- Predictive planning approaches
- Uncertainty-aware planning

### Section 2.5: Planning Optimization
- Multi-objective optimization in planning
- Planning with energy efficiency
- Human-aware planning
- Performance considerations

## Code Examples

### Example 2.1: A* Path Planning Algorithm
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from heapq import heappop, heappush

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')

        # Create subscriber for map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create subscriber for goal
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Create publisher for computed path
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None
        self.current_goal = None

    def map_callback(self, msg):
        """Handle incoming map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

        self.get_logger().info(f'Map received: {self.map_width}x{self.map_height}')

    def goal_callback(self, msg):
        """Handle incoming goal pose"""
        if self.map_data is not None:
            # Convert goal coordinates to map indices
            x = int((msg.pose.position.x - self.map_origin.position.x) / self.map_resolution)
            y = int((msg.pose.position.y - self.map_origin.position.y) / self.map_resolution)

            # Check if goal is within map bounds
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.current_goal = (x, y)
                self.get_logger().info(f'New goal set: ({x}, {y})')

                # Plan path to goal
                path = self.plan_path((x, y))
                if path:
                    self.publish_path(path)
                else:
                    self.get_logger().warn('No path found to goal')
            else:
                self.get_logger().warn('Goal is outside map bounds')

    def plan_path(self, goal):
        """Plan path using A* algorithm"""
        if self.map_data is None:
            return None

        # For simplicity, assume start is at (10, 10)
        start = (10, 10)

        # Check if start or goal is in obstacle
        if self.map_data[start[1], start[0]] > 50 or self.map_data[goal[1], goal[0]] > 50:
            self.get_logger().warn('Start or goal is in obstacle space')
            return None

        # A* algorithm implementation
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

        # Directions: up, down, left, right, and diagonals
        directions = [(-1, -1), (-1, 0), (-1, 1),
                      (0, -1),           (0, 1),
                      (1, -1),  (1, 0),  (1, 1)]

        # Initialize open and closed sets
        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            current = heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                # Check bounds
                if (0 <= neighbor[0] < self.map_width and
                    0 <= neighbor[1] < self.map_height):

                    # Skip if it's an obstacle
                    if self.map_data[neighbor[1], neighbor[0]] > 50:
                        continue

                    # Calculate tentative g_score
                    # Add cost for diagonal moves
                    tentative_g_score = g_score[current] + (1.414 if abs(dx) + abs(dy) == 2 else 1)

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def publish_path(self, path):
        """Publish the computed path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x * self.map_resolution + self.map_origin.position.x
            pose.pose.position.y = y * self.map_resolution + self.map_origin.position.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path)} waypoints')

def main(args=None):
    rclpy.init(args=args)
    astar_planner_node = AStarPlannerNode()

    try:
        rclpy.spin(astar_planner_node)
    except KeyboardInterrupt:
        astar_planner_node.get_logger().info('Interrupted, shutting down')
    finally:
        astar_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.2: Trajectory Generator for Smooth Motion
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')

        # Create subscriber for path
        self.path_subscription = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal state
        self.current_path = []
        self.current_index = 0
        self.is_moving = False
        self.path_follow_timer = None

        # Robot parameters
        self.linear_vel_max = 0.5  # m/s
        self.angular_vel_max = 1.0  # rad/s
        self.lookahead_distance = 0.3  # m
        self.arrival_threshold = 0.1  # m

    def path_callback(self, msg):
        """Handle incoming path message"""
        if len(msg.poses) > 0:
            # Convert path to list of (x, y) coordinates
            self.current_path = [(pose.pose.position.x, pose.pose.position.y)
                                for pose in msg.poses]
            self.current_index = 0
            self.is_moving = True

            # Start following the path
            if self.path_follow_timer is None:
                self.path_follow_timer = self.create_timer(0.1, self.follow_path)

            self.get_logger().info(f'Starting to follow path with {len(self.current_path)} waypoints')

    def follow_path(self):
        """Follow the current path"""
        if not self.is_moving or len(self.current_path) == 0:
            return

        # Get robot's current position (in a real implementation, this would come from localization)
        # For simulation, we'll track position based on commands
        # Here we'll just use a simple approach

        if self.current_index >= len(self.current_path):
            # Reached the end of the path
            self.stop_robot()
            self.is_moving = False
            self.get_logger().info('Reached end of path')
            return

        # Get current target point
        target_x, target_y = self.current_path[self.current_index]

        # Calculate distance to target
        # In a real implementation, this would use actual robot position
        # For simulation, we'll assume robot is at the last reached point
        if self.current_index > 0:
            robot_x, robot_y = self.current_path[self.current_index - 1]
        else:
            robot_x, robot_y = 0.0, 0.0  # Starting position

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < self.arrival_threshold:
            # Move to next waypoint
            self.current_index += 1
            if self.current_index >= len(self.current_path):
                self.stop_robot()
                self.is_moving = False
                self.get_logger().info('Reached end of path')
                return
            return

        # Calculate desired velocity
        linear_vel = min(self.linear_vel_max, distance * 1.0)  # Simple proportional control
        angular_vel = np.arctan2(dy, dx) * 1.0  # Simple heading control

        # Limit velocities
        linear_vel = max(-self.linear_vel_max, min(self.linear_vel_max, linear_vel))
        angular_vel = max(-self.angular_vel_max, min(self.angular_vel_max, angular_vel))

        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = float(linear_vel)
        cmd_msg.angular.z = float(angular_vel)
        self.cmd_vel_publisher.publish(cmd_msg)

    def stop_robot(self):
        """Stop the robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    trajectory_generator_node = TrajectoryGeneratorNode()

    try:
        rclpy.spin(trajectory_generator_node)
    except KeyboardInterrupt:
        trajectory_generator_node.get_logger().info('Interrupted, shutting down')
    finally:
        trajectory_generator_node.stop_robot()
        trajectory_generator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2.3: RRT Path Planning (Simplified Implementation)
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from random import random

class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner_node')

        # Create subscriber for map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create subscriber for goal
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Create publisher for computed path
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None
        self.current_goal = None

        # RRT parameters
        self.max_iterations = 1000
        self.step_size = 5  # pixels

    def map_callback(self, msg):
        """Handle incoming map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

        self.get_logger().info(f'Map received: {self.map_width}x{self.map_height}')

    def goal_callback(self, msg):
        """Handle incoming goal pose"""
        if self.map_data is not None:
            # Convert goal coordinates to map indices
            x = int((msg.pose.position.x - self.map_origin.position.x) / self.map_resolution)
            y = int((msg.pose.position.y - self.map_origin.position.y) / self.map_resolution)

            # Check if goal is within map bounds
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.current_goal = (x, y)
                self.get_logger().info(f'New goal set: ({x}, {y})')

                # Plan path to goal using RRT
                path = self.plan_path_rrt((x, y))
                if path:
                    self.publish_path(path)
                else:
                    self.get_logger().warn('No path found to goal')
            else:
                self.get_logger().warn('Goal is outside map bounds')

    def plan_path_rrt(self, goal):
        """Plan path using RRT algorithm"""
        if self.map_data is None:
            return None

        # For simplicity, assume start is at (10, 10)
        start = (10, 10)

        # Check if start or goal is in obstacle
        if self.map_data[start[1], start[0]] > 50 or self.map_data[goal[1], goal[0]] > 50:
            self.get_logger().warn('Start or goal is in obstacle space')
            return None

        # RRT implementation
        # Node structure: (x, y, parent_index)
        nodes = [start + (None,)]  # Start node with no parent

        for i in range(self.max_iterations):
            # Randomly sample a point (with bias toward goal)
            if random() < 0.1:  # 10% chance to sample goal
                rand_point = goal
            else:
                rand_point = (
                    int(random() * self.map_width),
                    int(random() * self.map_height)
                )

            # Find nearest node in tree
            nearest_idx = 0
            min_dist = float('inf')
            for j, node in enumerate(nodes):
                dist = np.sqrt((node[0] - rand_point[0])**2 + (node[1] - rand_point[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = j

            # Get nearest node
            nearest_node = nodes[nearest_idx]

            # Move from nearest node toward random point
            dx = rand_point[0] - nearest_node[0]
            dy = rand_point[1] - nearest_node[1]
            dist = np.sqrt(dx**2 + dy**2)

            if dist == 0:
                continue

            # Normalize direction and step toward random point
            step_x = int(nearest_node[0] + (dx / dist) * min(self.step_size, dist))
            step_y = int(nearest_node[1] + (dy / dist) * min(self.step_size, dist))

            # Check if new point is valid (not in obstacle)
            if (0 <= step_x < self.map_width and
                0 <= step_y < self.map_height and
                self.map_data[step_y, step_x] <= 50):

                # Add new node to tree
                new_node = (step_x, step_y, nearest_idx)
                nodes.append(new_node)

                # Check if we've reached the goal (with some tolerance)
                if np.sqrt((step_x - goal[0])**2 + (step_y - goal[1])**2) < self.step_size:
                    # Reconstruct path from goal to start
                    path = []
                    current_idx = len(nodes) - 1
                    while current_idx is not None:
                        node = nodes[current_idx]
                        path.append((node[0], node[1]))
                        current_idx = node[2]  # parent index
                    path.reverse()
                    return path

        return None  # Failed to find path

    def publish_path(self, path):
        """Publish the computed path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x * self.map_resolution + self.map_origin.position.x
            pose.pose.position.y = y * self.map_resolution + self.map_origin.position.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'Published RRT path with {len(path)} waypoints')

def main(args=None):
    rclpy.init(args=args)
    rrt_planner_node = RRTPlannerNode()

    try:
        rclpy.spin(rrt_planner_node)
    except KeyboardInterrupt:
        rrt_planner_node.get_logger().info('Interrupted, shutting down')
    finally:
        rrt_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercises

### Exercise 2.1: Implement Dijkstra's Algorithm
Create a path planning node that uses Dijkstra's algorithm instead of A* to find the shortest path in a grid map.

### Exercise 2.2: Create a Trajectory Smoother
Implement a node that takes a path with sharp turns and generates a smoother trajectory using cubic splines.

### Exercise 2.3: Dynamic Obstacle Avoidance
Extend the path planner to handle moving obstacles by implementing a replanning mechanism that updates the path when obstacles are detected.

## Assessment Criteria
- Students can implement fundamental path planning algorithms (A*, RRT)
- Students understand motion planning concepts for humanoid robots
- Students can generate smooth trajectories for robot movement
- Students can handle dynamic environments with replanning
- Students can integrate planning with perception and control systems