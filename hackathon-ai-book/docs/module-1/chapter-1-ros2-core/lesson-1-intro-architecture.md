---
sidebar_label: 'Lesson 1: Introduction to ROS 2 Architecture and Nodes'
sidebar_position: 1
---

# Lesson 1: Introduction to ROS 2 Architecture and Nodes

## Overview
In this lesson, you'll learn about the ROS 2 architecture and how to create basic nodes. We'll explore the node-based design concept and implement your first ROS 2 nodes in Python.

## Learning Objectives
- Understand the ROS 2 architecture and node-based design
- Create basic ROS 2 nodes in Python
- Understand the node lifecycle and initialization
- Implement proper error handling in nodes

## What is ROS 2 and Why It Matters for Humanoid Robotics

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

For humanoid robotics specifically, ROS 2 provides:
- Distributed computing capabilities for complex multi-joint systems
- Real-time communication between different subsystems (vision, control, planning)
- Extensive library support for robotics algorithms
- Simulation capabilities for testing before deployment

## The Node-Based Architecture Concept

In ROS 2, computation is broken down into nodes. A node is an executable that uses ROS 2 to communicate with other nodes. Nodes can:
- Publish data to topics
- Subscribe to topics
- Provide services
- Call services

This modular approach allows for:
- Code reuse across different robot platforms
- Independent development of different robot capabilities
- Easy debugging and testing of individual components

## Setting Up Your Development Environment

Before creating nodes, ensure you have:
- ROS 2 installed (Humble Hawksbill or later recommended)
- Python 3.8 or higher
- Basic understanding of Python programming

## Creating Your First ROS 2 Node

Let's create a basic ROS 2 node in Python:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.get_logger().info('Hello from basic_node!')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()
    node.get_logger().info('Node has been created')

    # Keep the node alive
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding the Node Lifecycle

A ROS 2 node typically follows this lifecycle:
1. **Initialization**: The node is created and initialized
2. **Activation**: The node becomes active and can communicate
3. **Running**: The node performs its tasks
4. **Shutdown**: The node is properly shut down

## Node Initialization and Spinning

The `rclpy.init()` function initializes the ROS 2 client library. The `rclpy.spin()` function keeps the node alive and processes callbacks. When you're done with the node, call `destroy_node()` and `rclpy.shutdown()` to properly clean up resources.

## Error Handling in Nodes

Always implement proper error handling in your nodes:

```python
def main(args=None):
    try:
        rclpy.init(args=args)
        node = BasicNode()

        # Perform operations
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Hands-On Exercise

Create a node that prints "Hello from [your_node_name]!" and runs for 10 seconds before shutting down.

## Summary

In this lesson, you learned about the ROS 2 architecture and how to create basic nodes. You now understand the node lifecycle and how to properly initialize and shut down nodes.

## Review Questions

1. What is a ROS 2 node?
2. What is the purpose of `rclpy.spin()`?
3. Why is proper error handling important in ROS 2 nodes?

## Next Lesson

Continue to Lesson 2: Publisher-Subscriber Pattern (Topics) to learn about asynchronous communication in ROS 2.