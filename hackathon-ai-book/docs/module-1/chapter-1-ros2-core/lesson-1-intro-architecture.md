# Lesson 1: Introduction to ROS 2 Architecture and Nodes

## Learning Objectives
By the end of this lesson, you will be able to:
- Explain the ROS 2 architecture and its key components
- Create and run a basic ROS 2 node in Python
- Understand the lifecycle of a ROS 2 node
- Use common ROS 2 tools for node management

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Concepts in ROS 2 Architecture

#### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. They contain the code that performs specific tasks and can receive, send, or process data.

Nodes are organized into packages, which are the basic building and distribution units in ROS 2. A package contains all the necessary files for a specific functionality, including source code, configuration files, and documentation.

#### Packages
A package is the smallest unit of functionality in ROS 2. It contains libraries, executables, scripts, or other files required for a specific functionality. A package typically contains:

- Source code files
- Launch files
- Configuration files
- Documentation
- Tests

## Creating Your First ROS 2 Node

### Prerequisites
Before creating your first node, ensure you have ROS 2 installed (Humble Hawksbill or later recommended). You should also have a workspace set up.

### Step 1: Create a Package
```bash
# In your ROS 2 workspace src directory
ros2 pkg create --build-type ament_python py_nodes_tutorial
```

### Step 2: Create the Node Script
Create a Python file in `py_nodes_tutorial/py_nodes_tutorial/simple_node.py`:

```python
import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('SimpleNode has been started')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    
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

### Step 3: Setup File Configuration
Update the `setup.py` file in your package directory to include the console script:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'py_nodes_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple Python nodes for tutorial',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = py_nodes_tutorial.simple_node:main',
        ],
    },
)
```

### Step 4: Building and Running the Node
```bash
# Build the package
colcon build --packages-select py_nodes_tutorial

# Source the workspace
source install/setup.bash

# Run the node
ros2 run py_nodes_tutorial simple_node
```

## Node Lifecycle

ROS 2 nodes have a well-defined lifecycle, which is an optional state machine that standardizes the states a node can be in during execution. The lifecycle provides improved fault handling and coordinated startup/shutdown across a system.

The states in the ROS 2 lifecycle are:
- Unconfigured: Initial state after creation
- Inactive: After successful configuration
- Active: After successful transition from inactive
- Finalized: After successful transition from any other state

## Essential ROS 2 CLI Commands for Node Management

### Listing Nodes
```bash
ros2 node list
```
This command shows all active nodes in the ROS 2 graph.

### Information About a Specific Node
```bash
ros2 node info <node_name>
```
This provides detailed information about a specific node, including its topics and services.

### Creating a Node from Command Line
Sometimes it's useful to create a temporary node for testing purposes:
```bash
ros2 run demo_nodes_cpp talker
```

## Node Communication Overview

In ROS 2, nodes communicate through several mechanisms:
- **Topics**: Asynchronous, many-to-many communication using publish/subscribe pattern
- **Services**: Synchronous, request/response communication 
- **Actions**: Asynchronous, goal-oriented communication with feedback

We'll explore these communication patterns in the upcoming lessons.

## Best Practices for Node Design

1. **Single Responsibility**: Each node should have a single, well-defined purpose
2. **Modularity**: Design nodes to be reusable and replaceable
3. **Error Handling**: Implement proper error handling and recovery mechanisms
4. **Logging**: Use appropriate log levels (debug, info, warn, error, fatal)
5. **Parameter Configuration**: Use ROS 2 parameters for configuration values
6. **Resource Management**: Properly clean up resources when shutting down

## Parameters in ROS 2 Nodes

Parameters allow nodes to be configured without recompilation. Here's how to implement parameters in your node:

```python
import rclpy
from rclpy.node import Node


class NodeWithParameters(Node):
    def __init__(self):
        super().__init__('node_with_parameters')
        
        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('threshold', 1.0)
        
        # Get parameter values
        my_param = self.get_parameter('my_parameter').value
        threshold = self.get_parameter('threshold').value
        
        self.get_logger().info(f'My parameter: {my_param}')
        self.get_logger().info(f'Threshold: {threshold}')


def main(args=None):
    rclpy.init(args=args)
    node = NodeWithParameters()
    
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

## Quality of Service (QoS) in Nodes

QoS settings allow you to control the behavior of publishers and subscribers in terms of reliability, durability, and other aspects:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Lesson Summary

In this lesson, you've learned about the ROS 2 architecture and how to create basic nodes. You now understand:
- The fundamental concepts of nodes and packages
- How to create, build, and run a ROS 2 node
- The lifecycle of a ROS 2 node
- Essential CLI commands for node management
- Best practices for node design
- How to work with parameters and QoS settings

## Exercises

1. Create a node called "robot_controller" with appropriate parameters like max_speed and operating_mode.
2. Explore the ROS 2 documentation to understand different QoS policies and when to use each one.
3. Write a simple node that prints its lifecycle changes.

## Next Steps

In the next lesson, we'll explore the publisher-subscriber communication pattern that enables asynchronous data exchange between nodes.