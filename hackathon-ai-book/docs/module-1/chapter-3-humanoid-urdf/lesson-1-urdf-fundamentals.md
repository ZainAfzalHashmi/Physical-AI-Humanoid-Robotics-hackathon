# Lesson 1: URDF Fundamentals and Basic Structure

## Learning Objectives
By the end of this lesson, you will be able to:
- Understand the URDF (Unified Robot Description Format) XML structure
- Create basic robot models with links and joints
- Define geometric, inertial, and visual properties
- Validate URDF files for correctness
- Use ROS 2 tools to visualize and inspect URDF models

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robots. It contains information about robot links, joints, inertial properties, visual and collision properties, and kinematic relationships. URDF is fundamental for robot simulation, visualization, and control in the ROS ecosystem.

### Key URDF Concepts:
- **Link**: A rigid body with physical properties (mass, inertia, visual representation)
- **Joint**: Connection between two links that defines their motion relationship
- **Transmission**: Defines how actuators connect to joints
- **Gazebo Plugin**: Extensions for simulation-specific properties

## Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Define links -->
  <link name="link_name">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Define joints -->
  <joint name="joint_name" type="fixed">
    <parent link="parent_link"/>
    <child link="child_link"/>
  </joint>
</robot>
```

## Creating a Simple Robot Model

Let's start with a simple differential drive robot to understand the basics:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="1.570796327 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="1.570796327 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Understanding Link Elements

### Inertial Properties
The `<inertial>` element defines the physical properties needed for simulation:

```xml
<inertial>
  <!-- Origin relative to link frame -->
  <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
  <!-- Mass in kilograms -->
  <mass value="2.0"/>
  <!-- Inertia matrix (3x3 symmetric matrix in kg*m^2) -->
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
</inertial>
```

### Visual Properties
The `<visual>` element defines how the link appears in visualization tools:

```xml
<visual>
  <!-- Origin relative to link frame -->
  <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
  <!-- Geometry definition -->
  <geometry>
    <!-- Options: box, cylinder, sphere, mesh -->
    <box size="0.5 0.3 0.2"/>
  </geometry>
  <!-- Material properties -->
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Properties
The `<collision>` element defines the collision boundaries for physics simulation:

```xml
<collision>
  <!-- Origin relative to link frame -->
  <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
  <!-- Geometry definition -->
  <geometry>
    <box size="0.5 0.3 0.2"/>
  </geometry>
</collision>
```

## Understanding Joint Elements

### Joint Types
URDF supports several joint types:
- `revolute`: Limited rotation around one axis
- `continuous`: Unlimited rotation around one axis
- `prismatic`: Limited translation along one axis
- `fixed`: No relative motion (rigid connection)
- `floating`: 6 DOF connection
- `planar`: Planar motion (3 DOF)

```xml
<!-- Revolute joint example -->
<joint name="hinge_joint" type="revolute">
  <parent link="base_link"/>
  <child link="moving_part"/>
  <!-- Position of joint in parent frame -->
  <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  <!-- Axis of rotation in child frame -->
  <axis xyz="0 0 1"/>
  <!-- Joint limits -->
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

## URDF Origins and Coordinate Systems

URDF uses the right-handed Cartesian coordinate system:
- X: Forward
- Y: Left
- Z: Up

### Origin Element
The `<origin>` element defines position and orientation with:
- `xyz`: Position vector [x, y, z]
- `rpy`: Roll, Pitch, Yaw angles in radians

```xml
<origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>
```

## Validating URDF Files

### Using check_urdf Command
```bash
# Install urdfdom tools if not already installed
sudo apt-get install ros-humble-urdfdom-tools

# Validate your URDF file
check_urdf /path/to/your/robot.urdf
```

### Using xacro for preprocessing
```bash
# If using xacro, check the preprocessed URDF
ros2 run xacro xacro /path/to/your/robot.xacro --check-order
```

## Visualizing URDF Models

### Using RViz
```xml
<!-- Add this to your URDF for better visualization -->
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
</gazebo>
```

### Launch file to visualize:
```xml
<!-- robot_visualize.launch.py -->
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('your_package'),
        'urdf',
        'your_robot.urdf.xacro'
    )
    
    robot_description_content = Command(['xacro ', robot_description_path])
    
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('your_package'), 'rviz', 'urdf_config.rviz')]
        )
    ])
```

## Common URDF Best Practices

### 1. Consistent Naming Convention
```xml
<!-- Good naming convention -->
<link name="base_link"/>
<link name="left_leg_upper"/>
<link name="left_leg_lower"/>
<joint name="left_knee_joint"/>
```

### 2. Proper Inertial Properties
```xml
<!-- Calculate realistic inertial values -->
<inertial>
  <mass value="0.1"/>  <!-- Use realistic masses -->
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

### 3. Appropriate Joint Limits
```xml
<!-- Set realistic joint limits -->
<limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
```

## Troubleshooting Common URDF Issues

### 1. Missing Joint Origins
```xml
<!-- WRONG -->
<joint name="missing_origin" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>

<!-- CORRECT -->
<joint name="proper_origin" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Always specify origin -->
  <axis xyz="0 0 1"/>
</joint>
```

### 2. Incorrect Mass Values
- Mass should be positive
- Mass values should be realistic for the robot
- Very small masses can cause simulation instability

### 3. Inconsistent Joint Types
- Make sure joint limits are appropriate for the joint type
- Continuous joints shouldn't have limits
- Fixed joints shouldn't have axes

## Using ROS 2 Tools for URDF Inspection

### 1. View URDF in Tree Format
```bash
# Install and use urdf_to_graphiz
sudo apt-get install ros-humble-urdfdom-tools
showmechanism.py your_robot.urdf
```

### 2. Validate URDF Structure
```python
# Python script to validate URDF
import xml.etree.ElementTree as ET
import os

def validate_urdf(urdf_path):
    try:
        # Parse the URDF file
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # Basic checks
        if root.tag != 'robot':
            raise ValueError("Root element must be 'robot'")
        
        if 'name' not in root.attrib:
            raise ValueError("Robot element must have 'name' attribute")
        
        print(f"URDF {root.attrib['name']} validated successfully")
        return True
        
    except ET.ParseError as e:
        print(f"XML parsing error: {e}")
        return False
    except Exception as e:
        print(f"Validation error: {e}")
        return False
```

## Lesson Summary

In this lesson, you've learned the fundamentals of URDF:
- Basic structure and XML elements
- Links and joints definitions
- Inertial, visual, and collision properties
- Coordinate systems and origins
- Validation and visualization techniques
- Best practices for URDF development

## Exercises

1. Create a simple URDF for a 2-link planar manipulator with revolute joints.
2. Define a mobile robot with 4 wheels using fixed joints.
3. Add materials and colors to your URDF for better visualization.
4. Validate your URDF using the check_urdf command and fix any errors.

## Next Steps

In the next lesson, we'll explore modeling humanoid kinematics, which involves more complex joint arrangements and link structures specific to bipedal robots.