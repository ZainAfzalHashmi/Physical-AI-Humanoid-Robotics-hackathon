---
sidebar_label: 'Chapter 3: Humanoid URDF - Robot Description for Humanoid Systems'
sidebar_position: 4
---

# Chapter 3: Humanoid URDF - Robot Description for Humanoid Systems

## Overview
This chapter introduces readers to URDF (Unified Robot Description Format) and Xacro (XML Macros) for describing humanoid robots. Readers will learn how to define robot geometry, kinematics, and dynamics using proper link and joint definitions, with a focus on creating a complete 3-DOF limb example.

## Learning Objectives
- Understand the structure and purpose of URDF files
- Learn how to use Xacro to simplify complex robot descriptions
- Define links with proper visual, collision, and inertial properties
- Create joint definitions with appropriate limits and dynamics
- Build a complete, commented 3-DOF limb URDF example
- Validate URDF files in simulation environments

## Content Structure

### Section 3.1: Introduction to URDF and Xacro
- What is URDF and why it's important for robotics
- The relationship between URDF and robot simulation
- Introduction to Xacro for reducing redundancy
- URDF vs other robot description formats

### Section 3.2: Links and Their Properties
- Understanding link elements in URDF
- Visual properties: geometry, material, origin
- Collision properties: geometry, origin
- Inertial properties: mass, inertia matrix
- Proper scaling and units for humanoid robots

### Section 3.3: Joint Definitions
- Different joint types: revolute, continuous, prismatic, fixed, etc.
- Joint limits: position, velocity, effort
- Joint dynamics: damping, friction
- Joint origins and transformations

### Section 3.4: Xacro Macros and Best Practices
- Creating reusable components with Xacro
- Parameterizing robot descriptions
- Including external files and libraries
- Organizing complex robot models

### Section 3.5: Complete 3-DOF Limb Example
- Step-by-step construction of a humanoid limb
- Proper naming conventions
- Validation and testing in simulation
- Extending the example to more complex structures

## Code Examples

### Example 3.1: Basic URDF Structure
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_limb" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First joint -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- First link -->
  <link name="link_1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

### Example 3.2: Complete 3-DOF Limb with Xacro
```xml
<?xml version="1.0"?>
<robot name="3dof_humanoid_limb" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="link_length" value="0.2" />
  <xacro:property name="link_radius" value="0.025" />
  <xacro:property name="joint_damping" value="0.5" />
  <xacro:property name="joint_friction" value="0.1" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="dark_grey">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  <material name="light_grey">
    <color rgba="0.8 0.8 0.8 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Macro for creating a simple link -->
  <xacro:macro name="simple_link" params="name xyz_length xyz_radius parent_joint_origin_xyz parent_joint_origin_rpy joint_axis joint_lower joint_upper joint_effort joint_velocity mass ixx iyy izz">

    <!-- Joint definition -->
    <joint name="${name}_joint" type="revolute">
      <parent link="${name}_parent"/>
      <child link="${name}_link"/>
      <origin xyz="${parent_joint_origin_xyz}" rpy="${parent_joint_origin_rpy}"/>
      <axis xyz="${joint_axis}"/>
      <limit lower="${joint_lower}" upper="${joint_upper}" effort="${joint_effort}" velocity="${joint_velocity}"/>
      <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
    </joint>

    <!-- Link definition -->
    <link name="${name}_link">
      <!-- Visual properties -->
      <visual>
        <origin xyz="0 0 ${xyz_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${xyz_length}" radius="${xyz_radius}"/>
        </geometry>
        <material name="orange"/>
      </visual>

      <!-- Collision properties -->
      <collision>
        <origin xyz="0 0 ${xyz_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${xyz_length}" radius="${xyz_radius}"/>
        </geometry>
      </collision>

      <!-- Inertial properties -->
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 ${xyz_length/2}" rpy="0 0 0"/>
        <inertia ixx="${ixx}" ixy="0.0" ixz="0.0" iyy="${iyy}" iyz="0.0" izz="${izz}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- First DOF: Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="1.0"/>
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0002" ixy="0.0" ixz="0.0" iyy="0.0002" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Second DOF: Elbow Joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100.0" velocity="1.0"/>
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder length="${link_length}" radius="${link_radius}"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder length="${link_length}" radius="${link_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Third DOF: Wrist Joint -->
  <joint name="wrist_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 ${link_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="50.0" velocity="1.0"/>
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
  </joint>

  <link name="forearm_link">
    <visual>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder length="${link_length}" radius="${link_radius}"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder length="${link_length}" radius="${link_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.0003"/>
    </inertial>
  </link>

  <!-- End effector (hand) -->
  <joint name="hand_joint" type="fixed">
    <parent link="forearm_link"/>
    <child link="hand_link"/>
    <origin xyz="0 0 ${link_length}" rpy="0 0 0"/>
  </joint>

  <link name="hand_link">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Transmission for ros_control -->
  <transmission name="shoulder_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="shoulder_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="elbow_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="elbow_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="elbow_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="wrist_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wrist_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="wrist_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/3dof_limb</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

### Example 3.3: URDF Validation and Testing Script
```python
#!/usr/bin/env python3
"""
URDF Validation and Testing Script
This script helps validate URDF files and test their properties
"""

import xml.etree.ElementTree as ET
import math

def validate_urdf(urdf_file_path):
    """
    Validate a URDF file for basic structural correctness
    """
    try:
        # Parse the URDF file
        tree = ET.parse(urdf_file_path)
        root = tree.getroot()

        # Check if it's a valid robot file
        if root.tag != 'robot':
            raise ValueError("Root element is not 'robot'")

        robot_name = root.attrib.get('name', 'unnamed')
        print(f"Validating robot: {robot_name}")

        # Find all links and joints
        links = root.findall('link')
        joints = root.findall('joint')

        print(f"Found {len(links)} links and {len(joints)} joints")

        # Check for common issues
        link_names = [link.attrib['name'] for link in links]
        joint_names = [joint.attrib['name'] for joint in joints]

        # Check for duplicate names
        if len(link_names) != len(set(link_names)):
            print("WARNING: Duplicate link names found")

        if len(joint_names) != len(set(joint_names)):
            print("WARNING: Duplicate joint names found")

        # Check joint connections
        for joint in joints:
            parent = joint.find('parent')
            child = joint.find('child')

            if parent is not None and child is not None:
                parent_name = parent.attrib['link']
                child_name = child.attrib['link']

                if parent_name not in link_names:
                    print(f"ERROR: Joint {joint.attrib['name']} references non-existent parent link: {parent_name}")

                if child_name not in link_names:
                    print(f"ERROR: Joint {joint.attrib['name']} references non-existent child link: {child_name}")

        print("URDF validation completed")
        return True

    except ET.ParseError as e:
        print(f"XML Parse Error: {e}")
        return False
    except Exception as e:
        print(f"Validation Error: {e}")
        return False

def analyze_urdf(urdf_file_path):
    """
    Analyze URDF file for kinematic properties
    """
    try:
        tree = ET.parse(urdf_file_path)
        root = tree.getroot()

        # Count different joint types
        joint_types = {}
        joints = root.findall('joint')

        for joint in joints:
            joint_type = joint.attrib.get('type', 'unknown')
            joint_types[joint_type] = joint_types.get(joint_type, 0) + 1

        print("Joint Type Analysis:")
        for jtype, count in joint_types.items():
            print(f"  {jtype}: {count}")

        # Analyze joint limits
        revolute_joints = [j for j in joints if j.attrib.get('type') == 'revolute']
        print(f"\nRevolute Joints: {len(revolute_joints)}")

        for joint in revolute_joints:
            limit = joint.find('limit')
            if limit is not None:
                lower = float(limit.attrib.get('lower', 0))
                upper = float(limit.attrib.get('upper', 0))
                range_val = upper - lower
                print(f"  {joint.attrib['name']}: range = {range_val:.3f} rad ({math.degrees(range_val):.1f}°)")

    except Exception as e:
        print(f"Analysis Error: {e}")

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: python urdf_validator.py <urdf_file_path>")
        return

    urdf_file = sys.argv[1]

    print("=== URDF Validation ===")
    if validate_urdf(urdf_file):
        print("Validation passed!")
    else:
        print("Validation failed!")

    print("\n=== URDF Analysis ===")
    analyze_urdf(urdf_file)

if __name__ == '__main__':
    main()
```

### Example 3.4: URDF Launch File for RViz Visualization
```xml
<launch>
  <!-- Load the URDF file -->
  <param name="robot_description" command="xacro $(find-pkg-share 3dof_humanoid_limb)/urdf/3dof_limb.urdf.xacro" />

  <!-- Start the robot state publisher -->
  <node pkg="robot_state_publisher" executable="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" value="50.0"/>
  </node>

  <!-- Start RViz with a configuration for viewing the robot -->
  <node pkg="rviz2" executable="rviz2" name="rviz2" args="-d $(find-pkg-share 3dof_humanoid_limb)/rviz/robot_description.rviz">
  </node>

  <!-- Optional: Start joint state publisher for manual control -->
  <node pkg="joint_state_publisher_gui" executable="joint_state_publisher_gui" name="joint_state_publisher_gui">
  </node>
</launch>
```

## Hands-on Exercises

### Exercise 3.1: Create a Simple 2-DOF Arm
Create a URDF file for a simple 2-DOF arm with revolute joints, including proper visual, collision, and inertial properties.

### Exercise 3.2: Extend the 3-DOF Limb
Extend the provided 3-DOF limb example by adding a 4th DOF at the wrist for additional rotation.

### Exercise 3.3: Create a Leg Structure
Design a URDF for a humanoid leg with hip, knee, and ankle joints, following the same principles as the arm example.

## Assessment Criteria
- Students can create valid URDF files with proper link and joint definitions
- Students understand the purpose of visual, collision, and inertial properties
- Students can use Xacro to create parameterized and reusable robot components
- Students can validate URDF files for structural correctness
- Students can visualize their URDF models in RViz
- Students understand the relationship between URDF and robot simulation