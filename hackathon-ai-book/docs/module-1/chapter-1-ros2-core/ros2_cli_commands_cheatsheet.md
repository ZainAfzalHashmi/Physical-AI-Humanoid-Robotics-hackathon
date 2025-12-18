# Essential ROS 2 CLI Commands Reference

## Node Commands
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info <node_name>

# Get detailed information about a node's publications, subscriptions, services, etc.
ros2 node info /minimal_publisher
```

## Topic Commands
```bash
# List all active topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Publish a message to a topic directly from command line
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello World'"

# Get information about a topic
ros2 topic info /topic_name

# Show the type of a topic
ros2 topic type /topic_name

# List the publishers of a topic
ros2 topic publishers /topic_name

# List the subscribers of a topic
ros2 topic subscribers /topic_name
```

## Service Commands
```bash
# List all available services
ros2 service list

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Find all services of a specific type
ros2 service find example_interfaces/srv/AddTwoInts

# Show the type of a service
ros2 service type /add_two_ints
```

## Parameter Commands
```bash
# List all parameters of a node
ros2 param list /node_name

# Get a parameter value
ros2 param get /node_name param_name

# Set a parameter value
ros2 param set /node_name param_name value

# Dump all parameters to a file
ros2 param dump /node_name
```

## Action Commands (Bonus)
```bash
# List all available actions
ros2 action list

# Send a goal to an action
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

## Package and Launch Commands
```bash
# List all packages
ros2 pkg list

# Create a new package
ros2 pkg create --build-type ament_python my_robot_package

# Run a launch file
ros2 launch package_name launch_file.py

# Run a specific executable
ros2 run package_name executable_name
```

## Common Message Types
```bash
# Standard message types you'll encounter
std_msgs/msg/String
std_msgs/msg/Int32
std_msgs/msg/Float64
geometry_msgs/msg/Point
geometry_msgs/msg/Pose
geometry_msgs/msg/Twist
sensor_msgs/msg/Image
sensor_msgs/msg/LaserScan
```

## Practical Examples
```bash
# Monitor a temperature topic
ros2 topic echo /temperature std_msgs/msg/Float64

# Publish a velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# Check if a service is available
ros2 service list | grep service_name
```