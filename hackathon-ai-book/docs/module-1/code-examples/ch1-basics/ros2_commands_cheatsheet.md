# ROS 2 Commands Cheatsheet

## Node Commands
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info <node_name>
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
```

## Service Commands
```bash
# List all available services
ros2 service list

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Find all services of a specific type
ros2 service find example_interfaces/srv/AddTwoInts
```

## Parameter Commands
```bash
# List all parameters of a node
ros2 param list /node_name

# Get a parameter value
ros2 param get /node_name param_name

# Set a parameter value
ros2 param set /node_name param_name value
```

## Package and Launch Commands
```bash
# List all packages
ros2 pkg list

# Run a launch file
ros2 launch package_name launch_file.py

# Run a specific executable
ros2 run package_name executable_name
```