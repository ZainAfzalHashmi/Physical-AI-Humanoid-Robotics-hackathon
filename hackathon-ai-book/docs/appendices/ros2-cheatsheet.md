# ROS 2 Cheatsheet

This appendix contains quick reference information for common ROS 2 commands and concepts.

## Essential Commands
- `ros2 node list` - List all active nodes
- `ros2 topic list` - List all active topics
- `ros2 service list` - List all available services
- `ros2 run <package> <executable>` - Run a specific executable
- `ros2 launch <package> <launch_file>` - Run a launch file

## Common Message Types
- `std_msgs/msg/String` - String data
- `std_msgs/msg/Int32` - 32-bit integer
- `geometry_msgs/msg/Twist` - Linear and angular velocities
- `sensor_msgs/msg/Image` - Image data

## Quality of Service Settings
- Reliability: RELIABLE or BEST_EFFORT
- Durability: VOLATILE or TRANSIENT_LOCAL
- History: KEEP_LAST or KEEP_ALL