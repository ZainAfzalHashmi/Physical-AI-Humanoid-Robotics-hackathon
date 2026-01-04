# Lesson 4: Performance Optimization and Best Practices

## Learning Objectives
By the end of this lesson, you will be able to:
- Optimize Python code for real-time robotics applications
- Implement efficient memory management in ROS 2 nodes
- Use profiling tools to identify and resolve performance bottlenecks
- Apply best practices for production ROS 2 deployments
- Design scalable and maintainable Python-based robotics systems

## Introduction to Performance Optimization

Performance optimization in robotics applications is crucial, especially for real-time systems where delays can lead to poor robot behavior or safety issues. Unlike traditional applications, robotics systems often require consistent timing guarantees, efficient resource utilization, and predictable behavior under varying loads.

### Performance Metrics in Robotics
- **Latency**: Time delay between input and response (critical for control loops)
- **Throughput**: Number of operations per second (important for sensor processing)
- **Jitter**: Variation in latency (important for smooth control)
- **CPU and Memory Usage**: Resource consumption that affects system stability
- **Real-time Performance**: Predictable response times under all conditions

## Python-Specific Optimization Strategies

### 1. Algorithmic Improvements

Efficient algorithms are the foundation of performance optimization. Let's explore several optimization techniques:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import numba
from typing import List, Tuple


class OptimizedPointCloudProcessor(Node):
    def __init__(self):
        super().__init__('optimized_pointcloud_processor')
        
        # Subscriber for point cloud data
        self.pc_sub = self.create_subscription(
            PointCloud2,
            'pointcloud_raw',
            self.optimized_pc_callback,
            10
        )
        
        # Publisher for processed data
        self.pc_pub = self.create_publisher(
            PointCloud2,
            'pointcloud_filtered',
            10
        )
        
        # Cache for optimization
        self.point_cache = {}
        self.cache_size_limit = 1000

    def optimized_pc_callback(self, msg):
        """Optimized callback using algorithmic improvements"""
        start_time = self.get_clock().now().nanoseconds
        
        # Convert PointCloud2 to NumPy array efficiently
        points = self.convert_pointcloud_optimized(msg)
        
        # Apply optimized filtering algorithm
        filtered_points = self.filter_ground_plane_optimized(points)
        
        # Publish results
        filtered_msg = self.array_to_pointcloud2(filtered_points, msg.header)
        self.pc_pub.publish(filtered_msg)
        
        end_time = self.get_clock().now().nanoseconds
        callback_time = (end_time - start_time) / 1_000_000  # Convert to milliseconds
        self.get_logger().debug(f'Optimized callback took {callback_time:.2f}ms')

    def convert_pointcloud_optimized(self, msg):
        """Optimized conversion using NumPy vectorization"""
        # Use NumPy for efficient data conversion
        data = np.frombuffer(msg.data, dtype=np.float32)
        
        # Reshape based on point stride (assumes x, y, z layout)
        points = data.reshape(-1, msg.point_step // 4)[:, :3]  # Only x, y, z
        
        return points

    def filter_ground_plane_optimized(self, points):
        """Efficient ground plane filtering using vectorization"""
        # Use NumPy boolean indexing for fast filtering
        mask = points[:, 2] > 0.1  # Only points above 10cm (vectorized operation)
        return points[mask]

    @staticmethod
    @numba.jit(nopython=True)
    def fast_distance_calculation(point1, point2):
        """High-performance distance calculation using Numba"""
        return ((point1[0] - point2[0])**2 + 
                (point1[1] - point2[1])**2 + 
                (point1[2] - point2[2])**2)**0.5


def main(args=None):
    rclpy.init(args=args)
    processor = OptimizedPointCloudProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Memory Management

Memory efficiency is crucial for long-running robotics applications:

```python
import gc
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import weakref


class MemoryEfficientNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_node')
        
        # Use deque for efficient memory management
        self.message_history = deque(maxlen=100)
        
        # Publisher and subscriber
        self.pub = self.create_publisher(String, 'optimized_topic', 10)
        self.sub = self.create_subscription(
            String,
            'input_topic',
            self.memory_managed_callback,
            10
        )
        
        # Timer for periodic cleanup
        self.cleanup_timer = self.create_timer(60.0, self.periodic_cleanup)
        
        # Reference to large objects that may need cleanup
        self.large_data_structures = {}

    def memory_managed_callback(self, msg):
        """Process message with attention to memory usage"""
        # Add to history
        self.message_history.append(msg)
        
        # Process message
        processed_data = self.process_message_optimized(msg.data)
        
        # Publish result
        result_msg = String()
        result_msg.data = processed_data
        self.pub.publish(result_msg)
        
        # Monitor memory usage
        if len(self.message_history) % 100 == 0:  # Every 100 messages
            self.log_memory_usage()

    def process_message_optimized(self, input_string: str) -> str:
        """Optimized message processing avoiding unnecessary copies"""
        # Use generator expressions instead of list comprehensions when possible
        # Process in-place when possible
        words = input_string.split()
        
        # Use join instead of concatenation in loops
        processed_words = [word.upper() for word in words if len(word) > 1]
        return ' '.join(processed_words)

    def log_memory_usage(self):
        """Log memory usage information"""
        import psutil
        import os
        
        process = psutil.Process(os.getpid())
        memory_mb = process.memory_info().rss / 1024 / 1024
        self.get_logger().info(f'Current memory usage: {memory_mb:.2f} MB')
        
        # Log object counts
        obj_counts = {}
        for obj in gc.get_objects():
            obj_type = type(obj).__name__
            obj_counts[obj_type] = obj_counts.get(obj_type, 0) + 1
        
        # Note: This is just for debugging; don't do this in production
        # as it can be expensive
        total_objects = sum(obj_counts.values())
        self.get_logger().debug(f'Total objects: {total_objects}')
        
        # Check for potential memory leaks
        if memory_mb > 100:  # 100MB threshold
            self.get_logger().warn('High memory usage detected')

    def periodic_cleanup(self):
        """Periodic cleanup operations"""
        # Force garbage collection
        collected = gc.collect()
        self.get_logger().debug(f'Garbage collected {collected} objects')
        
        # Clean up the message history if needed
        # (in practice, the deque already handles this with maxlen)
        if len(self.message_history) == self.message_history.maxlen:
            self.get_logger().debug('Message history at maximum capacity')


def main(args=None):
    rclpy.init(args=args)
    node = MemoryEfficientNode()
    
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

### 3. Using Compiled Extensions

For computationally intensive operations, compiled extensions provide significant performance benefits:

```python
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import cKDTree
from geometry_msgs.msg import Point
import time


class OptimizedPathPlanner(Node):
    def __init__(self):
        super().__init__('optimized_path_planner')
        
        # Initialize with some obstacle points
        self.obstacles = np.array([
            [1.0, 1.0],
            [2.0, 2.0],
            [3.0, 1.0],
            [1.5, 3.0]
        ])
        
        # Build KD-tree for fast nearest neighbor search
        self.obstacle_tree = cKDTree(self.obstacles)
        
        # Timer for path planning
        self.planning_timer = self.create_timer(0.1, self.optimized_planning_loop)

    def optimized_planning_loop(self):
        """Optimized path planning with efficient algorithms"""
        start_time = self.get_clock().now().nanoseconds
        
        # Example: Find closest obstacle to a point
        query_point = np.array([0.5, 0.5])
        
        # Efficient nearest neighbor search using KD-tree (O(log n))
        distance, index = self.obstacle_tree.query(query_point)
        
        end_time = self.get_clock().now().nanoseconds
        planning_time = (end_time - start_time) / 1_000_000  # ms
        
        if distance < 0.5:  # Obstacle too close
            self.get_logger().warn(
                f'Obstacle at {self.obstacles[index]} is {distance:.2f}m away'
            )
        else:
            self.get_logger().debug(f'Closest obstacle {distance:.2f}m away')
        
        self.get_logger().debug(f'Planning took {planning_time:.3f}ms')

    def optimized_astar_pathfinding(self, start, goal, grid_resolution=0.1):
        """Optimized A* pathfinding using heapq instead of full implementation"""
        import heapq
        
        # This is a simplified example - a full implementation would be more complex
        # Calculate grid indices
        start_idx = (int(start[0] / grid_resolution), int(start[1] / grid_resolution))
        goal_idx = (int(goal[0] / grid_resolution), int(goal[1] / grid_resolution))
        
        # Use heapq for priority queue (more efficient than sorted list)
        frontier = [(0.0, start_idx)]  # (cost, position)
        came_from = {start_idx: None}
        cost_so_far = {start_idx: 0.0}
        
        # This is a simplified version - real implementation would check grid cells
        # and handle obstacles appropriately
        self.get_logger().info(f'Optimized pathfinding from {start_idx} to {goal_idx} started')
        
        return [[start[0], start[1]], [goal[0], goal[1]]]  # Simplified result


def main(args=None):
    rclpy.init(args=args)
    planner = OptimizedPathPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Concurrency and Parallel Processing

### 1. Threading for I/O Bound Operations

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
import threading
import time
from concurrent.futures import ThreadPoolExecutor


class ConcurrentNode(Node):
    def __init__(self):
        super().__init__('concurrent_node')
        
        # Use mutually exclusive callback groups for thread safety
        self.cbg_io = MutuallyExclusiveCallbackGroup()
        self.cbg_processing = MutuallyExclusiveCallbackGroup()
        
        # Publishers and subscribers
        self.input_sub = self.create_subscription(
            String,
            'input_topic',
            self.concurrent_input_callback,
            10
        )
        
        self.result_pub = self.create_publisher(String, 'result_topic', 10)
        
        # Thread pool for I/O operations
        self.io_pool = ThreadPoolExecutor(max_workers=4)
        self.processor_pool = ThreadPoolExecutor(max_workers=2)
        
        # Shared state with locks
        self.state_lock = threading.Lock()
        self.shared_state = {'last_processed': 0}

    def concurrent_input_callback(self, msg):
        """Handle input with concurrency"""
        # Submit I/O-bound tasks to thread pool
        future = self.io_pool.submit(self.process_io_intensive_task, msg.data)
        future.add_done_callback(self.on_io_complete)

    def process_io_intensive_task(self, input_data):
        """Simulate I/O intensive task that can run in a thread"""
        # Simulate database lookup, file operation, etc.
        time.sleep(0.1)  # Simulate I/O delay
        
        # Process the data
        processed = input_data.upper()
        
        # Simulate more I/O
        time.sleep(0.05)
        
        return processed

    def on_io_complete(self, future):
        """Callback when I/O task is complete"""
        try:
            result = future.result()
            
            # Submit CPU-intensive task to processor pool
            processing_future = self.processor_pool.submit(
                self.process_cpu_intensive_task, 
                result
            )
            processing_future.add_done_callback(self.on_processing_complete)
            
        except Exception as e:
            self.get_logger().error(f'IO task failed: {str(e)}')

    def process_cpu_intensive_task(self, input_data):
        """CPU-intensive task that benefits from parallel processing"""
        # Simulate CPU-intensive processing
        result = input_data.lower()
        
        # Update shared state safely
        with self.state_lock:
            self.shared_state['last_processed'] = self.get_clock().now().nanoseconds
        
        return result

    def on_processing_complete(self, future):
        """Callback when processing is complete"""
        try:
            result = future.result()
            
            # Publish result
            output_msg = String()
            output_msg.data = f'Processed: {result}'
            self.result_pub.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'Processing task failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ConcurrentNode()
    
    # Use multi-threaded executor to handle concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.io_pool.shutdown(wait=True)
        node.processor_pool.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Using asyncio for Asynchronous Operations

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
import asyncio
from std_msgs.msg import String


class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')
        
        # Publisher
        self.pub = self.create_publisher(String, 'async_topic', 10)
        
        # Timer to trigger async operations
        self.async_timer = self.create_timer(2.0, self.trigger_async_operations)

    async def async_operation(self, operation_id: int):
        """Simulate an asynchronous operation"""
        self.get_logger().info(f'Starting async operation {operation_id}')
        
        # Simulate async I/O
        await asyncio.sleep(1.0)
        
        # Perform some computation
        result = operation_id * operation_id
        
        self.get_logger().info(f'Completed async operation {operation_id}: {result}')
        
        # Publish result
        msg = String()
        msg.data = f'Async result {operation_id}: {result}'
        self.pub.publish(msg)
        
        return result

    async def concurrent_operations(self):
        """Run multiple async operations concurrently"""
        tasks = [
            self.async_operation(i) for i in range(3)
        ]
        
        results = await asyncio.gather(*tasks)
        self.get_logger().info(f'All async operations completed: {results}')

    def trigger_async_operations(self):
        """Trigger async operations from timer callback"""
        # Run the async function in the event loop
        asyncio.create_task(self.concurrent_operations())


def run_async_node():
    """Run the async node with asyncio event loop"""
    rclpy.init()
    node = AsyncNode()
    
    # Create ROS executor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Create asyncio event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        # Run both ROS and asyncio loops
        loop.run_in_executor(None, executor.spin)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == '__main__':
    run_async_node()
```

## Profiling and Monitoring

### 1. Performance Profiling

```python
import rclpy
from rclpy.node import Node
import cProfile
import pstats
import io
import pstats
from contextlib import contextmanager
from functools import wraps
import time


class ProfilingNode(Node):
    def __init__(self):
        super().__init__('profiling_node')
        
        # Start profiling
        self.profile = cProfile.Profile()
        
        # Timer for profiling callback
        self.profiling_timer = self.create_timer(10.0, self.report_profiling_results)
        
        # Counter for performance tracking
        self.callback_count = 0
        self.total_execution_time = 0.0
        self.min_execution_time = float('inf')
        self.max_execution_time = 0.0

    @contextmanager
    def performance_context(self, operation_name):
        """Context manager for measuring performance"""
        start_time = time.perf_counter()
        try:
            yield
        finally:
            end_time = time.perf_counter()
            duration_ms = (end_time - start_time) * 1000
            
            # Update statistics
            self.total_execution_time += duration_ms
            self.min_execution_time = min(self.min_execution_time, duration_ms)
            self.max_execution_time = max(self.max_execution_time, duration_ms)
            self.callback_count += 1
            
            self.get_logger().debug(f'{operation_name} took {duration_ms:.3f}ms')

    def performance_decorator(self, func_name):
        """Decorator for measuring function performance"""
        def decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                with self.performance_context(func_name):
                    return func(*args, **kwargs)
            return wrapper
        return decorator

    @performance_decorator('main_callback')
    def example_callback(self):
        """Example callback to demonstrate profiling"""
        # Simulate some work
        total = 0
        for i in range(1000):
            total += i * 2
        return total

    def profiled_operation(self):
        """Operation that will be profiled"""
        # Start profiling if not already
        if not self.profile.getstats():
            self.profile.enable()
        
        # Perform some operations
        self.example_callback()
        time.sleep(0.01)  # Simulate delay
        self.example_callback()

    def report_profiling_results(self):
        """Report profiling statistics"""
        self.profile.disable()
        
        # Get profiling stats
        s = io.StringIO()
        ps = pstats.Stats(self.profile, stream=s)
        ps.sort_stats('cumulative')
        ps.print_stats(10)  # Top 10 functions
        
        profile_output = s.getvalue()
        self.get_logger().info('=== PROFILING RESULTS ===')
        self.get_logger().info(profile_output)
        
        # Report custom statistics
        if self.callback_count > 0:
            avg_time = self.total_execution_time / self.callback_count
            self.get_logger().info(
                f'Performance Stats: '
                f'Calls={self.callback_count}, '
                f'Avg={avg_time:.3f}ms, '
                f'Min={self.min_execution_time:.3f}ms, '
                f'Max={self.max_execution_time:.3f}ms'
            )
        
        # Reset for next profiling cycle
        self.profile = cProfile.Profile()
        self.profile.enable()
        
        # Reset counters
        self.callback_count = 0
        self.total_execution_time = 0.0
        self.min_execution_time = float('inf')
        self.max_execution_time = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ProfilingNode()
    
    # Run profiling operations
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

### 2. Real-Time Performance Monitoring

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
import time
import threading
import collections


class RealtimeMonitoringNode(Node):
    def __init__(self):
        super().__init__('realtime_monitoring_node')
        
        # Publishers for monitoring data
        self.latency_pub = self.create_publisher(Float32, 'callback_latency', 10)
        self.jitter_pub = self.create_publisher(Float32, 'callback_jitter', 10)
        
        # Timing history for jitter calculation
        self.execution_times = collections.deque(maxlen=100)
        self.interval_times = collections.deque(maxlen=100)
        
        # Monitor timer (runs at 10Hz)
        self.monitor_timer = self.create_timer(0.1, self.monitored_callback)
        
        # Statistics
        self.last_execution_time = 0.0
        self.period_duration = 0.1  # 10Hz = 0.1s period

    def monitored_callback(self):
        """Callback with real-time performance monitoring"""
        start_time = time.perf_counter()
        
        # Do actual work here
        self.do_work()
        
        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000  # Convert to ms
        
        # Track execution time for jitter calculation
        self.execution_times.append(execution_time)
        
        # Calculate and publish metrics
        self.publish_performance_metrics(execution_time)

    def do_work(self):
        """The actual work to be monitored"""
        # Simulate processing work
        total = 0
        for i in range(10000):
            total += i * 0.001
        return total

    def publish_performance_metrics(self, execution_time):
        """Publish performance metrics"""
        # Publish latency (current execution time)
        latency_msg = Float32()
        latency_msg.data = execution_time
        self.latency_pub.publish(latency_msg)
        
        # Calculate jitter (variability in execution time)
        if len(self.execution_times) > 1:
            execution_times_list = list(self.execution_times)
            mean_exec_time = sum(execution_times_list) / len(execution_times_list)
            variance = sum((t - mean_exec_time) ** 2 for t in execution_times_list) / len(execution_times_list)
            jitter = variance ** 0.5  # Standard deviation
            
            # Publish jitter
            jitter_msg = Float32()
            jitter_msg.data = jitter
            self.jitter_pub.publish(jitter_msg)
            
            # Warn if jitter exceeds threshold
            if jitter > 5.0:  # 5ms threshold
                self.get_logger().warn(f'High jitter detected: {jitter:.3f}ms')

    def get_performance_summary(self):
        """Get a summary of performance metrics"""
        if not self.execution_times:
            return "No performance data available"
        
        times = list(self.execution_times)
        mean_time = sum(times) / len(times)
        min_time = min(times)
        max_time = max(times)
        std_dev = (sum((t - mean_time) ** 2 for t in times) / len(times)) ** 0.5
        
        return (
            f"Mean: {mean_time:.3f}ms, "
            f"Min: {min_time:.3f}ms, "
            f"Max: {max_time:.3f}ms, "
            f"StdDev: {std_dev:.3f}ms"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeMonitoringNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print final performance summary
        print("\nFinal Performance Summary:")
        print(node.get_performance_summary())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices for Production Deployments

### 1. Code Quality and Maintainability

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange
from std_msgs.msg import String
from typing import Optional
import logging


class BestPracticeNode(Node):
    """
    A ROS 2 node demonstrating best practices for production deployments.
    
    This node follows these best practices:
    - Comprehensive parameter validation
    - Proper error handling
    - Type hints and documentation
    - Resource management
    - Configuration flexibility
    """
    
    def __init__(self):
        super().__init__('best_practice_node')
        
        # Validate and declare parameters with descriptions and ranges
        self.declare_parameters_with_descriptions()
        
        # Initialize publishers and subscribers
        self.setup_communication_interfaces()
        
        # Initialize timers and scheduling
        self.setup_timers()
        
        # Initialize internal state
        self.initialize_state()
        
        self.get_logger().info('Best Practice Node initialized successfully')

    def declare_parameters_with_descriptions(self):
        """Declare parameters with descriptions and validation ranges."""
        # Define parameter descriptors with ranges and descriptions
        int_descriptor = ParameterDescriptor(
            description='Processing frequency in Hz',
            integer_range=[IntegerRange(from_value=1, to_value=100, step=1)]
        )
        
        float_descriptor = ParameterDescriptor(
            description='Distance threshold in meters',
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=10.0, step=0.01)]
        )
        
        string_descriptor = ParameterDescriptor(
            description='Node operating mode'
        )
        
        # Declare parameters with defaults and descriptors
        self.declare_parameter('processing_frequency', 10, descriptor=int_descriptor)
        self.declare_parameter('distance_threshold', 2.0, descriptor=float_descriptor)
        self.declare_parameter('operating_mode', 'normal', descriptor=string_descriptor)
        
        # Get parameters with type validation
        try:
            self.processing_freq = self.get_parameter('processing_frequency').get_parameter_value().integer_value
            self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
            self.operating_mode = self.get_parameter('operating_mode').get_parameter_value().string_value
        except Exception as e:
            self.get_logger().fatal(f'Failed to initialize parameters: {str(e)}')
            raise
        
        # Validate parameter values
        if self.processing_freq <= 0:
            raise ValueError(f'Processing frequency must be positive, got {self.processing_freq}')
        if self.distance_threshold <= 0:
            raise ValueError(f'Distance threshold must be positive, got {self.distance_threshold}')
        
        # Log parameter values
        self.get_logger().info(
            f'Initialized with: freq={self.processing_freq}Hz, '
            f'threshold={self.distance_threshold}m, mode={self.operating_mode}'
        )

    def setup_communication_interfaces(self):
        """Setup all publishers and subscribers."""
        # Create QoS profiles
        reliable_qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'status', reliable_qos)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            reliable_qos
        )
        
        self.get_logger().info('Communication interfaces setup completed')

    def setup_timers(self):
        """Setup timer for periodic operations."""
        if hasattr(self, 'processing_freq'):
            timer_period = 1.0 / self.processing_freq
            self.main_timer = self.create_timer(timer_period, self.main_processing_callback)
            self.get_logger().info(f'Main timer created with period {timer_period:.3f}s')

    def initialize_state(self):
        """Initialize internal state variables."""
        self.is_running = True
        self.error_count = 0
        self.success_count = 0
        self.last_error_time = None

    def command_callback(self, msg: String):
        """
        Handle incoming commands.
        
        Args:
            msg: Command message with command string
        """
        try:
            command = msg.data.strip().lower()
            
            if command == 'start':
                self.is_running = True
                self.get_logger().info('Started processing')
            elif command == 'stop':
                self.is_running = False
                self.get_logger().info('Stopped processing')
            elif command == 'reset':
                self.reset_state()
                self.get_logger().info('State reset completed')
            else:
                self.get_logger().warn(f'Unknown command: {command}')
                
        except Exception as e:
            self.handle_error(f'Command callback error: {str(e)}')

    def main_processing_callback(self):
        """
        Main processing callback called periodically.
        """
        if not self.is_running:
            return
            
        try:
            # Perform main processing
            result = self.perform_main_processing()
            
            if result is not None:
                self.success_count += 1
                self.publish_status(f'Success: {result}')
            else:
                self.get_logger().warn('Processing returned None')
                
        except Exception as e:
            self.handle_error(f'Main processing error: {str(e)}')

    def perform_main_processing(self) -> Optional[str]:
        """
        Perform the main processing work.
        
        Returns:
            Processing result or None if no result generated
        """
        # Simulate processing work
        import random
        
        # Check operating mode
        if self.operating_mode == 'test':
            # In test mode, simulate occasional failures
            if random.random() < 0.1:  # 10% chance of failure
                raise RuntimeError('Simulated processing failure in test mode')
        
        # Simulate processing
        if random.random() < 0.95:  # 95% success rate
            return f'Processed {random.randint(1, 100)} items'
        else:
            return None

    def publish_status(self, status: str):
        """
        Publish status message.
        
        Args:
            status: Status string to publish
        """
        try:
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish status: {str(e)}')

    def handle_error(self, error_msg: str):
        """
        Handle errors in a consistent manner.
        
        Args:
            error_msg: Error message to log
        """
        self.error_count += 1
        self.last_error_time = self.get_clock().now()
        
        self.get_logger().error(f'{error_msg} (errors: {self.error_count})')
        
        # Publish error status
        self.publish_status(f'Error: {error_msg}')

    def reset_state(self):
        """Reset node state to initial values."""
        self.error_count = 0
        self.success_count = 0
        self.last_error_time = None
        self.is_running = True

    def destroy_node(self):
        """Override destroy_node to perform cleanup."""
        self.get_logger().info('Cleaning up resources...')
        
        # Cancel timers
        if hasattr(self, 'main_timer') and self.main_timer is not None:
            self.main_timer.cancel()
        
        # Close any open resources
        # Perform any necessary cleanup
        
        super().destroy_node()
        self.get_logger().info('Node destroyed successfully')


def main(args=None):
    """Main function with proper error handling."""
    try:
        rclpy.init(args=args)
        node = BestPracticeNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Node interrupted by user')
        except Exception as e:
            node.get_logger().fatal(f'Unexpected error: {str(e)}')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f'Failed to initialize node: {str(e)}')
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
```

### 2. Configuration Management

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml
import os
from typing import Dict, Any


class ConfigurableNode(Node):
    """
    A node that loads configuration from multiple sources.
    Demonstrates best practices for configuration management.
    """
    
    def __init__(self, config_path: str = None):
        super().__init__('configurable_node')
        
        # Load configuration
        self.config = self.load_configuration(config_path)
        
        # Apply configuration
        self.apply_configuration()
        
        self.get_logger().info('Configurable node initialized with configuration')

    def load_configuration(self, config_path: str = None) -> Dict[str, Any]:
        """
        Load configuration from multiple sources:
        1. YAML file (if provided)
        2. ROS 2 parameters
        3. Environment variables
        4. Default values
        """
        config = {}
        
        # 1. Load from YAML file if provided
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    yaml_config = yaml.safe_load(f) or {}
                    config.update(yaml_config)
                self.get_logger().info(f'Loaded configuration from {config_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load config from {config_path}: {str(e)}')
        
        # 2. Override with ROS 2 parameters
        # First declare the parameters we expect
        expected_params = {
            'robot_name': ('string', 'default_robot'),
            'max_velocity': ('double', 1.0),
            'safety_factor': ('double', 1.2),
            'debug_mode': ('bool', False),
            'sensor_topics': ('string_array', ['sensor1', 'sensor2'])
        }
        
        for param_name, (param_type, default_value) in expected_params.items():
            # Construct parameter descriptor
            if param_type == 'string':
                self.declare_parameter(param_name, default_value)
            elif param_type == 'double':
                self.declare_parameter(param_name, default_value)
            elif param_type == 'bool':
                self.declare_parameter(param_name, default_value)
            elif param_type == 'string_array':
                self.declare_parameter(param_name, default_value)
        
        # Get parameter values
        for param_name in expected_params:
            try:
                param_value = self.get_parameter(param_name).value
                config[param_name] = param_value
            except Exception:
                # If parameter not set, use from YAML or skip
                pass
        
        # 3. Override with environment variables
        env_mappings = {
            'ROBOT_NAME': 'robot_name',
            'MAX_VELOCITY': 'max_velocity',
            'SAFETY_FACTOR': 'safety_factor',
            'DEBUG_MODE': 'debug_mode'
        }
        
        for env_var, config_key in env_mappings.items():
            env_value = os.environ.get(env_var)
            if env_value is not None:
                # Convert to appropriate type
                current_value = config.get(config_key)
                if isinstance(current_value, bool):
                    config[config_key] = env_value.lower() in ('true', '1', 'yes', 'on')
                elif isinstance(current_value, float):
                    try:
                        config[config_key] = float(env_value)
                    except ValueError:
                        self.get_logger().warn(f'Invalid value for {config_key}: {env_value}')
                elif isinstance(current_value, int):
                    try:
                        config[config_key] = int(env_value)
                    except ValueError:
                        self.get_logger().warn(f'Invalid value for {config_key}: {env_value}')
                else:
                    config[config_key] = env_value
        
        # 4. Ensure defaults for missing values
        defaults = {
            'robot_name': 'configurable_robot',
            'max_velocity': 0.5,
            'safety_factor': 1.1,
            'debug_mode': False,
            'sensor_topics': ['default_sensor'],
            'processing_frequency': 10
        }
        
        for key, default_value in defaults.items():
            if key not in config:
                config[key] = default_value
        
        self.get_logger().info('Configuration loaded successfully')
        return config

    def apply_configuration(self):
        """Apply the loaded configuration to the node."""
        # Apply robot-specific settings
        self.robot_name = self.config['robot_name']
        self.max_velocity = self.config['max_velocity']
        self.safety_factor = self.config['safety_factor']
        self.debug_mode = self.config['debug_mode']
        self.sensor_topics = self.config['sensor_topics']
        self.processing_frequency = self.config.get('processing_frequency', 10)
        
        # Set log level based on debug mode
        if self.debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Log applied configuration
        self.get_logger().info(f'Applied configuration:')
        self.get_logger().info(f'  Robot name: {self.robot_name}')
        self.get_logger().info(f'  Max velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'  Safety factor: {self.safety_factor}')
        self.get_logger().info(f'  Debug mode: {self.debug_mode}')
        self.get_logger().info(f'  Sensors: {self.sensor_topics}')

    def get_effective_config(self) -> Dict[str, Any]:
        """Return the effective configuration with all overrides applied."""
        return {
            'robot_name': self.robot_name,
            'max_velocity': self.max_velocity,
            'safety_factor': self.safety_factor,
            'debug_mode': self.debug_mode,
            'sensor_topics': self.sensor_topics,
            'processing_frequency': self.processing_frequency
        }


def create_config_file_example():
    """Create an example configuration file."""
    example_config = {
        'robot_name': 'production_robot',
        'max_velocity': 0.8,
        'safety_factor': 1.3,
        'debug_mode': False,
        'sensor_topics': [
            '/front_lidar/scan',
            '/rear_lidar/scan',
            '/camera/depth/image_rect_raw'
        ],
        'processing_frequency': 20,
        'control_parameters': {
            'kp': 1.0,
            'ki': 0.1,
            'kd': 0.05
        }
    }
    
    config_dir = os.path.expanduser('~/.ros/config')
    os.makedirs(config_dir, exist_ok=True)
    
    config_path = os.path.join(config_dir, 'robot_config.yaml')
    with open(config_path, 'w') as f:
        yaml.dump(example_config, f, default_flow_style=False)
    
    print(f'Example configuration file created at: {config_path}')
    return config_path


def main(args=None, config_path: str = None):
    """Main function with configuration support."""
    rclpy.init(args=args)
    
    # Create example config if none provided
    if config_path is None:
        config_path = create_config_file_example()
        print(f'Using example config from: {config_path}')
    
    try:
        node = ConfigurableNode(config_path)
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Node interrupted by user')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f'Failed to initialize configurable node: {str(e)}')
        return 1
    
    return 0


if __name__ == '__main__':
    import sys
    config_file = sys.argv[1] if len(sys.argv) > 1 else None
    exit(main(config_path=config_file))
```

## Chapter Summary

In this lesson, you've learned about performance optimization and best practices for Python-based ROS 2 applications:
- Algorithmic improvements and memory management techniques
- Concurrency and parallel processing patterns
- Profiling and monitoring tools for performance analysis
- Best practices for production deployments
- Configuration management strategies

## Exercises

1. Profile the performance of a simple publisher/subscriber pair and identify bottlenecks.
2. Implement memory-efficient data processing for sensor streams.
3. Create a configuration management system for a multi-robot system.
4. Develop a real-time performance monitoring dashboard.

## Next Steps

With all four lessons of Chapter 2 completed, you now have a comprehensive understanding of Python bridging in ROS 2. The next step is to apply these concepts to create sophisticated Python-based robotics systems that are efficient, maintainable, and production-ready.