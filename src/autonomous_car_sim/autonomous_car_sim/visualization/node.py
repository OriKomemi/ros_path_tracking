import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64MultiArray
import sensor_msgs_py.point_cloud2 as pc2

class VisualizationNode(Node):
    """Node to visualize planned path in RViz"""

    def __init__(self):
        super().__init__('visualization_node')
    
        self.get_logger().info('Visualization Node initialized')

        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/robot/full_state',
            self.state_callback,
            10
        )

        self.state_sub = self.create_subscription(
            PointCloud2,
            '/lidar/detections',
            self.lidar_detection_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(Path, '/planned_path', self._on_path, 10)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        plt.ion()
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Path')
        self.cone_unknown_sc = self.ax.scatter([], [], c='gray', s=30, label='Unknown cones', zorder=5)
        # self.cone_left_sc = self.ax.scatter([], [], c='yellow', s=40, edgecolors='black', label='Left cones', zorder=5)
        # self.cone_right_sc = self.ax.scatter([], [], c='blue', s=40, label='Right cones', zorder=5)
        self.car_marker, = self.ax.plot([], [], 'r^', markersize=10, label='Car', zorder=6)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Auto Cross Path")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.legend(loc='upper right')
        plt.show(block=False)
        self.car_position = None
        self.car_direction = None
    
    def state_callback(self, msg):
        """
        Receive full state from SuperStateSpy.
        """
        x = msg.data[0]
        y = msg.data[1]
        self.car_position = np.array([x, y])

        yaw = msg.data[5]
        self.car_direction = np.array([np.cos(yaw), np.sin(yaw)])
        
        # 1. FIX: Actually update the car marker's data on the plot
        self.car_marker.set_data([x], [y]) 
        window_size = 15.0  # Adjust this to see more or less of the track (in meters)
        self.ax.set_xlim(x - window_size, x + window_size)
        self.ax.set_ylim(y - window_size, y + window_size)

    def lidar_detection_callback(self, msg):
        """Callback for LIDAR detections to update visualization"""
        self.get_logger().info('Callback for LIDAR detections to update visualization')
        
        n = msg.width * msg.height
        if n == 0:
            return 
            
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        
        if self.car_position is None:
            self.get_logger().warn('Received lidar detections but car position is unknown, ignoring.')
            return
            
        car_xy = np.array(self.car_position[:2], dtype=np.float32)
        xy = raw[:, :8].view(np.float32).reshape(n, 2).copy()

        xy += car_xy

        if xy is not None and xy.ndim == 2 and len(xy) > 0:
            self.cone_unknown_sc.set_offsets(xy) 
        else:
            # 2. FIX: Corrected typo (added self.cone_unknown_sc.)
            self.cone_unknown_sc.set_offsets(np.empty((0, 2))) 
            

        self.get_logger().info(f'Updated visualization with {len(xy)} lidar detections')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events() 

    def _on_path(self, msg: Path):
        """Callback for receiving planned path"""
        self.get_logger().info('Received new planned path with %d poses' % len(msg.poses))
        if len(msg.poses) == 0:
            self.path_line.set_data([], [])
            return

        x = [pose.pose.position.x for pose in msg.poses]
        y = [pose.pose.position.y for pose in msg.poses]
        self.path_line.set_data(x, y)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()    

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()
