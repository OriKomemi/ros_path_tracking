import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray

from bgr_description.srv import GetTrack
from bgr_description.msg import Cone, CarPose

class PathPlotterNode(LifecycleNode):
    def __init__(self):
        super().__init__('path_plotter')
        self.get_logger().info('PathPlotterNode initialized')
        self.track_client = None
        self.path_sub = None
        self.state_sub = None

    def on_configure(self, state :LifecycleState) -> TransitionCallbackReturn:
        # client
        self.track_client = self.create_client(GetTrack, '/track_server/get_track')

        # subscribes
        self.path_sub = self.create_subscription(
                Path,
                '/planner/path',
                self.path_callback,
                10
            )
        
        self.state_sub = self.create_subscription(
                Float64MultiArray,
                '/robot/full_state',
                self.state_callback,
                10
            )


        return super().on_configure(state)