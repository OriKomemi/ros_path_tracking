import rclpy
from rclpy.node import Node

from rclpy.lifecycle import LifecycleState
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

class LifecycleManager(Node):
    """
    An orchestrator node that manages the lifecycle of all other nodes in the autonomous car simulation. 
    It ensures that all nodes are started, stopped, and restarted in a coordinated manner to maintain the overall system's stability and performance.
    """
    def __init__(self, managed_nodes: list[str]):
        super().__init__('lifecycle_manager')

        self.managed_nodes = managed_nodes
        self.nodes = {}

        for node_name in self.managed_nodes:
            self.nodes[node_name] = {
                'get_state': self.create_client(
                    GetState,
                    f'/{node_name}/get_state'
                ),
                'change_state': self.create_client(
                    ChangeState,
                    f'/{node_name}/change_state'
                )
            }

    # wait_for_service is not async, so we use it sequentially.
    # add timeout and error handling to avoid infinite waiting if a node fails to start
    def wait_for_managed_nodes(self) -> bool:
        self.get_logger().info('Waiting for lifecycle services of all managed nodes...')

        all_ready = True

        for node_name, node_clients in self.nodes.items():
            get_state_client = node_clients['get_state']
            change_state_client = node_clients['change_state']

            while not get_state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'/{node_name}/get_state not available, waiting...')

            while not change_state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'/{node_name}/change_state not available, waiting...')

            self.get_logger().info(f'Lifecycle services ready for {node_name}')

        return all_ready
    
    # Assume every node in managed_nodes has get_state services
    def get_state(self, node_name: str) -> LifecycleState | None:
        if node_name not in self.nodes:
            self.get_logger().error(f'Unknown managed node: {node_name}')
            return None

        try:
            req = GetState.Request()
            future = self.nodes[node_name]['get_state'].call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error(f'Failed to get {node_name} state')
                return None

            state = future.result().current_state
            self.get_logger().info(f'{node_name} state: {state.label} ({state.id})')
            return state

        except Exception as e:
            self.get_logger().error(
                f'Exception while getting state for {node_name}: {e}'
            )
            return None

    # Assume every node in managed_nodes has change_state services
    def change_state(self, node_name: str, transition : Transition) -> bool | rclpy.task.Future:
        if node_name not in self.nodes:
            self.get_logger().error(f'Unknown managed node: {node_name}')
            return False

        try:
            req = ChangeState.Request()
            req.transition.id = transition.id
            req.transition.label = transition.label

            future = self.nodes[node_name]['change_state'].call_async(req) 

            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response is None:
                self.get_logger().error(
                    f'Failed to call transition {transition.label} for {node_name}'
                )
                return False
            
            elif not response.success:
                self.get_logger().error(
                    f'Transition {transition.label} failed for {node_name}'
                )
                return False

            self.get_logger().info(
                f'Transition {transition.label} succeeded for {node_name}'
            )
            return response.success

        except Exception as e:
            self.get_logger().error(
                f'Exception while calling transition {transition.label} for {node_name}: {e}'
            )
            return False

    def get_all_states(self):
        futures = []
        for node_name in self.managed_nodes:
            req = GetState.Request()
            futures.append(self.nodes[node_name]['get_state'].call_async(req))

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(f.done() for f in futures):
                break

        results = [f.result() for f in futures]
        if None in results:
            self.get_logger().error('Failed to get states for all nodes')
            return None
                
        return {
            node_name: result.current_state 
            for node_name, result in zip(self.managed_nodes, results)
            }

    def change_state_all(self, transition : Transition) -> bool:
        futures = []
        for node_name in self.managed_nodes:
            req = ChangeState.Request()
            req.transition.id = transition.id
            req.transition.label = transition.label
            futures.append(self.nodes[node_name]['change_state'].call_async(req)) 

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(f.done() for f in futures):
                break
        
        results = [f.result() for f in futures]
        if None in results or not all(r.success for r in results):
            self.get_logger().error(f'Failed to call transition {transition.label} for all nodes')
            return False
        
        self.get_logger().info(f'all nodes transitioned to {transition.label} successfully')
        return True
 

def manage_nodes(manager: LifecycleManager):
    # Initialize get/change subservices for all nodes
    manager.wait_for_managed_nodes()

    # Configure nodes in parallel
    if not manager.change_state_all(Transition(label='configure', id=Transition.TRANSITION_CONFIGURE)):
        manager.get_logger().error('Failed to configure all nodes, shutting down')
        # shutdown_nodes(manager)
        return
    
    # Activate nodes in parallel
    if not manager.change_state_all(Transition(label='activate', id=Transition.TRANSITION_ACTIVATE)):
        manager.get_logger().error('Failed to activate all nodes, shutting down')
        # shutdown_nodes(manager)
        return

# check how to shutdown properly
# def shutdown_nodes(manager: LifecycleManager):
#     # Shutdown nodes in parallel
#     if not manager.change_state_all(Transition(label='shutdown', id=Transition.TRANSITION_SHUTDOWN)):
#         manager.get_logger().error('Failed to shutdown all nodes')
#     rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)

    managed_nodes = [
        #'perception_node',
        #'mapping_node',
        'path_planner',
        #'vehicle_controller',
    ]
    # TODO: Find the name of the nodes of the simulator and plotting nodes 
    # if args is list:
    #     if 'simulation' in args:
    #         managed_nodes.append('simulation_node')
    #     if 'plot' in args:
    #         managed_nodes.append('plotting_node')
        

    manager = LifecycleManager(managed_nodes)

    manage_nodes(manager)

    rclpy.spin(manager)
    rclpy.shutdown()


    