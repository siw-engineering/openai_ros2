from typing import Optional
import numpy
import rclpy
import rclpy.node
from ros2_control_interfaces.srv import CreateMarker, RandomPositions


def create_marker(node: rclpy.node.Node, x: float, y: float, z: float, diameter: float = 0.02, id: int = 0) -> bool:
    if "client" not in create_marker.__dict__:
        create_marker.client = node.create_client(CreateMarker, '/create_marker')
    if "req" not in create_marker.__dict__:
        create_marker.req = CreateMarker.Request()
        create_marker.req.roll = 0.0
        create_marker.req.pitch = 0.0
        create_marker.req.yaw = 0.0
    if create_marker.client.wait_for_service(timeout_sec=5.0):
        create_marker.req.id = id
        create_marker.req.diameter = diameter
        create_marker.req.x = x
        create_marker.req.y = y
        create_marker.req.z = z
        # node.get_logger().debug('Calling service /create_marker')
        srv_call = create_marker.client.call_async(create_marker.req)
        while rclpy.ok():
            if srv_call.done():
                # node.get_logger().debug('Move status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_once(node)
        return srv_call.result().success
    node.get_logger().error('Service /create_marker unavailable')
    return False


def random_positions(node: rclpy.node.Node) -> Optional[numpy.ndarray]:
    if "client" not in random_positions.__dict__:
        random_positions.client = node.create_client(RandomPositions, '/random_positions')
    if random_positions.client.wait_for_service(timeout_sec=5.0):
        # node.get_logger().debug('Calling service /create_marker')
        srv_call = random_positions.client.call_async(RandomPositions.Request())
        while rclpy.ok():
            if srv_call.done():
                break
            rclpy.spin_once(node)
        # Sort the joint and position pairs such that arm_1_joint is first, and arm_3_joint is last
        joint_val_pairs = list(zip(srv_call.result().joints, srv_call.result().positions))
        joint_val_pairs.sort()
        positions = [v for _, v in joint_val_pairs]
        return numpy.array(positions)
    node.get_logger().error('Service /random_positions unavailable')
    return None
