from parameter_server_interfaces.srv import GetAllJoints, GetRobots, GetGymUpdateRate
from rclpy.qos import qos_profile_services_default
from typing import Collection
import rclpy
from rclpy.node import Node


def get_robots(node: Node):
    get_robots_client = node.create_client(GetRobots, "/GetRobots")
    req = GetRobots.Request()
    while not get_robots_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().info('Parameter service not available, waiting again...')

    future = get_robots_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        robot_names = future.result().robots
        node.get_logger().info('Number of robots: %d' % (len(robot_names)))
        return robot_names
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))


def get_joints(node: Node, robot_name: str) -> Collection[float]:
    get_joints_client = node.create_client(GetAllJoints, "/GetAllControlJoints")
    req = GetAllJoints.Request()
    req.robot = robot_name
    while not get_joints_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().info('Parameter service not available, waiting again...')

    future = get_joints_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        joint_names = future.result().joints
        node.get_logger().info('Number of joints: %d' % (len(joint_names)))
        return joint_names
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))
        return []


def get_update_rate(node: Node) -> float:
    get_update_rate_client = node.create_client(
        GetGymUpdateRate, "/GetGymUpdateRate", qos_profile=qos_profile_services_default)
    req1 = GetGymUpdateRate.Request()
    while not get_update_rate_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().warn('Parameter service not available, waiting again...')

    future = get_update_rate_client.call_async(req1)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        update_rate = future.result().update_rate
        node.get_logger().info(f'Gym update rate set to: {update_rate}Hz')
        return update_rate
    else:
        node.get_logger().warn(f'Service call failed {future.exception}')
