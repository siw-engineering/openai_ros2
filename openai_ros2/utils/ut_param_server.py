from parameter_server_interfaces.srv import GetAllJoints
from parameter_server_interfaces.srv import GetRobots
from rclpy.qos import qos_profile_services_default
import rclpy


def getRobots(node):
    get_robots_client = node.create_client(
        GetRobots, "/GetRobots", qos_profile=qos_profile_services_default)
    req = GetRobots.Request()
    while not get_robots_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().info('service not available, waiting again...')

    future = get_robots_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        robot_names = future.result().robots
        node.get_logger().info('Number of robots: %d' % (len(robot_names)))
        return robot_names
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))


def getJoints(node, robot_name):
    get_joints_client = node.create_client(
        GetAllJoints, "/GetAllControlJoints", qos_profile=qos_profile_services_default)
    req = GetAllJoints.Request()
    req.robot = robot_name
    while not get_joints_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().info('service not available, waiting again...')

    future = get_joints_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        joint_names = future.result().joints
        node.get_logger().info('Number of joints: %d' % (len(joint_names)))
        return joint_names
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))
