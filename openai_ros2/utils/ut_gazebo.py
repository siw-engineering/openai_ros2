import os
from typing import Tuple
import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import qos_profile_services_default
from gazebo_msgs.srv import DeleteEntity
from xml.etree import ElementTree
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity


def remove_target_marker(node: rclpy.node.Node) -> Tuple[bool, str]:
    remove_marker_client = node.create_client(DeleteEntity, "/delete_entity", qos_profile=qos_profile_services_default)
    req1 = DeleteEntity.Request()
    req1.name = 'target_marker'
    while not remove_marker_client.wait_for_service(timeout_sec=3.0):
        node.get_logger().warn('/delete_entity service not available, check that Gazebo is launched properly, waiting again...')

    future = remove_marker_client.call_async(req1)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        result: DeleteEntity.Response = future.result()
        success = result.success
        status_msg = result.status_message
        return success, status_msg
    else:
        node.get_logger().warn(f'Service call failed {future.exception}')
        return False, ''


def spawn_target_marker(node: rclpy.node.Node, x: float, y: float, z: float) -> bool:
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    marker_urdf_path = os.path.join(lobot_desc_share_path, 'robots/target_indicator.urdf')
    try:
        f = open(marker_urdf_path, 'r')
        entity_xml = f.read()
    except IOError as e:
        node.get_logger().error('Error reading file {}: {}'.format(marker_urdf_path, e))
        return False
    if entity_xml == '':
        node.get_logger().error('Error: file %s is empty', marker_urdf_path)
        return False

    try:
        xml_parsed = ElementTree.fromstring(entity_xml)
    except ElementTree.ParseError as e:
        node.get_logger().error('Invalid XML: {}'.format(e))
        return False

    # Form requested Pose from arguments
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z

    success = __spawn_entity(node, entity_xml, initial_pose, 'target_marker')
    if not success:
        node.get_logger().error('Spawn service failed. Exiting.')
    return success


def __spawn_entity(node, entity_xml, initial_pose, name) -> bool:
    node.get_logger().info('Waiting for service /spawn_entity')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    if client.wait_for_service(timeout_sec=5.0):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = entity_xml
        req.initial_pose = initial_pose
        node.get_logger().info('Calling service /spawn_entity')
        srv_call = client.call_async(req)
        while rclpy.ok():
            if srv_call.done():
                node.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_once(node)
        return srv_call.result().success
    node.get_logger().error(
        'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')
    return False