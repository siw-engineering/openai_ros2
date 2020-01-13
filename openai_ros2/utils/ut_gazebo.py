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
from ros2_control_interfaces.srv import CreateMarker

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
