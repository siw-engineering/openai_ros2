import numpy as np
import rclpy
import os
from xml.etree import ElementTree
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity


def processObservations(message, agent):
    """
    Helper fuinction to convert a ROS message to joint angles and velocities.
    Check for and handle the case where a message is either malformed
    or contains joint values in an order different from that expected observation_callback
    in hyperparams['jointOrder']
    """
    if not message:
        print("Message is empty")
        return None
    else:
        # # Check if joint values are in the expected order and size.
        # if len(message.name) != len(agent['jointOrder']):
        #     # Check that the message is of same size as the expected message.
        #     if message.name != agent['jointOrder']:
        #         raise Exception
        pos_arr = np.array(message.position)
        vel_arr = np.array(message.velocity)
        state_arr = np.concatenate((pos_arr, vel_arr))
        return state_arr


def positionsMatch(action, lastObservation):
    """
    Compares a given action with the observed position.
    Returns: bool. True if the position is final, False if not.
    """
    acceptedError = 0.01
    for i in range(action.size - 1):  # lastObservation loses last pose
        if abs(action[i] - lastObservation[i]) > acceptedError:
            return False
    return True


def spawn_robot(urdfPath, robotName, node):
    node.get_logger().info('Loading entity XML from file %s' % urdfPath)
    if not os.path.exists(urdfPath):
        node.get_logger().error('Error: specified file %s does not exist', urdfPath)
        return 1
    if not os.path.isfile(urdfPath):
        node.get_logger().error('Error: specified file %s is not a file', urdfPath)
        return 1
    # load file
    try:
        f = open(urdfPath, 'r')
        entity_xml = f.read()
    except IOError as e:
        node.get_logger().error('Error reading file {}: {}'.format(urdfPath, e))
        return 1
    if entity_xml == '':
        node.get_logger().error('Error: file %s is empty', urdfPath)
        return 1
    # Parse xml to detect invalid xml before sending to gazebo
    try:
        xml_parsed = ElementTree.fromstring(entity_xml)
    except ElementTree.ParseError as e:
        node.get_logger().error('Invalid XML: {}'.format(e))
        return 1
    # Encode xml object back into string for service call
    entity_xml = ElementTree.tostring(xml_parsed)

    # Set Pose
    initial_pose = Pose()
    initial_pose.position.x = float(0)
    initial_pose.position.y = float(0)
    initial_pose.position.z = float(0.3)

    # Service call
    node.get_logger().info('Waiting for service /spawn_entity')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    if client.wait_for_service(timeout_sec=5.0):
        req = SpawnEntity.Request()
        req.name = robotName
        req.xml = str(entity_xml, 'utf-8')
        req.robot_namespace = ''
        req.initial_pose = initial_pose
        req.reference_frame = ''
        node.get_logger().info('Calling service /spawn_entity')
        srv_call = client.call_async(req)
        while rclpy.ok():
            if srv_call.done():
                node.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_until_future_complete(node, srv_call)
        return srv_call.result().success
    node.get_logger().error(
        'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')
    return False

    # node.get_logger().info('Waiting for service /spawn_entity')
    # client = node.create_client(SpawnEntity, '/spawn_entity')
    # if client.wait_for_service(timeout_sec=5.0):
    #     req = SpawnEntity.Request()
    #     req.name = robotName
    #     req.xml = str(entity_xml, 'utf-8')
    #     req.robot_namespace = ''
    #     req.initial_pose = initial_pose
    #     req.reference_frame = ''
    #     node.get_logger().info('Calling service /spawn_entity')
    #     srv_call = client.call_async(req)
    #     while rclpy.ok():
    #         if srv_call.done():
    #             node.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
    #             break
    #         rclpy.spin_once(node)
    #     return srv_call.result().success
    #     node.get_logger().error(
    #         'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')
    # # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    # spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #                     arguments=['-entity', robotName,'-file',urdfPath, '-z', "0.22"],
    #                     output='screen')
