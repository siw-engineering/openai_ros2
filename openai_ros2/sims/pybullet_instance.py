from typing import List, Tuple
import copy
import threading
import math
import pybullet as bullet
import pybullet_data
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default, qos_profile_parameters
import numpy

from ros2_control_interfaces.msg import JointControl
from sensor_msgs.msg import JointState


class WorldParams:
    def __init__(self):
        self.urdf_path: str = "plane.urdf"
        self.gravity_xyz: Tuple = (0.0, 0.0, -9.80665)
        self.time_step: float  = None  


def spin_node(node):
    rclpy.spin(node)
class Bullet(object):
    ''' 
    PyBullet internals
    
    '''
    def __init__(self, node, use_gui=True, robot_urdf_path: str = None, world : WorldParams = None):

        # ROS2 content
        self.node = node
        # context = rclpy.get_default_context()
        # if not context.ok():
        #     rclpy.init()
        # self.node = Node("openai_ros2_pybullet_node")
        self.joint_state_goals = numpy.array([0.0, 0.0, 0.0])

        joint_control_topic = 'arm_standalone/control'
        self._control_sub = self.node.create_subscription(JointControl, joint_control_topic,
            self.__control_subscription_callback, qos_profile_parameters)
        self._joint_state_pub = self.node.create_publisher(JointState, '/joint_states', qos_profile_parameters)

        # Spin the node in a different thread
        # spin_thread = threading.Thread(target = spin_node, kwargs = {'node': self.node})
        # spin_thread.daemon = True
        # spin_thread.start()
        
        # Pybullet 
        self.sim_time = 0.0 # must be updated on each call of stepSimulation
        if use_gui:
            self.clientID = bullet.connect(bullet.GUI)
            bullet.setRealTimeSimulation(0) # Non real-time, deterministic
        else:
            self.clientID = bullet.connect(bullet.DIRECT)
        bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        bullet.resetSimulation()
               
        if world.time_step is not None:
            bullet.setTimeStep(world.time_step)
            self.time_step = world.time_step
        else:
            self.time_step = 1/240.0
        g_x, g_y, g_z = world.gravity_xyz
        bullet.setGravity(g_x, g_y, g_z)
        
        self.planeID = bullet.loadURDF(world.urdf_path)
        self.robotID = bullet.loadURDF(robot_urdf_path, useFixedBase=1)
        #Find revolute joints
        self.motor_joints = []
        for i in range(bullet.getNumJoints(self.robotID)):
            joint_info = bullet.getJointInfo(self.robotID, i)
            if joint_info[2] == 0: # if joint is revolute
                self.motor_joints.append(i)
        #TODO : set initial positions



    def __control_subscription_callback(self, message: JointControl) -> None:
        # Check whether the control message is outdated
        msg_time = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if msg_time < self.sim_time*1000000000 - 20000000:
            sim_time_sec = math.floor(self.sim_time)
            sim_time_nsec = int((self.sim_time * 1000000000) % 1000000000)
            self.node.get_logger().warn(f'Ignoring outdated control message,' 
                    f'msg time: [{message.header.stamp.sec}][{message.header.stamp.nanosec}], last update time: [{sim_time_sec}][{sim_time_nsec}]')
            return
        self.joint_state_goals = message.goals

    
    def calculate_forces(self, goals):
        '''
        Calculate the required torque for desired joint states
        '''
        #TODO : custom controller here
        pass

    def apply_forces(self, goals: numpy.ndarray):
        '''
        Apply the forces to joints, applyExternalForce/Torque API
        '''
        # forces = self.calculate_forces(goals)
        # TODO: Use custom controller and use torque control
        uj1, uj2, uj3 = goals
        bullet.setJointMotorControl2(self.robotID, self.motor_joints[0], bullet.POSITION_CONTROL, targetPosition=uj1)
        bullet.setJointMotorControl2(self.robotID, self.motor_joints[1], bullet.POSITION_CONTROL, targetPosition=uj2)
        bullet.setJointMotorControl2(self.robotID, self.motor_joints[2], bullet.POSITION_CONTROL, targetPosition=uj3)
        pass


    def step(self):
        goals = copy.copy(self.joint_state_goals)
        self.apply_forces(goals)
        bullet.stepSimulation()
        self.sim_time += self.time_step
        state_msg = JointState()
        state_msg.position = []
        state_msg.velocity = []
        for joint in self.motor_joints:
            joint_angle, joint_velocity, _ , _ = bullet.getJointState(self.robotID, joint)
            state_msg.position.append(joint_angle)
            state_msg.velocity.append(joint_velocity)
        state_msg.header.stamp.sec = math.floor(self.sim_time)
        state_msg.header.stamp.nanosec = int((self.sim_time * 1000000000) % 1000000000)
        self._joint_state_pub.publish(state_msg)

    def reset(self):
        #TODO : 
        
        for joint in self.motor_joints: 
            bullet.resetJointState(self.robotID,joint,0)
            bullet.setJointMotorControl2(self.robotID, joint, bullet.POSITION_CONTROL, targetPosition=0.0)