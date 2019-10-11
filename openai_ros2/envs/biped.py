from ros2pkg.api import get_prefix_path
from geometry_msgs.msg import Pose
from ros2_control_interfaces.msg import JointControl
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity
from sensor_msgs.msg import Imu
from parameter_server_interfaces.srv import GetAllJoints
# Used for publishing mara joint angles.
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import rclpy
import builtin_interfaces
from gym.utils import seeding
from biped_gym.utils import ut_generic, ut_launch, ut_biped
from gym import utils, spaces
import numpy as np
import time
import gym
import copy
import os
import threading
gym.logger.set_level(40)  # hide warnings

class BipedEnv(gym.Env):

    def __init__(self):
        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getParserArgsRobot().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        # self.realSpeed = True
        self.debug = args.debug
        self.multiInstance = args.multiInstance
        self.port = args.port
        # Set the path of the corresponding URDF file
        if self.realSpeed:
            urdf = "biped.urdf"
            self.urdfPath = get_prefix_path(
                "lobot_description") + "/share/lobot_description/robots/" + urdf
        else:
            print("Non real speed not yet supported. Use real speed instead. ")

        # TODO: Include launch logic here, refer to code from the .launch.py files
        # Note that after including the launch logic the code will no longer be debuggable due to multi process stuff

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 1024 #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True
        self._collision_msg = None

        #############################
        #   Environment hyperparams
        #############################
        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])

        # # Topics for the robot publisher and subscriber.
        JOINT_PUBLISHER = '/lobot/control'
        # Get Joint names from the parameter server
        get_joints_client = self.node.create_client(GetAllJoints, "/GetAllControlJoints", qos_profile=qos_profile_services_default)
        req = GetAllJoints.Request()
        req.robot = "lobot"
        while not get_joints_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().info('service not available, waiting again...')

        future = get_joints_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            joint_names = future.result().joints
            self.node.get_logger().info(
                'Number of joints: %d' %
                (len(joint_names)))
        else:
            self.node.get_logger().info('Service call failed %r' % (future.exception(),))
        JOINT_ORDER = joint_names
        INITIAL_JOINTS = np.full((len(joint_names)), 0.0).tolist()
        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
             'initial_velocities': []
        }
        #############################

        m_jointOrder = copy.deepcopy(JOINT_ORDER)

        # Initialize target end effector position
        self.environment = {
            'jointOrder': m_jointOrder,
            'reset_conditions': reset_condition,
            'tree_path': self.urdfPath,
            'end_effector_points': EE_POINTS,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointControl, JOINT_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointState, "/joint_states", self.observation_callback, qos_profile_sensor_data)

        # TODO: Make the clock node run on a separate thread so weird issues like outdated clock can stop happening
        self.lock = threading.Lock()
        self.clock_node = rclpy.create_node(self.__class__.__name__+"_clock")
        self._sub_clock = self.clock_node.create_subscription(RosClock, '/clock', self.clock_callback, qos_profile=qos_profile_sensor_data)
        self.exec = rclpy.executors.MultiThreadedExecutor()
        self.exec.add_node(self.clock_node)
        t1 = threading.Thread(target=self.spinClockNode, daemon=True)
        t1.start()
        # self._imu_sub = self.node.create_subscription(JointState, "/lobot_IMU_controller/out", self.imu_callback, qos_profile_sensor_data)
        # self._sub = self.node.create_subscription(JointTrajectoryControllerState, JOINT_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)
        self._reset_sim = self.node.create_client(Empty, '/reset_simulation')
        self._physics_pauser = self.node.create_client(Empty, '/pause_physics')
        self._robot_resetter = self.node.create_client(Empty, '/lobot/reset')
        self._physics_unpauser = self.node.create_client(Empty, '/unpause_physics')
        self.delete_entity = self.node.create_client(DeleteEntity, '/delete_entity')
        self.numJoints = len(JOINT_ORDER)
        # Initialize a KDL Jacobian solver from the chain.
        # self.jacSolver = ChainJntToJacSolver(self.mara_chain)

        # Observable dimensions, each joint has 2 (joint position + joint velocity), the IMU gives 6
        self.obs_dim = self.numJoints * 2 + 6 

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]

        low = -np.pi * np.ones(self.numJoints) * 0.4
        high = np.pi * np.ones(self.numJoints) * 0.4

        self.action_space = spaces.Box(low, high)

        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        self.seed()
        self.buffer_dist_rewards = []
        self.buffer_tot_rewards = []
        self.collided = 0

        # Set the time source
        self._sim_time = 0
        self._sim_time_msg = builtin_interfaces.msg.Time()

    def observation_callback(self, message):
        self._observation_msg = message

    def imu_callback(self, message):
        self._imu_msg = message

    def collision_callback(self, message):
        """
        Callback method for the subscriber of Collision data
        """
        print("Not implemented yet, collision to be implemented soon...")
        self._collision_msg = message

    def clock_callback(self, message):
        with self.lock:
            self._sim_time = self.get_time_from_time_msg(message.clock)
            self._sim_time_msg = message.clock

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def get_time_from_time_msg(self, time_msg):
        secStr = str(time_msg.sec)
        nanoSecStr = str(time_msg.nanosec)
        if(len(nanoSecStr) < 9):
            lengthDiff = 9-len(nanoSecStr)
            for i in range(1,lengthDiff):
                nanoSecStr = "0" + nanoSecStr
        msg_time = int(secStr + nanoSecStr)
        return msg_time

    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg

        # Check that the observation is not prior to the action
        if(obs_message != None): 
            msg_time = self.get_time_from_time_msg(obs_message.header.stamp)
        else:
            msg_time = -1
        
        while obs_message is None or msg_time < self.last_action_send_time:
            if(obs_message is not None):
                if(msg_time < self.last_action_send_time):
                    print("observation outdated, msg time: %d, last action send time: %d" %(msg_time, self.last_action_send_time) )
                    # print("Sec: %d" % self._observation_msg.header.stamp.sec)
                    # print("Nsec: %d" % self._observation_msg.header.stamp.nanosec) 
            # else:
                # print("I am in obs_message is none")
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg

            if(obs_message != None): 
                msg_time = self.get_time_from_time_msg(obs_message.header.stamp)
            else:
                msg_time = -1

        # print("Observation taken!")
        lastObservations = ut_biped.processObservations(obs_message, self.environment)
        # lastImuData = self._imu_msg
        # Set observation to None after it has been read.
        self._observation_msg = None

        state = np.r_[np.reshape(lastObservations, -1)]

        return state

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - action
            - observation
            - reward
            - done (status)
        """
        self.iterator+=1
        # Execute "action"
        msg = JointControl()
        msg.joints = self.environment['jointOrder']
        msg.goals = action.tolist()
        self.last_action_send_time = self._sim_time
        msg.header.stamp = self._sim_time_msg
        self._pub.publish(msg)

        # Take an observation
        obs = self.take_observation()
        # Compute reward
        # TODO: Implement reward function
        reward = 1
        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)

        info = {}

        # TODO: Function to publish data every 10 iteration or so

        # Return the corresponding observations, rewards, etc.
        return obs, reward, done, info

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0
        if self.reset_jnts is True:
            # pause simulation
            while not self._physics_pauser.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/pause_physics service not available, waiting again...')
            pause_future = self._physics_pauser.call_async(Empty.Request())
            print("Pausing physics")
            rclpy.spin_until_future_complete(self.node, pause_future)

            # reset controllers
            while not self._robot_resetter.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/lobot/reset service not available, waiting again...')
            reset_robot_future = self._robot_resetter.call_async(Empty.Request())
            print("Resetting controller initial positions")
            rclpy.spin_until_future_complete(self.node, reset_robot_future)

            # reset simulation
            while not self._reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')
            reset_future = self._reset_sim.call_async(Empty.Request())
            print("Resetting simulation")
            rclpy.spin_until_future_complete(self.node, reset_future)
            
            # unpause simulation
            while not self._physics_unpauser.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/unpause_physics service not available, waiting again...')
            unpause_future = self._physics_unpauser.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, unpause_future)
            print("Unpausing simulation")

    def spinClockNode(self):
        self.exec.spin()

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        # parent = psutil.Process(self.launch_subp.pid)
        # for child in parent.children(recursive=True):
        #     child.kill()
        # parent = psutil.Process(self.param_subp.pid)
        # for child in parent.children(recursive=True):
        #     child.kill()
        rclpy.shutdown()
        # parent.kill()
