import numpy
from biped_gym.envs.robot_gazebo_env import RobotGazeboEnv
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
import rclpy
from std_srvs.srv import Empty

class RobotArmEnv(RobotGazeboEnv):


    def __init__(self):

        self.controllers_list = []

        self.robot_name_space = ""
        super(RobotArmEnv, self).__init__(controllers_list=self.controllers_list,
                                            robot_name_space=self.robot_name_space,
                                            reset_controls=False,
                                            start_init_physics_parameters=False,
                                            reset_world_or_sim="WORLD")


        self.gazebo.unpauseSim()
        self._sub = self.node.create_subscription(JointState, "/joint_states", self.jointStateSubscriber, qos_profile_sensor_data)
        #self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #self._check_publishers_connection()
        self.gazebo.pauseSim()

    
    def jointStateSubscriber(self, message):
        self._observation_msg = message
        return True
  
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    def resetController(self):   #TODO call robot_gazebpo_env resetcontorler than replace with below
            while not self._robot_resetter.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/'+self.robot_name+'/reset service not available, waiting again...')
            reset_robot_future = self._robot_resetter.call_async(Empty.Request())
            print("Resetting controller initial positions")
            rclpy.spin_until_future_complete(self.node, reset_robot_future) 