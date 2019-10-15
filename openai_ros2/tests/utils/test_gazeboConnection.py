from unittest import TestCase
import rclpy
from std_srvs.srv import Empty
from rclpy.executors import SingleThreadedExecutor
from openai_ros2.utils.gazebo_connection import GazeboConnection
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity


class MockGazebo:
    def __init__(self, node):
        self._pause_called = False
        self._reset_called = False
        self._unpause_called = False
        self.reset_sim_srv = node.create_service(Empty, '/reset_simulation', self.reset_sim_callback)
        self.pause_sim_srv = node.create_service(Empty, '/pause_physics', self.pause_sim_callback)
        self.unpause_sim_srv = node.create_service(Empty, '/unpause_physics', self.unpause_sim_callback)

    def reset_sim_callback(self, request, response):
        self._reset_called = True
        return response

    def pause_sim_callback(self, request, response):
        self._pause_called = True
        return response

    def unpause_sim_callback(self, request, response):
        self._unpause_called = True
        return response


class TestGazeboConnection(TestCase):
    context = None
    node = None
    gazebo_connection = None

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestGazeboConnectionClient', context=cls.context)
        cls.mock_gazebo = MockGazebo(cls.node)
        cls.gazebo_connection = GazeboConnection(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_pauseSim(self):
        self.gazebo_connection.pause_sim()
        self.assertTrue(self.mock_gazebo._pause_called, "Pause service not called")

    def test_unpauseSim(self):
        self.gazebo_connection.unpause_sim()
        self.assertTrue(self.mock_gazebo._unpause_called, "Unpause service not called")

    def test_resetSim(self):
        self.gazebo_connection.reset_sim()
        self.assertTrue(self.mock_gazebo._reset_called, "Reset service not called")
