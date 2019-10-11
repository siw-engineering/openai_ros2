from unittest import TestCase
import rclpy
from std_srvs.srv import Empty
from rclpy.executors import SingleThreadedExecutor


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

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestGazeboConnectionClient', context=cls.context)
        cls.mock_gazebo = MockGazebo(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def call_service(self, service_name):
        client = self.node.create_client(Empty, service_name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(Empty.Request())
            executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
            rclpy.spin_until_future_complete(self.node, future, executor=executor)
            self.assertTrue(future.result() is not None)
        finally:
            self.node.destroy_client(client)

    def test_pauseSim(self):
        self.call_service('/pause_physics')
        self.assertTrue(self.mock_gazebo._pause_called, "Pause service not called")

    def test_unpauseSim(self):
        self.call_service('/unpause_physics')
        self.assertTrue(self.mock_gazebo._unpause_called, "Unpause service not called")

    def test_resetSim(self):
        self.call_service('/reset_simulation')
        self.assertTrue(self.mock_gazebo._reset_called, "Unpause service not called")
