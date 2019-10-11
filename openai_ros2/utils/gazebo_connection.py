from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity
import rclpy
import gym
gym.logger.set_level(40)  # hide warnings

class GazeboConnection():

    def __init__(self, node):
        self.node = node
        self._reset_sim = self.node.create_client(Empty, '/reset_simulation')
        self._physics_pauser = self.node.create_client(Empty, '/pause_physics')
        self._physics_unpauser = self.node.create_client(Empty, '/unpause_physics')
        self.delete_entity = self.node.create_client(DeleteEntity, '/delete_entity')

    def pauseSim(self):
            while not self._physics_pauser.wait_for_service(timeout_sec=1.0):  #rospy.wait_for_service('/gazebo/pause_physics')
                self.node.get_logger().info('/pause_physics service not available, waiting again...') #rospy.logwarn("XXX")
            pause_future = self._physics_pauser.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, pause_future)
            print("Pausing physics")
            
    def unpauseSim(self):
            while not self._physics_unpauser.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/unpause_physics service not available, waiting again...')
            unpause_future = self._physics_unpauser.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, unpause_future)
            print("Unpausing simulation")

    def resetSim(self): 
            while not self._reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')
            reset_future = self._reset_sim.call_async(Empty.Request())
            print("Resetting simulation")
            rclpy.spin_until_future_complete(self.node, reset_future)