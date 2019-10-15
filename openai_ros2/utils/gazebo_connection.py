from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity
import rclpy


class GazeboConnection:

    def __init__(self, node):
        self.node = node
        rclpy.init()
        self._reset_sim = self.node.create_client(Empty, '/reset_simulation')
        self._physics_pause_client = self.node.create_client(Empty, '/pause_physics')
        self._physics_unpause_client = self.node.create_client(Empty, '/unpause_physics')
        self._entity_delete = self.node.create_client(DeleteEntity, '/delete_entity')

    def pause_sim(self) -> None:
        while not self._physics_pause_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/pause_physics service not available, waiting again...')
        pause_future = self._physics_pause_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, pause_future)
        print("Pausing simulation")

    def unpause_sim(self) -> None:
        while not self._physics_unpause_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/unpause_physics service not available, waiting again...')
        unpause_future = self._physics_unpause_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, unpause_future)
        print("Unpausing simulation")

    def reset_sim(self) -> None:
        while not self._reset_sim.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/reset_simulation service not available, waiting again...')
        reset_future = self._reset_sim.call_async(Empty.Request())
        print("Resetting simulation")
        rclpy.spin_until_future_complete(self.node, reset_future)

    def delete_entity(self, entity_name: str) -> bool:
        """
        Deletes an entity from gazebo by name of the entity
        :param entity_name:
        :return: bool representing whether the delete is successful
        """
        while not self._entity_delete.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/delete_entity service not available, waiting again...')
        delete_future = self._entity_delete.call_async(DeleteEntity.Request(name=entity_name))
        print("Deleting entity")
        rclpy.spin_until_future_complete(self.node, delete_future)
        if delete_future.done():
            delete_result = delete_future.result()
            delete_success_status = delete_result.success
            delete_message = delete_result.status_message
            print(delete_message)
            return delete_success_status


