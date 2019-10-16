import rclpy
from builtin_interfaces.msg import Time


class LobotArmHelper:
    @staticmethod
    def get_time_from_time_msg(time_msg: Time) -> int:
        """
        Converts from a message with separate second and nanoseconds component into just nanoseconds
        :param time_msg:
        :return:
        """
        return time_msg.sec*1000000000 + time_msg.nanosec
