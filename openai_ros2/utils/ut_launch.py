import socket
import random
import os
import pathlib

from datetime import datetime
from billiard import Process

from ament_index_python.packages import get_package_prefix
from launch import LaunchService, LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from biped_gym.utils import ut_generic

def startLaunchServiceProcess(launchDesc):
    """Starts a Launch Service process. To be called from subclasses.

    Args:
         launchDesc : LaunchDescription obj.
    """
    # Create the LauchService and feed the LaunchDescription obj. to it.
    launchService = LaunchService()
    launchService.include_launch_description(launchDesc)
    process = Process(target=launchService.run)
    #The daemon process is terminated automatically before the main program exits,
    # to avoid leaving orphaned processes running
    process.daemon = True
    process.start()

    return process

def isPortInUse(port):
    """Checks if the given port is being used.

    Args:
        port(int): Port number.

    Returns:
        bool: True if the port is being used, False otherwise.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket1:
        return socket1.connect_ex(('localhost', port)) == 0

def getExclusiveNetworkParameters():
    """Creates appropriate values for ROS_DOMAIN_ID and GAZEBO_MASTER_URI.

    Returns:
        Dictionary {ros_domain_id (string), ros_domain_id (string)}
    """

    randomPortROS = random.randint(0, 230)
    randomPortGazebo = random.randint(10000, 15000)
    while isPortInUse(randomPortROS):
        print("Randomly selected port is already in use, retrying.")
        randomPortROS = random.randint(0, 230)

    while isPortInUse(randomPortGazebo):
        print("Randomly selected port is already in use, retrying.")
        randomPortGazebo = random.randint(10000, 15000)

    # Save network segmentation related information in a temporary folder.
    tempPath = '/tmp/gym-gazebo-2/running/'
    pathlib.Path(tempPath).mkdir(parents=True, exist_ok=True)

    # Remove old tmp files.
    ut_generic.cleanOldFiles(tempPath, ".log", 2)

    filename = datetime.now().strftime('running_since_%H_%M__%d_%m_%Y.log')

    file = open(tempPath + '/' + filename, 'w+')
    file.write(filename + '\nROS_DOMAIN_ID=' + str(randomPortROS) \
        + '\nGAZEBO_MASTER_URI=http://localhost:' + str(randomPortGazebo))
    file.close()

    return {'ros_domain_id':str(randomPortROS),
            'gazebo_master_uri':"http://localhost:" + str(randomPortGazebo)}
