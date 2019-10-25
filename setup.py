from setuptools import setup

package_name = 'openai_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'examples', 'openai_ros2/envs', 'openai_ros2/envs/robot_envs', 'openai_ros2/envs/task_envs/lobot_arm', 'openai_ros2/utils', 'openai_ros2/robots', 'openai_ros2/robots/lobot', 'openai_ros2/robots/lobot/tasks'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gym', 'billiard', 'numpy'],
    zip_safe=True,
    author='Poh Zhi-Ee',
    author_email='zhiee.poh@httechnology.com',
    maintainer='Poh Zhi-Ee',
    maintainer_email='zhiee.poh@httechnology.com',
    keywords=['ROS2', 'OpenAI', 'Gym'],
    description='ros2 implementation of openai_ros',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'biped_random = examples.biped_random:main',
            'robot_arm_random = examples.robot_arm_random:main'
        ],
    },
)
