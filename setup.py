from setuptools import setup

package_name = 'openai_ros2'

setup(
    name=package_name,
    version='0.8.0',
    packages=[package_name, 'examples'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gym', 'billiard'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'biped_random = examples.biped_random:main',
            'robot_arm_random = examples.robot_arm_random:main'
        ],
    },
)
