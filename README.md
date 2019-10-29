# Installation instructions
## 1. Install openai gym from source
```
git clone https://github.com/openai/gym.git ~/gym
cd ~/gym
pip3 install -e .
```

## 2. Install the biped_ros2 repository
See installation instructions [here](https://github.com/siw-engineering/biped_ros2)


## 3a. Run the example script (robot arm)
In terminal #1, launch the simulation. 
```
rosbiped
ros2 launch arm_simulation gym_launch_all.launch.py
```
In terminal #2, run the script
```
rosbiped
cd ~/biped_ros2/src/openai_ros2/examples
python3 robot_arm_random.py
```

#### Notes
1. You can use multi-tabbed terminals to run the script for easier management, using tab 1 to run the simulation and tab 2 to run the script
  - Ctrl + Alt + T to launch a terminal
  - Ctrl + Shift + T to launch new tab
  - Alt + \<number\> to change tab. E.g. Alt + 1 to change to first tab
  
2. As of 26/9/2019 the gazebo launching needs to be done separately due to debugging concerns. 
The python debugger will simply not work when launching other processes as part of the python script, and so we try to make
the environment 1 process for now.
Definitely needs to be changed in the future, not sure when.

## 4. Create your own agents!

# Other Notes
This package is integrated as part of the `colcon build` process. This means as long as you source
the `install/setup.bash` you will be able to import this package and the relevant utilities
provided by this package. As such, no need to run `pip install -e .` or similar commands.