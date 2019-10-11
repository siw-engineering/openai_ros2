# Installation instructions
## 1. Install openai gym from source
```
git clone https://github.com/openai/gym.git ~/gym
cd ~/gym
pip3 install -e .
```

## 2. Install this environment
```
git clone https://github.com/pohzhiee/biped_gym.git ~/biped_gym
cd ~/biped_gym
pip3 install -e .
```

## 3a. Run the example script (biped)
In terminal #1, launch the simulation. 
```
rosbiped
ros2 launch lobot_simulation launch_all.launch.py
```
In terminal #2, run the script
```
cd ~/biped_gym/examples
python3 biped_random.py
```

## 3b. Run the example script (robot arm)
In terminal #1, launch the simulation. 
```
rosbiped
ros2 launch arm_simulation launch_all.launch.py
```
In terminal #2, run the script
```
cd ~/biped_gym/examples
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

3. There is currently a bug where the controllers and the robot plugins do not discover each other quickly enough.
This causes the robot to seem like it is not controlled when it is spawned. You might need to wait for some time before 
they discover each other and the robot move into a fixed position. This bug is partially beyond our control as some of it
has to do with the discovery mechanism of the ROS publisher/subscribers.

## 4. Create your own agents!

# Other Notes
1. If your python package happen to be part of a `colcon build` directory 
somehow the links made during the `pip3 install -e` will break every time you run colcon install.
So try to avoid putting python packages that needs to be installed to pip into a directory affected by `colcon build`
