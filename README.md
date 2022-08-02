# KIMM_SPOT_KINOVA PROJECT
IMPORTANT: This package is optimized on Ubuntu 20.04 with ROS-noetic and Python 3


## Packages
This repository contains multiple packages to control the spot_with_kinova robot on both real and simulation world. 
* aruco_mapping: The package for QR Marker detection. With QR marker, the robot can walk to the marker and pick object up with kinova arm.
* kimm_spot_kinova: HQP controller for whole-body motion.
* spot_kinova_description: description files.
* spot_kinova_framework: a controller launcher with kimm_spot_kinova.
* spot_kinova_msgs: action and message definitions.

## Tip
Open bashrc
```
gedit ~/.bashrc
```
Add the following line
```
alias cm='catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0'
export spot_kinova=~/$(your workspace)/src/kimm_spot_kinova/spot_kinova_framework/python_client
```
Then, you can build catkin_project with python3. (Type just `cm` in workpace)
And you can easily move the simulation foler. (Type cd $spot_kinova)

## Prerequisite
### For Ros with Python3
```
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-noetic-desktop-full --fix-missing
sudo apt install ros-noetic-pinocchio 
sudo rm /etc/ros/rosdep/sources.list.d/ -R
sudo -H pip3 install rosdep
sudo rosdep init
rosdep update
```
### For Spot-ros
```
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
pip3 install empy
pip3 install cython
git clone https://github.com/ggory15/spot_ros/ --recursive
cd ..
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

### For Champ
```
sudo apt install ros-noetic-hector-sensors-description
sudo apt install ros-noetic-robot-localization
sudo apt install ros-noetic-move-base ros-noetic-effort-controllers
sudo apt install ros-noetic-gmapping ros-noetic-amcl
sudo apt install ros-noetic-ecl-license
git clone https://github.com/chvmp/champ --recursive
git clone https://github.com/chvmp/champ_teleop --recursive
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

### For PyYaml
```
sudo pip3 install pyyaml -U
```

### For Aruco
```
sudo apt install ros-noetic-aruco
```

### For Kinova-RBDL
```
sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base
sudo apt-get install ros-noetic-rgbd-launch
git clone https://github.com/Kinovarobotics/ros_kortex_vision --recursive
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

### For MechMind
```
sudo apt install ros-noetic-rosbridge-server
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

## Install for This Package
```
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

## Unittest
Open Terminal (Warning: Please, don't use terminel in Vscode.)
```
roslaunch kimm_spot_kinova unittest_spot_kinova.launch
```
Then, you can see the movement of the spot.

or
```
roslaunch kimm_spot_kinova unittest_joy_spot_kinova.launch
```
Then, you can move the spot with PS5 controller.

## QuickStart for Simulation
Open Terminal (Warning: Please, don't use terminel in Vscode.)
```
roslaunch spot_kinova_framework simulation.launch
```
Open Another Terminal, and go to the "/spot_kinova_framework/python_client/" folder
```
python3 simulation.py
```
you can type "home", "walk", "reach", and so on.
You can also move spot with kinova on rviz (press 2D Nav Goal in Rviz)

## QuickStart for Real Robot
Open Terminal (Warning: Please, don't use terminel in Vscode.)
```
roslaunch spot_kinova_framework real.launch
```
Open Another Terminal, and go to the "/spot_kinova_framework/python_client/" folder
```
python3 real.py
```
you can type "home", "walk", "reach", and so on.

## With State Machine
Open Terminal (Warning: Please, don't use terminel in Vscode.)
```
roslaunch spot_kinova_framework simulation.launch
rosrun spot_kinova_smach task_manager_simul.py
```
Then, you can start state transition with rostopic msg.
```
rostopic pub /spot_kinova/state_transition std_msgs/String "data: 'pick'"
```