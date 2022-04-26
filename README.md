# KIMM_SPOT_KINOVA PROJECT
IMPORTANT: This package is optimized on Ubuntu 18.04 with ROS-melodic and Python 3

## Tip
Open bashrc
```
gedit ~/.bashrc
```
Add the following line
```
alias cm='catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0'
export spot_kinova=~/$(your workspace)/src/kimm_spot_kinova/python/simulation_script
```
Then, you can build catkin_project with python3. (Type just `cm` in workpace)
And you can easily move the simulation foler. (Type cd $spot_kinova)

## Prerequisite
### For Ros with Python3

```
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-melodic-desktop-full --fix-missing
sudo apt install ros-melodic-pinocchio 
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
git clone https://github.com/clearpathrobotics/spot_ros/ --recursive
git clone https://github.com/ros/geometry2 --branch 0.6.5
cd ..
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

### For Champ
```
sudo apt install ros-melodic-hector-sensors-description
sudo apt install ros-melodic-robot-localization
sudo apt install ros-melodic-move-base ros-melodic-effort-controllers
sudo apt install ros-melodic-gmapping ros-melodic-amcl
sudo apt install ros-melodic-ecl-license
git clone https://github.com/chvmp/champ --recursive
git clone https://github.com/chvmp/champ_teleop --recursive
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make 
```

### For PyYaml
```
sudo pip3 install pyyaml -U
```

## Install
```
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0
```

## Quick Start
Open Terminal (Warning: Please, don't use terminel in Vscode.)
```
roslaunch kimm_spot_kinova unittest_spot_kinova.launch
```
Then, you can see the moving of the spot.

or
```
roslaunch kimm_spot_kinova unittest_joy_spot_kinova.launch
```
Then, you can move the spot with PS5 controller.

