## INSTALLATION 
```
sudo apt-get install ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-common-plugins
# Install the dependency of april_docking as well, refer to its compile guide
# Put the source code in your catkin_ws/src
catkin_make
```

## Running
```
rqt --standalone rqt_docking_helper
```
