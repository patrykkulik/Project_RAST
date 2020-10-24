#### Instructions for Building Iris Drone with Ultrasonic Sensors


final static version of drone.

This folder is designed to run as a Catkin Package and hence will be required to reside in your Catkin workspace.

Required packages (4): PX4, ROS "Melodic" (Melodic is preferred), Gazebo 9 simulator and MAVROS
Tested to work with ROS "Melodic".

Also required to install Hector:

```bash
sudo apt-get install ros-melodic-hector-gazebo-plugins
```



After cloning the git folder, cut and paste the ENTIRE `drone_statc` folder into the source folder of your `catkin` workspace.
(usually `~/catkin_ws/src`)

Run `catkin build` to build the package

Note: every time you launch a new terminal, you may need to run: 

```bash
source devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
```

Run in terminal:

```bash
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/drone_static/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/drone_static/lib' >> ~/.bashrc
. ~/.bashrc
```

Command to load a single drone: 

```
roslaunch drone_static single.launch
```




