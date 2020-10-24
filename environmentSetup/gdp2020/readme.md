#Final Drone Model for GDP 2020 Autonomous Drone Delivery - Project RAST

This package will load the GDP2020 Custom Drone in a flooded environment with a single house, controlled on PX4 / QGroundControl. There is a survivor stuck on the first floor and moving debri floating on the ground floor. From the launch raft, the UAV will encounter a tree as an obstacle.

This folder is designed to run as a Catkin Package and hence will be required to reside in your Catkin workspace.

Required packages (4): PX4, ROS "Melodic" (Melodic is preferred), Gazebo 9 simulator and MAVROS
Tested to work with ROS "Melodic".

# Option 1 (Automatic):
# NOT IN USE
Run in terminal:
```bash
bash gdp2020_load.sh
```

# Option 2 (Manual):
Git Address: https://github.com/mirkokovac/GDP2020/tree/Environment_Setup

After cloning the git folder, cut and paste the ENTIRE gdp2020 folder into the source folder of your catkin workspace.
(usually ~/catkin_ws/src)

Run catkin build to build the package:

```bash
catkin build gdp2020
```
This action builds the gdp2020 workspace.

Run in terminal:

```bash
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/gdp2020/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/gdp2020/include/lib' >> ~/.bashrc
. ~/.bashrc
```
This action will link the MODELS and LIB directories to the Gazebo.
-------------------------------------------------------------------------------------------
Command to load: 

```bash
roslaunch gdp2020 gdp2020.launch
```

Retrieve the rostopics via 'rostopic list'
Obtain the live readings via 'rostopic echo <topic>'
Obtain topic information via 'rosmsg show <msgtype>' (e.g. sensor_msgs\Range)

## If required ##
```bash
sudo apt install libeigen3-dev
sudo apt install ros-melodic-hector-gazebo-plugins
```
Geographic Set:
```bash
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```

Written by Tan Tai Ming. 11 June 2020
