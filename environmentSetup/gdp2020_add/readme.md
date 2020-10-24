#### Instructions for Building Iris Drone with Ultrasonic Sensors



This folder is created to test the ultrasonic sensors with the Iris model in preparation for GDP 2020.

This folder is designed to run as a Catkin Package and hence will be required to reside in your Catkin workspace.

Required packages (4): PX4, ROS "Melodic" (Melodic is preferred), Gazebo 9 simulator and MAVROS
Tested to work with ROS "Melodic".

Also required to install Hector:

```bash
sudo apt-get install ros-melodic-hector-gazebo-plugins
```



After cloning the git folder, cut and paste the ENTIRE `gdp2020_add` folder into the source folder of your `catkin` workspace.
(usually `~/catkin_ws/src`)

Run `catkin build` to build the package

Run in terminal:

```bash
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/gdp2020_add/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/gdp2020_add/gdp2020_cpp/build' >> ~/.bashrc
. ~/.bashrc
```

Command to load: 

```
roslaunch gdp2020_add gdp2020.launch
```

You will load an empty world, with a ground plane and an Iris model with 8 ultrasonic sensors attached for distance measurement. PX4 will also loaded and you will be able to command the drone using PX4.
Min range 0.01, Max range 1.0

###### Retrieving Sensor Data

To retrieve the rostopics run:

```bash
rostopic list
```

To retrieve live data from an ultrasonic sensor run:

```bash
rostopic echo /distance_0_bottom  # (Or the desired topic)
```

To list the publishers and subscribers, run:

```bash
rostopic info /distance_0_bottom  # (Or the desired topic)
```



All topics are ranges only.

These ultrasonic sensors are designed for the sole purposes and usage for the environmental simulation team.
Not intended for other purposes.

Written by Tai Ming. v1.0. 26 May 2020
Document Version Control:
Tai Ming v1.1 27 May 2020: Added PX4 integration, Added required package list
