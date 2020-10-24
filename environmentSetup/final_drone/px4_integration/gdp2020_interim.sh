#!/bin/bash

#Written by Tan Tai Ming, tmt111@ic.ac.uk
#As in interim solution to PX4 and QGroundControl controller integration with the GDP2020 Drone
#3 June 2020

cp gdpdrone.xacro ~/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf
cp gdpdrone_base.xacro ~/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf
cp multirotor_base_gdp2020.xacro ~/Firmware/Tools/sitl_gazebo/models/rotors_description/urdf
cp gdpdrone.launch ~/Firmware/launch
cp gdpdrone_spawn.launch ~/Firmware/launch

echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/final_drone/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/gdp2020_add/gdp2020_cpp/build' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo' >> ~/.bashrc
