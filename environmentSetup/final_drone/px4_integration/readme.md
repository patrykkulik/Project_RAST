#### Instructions for Building GDP Drone with PX4 Integration - Interim Solution

Tan Tai Ming, tmt111@ic.ac.uk
For Imperial College London, Group Design Project 2020, Autonomous Drone Delivery

This folder is designed to run as a Catkin Package and hence will be required to reside in your Catkin workspace.

Required packages (4): PX4, ROS "Melodic" (Melodic is preferred), Gazebo 9 simulator and MAVROS
Tested to work with ROS "Melodic".

Also required to install Hector:

```bash
sudo apt-get install ros-melodic-hector-gazebo-plugins
```

Install ROS Effort controllers:
```bash
sudo apt-get install ros-melodic-effort-controllers
```
Install geographic:
```bash
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
```

If the above fails, instead try:

```bash
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```

Also try:
```bash
sudo geographiclib-get-geoids egm96-5
```


After cloning the git folder:
1. Cut and paste the ENTIRE `gdp2020_add` folder into the source folder of your `catkin` workspace. (for the utilisation of the gdp2020 motor plugin. Interim solution)
2. Cut and paste the ENTIRE `final_drone` folder into the source folder of your `catkin` workspace.
(usually `~/catkin_ws/src`)

In your Catkin Workspace,
Run `catkin build final_drone` to build the package

In Terminal,
```bash
cd ~/catkin_ws/src/final_drone/px4_integration
bash gdp2020_interim.sh
```

## Launch QGroundControl
Open QGroundControl by selecting/double-click on the icon where you saved it.

## Launch Drone with PX4

```bash
roslaunch px4 gdpdrone.launch 
```

## Using QGroundControl
You should see the drone connected to QGroundControl interface.
Create a mission with take-off -> waypoints -> land.
Within the take-off tab, ensure to check the speed and set speed to 2 m/s. (Do not place speed > 5 m/s as it WILL cause the drone to de-stabilise mid-flight and crash.
