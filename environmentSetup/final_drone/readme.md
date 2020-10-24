#### Instructions for Building Final Drone 


This folder is designed to run as a Catkin Package and hence will be required to reside in your Catkin workspace.

Required packages (4): PX4, ROS "Melodic" (Melodic is preferred), Gazebo 9 simulator and MAVROS
Tested to work with ROS "Melodic".

Also required to install Hector:

```bash
sudo apt-get install ros-melodic-hector-gazebo-plugins
```

Install ROS joint controllers:
```bash
sudo apt-get install ros-melodic-effort-controllers
```
Install geographic:
```bash
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
```

If the above fails, instead try:

```bash
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```

After cloning the git folder, cut and paste the ENTIRE `final_drone` folder into the source folder of your `catkin` workspace.
(usually `~/catkin_ws/src`)

Run `catkin build` to build the package

Add to `.bashrc`: 

```bash
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/final_drone/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc

echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/final_drone/models/final_drone:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
```

In directory: `~/catkin_ws/src/final_drone/models/final_drone`

Run in terminal: 

```bash
rosrun xacro xacro final_drone.xacro > final_drone.urdf
```


Commands to load in `catkin_ws`:

```bash
source devel/setup.bash
```



## Launches World and  Joint Controllers

```bash
roslaunch final_drone final_drone_empty.launch 
```

## To Rotate Arms
Type `rostopic list` to see available topics
For example to rotate arm_0 by 1.75 rads:
note: this makes it straight for some reason
```bash
rostopic pub /drone/arm_1_position_controller/command std_msgs/Float64 "data: 1.75"
```

Ethan's .bashrc file:

```bash
source /opt/ros/melodic/setup.bash
export SVGA_VGPU10=0
source ~/catkin_ws/devel/setup.bash 
export GAZEBO_MODEL_PATH=~/catkin_ws/src/gdp2020_add/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/home/ethan/Firmware/Tools/sitl_gazebo/models:${GAZEBO_MODEL_PATH}
source /home/ethan/catkin_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/Firmware/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Firmware/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Firmware/build/px4_sitl_default/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/gdp2020_add/gdp2020_cpp/build


export GAZEBO_MODEL_PATH=~/catkin_ws/src/final_drone/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/catkin_ws/src/final_drone/models/final_drone:${GAZEBO_MODEL_PATH}
```


Written by Ethan Hy
