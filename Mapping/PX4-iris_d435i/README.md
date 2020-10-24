# Launch mavros & px4 with the iris model & Intel Realsense d435 attached 

- All the configurations must happen inside the Px4 Firmware repository. If you don't have it clone it from git & run make 

- A similar approach can be used for any custom model.

# Pre-requisites 

You must have configured the realsense_gazebo_plugin package i.e either have librealsense_gazebo_plugin.so in /opt/ros/melodic/lib or your catkin_ws/devel/lib and source accordingly! 

1. Copy the two folders {iris_d435, realsense_d435} in path/to/your/Firmware/Tools/sitl_gazebo/models

2. Copy the file 1078_iris_d435 to path/to/your/Firmware/ROMFS/px4fmu_common/init.d-posix

3. In file .../Firmware/platforms/posix/sitl_target.cmake Look for the line that starts with set(models ... 

Append this list with: iris_d435

It should look like this: 
```
set(models none shell
	if750a iris iris_opt_flow iris_vision iris_rplidar iris_irlock iris_obs_avoid solo typhoon_h480 iris_d435
	plane
	standard_vtol tailsitter tiltrotor
	hippocampus rover)
```

# Source/export your paths:
the most easy way is to run nano ~/.bashrc and add:
```
source /opt/ros/melodic/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib/:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=path/to/your/Firmware/Tools/sitl_gazebo/models/:${GAZEBO_MODEL_PATH}
source .../Firmware/Tools/setup_gazebo.bash .../Firmware .../Firmware/build/px4_sitl_default
```

4. In .../Firmware/launch/mavros_posix_sitl.launch:

change line 14:
```
<arg name="vehicle" default="iris_d435"/>
```

change line 26 where local_host is your IP address:
```
<arg name="fcu_url" default="udp://:14540@local_host:14557"/>
```


5. To run the simulation: 

while in Firmware folder:
```
DONT_RUN=1 make px4_sitl_default gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```

# NB:

 If there are any issues after doing this you may want to take a look @ .../Firmware/Tools/setup_gazebo.bash and source appropriately 

 
