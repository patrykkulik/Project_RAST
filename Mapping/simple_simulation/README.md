# Simple Simulation Setup 

Follow the instructions below to setup a simple simulation environment for testing of sensors.
 
# Install Mavros, Mavproxy and Ardupilot 

Follow instructions in these links:

```
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/Installing_Ardupilot.md
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_gazebo_arduplugin.md
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_ros.md
```
If you have already set up the workspace, you don't have to set up things again.

Once everything has been set up, download the mapping_team folder to your workspace and build the workspace

Launch the world using:

```
roslaunch mapping_team droneOnly.launch
```

Launch autopilot using:

```
roslaunch mapping_team atm.launch
```
Launch SITL using:

```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

