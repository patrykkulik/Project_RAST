# How to use these files:

* First please install PX4 as instructed in the PX4 Setup folder.

* Put the launch scripts inside the PX4 launch folder.

* Put the worlds file inside PX4 Firmwares/Tools/sitl_gazebo/worlds folder.

* Put the models inside Firmwares/Tools/sitl_gazebo/models folder.

# Running sequence:

```
roslaunch px4 indoor3.launch
roslaunch px4 realsense_rtabmap_2.launch
python (PATH TO SCRIPTS FOLDER) rtabmap_transfer.py iris (This file publishes pose info to mavros/vision/pose)
python (PATH TO SCRIPTS FOLDER) multirotor_communication.py iris 0 (Communication node)
python (PATH TO SCRIPTS FOLDER) multirotor_keyboard_control.py iris 1 vel (Keyboard input)
```
