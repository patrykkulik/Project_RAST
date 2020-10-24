# Adaptive 3D Greedy D* Lite 

### Requirements:
The following should be installed according to instructions from Mapping and Simulation
* Python 2.7 (scripts must be written in Python 2.7) 
* Ubuntu
* ROS, Gazebo
* Matplotlib
* rospy

This needs to be installed as well:
* `ros-numpy` 
    * `git clone https://github.com/eric-wieser/ros_numpy `
    * `cd ros_numpy`
    * `python setup.py install`

### Integration with ROS instructions:
 
* Navigate into the catkin workspace source `cd ~/catkin_ws/src`
* Navigate into package directory `cd packageName` (replace as appropriate)
* Copy contents of this folder into the package root directory
* Make the python script `integration.py` executable with `chmod +x integration.py`
* `cd ~/catkin_ws`
* Build catkin workspace `catkin build`
* `source ~/.bashrc`
* `source devel/setup.bash`
* `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware`
* `roslaunch packageName launchfile.launch`

Open a new terminal and run
* `source devel/setup.bash`
* `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware`

Now to run the script:
* `rosrun packageName integration.py`

### Test procedures:
For testing the SLAM integration:
* Ensure that a `rosbag` file containing a recording of SLAM data is available. An example file `2020-06-13-00-18-21.bag` is provided in this folder.
* In a terminal, run the following commands:
    * `cd ~/catkin_ws`
    * `catkin build`
    * `source ~/.bashrc`
    * `roscore`
    * `source devel/setup.bash`
    * `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware`
* In two additional terminals, run
    * `source devel/setup.bash`
    * `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware`
* Run the following two commands simultaneously in each of the two additional terminals:
    * `rosbag play 2020-06-13-00-18-21.bag` (must `cd` to the file location and change the filename to your `rosbag` file)
    * `rosrun packageName integration.py` (change the package name as appropriate)
    
Note: For convenience, it may be easier to run the `rosbag` file as a for-loop as so: 
* `for i in {1..30}; do rosbag play 2020-06-13-00-18-21.bag ; done`

The processed point cloud can be printed by uncommenting the following lines in `adaptive3.py move_to_goal()`

    for a in observation:
    if observation[a] == '#':
        (ax,ay,az) = (a[0]*self.real_world_multiplier,a[1]*self.real_world_multiplier,a[2]*self.real_world_multiplierZ)
        print("{},".format((ax,ay,az)))

This can be plotted in an application of your choice; a `plotter.py` file is provided as well. The actual point cloud can be viewed with Rviz (in a new terminal, `rosrun rviz rviz`; add a PointCloud2 display in the left pane with the topic set to `/rtabmap/cloud_obstacles`)