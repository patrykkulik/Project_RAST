## Full Guide to Setting up and Simulating an Autonomous Drone

- This document is a step-by-step guide to setting up and designing a simple autonomous drone
- The full process is run using an Ubuntu Linux environment

###### Links to original content by Intelligent Quads:

[Installing Ardupilot](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/Installing_Ardupilot.md)

[Installing QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_qgc.md)

[Installing Gazebo and ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_gazebo_arduplugin.md)

[Installing ROS and Setting up Catkin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/installing_ros.md)

[ROS Introduction with Autonomous Drones](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/ros_intro.md)

[Guidance Navigation and Control](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/gnc_tutorial.md)

[ArduCopter and MAVproxy Commands](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/ardu_params_and_commands.md)

[Gazebo World Modelling](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/gazebo_world_modeling_intro.md)

[YOLO Image Recognition](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/intro_to_yolo.md)

[First Subscriber in ROS](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/basic_ros_sub.md)

[Search and Rescue Mission](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/search_and_rescue.md)

[Adding a Sensor in Gazebo](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/adding_a_sensor.md)





#### Installing Ardupilot and MAVProxy

Install git and use to clone the Ardupilot repository:

```bash
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-3.6
git submodule update --init --recursive
```

Necessary Python dependencies:

```bash
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

Install MavProxy using pip:

```bash
sudo pip install future pymavlink MAVProxy
```

Edit the bashrc file with the following:

```bash
gedit ~/.bashrc # Opens bashrc file to edit
export PATH=$PATH:$HOME/ardupilot/Tools/autotest  # Adds the lines
export PATH=/usr/lib/ccache:$PATH
. ~/.bashrc
```

Set parameters by running SITL (software in the loop):

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```



#### Installing QGroundControl

Downloading the AppImage:

```bash
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```

Change permissions and launch application:

```bash
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage
```

To connect SITL with QGroundControl:

```bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py
```



#### Installing Gazebo and ArduPilot Plugin

Setup to allow packages:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Update/Upgrade software:

```bash
sudo apt update
sudo apt upgrade
```

Install Gazebo (Skip if already installed through Dr.Kovac's process):

```bash
sudo apt install gazebo9 libgazebo9-dev
```

###### Installing ArduPilot Plugin:

```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

Edit bashrc file:

```bash
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

To run the Gazebo simulator (opens up a runway with an Iris drone):

```bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

Ensure that in the console that opens up the following lines appear (will not work properly otherwise):

```
APM: EKF2 IMU0 is using GPS
APM: EKF2 IMU1 is using GPS
```



#### Installing ROS (Robot Operating Software)

Follow all steps to install ROS Melodic from this [link](http://wiki.ros.org/melodic/Installation/Ubuntu). Ensure to use Desktop-Full install at Step 1.4

Set Up Catkin Workspace:

```bash
# Sets up Workspace
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
# Initialises Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

###### Installing Dependencies

```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src

# Installing mavros and mavlink from source
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

Edit bashrc file:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

###### Clone Intelligent Quads Simulation

```bash
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
```

Build Catkin:

```bash
cd ~/catkin_ws
catkin build  # Builds catkin
source ~/.bashrc  # Update global variables after building
```



#### Introduction to ROS with Autonomous Drones

Launch Gazebo runway world with ROS:

```bash
roslaunch iq_sim runway.launch
```

Copy to root directory and launch SITL:

```bash
cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~
~/startsitl.sh  # Use this from now on to launch SITL
```

To see the drone data being published (Don't run any new commands in a currently running terminal. Create a new terminal and run where a $ prefix can be seen):

```bash
rostopic echo /gazebo/model_states
```

A simple command to the drone. In the maxproxy terminal (`./startsitl` terminal):

```bash
mode guided
arm throttle
takeoff 15
```

Starting mavros to retrieve telemetry data (new terminal):

```bash
roslaunch iq_sim apm.launch
```

Some useful commands:

```bash
rostopic echo /mavros/global_position/local     # Position of drone in local frame
rostopic list -v /mavros/global_position/local  # Data available to C++ programs
rosmsg show nav_msgs/Odometry										# Show structure of message
```



#### C++ GNC (Guidance Navigation Control)

Simple program for allowing drone to take off, fly to waypoints and land

Download Sublime Text: (other text editors are good too)

```bash
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
```

Clone GNC ROS Package:

```bash
git clone https://github.com/Intelligent-Quads/iq_gnc.git
```

Add executable to CMakeLists.txt

```cmake
add_executable(square src/square.cpp) # Or the appropriate project/cpp file
target_link_libraries(square ${catkin_LIBRARIES})
```

Example Code (with comments below). A less commented version of gnc_tutorial is in the iq_gnc/src directory:

```cpp
#include <gnc_functions.hpp>
// Include API 

int main(int argc, char** argv)
{
	// Initialize ROS 
	ros::init(argc, argv, "gnc_node");  // Initialised a node handler
	ros::NodeHandle gnc_node;
	
	// Initialize control publisher/subscribers
	// Publishes push data away from node, subscribers pull data into the node
	init_publisher_subscriber(gnc_node);

  	// Wait for FCU connection
	wait4connect();

	// Wait for used to switch to mode GUIDED
	wait4start();

	// Create local reference frame 
	initialize_local_frame();

	// Request Takeoff
	takeoff(3);  // Height 3 meters

	// Specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;

	// Specifying square path
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	// Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. 
	// Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);  // Measured in Hz
	int counter = 0;
	// ROS function to only run when ROS is still operating normally
	while(ros::ok())
	{
		// Allows all publishers and subscribes to push and pull data 
		ros::spinOnce();
		// Enforce rate of spinning 
		rate.sleep();
		// If condition to check if the waypoint is reached
		if (check_waypoint_reached(.3) == 1)
		{
			// If waypointList hasn't reached the end
			if (counter < waypointList.size())
			{
				// Set the next destination to the next waypoint location
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			} else {
				// Land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;
}
```

###### Building and Running Code

Build finished code:

```bash
cd ~/catkin_ws
catkin build     # Builds code
source ~/.bashrc # Updates global variables
```

Running the code:

```bash
roslaunch iq_sim runway.launch  # Starts up ArduCopter
# New Terminal
./startsitl.sh                  # Starts up Gazebo
# New Terminal
roslaunch iq_sim apm.launch     # Starts ArduCopter-Mavros communiation
# New Terminal 
rosrun iq_gnc square            # Runs the program (change square to name of program)
# Not the project name from top of CMakeLists, the name of the file attached to add_executable
# E.g. for "add_executable(gnc_example src/gnc_tutorial.cpp)", use gnc_example
```

Finally, go to the MAXproxy terminal and initiate guided mode to start the program:

```bash
mode guided 
```



#### Setting Up Gazebo World

Install Mercurial (similar to Git):

```bash
sudo apt install mercurial
```

Retrieve many Gazebo models:

```bash
hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
```

Update bashrc file:

```bash
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
source ~/.bashrc
```

Create new world in the following workspace (using text editor, call it hills.world):

```bash
cd ~/catkin_ws/src/iq_sim/worlds/
```

Copy this sdf code into the file:

```html
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.9</erp>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>-1</real_time_update_rate>
      <!-- <max_step_size>0.0020</max_step_size> -->
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="iris">
      <include>
        <uri>model://iris_with_standoffs_demo</uri>
      </include>

      <!-- add new camera -->
      <link name='camera'>
        <pose>0 -0.01 0.070 .8 0 1.57</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>0.025</length>
            </cylinder>
          </geometry>
           <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

        <sensor name="camera" type="camera">
          <pose>0 0 0 -1.57 -1.57 0</pose>
          <camera>
            <horizontal_fov>1.0472</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>1000</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>true</visualize>

         <!--  <plugin name="irlock" filename="libArduCopterIRLockPlugin.so">
              <fiducial>irlock_beacon_01</fiducial>
          </plugin> -->
          <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>0.0</updateRate>
          <cameraName>webcam</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>camera_link</frameName>
          <hackBaseline>0.07</hackBaseline>
          <distortionK1>0.0</distortionK1>
          <distortionK2>0.0</distortionK2>
          <distortionK3>0.0</distortionK3>
          <distortionT1>0.0</distortionT1>
          <distortionT2>0.0</distortionT2>
      </plugin>

        </sensor>

      </link>

      <!-- attach camera -->
      <joint type="revolute" name="base_camera_joint">
        <pose>0 0 0.0 0 0 0</pose>
        <parent>iris::iris_demo::gimbal_small_2d::tilt_link</parent>
        <child>camera</child>
        <axis>
          <limit>
            <lower>0</lower>
            <upper>0</upper>
          </limit>
          <xyz>0 0 1</xyz>
          <use_parent_model_frame>true</use_parent_model_frame>
        </axis>
      </joint>
    </model>

  </world>
</sdf>
```

Create a launch file inside `~/catkin_ws/src/iq_sim/launch` called `hills.launch` and copy this in:

```html
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find iq_sim)/worlds/hills.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
```

Launch the new world using:

```bash
roslaunch iq_sim hills.launch
```

Go to the `Insert` tab and click on `Winding Valley Heightmap` to add it. Delete the flat plane. Add other objects as desired (person, pickup truck etc)

Use Save As and navigate to the worlds directory to overwrite the current world



#### YOLO Image Recognition

Installing CUDA (For computers with GPU)

```bash
sudo apt install nvidia-cuda-toolkit
```

Clone Darknet/YOLO:

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```

Build Darknet:

```bash
catkin build -DCMAKE_BUILD_TYPE=Release  # For all others
catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-6  # For Ubuntu 18.04
```

IF error arises from building (Something about a GCC-6 Compiler error), run this and repeat:

```bash
sudo apt-get update -y
sudo apt-get install -y gcc-6
```

##### YOLO Configurations

Find the file `ros.yaml` in the `darknet_ros/darknet_ros/config` directory and change `/camera/rgb/image_raw` to `/webcam/image_raw`

Find the `darknet_ros.launch` file inside the `launch` directory, and on line `14` note that different versions of YOLO can be configured:

`yolov1-tiny`,  `yolov2-tiny`, `yolov3-tiny`, or without the tiny, which is slower but more accurate. 

Recommended is `yolov2-tiny`

Returning to the `config` directory, in each `.yaml` file the threshold of image detection accuracy can be set

###### Starting up and using YOLO

Open Gazebo and SITL:

```bash
roslaunch iq_sim hills.launch  # Starts up ArduCopter
# New Terminal
./startsitl.sh                  # Starts up Gazebo
# New Terminal
roslaunch darknet_ros darknet_ros.launch  # Starts up Darknet
```

A terminal will open showing the images that the drone is detecting, and a view of the camera

Now start flying around and see what the drone detects:

```bash
mode GUIDED
arm throttle
takeoff 10
```



#### Subscriber with ROS

Within the `iq_gnc` directory create a new `.cpp` file called `sub.cpp`

Add the executable to the `CMakeLists.txt` file:

```cmake
add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
```

Enter this code into th `sub.cpp` file:

```cpp
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// Function to print ot what object YOLO is seeing
void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	// Bounding boxes is an array of bounding boxes, use a for loop
	for (int i=0; i < msg -> bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
	}	

}


int main(int argc, char **argv)
{
	// Compulsory for all ROS applications
	ros::init(argc, argv, "detection_sub");
	// Creating the ROS node handle
	ros::NodeHandle n;

	// ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

	ros::spin();

	return 0;
}
```

Now again using `catkin build` and then launch the file:

```bash
# New terminal
cd catkin_ws/
catkin build  # Build the new code

roslaunch iq_sim hills.launch  						# Opens gazebo
./startsitl.sh  												  # Run ArduCopter SITL (New Terminal)
roslaunch darknet_ros darknet_ros.launch  # Opens YOLO (New Terminal)

rosrun iq_gnc sub													# Runs the program (New Terminal)
```

Then navigate the drone however you like using MAVproxy commands and watch for objects being detected



#### Search and Rescue

Integrates a lot of the previous sections into a search and rescue operation

Open a new `.cpp` file inside `src/iq_gnc` called `sr.cpp` 

Add the executable to the CMake file:

```cmake
add_executable(sr src/sr.cpp)
target_link_libraries(sr ${catkin_LIBRARIES})
```

Insert the following `cpp` code into the file (comments included):

```cpp
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>

// mode_g denotes the flight opperations
//		0 - search
//		1 - rescue 
int mode_g = 0;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
		// Similar to sub, but now we are looking for a person
		if(msg->bounding_boxes[i].Class == "person")
		{
			mode_g = 1; 
			ROS_INFO("Person found. Starting Rescue Operation");
		}
	}	
}


int main(int argc, char **argv)
{
	// Initialize ROS 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
	// Initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// Wait for FCU connection
	wait4connect();

	// Wait for used to switch to mode GUIDED
	wait4start();

	// Create local reference frame 
	initialize_local_frame();

	// Request takeoff
	takeoff(10);


	// Specify some waypoints (Winding path)
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	float range = 50;
	float spacing = 10;
	int rows = 5; 
	int row;
	for(int i=0; i<5; i++)
	{
		row = i*2; 
		nextWayPoint.x = row*spacing;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = row*spacing;
		nextWayPoint.y = range;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = (row+1)*spacing;
		nextWayPoint.y = range;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);

		nextWayPoint.x = (row+1)*spacing;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
	}
	
	// Specify control loop rate. We recommend a low frequency to not over load the FCU with messages.
	// Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		if(mode_g == 0)
		{
			ros::spinOnce();
			rate.sleep();
			if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;	
				} else{
					// Land after all waypoints are reached
					land();
				}	
			}
		}
		if(mode_g == 1)
		{
			land();
			ROS_INFO("Landing Started");
			break;
		}	
		
	}
	return 0;
}

```

Save, build and run:

```bash
# New terminal
cd catkin_ws/
catkin build  # Build the new code

roslaunch iq_sim hills.launch  						# Opens gazebo
./startsitl.sh  												  # Run ArduCopter SITL (New Terminal)
roslaunch iq_sim apm.launch               # Initiate MAVlink communications (New Terminal)
roslaunch darknet_ros darknet_ros.launch  # Opens YOLO (New Terminal)

rosrun iq_gnc sr													# Runs the program (New Terminal)
```

Watch the drone follow its search path and land once the person is spotted!



#### Adding a Sensor in Gazebo

Open up the `runway.world` file in the `iq_sim/worlds` directory

Add the following code just before the closing model tag to add a LIDAR to the drone:

```html
<!--add lidar-->
<link name="hokuyo_link">
<pose>0 0 0 0 0 0</pose>
<collision name="collision">
  <pose>0 0 0.3 0 0 0</pose>
  <geometry>
    <box>
      <size>0.1 0.1 0.1</size>
    </box>
  </geometry>
</collision>
<visual name="visual">
  <pose>0 0 0.27 0 0 0</pose>
  <geometry>
    <mesh>
      <uri>model://hokuyo/meshes/hokuyo.dae</uri>
    </mesh>
  </geometry>
</visual>
<inertial>
  <mass>0.016</mass>
  <inertia>
    <ixx>0.0001</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>0.0001</iyy>
    <iyz>0</iyz>
    <izz>0.0001</izz>
    <!-- low intertia necessary to avoid not disturb the drone -->
  </inertia>
</inertial>

<sensor type="ray" name="laser">
  <pose>0 0 0.3 0 0 1.57</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.141593</min_angle>
        <max_angle>3.141593</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
      <resolution>0.1</resolution>
    </range>
    <!-- <noise>
<type>Gaussian</type>
<mean>0.0</mean>
<stddev>0.01</stddev>
</noise> -->
  </ray>
  <plugin name="hokuyo_node" filename="libgazebo_ros_laser.so">
    <robotNamespace></robotNamespace>
    <topicName>/spur/laser/scan</topicName>
    <frameName>/hokuyo_sensor_link</frameName>
  </plugin>
</sensor>
</link>

<joint name="hokuyo_joint" type="fixed">
  <pose>0 0 0 0 0 0</pose>
  <parent>iris::iris_demo::iris::base_link</parent>
  <child>hokuyo_link</child>
</joint>
```

Start the runway world and observe the LIDAR sensor rays:

```bash
roslaunch iq_sim runway.launch
```

You can place objects in the way and observe the updates rays. This command shows the sensor ray distances:

```bash
rostopic echo /spur/laser/scan
```



#### Obstacle Avoidance

Open a new `.cpp` file inside `src/iq_gnc` called `avoidance.cpp`

Add exectuable to `CMakeLists.txt`:

```cmake
add_executable(avoidance_sol src/avoidance_sol.cpp)
target_link_libraries(avoidance_sol ${catkin_LIBRARIES})
```

Copy this `cpp` code into the avoidance file (functionality is commented):

```cpp
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// Using this ROS message
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>


// Callback function for avoidance algorithm
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Initialise variable
	sensor_msgs::LaserScan current_2D_scan;
  	current_2D_scan = *msg;
	
	// Initialise avoidance force vectors and boolean for avoiding
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;
	
	// Start all 
	for (int i=1; i<current_2D_scan.ranges.size(); i++)
	{
		// Initialise distance from when avoidance should begin
		float d0 = 3; 
		// Proportional factor for avoidance
		float k = 0.5;

		// If within certain range of distance from object
		if (current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .35)
		{
			// Begin to avoid, set avoidance vectors
			avoid = true;
			float x = cos(current_2D_scan.angle_increment*i);
			float y = sin(current_2D_scan.angle_increment*i);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;

		}
	}

	// Retrieve current drone heading and define avoidance vectors
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

	if(avoid)
	{
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y, 2, 0);	
	}
	

}

int main(int argc, char **argv)
{
	// Initialize ROS 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;  // Same process as before

	// Similar format; subscriber name, nodehandler.subscribe(event... etc)
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, scan_cb);
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	
	// Initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// Wait for FCU connection
	wait4connect();

	// Wait for used to switch to mode GUIDED
	wait4start();

	// Create local reference frame 
	initialize_local_frame();

	// Request takeoff
	takeoff(2);

	set_destination(0,0,2,0);
	// Specify control loop rate. We recommend a low frequency to not over load the FCU with messages.
	// Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
```

Save and do the same thing to build the code and run the project:

```bash
# New terminal
cd catkin_ws/
catkin build  # Build the new code

roslaunch iq_sim lidar.launch  		  			# Opens gazebo
./startsitl.sh  												  # Run ArduCopter SITL (New Terminal)
roslaunch iq_sim apm.launch								# Initiate MAVlink communications (New Terminal)

rosrun iq_gnc avoidance										# Runs the program (New Terminal)
```

