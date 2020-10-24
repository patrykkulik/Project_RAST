# How to use the keyboard controller

* Two scripts are provided, the communication script establishes communication of the drone with MAVROS and the keyboard controller publishes velocity commands. 

* The script was originally written for the XTDrone (https://www.yuque.com/xtdrone/manual_en) and thus topic names might be different. 

* For the sake of time I'm not changing each topic inside the script, instead we will match the topic names used inside the scirpts.

* Inside the px4 launch script, locate where MAVROS is launched and change to the following:
```
<!-- MAVROS -->
    <group ns="iris_0">
        <include file="$(find mavros)/launch/px4.launch">
            <!-- GCS link is provided by SITL -->
            <arg name="gcs_url" value=""/>
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="respawn_mavros" value="$(arg respawn_mavros)"/>
        </include>
    </group>
```
we basically add a grouping (iris_0) to all the MAVROS topics

* The scripts were originally designed for multi-drone communications and control, thus to launch it we need to pass some parameters:
```
python multirotor_communication.py iris 0
python multirotor_keyboard_control.py iris 1 vel
```
remember to launch these scripts inside the folder where the scripts are.
