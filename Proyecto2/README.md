# Proyecto 2

## Gazebo

Setup your computer to accept software from packages.osrfoundation.org and setup keys
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```
Install Gazebo5
```
sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control
```
This should install Gazebo5 with the packages for using gazebo with ROS. To check if Gazebo version 5.3.0 was install succesfully, run:
```
gazebo --version
gazebo
```
Gazebo should open, the first time might take a while.
## Setup

Clone the repository
```
git clone https://github.com/EagleKnights/SDI-11911.git
git checkout PERSONAL_BRANCH
```
(git pull can also be used).

Go into the new directories then initialize and build the workspace:
```
cd SDI-11911/Proyecto2/src/
catkin_init_workspace
cd ..
catkin_make
```

Verify there are no errors.

Copy the model to Gazebo directory, in a new terminal:
```
cd ~/.gazebo/models/
mkdir EKBot
cp PATH_TO_REPO/SDI-11911/Proyecto2/src/ekbot_description/model.* EKBot/
```

## Run gazebo with the EKBot 

```
source devel/setup.bash
roscore
roslaunch ekbot_gazebo ekbot.launch #launches gazebo with EKBot in (0,0,0)
```
Run each of the following commands in a different terminal (source devel/setup.bash in each one):

```
rosrun ekbot_ctrl robot_velocity_node #launches the node to convert x,y,w vel to motor
vel
rostopic pub /target_position_topic geometry_msgs/Twist -- '[3,0,0]' '[0,0,0]' #topic to publish desired position [x,y,z] [p,r,w] (only x,y & w are used)
rosrun ekbot_ctrl robot_trajectory_node #node to calculate a simple trajectory
```
## AutoNOMOS
The directories for the AutoNOMOS are: 
* autonomos_gazebo ==> model files for the simulation (the ones gazebo uses)
* autonomos_simulation ==> files for simulation camera output

### Run gazebo with AutoNOMOS
Copy all the files to the local gazebo dir, usually at ~/.gazebo/models
```
cp -r src/autonomos_gazebo/AutoNOMOS_* ~/.gazebo/models
```
There are diferent .launch files in src/autonomos_gazebo/launch/ and new ones can be created.
roscore must be running
```
roslaunch autonomos_gazebo FILE.launch 
```

### Run the simulation
There are three nodes that must be launch and gazebo must be running
```
rosrun autonomos_simulation tf2_broadcaster_node 
rosrun autonomos_simulation vision_node
rosrun autonomos_simulation perception_node
```
or with a .launch file
```
roslaunch autonomos_simulation ros_autonomos.launch
```
When launching the nodes, there might be some ERROR messages from vision_node, its normal since it needs to wait for the other nodes.
### AutoNOMOS simulation Nodes
![alt tag] (images/autonomos_simulation_nodes.png)

### AutoNOMOS laser
After launching gazebo with the autonomos-mini, verify that the laser can be seen (a blue circle with the robot in its center) and run:
```
rostopic list
```
The output should have a topic named "/laser_scan". The output is of type: "sensor_msgs/LaserScan", to see it run: 
```
rostopic echo -n1 /laser_scan 
```


## Using other robots (turtlebot)
1. The model must be first downloaded and placed in ~/.gazebo/models/.
2. The files in src/autonomos_gazebo/worlds/ can be changed (or add a new one) to directly spawn a speceific robot.
3. To be sure the points being detected by vision_node are the correct ones, after running gazebo with the robot and the scenario, run 
  ```
  rostopic echo -n1 /gazebo/link_states | more
  ```
4. This should list all the links in the model, for the autonomos_mini: 

  * ['ground_plane::link', 'ackermann::base_link', 'ackermann::back_left_wheel_link', 'ackermann::back_right_wheel_link', 'ackermann::front_left_bar_link', 'ackermann::front_left_wheel_link', 'ackermann::front_right_bar_link', 'ackermann::front_right_wheel_link', 'ackermann::steer_link', 'ackermann::ackermann_bar_link', 'AutoNOMOS_mini_intersection::field', 'AutoNOMOS_mini_intersection::L1_p1', ... ] 
5. Look for the first link named "AutoNOMOS[...]::L1_p1" and get its position (starting from 0), in this example is 11. Change if necessary the value of the variable "first_point" at src/autonomos_simulation/vision_node.cpp to this value.

## ROSBags
There are two bags:
* intersection.bag
* road.bag
To use them, run (while running roscore in a different terminal):
```
rosbag play BAG_TO_USE
```
To display all the topics published by the bag use:
```
rosbag info SOME_BAG
```
To view the rgb images, play the bag and run:
```
rosrun image_view image_view image:=/app/camera/rgb/image_raw
```

## Directories
* autonomos_gazebo     ==> files for gazebo
  * AutoNOMOS_curved_road  
  * AutoNOMOS_mini_Intersection  
  * AutoNOMOS_mini
  * AutoNOMOS_straight_road
  * launch 
  * worlds

* autonomos_simulation  ==> files to simulate vision
  * tf2_broadcaster_node.cpp
  * perception_node.cpp  
  * vision_node.cpp
  * launch 
* ekbot_description
  * model.config
  * model.sdf
* ekbot_ctrl
  * robot_trajectory_node.cpp 
  * robot_velocity_node.cpp
* ekbot_gazebo
  * launch 
  * worlds



## Links
Gazebo - http://gazebosim.org/tutorials
Point Cloud Library - http://docs.pointclouds.org/trunk/index.html
sensor_msgs/LaserScan - http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
