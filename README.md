# SDI-11911

## ROS 
ROS versions tested:
* indigo
* kinetic

## Gazebo
Gazebo versions tested:
* 5
* 7
To verify the gazebo version use:
```
gazebo --version
```
Launch Gazebo:
```
gazebo
```
Gazebo should be able to be launched using ROS:
```
roscore # At first terminal
rosrun gazebo_ros gazebo # At second terminal
```

## AutoNOMOS
The directories for the AutoNOMOS are: 
* autonomos_gazebo ==> model files for the simulation (the ones gazebo uses)
* autonomos_simulation ==> files for simulation camera output

## Setup
Gazebo needs the model files for the robots. 
```
cp EK_AutoNOMOS/src/autonomos_gazebo/models/* ~/.gazebo/models/
```
To verify that the models are correct, open gazebo and try to import the "AutoNOMOS_mini" model.

## Run the simulation
There are two nodes that must be launch and gazebo must be running
```
rosrun autonomos_simulation tf2_broadcaster_node 
rosrun autonomos_simulation autoNOMOS_gazebo_bridge
```
or with a .launch file
```
roslaunch autonomos_simulation ros_autonomos.launch
```

## Topics 
To see all the topics use:
```
rostopic list
```
To see the last output of a topic:
```
rostopic echo -n1 /topic_name
```
To publish to a topic from the command line:
```
rostopic pub 
```

## Gazebo Topics
To get the position of the robots use the topic /gazebo/model_states of type link_states.

To set the steering value for the robot use the topic: /manual_control/steering of type std_msgs::Int16. The posible values for the steering angle are: [-30, 30]

To set the steering angle (in degrees) for the robot use the topic: /manual_control/speed of type std_msgs::Int16. The posible values for the steering angle are: [-30, 30]

<!-- 
```
```
```
```
```
```
```
```
```
```
```
```
```
``` -->





