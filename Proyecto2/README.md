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

## Important Files
The EKBot description file - src/ekbot_description/model.sdf
The launch file - src/ekbot_gazebo/launch/ekbot.launch 
The trajectory generator - src/ekbot_ctrl/robot_trajectory_node.cpp

## Links
Gazebo - http://gazebosim.org/tutorials

