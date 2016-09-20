# Proyecto 2

# Gazebo
Install Gazebo5
```
sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control
```
This should install Gazebo5 with the packages for using gazebo with ROS
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

To run gazebo with the EKBot (omnidirection robot)
```
source devel/setup.bash
export GAZEBO_MODEL_PATH=PATH_TO_REPO/SDI-11911/Proyecto2/src/ #or include it in your ba
shrc
roslaunch ekbot_gazebo ekbot.launch #launches gazebo with EKBot in the (0,0,0)
```
Run each of the following commands in a different terminal (source devel/setup.bash needed in each one):

```
rosrun ekbot_ctrol robot_velocity_node #launches the node to convert x,y,w vel to motor
vel
rostopic pub /target_position_topic geometry_msgs/Twist -- '[-3,0,0]' '[0,0,0]' #topic to publish desired position [x,y,z] [p,r,w] (only x,y & w are used)
rosrun ekbot_ctrl robot_trajectory_node #node to calculate a simple trajectory
```

