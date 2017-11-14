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
rostopic pub topic-name topic-type data...
```

## Gazebo Topics
To get the position of the robots use the topic /gazebo/model_states of type link_states.

To set the steering value for the robot use the topic: /manual_control/steering of type std_msgs::Int16. The posible values for the steering angle are: [-30, 30]

To set the steering angle (in degrees) for the robot use the topic: /manual_control/speed of type std_msgs::Int16. The posible values for the steering angle are: [-30, 30]


## Git
Git usually comes built-in with the main OS's. 
If it is not install or a newer version is needed, it can be downloaded from https://git-scm.com/downloads

This guide was tested in Linux.

### Cloning the repository
1. Open Terminal
2. cd to where the repository will be downloaded
3. git clone https://github.com/EagleKnights/SDI-11911.git --single-branch
4. cd into the downloaded repository

### Branches
Git uses branches to keep different versions of the code in the same repository.
The main branch is call "master" and in this repository only administrator will be able to make changes to it. After making git clone, the main branch will be "master".
To make a new branch, be sure to be inside the repo and type: 
```
git branch NEW_BRANCH
```
This only creates tha branch, to switch into the new branch:
```
git checkout NAME_OF_THE_BRANCH
```

In this new branch is where changes can be made and push to github.

### Add - Commit - Push
The basic workflow to save changes into git is:
```
git add FILES_TO_ADD
git commit -m "Usefull comments about this commit"
```
After executing git commit, the changes are store

NOTE: if the option -m is not use, git will open VIM to enter the comment. A VIM cheat-sheet can be found at http://vim.rtorr.com/ .

After executing commit, the changes are store locally. To store the changes to the repo at github, use:
```
git push origin BRANCH_TO_PUSH
```

### Pull
If there are new changes to the main github repo (called origin), use the following command (be sure to save your last changes):
```
git pull
```

### Other Stuff
* To print commit history
```
git log
```

* Graphically show the branches
```
git log --oneline --decorate --graph --all
```

* Git Manual
```
man git
```

* Official git web page: https://git-scm.com/

* Pro Git Book Available at: https://git-scm.com/book/en/v2
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





