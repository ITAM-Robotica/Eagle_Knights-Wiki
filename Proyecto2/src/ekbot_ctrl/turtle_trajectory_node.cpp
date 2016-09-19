#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <math.h>
#include <stdio.h>
#include <turtlesim/Pose.h>
#include <Eigen/Geometry>

using namespace Eigen;

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Twist robot_position;
geometry_msgs::Twist target_position;

//rate_hz assignment
double rate_hz = 30;

//Assign the position of the robot (from other topic) to robot_position.
//Assuming the topic that generate the robot position uses geometry_msgs::Twist
//and the information is in dummy.linear. This might need to be modifed
void getRobotPose(const geometry_msgs::Twist& msg) {
	robot_position.linear.x = msg.linear.x;
	robot_position.linear.y = msg.linear.y;
    robot_position.angular.z = msg.linear.z; 
}

//Assign the position of the target (from other topic) to target_position.
//Assuming the topic that generate the robot position uses geometry_msgs::Twist
//and the information is in dummy.linear. This might need to be modifed
void getTargetPose(const geometry_msgs::Twist& msg) {
	target_position.linear.x = msg.linear.x;
	target_position.linear.y = msg.linear.y;
    target_position.angular.z = msg.linear.z; 
}

//Function to determine if the goal is still far.
//Variables epsilon_x and epsilon_y used to determined if the goal has been reach
bool isGoalFar(geometry_msgs::Twist p_start, geometry_msgs::Twist p_goal) {
	double epsilon_x, epsilon_y;
	epsilon_x = 150; 
	epsilon_y = 150;
	double distance_x = abs(p_goal.linear.x - p_start.linear.x);
	double distance_y = abs(p_goal.linear.y - p_start.linear.y);
	if (distance_x > epsilon_x || distance_y > epsilon_y)
		return true;
	else
		return false;
}

//Function to generate a Linear Constant Velocity from robot's position to the target's position
geometry_msgs::Twist generateConstantVelocity(double constant_speed, geometry_msgs::Twist p_start, geometry_msgs::Twist p_goal){

    // Compute direction to goal
	Vector3d p_start_vector(p_start.linear.x,p_start.linear.y,p_start.angular.z);
	Vector3d p_goal_vector(p_goal.linear.x, p_goal.linear.y, p_goal.angular.z);
	Vector3d goal_direction_vector = p_goal_vector-p_start_vector;

    // Compute speed in the direction to goal
	Vector3d velocity_vector = constant_speed * (goal_direction_vector/ goal_direction_vector.norm());

	geometry_msgs::Twist velocity;

	velocity.linear.x = velocity_vector.x();
	velocity.linear.y = velocity_vector.y();
	velocity.angular.z = velocity_vector.z();

	return velocity;
}

//Function to generate an Angular Constant Velocity from robot's position to the target's position
geometry_msgs::Twist rotateVelocity(geometry_msgs::Twist velocity, double rotation_angle){

    // Compute direction to goal
	Vector3d original_vector(velocity.linear.x,velocity.linear.y,velocity.angular.z);

	geometry_msgs::Twist velocity_new;

	velocity_new.linear.x = original_vector.x() * cos(rotation_angle) - original_vector.y() * sin(rotation_angle);
	velocity_new.linear.y = original_vector.x() * sin(rotation_angle) + original_vector.y() * cos(rotation_angle);
	velocity_new.angular.z = original_vector.z();

	return velocity_new;
}

// Function to keep velocity under the allowed robot limits
geometry_msgs::Twist boundVelocity(geometry_msgs::Twist velocity) {

	double max_linear_speed = 30;
	double min_linear_speed = 0;
	double max_angular_speed = M_PI*4;
	double min_angular_speed = M_PI/16;

	if (velocity.linear.x > max_linear_speed)
		velocity.linear.x = max_linear_speed;
	else if (velocity.linear.x < -max_linear_speed)
		velocity.linear.x = -max_linear_speed;
	if (velocity.linear.y > max_linear_speed)
		velocity.linear.y = max_linear_speed;
	else if (velocity.linear.y < -max_linear_speed)
		velocity.linear.y = -max_linear_speed;
	if (velocity.angular.z > max_angular_speed)
		velocity.angular.z = max_angular_speed;
	else if (velocity.angular.z < -max_angular_speed)
		velocity.angular.z = -max_angular_speed;

    // Lower speed bounds
	if (velocity.linear.x > 0 && velocity.linear.x < min_linear_speed)
		velocity.linear.x = min_linear_speed;
	else if (velocity.linear.x < 0 && velocity.linear.x > -min_linear_speed)
		velocity.linear.x = -min_linear_speed;
	if (velocity.linear.y > 0 && velocity.linear.y<min_linear_speed)
		velocity.linear.y = min_linear_speed;
	else if (velocity.linear.y < 0 && velocity.linear.y > -min_linear_speed)
		velocity.linear.y = -min_linear_speed;
	if (velocity.angular.z >0 && velocity.angular.z < min_angular_speed)
		velocity.angular.z = min_angular_speed;
	else if (velocity.angular.z < 0 && velocity.linear.z > - min_angular_speed )
		velocity.angular.z = -min_angular_speed;

}


int main(int argc, char **argv){
	ros::init(argc,argv,"turtle_trajectory_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("turtle_trajectory_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	
	//Topic to publish velocity command, queue size equals rate_hz to keep up with the rate at which messages are generated,

    //Publish to the turtle topic "/turtle1/cmd_vel at rate_hz"
	ros::Publisher pub_vel_turtle = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", rate_hz);

	//Topics to acquire robot and target position (from the vision node) 
	ros::Subscriber sub_attacker_pos = nh.subscribe("/robot_position_topic", 1, &getRobotPose); 
	ros::Subscriber sub_ball_pos = nh.subscribe("/target_position_topic", 1, &getTargetPose);

    //Twist variable to publish velocity (trajectories)
	geometry_msgs::Twist desired_velocity;
	double tiempo = 0;

    //define the max speed
	double cruise_speed = 50;

    //define the rate
	ros::Rate rate(rate_hz);

	while (ros::ok())
	{
        //ROS_INFO_STREAM use for debugging 
		ROS_INFO_STREAM("Robot Position"
			<<",X,"<<robot_position.linear.x
			<<",Y,"<<robot_position.linear.y
			<<",W,"<<robot_position.angular.z);
		ROS_INFO_STREAM("Target position:"
			<<",X,"<<target_position.linear.x
			<<",Y,"<<target_position.linear.y
			<<",W,"<<target_position.angular.z);

        //Define is goal hasn't been reach
		if (isGoalFar(robot_position, target_position)) {	
            //generate the new velocity
			desired_velocity = generateConstantVelocity(cruise_speed, robot_position, target_position);
            //rotate the omnidirectional robot
			desired_velocity = rotateVelocity(desired_velocity, -robot_position.angular.z);
            //bound the velocity
			desired_velocity = boundVelocity(desired_velocity);
		} else { 
            // Goal has been reach ==> dont move
			desired_velocity.linear.x = 0;
			desired_velocity.linear.y = 0;
			desired_velocity.angular.z = 0;
		}
		//ROS_INFO_STREAM use for debugging 
		ROS_INFO_STREAM("Desired Velocity:"
			<<"X:"<<desired_velocity.linear.x
			<<",Y:"<<desired_velocity.linear.y
			<<",W:"<<desired_velocity.angular.z);

		//publish the new velocity
		pub_vel_turtle.publish(desired_velocity);
		
		ros::spinOnce();
		rate.sleep();
        tiempo+=(1/rate_hz); 
    }
    return 0;
}