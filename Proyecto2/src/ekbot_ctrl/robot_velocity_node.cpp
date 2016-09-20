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
#include "gazebo_msgs/ApplyJointEffort.h"
#include <math.h>

using namespace Eigen;

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Twist robot_position;
geometry_msgs::Twist velocity_msg;

//rate_hz assignment
double rate_hz = 1;


double* getMotorValue(int x_velocity, int y_velocity, int w_velocity){
	double deg1 = 3 * M_PI / 10;
	double deg2 = 3 * M_PI / 10;
	double deg3 = 7 * M_PI / 30;
	double deg4 = 7 * M_PI / 30;
	double s1 = sin(deg1), c1 = cos(deg1);
	double s2 = sin(deg2), c2 = cos(deg2);
	double s3 = sin(deg3), c3 = cos(deg3);
	double s4 = sin(deg4), c4 = cos(deg4);
	double R = 8.5;
	double r = 3.4;

	double velXMod = x_velocity * 3.5 / ( 2 * M_PI * r );
    double velYMod = y_velocity * 3.5 / ( 2 * M_PI * r );
    double velWMod = w_velocity * 3.5 / ( 2 * M_PI * r );

    velWMod = R * velWMod;
    double velMots[4];

    velMots[0] = (double)( s1 * velXMod + c1 * velYMod + velWMod);
    velMots[1] = (double)(-s2 * velXMod + c2 * velYMod + velWMod);
    velMots[2] = (double)(-s3 * velXMod - c3 * velYMod + velWMod);
    velMots[3] = (double)( s4 * velXMod - c4 * velYMod + velWMod);

    return velMots;
}

void get_vel_vec(const geometry_msgs::Twist& msg) {
	velocity_msg.linear.x = msg.linear.x;
	velocity_msg.linear.y = msg.linear.y;
    velocity_msg.angular.z = msg.linear.z; 
}


int main(int argc, char **argv){
	ros::init(argc,argv,"robot_velocity_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("robot_velocity_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	

	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort eff_msg[4];
	

	ros::Subscriber sub_vel = nh.subscribe("/target_vel_topic", 1000, &get_vel_vec);


	//Topic to publish velocity command, queue size equals rate_hz to keep up with the rate at which messages are generated,

 //    //Publish to the turtle topic "/turtle1/cmd_vel at rate_hz"
	// ros::Publisher pub_vel_turtle = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", rate_hz);

	// //Topics to acquire robot and target position (from the vision node) 
	// ros::Subscriber sub_attacker_pos = nh.subscribe("/robot_position_topic", 1, &getRobotPose); 
	// ros::Subscriber sub_ball_pos = nh.subscribe("/target_position_topic", 1, &getTargetPose);

 //    //Twist variable to publish velocity (trajectories)
	// geometry_msgs::Twist desired_velocity;
	double tiempo = 0;

    //define the max speed
	double cruise_speed = 50;

    //define the rate
	ros::Rate rate(rate_hz);
	ros::Time start_time ;
	ros::Duration duration ;
	// ros::Rate rate(rate_hz);

	double effort[4];
	
	while (ros::ok())
	{
	// ROS_INFO_STREAM("Vels x: " << velocity_msg.linear.x << " y: " << velocity_msg.linear.y << " z: " << velocity_msg.angular.z);

	double* velMots = getMotorValue(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.angular.z);
	
	effort [0] = velMots[0];
	effort [1] = velMots[1];
	effort [2] = velMots[2];
	effort [3] = velMots[3];

	
  
		if(client.exists()){
			


			start_time.sec = 0;
			start_time.nsec = 0;
			duration.sec = 1/rate_hz;
			duration.nsec = 0;

			// Wheel-Joint 1
			eff_msg[0].request.joint_name = "chassis_JOINT_1";
			eff_msg[0].request.duration = duration;
			eff_msg[0].request.effort = effort[0];
			eff_msg[0].request.start_time = start_time;

			// Wheel-Joint 2
			eff_msg[1].request.joint_name = "chassis_JOINT_2";
			eff_msg[1].request.duration = duration;
			eff_msg[1].request.effort = effort[1];
			eff_msg[1].request.start_time = start_time;

			// Wheel-Joint 3
			eff_msg[2].request.joint_name = "chassis_JOINT_3";
			eff_msg[2].request.duration = duration;
			eff_msg[2].request.effort = effort[2];
			eff_msg[2].request.start_time = start_time;

			// Wheel-Joint 4
			eff_msg[3].request.joint_name = "chassis_JOINT_4";
			eff_msg[3].request.duration = duration;
			eff_msg[3].request.effort = effort[03];
			eff_msg[3].request.start_time = start_time;

			// if(client.call(eff_msg[3])){
			// 	if(client.call(eff_msg[2])){
			// 		if(client.call(eff_msg[1])){
			// 			if(client.call(eff_msg[0])){
			// 				ROS_INFO_STREAM("Joint 1: " << (eff_msg[0].response.success) == 1 ? "TRUE" : "FALSE";
			// 			}
			// 			ROS_INFO_STREAM("Joint 2: " << eff_msg[1].response.success);
			// 		}
			// 		ROS_INFO_STREAM("Joint 3: " << eff_msg[2].response.success);
			// 	}
			// 	ROS_INFO_STREAM("Joint 4: " << eff_msg[3].response.success);
				// ROS_INFO_STREAM("RES: " << eff_msg.response.success << "MSG: \n" << eff_msg.response.status_message);
			// } else{
			// 	ROS_INFO_STREAM("Something bad happened...");
			// }
			client.call(eff_msg[0]);
			client.call(eff_msg[1]);																																																																								
			client.call(eff_msg[2]);
			client.call(eff_msg[3]);
			ROS_INFO_STREAM("Joints ==> 1: " << ((eff_msg[0].response.success == 1) ? "TRUE" : "FALSE") <<
			" 2: " << ((eff_msg[1].response.success == 1) ? "TRUE" : "FALSE") <<
			" 3: " << ((eff_msg[2].response.success == 1) ? "TRUE" : "FALSE") <<
			" 4: " << ((eff_msg[3].response.success == 1) ? "TRUE" : "FALSE"));
		}																																																																																																
        //ROS_INFO_STREAM use for debugging 
		// ROS_INFO_STREAM("Robot Position"
		// 	<<",X,"<<robot_position.linear.x
		// 	<<",Y,"<<robot_position.linear.y
		// 	<<",W,"<<robot_position.angular.z);
		// ROS_INFO_STREAM("Target position:"
		// 	<<",X,"<<target_position.linear.x
		// 	<<",Y,"<<target_position.linear.y
		// 	<<",W,"<<target_position.angular.z);

  //       //Define is goal hasn't been reach
		// if (isGoalFar(robot_position, target_position)) {	
  //           //generate the new velocity
		// 	desired_velocity = generateConstantVelocity(cruise_speed, robot_position, target_position);
  //           //rotate the omnidirectional robot
		// 	desired_velocity = rotateVelocity(desired_velocity, -robot_position.angular.z);
  //           //bound the velocity
		// 	desired_velocity = boundVelocity(desired_velocity);
		// } else { 
  //           // Goal has been reach ==> dont move
		// 	desired_velocity.linear.x = 0;
		// 	desired_velocity.linear.y = 0;
		// 	desired_velocity.angular.z = 0;
		// }
		// //ROS_INFO_STREAM use for debugging 
		// ROS_INFO_STREAM("Desired Velocity:"
		// 	<<"X:"<<desired_velocity.linear.x
		// 	<<",Y:"<<desired_velocity.linear.y
		// 	<<",W:"<<desired_velocity.angular.z);

		// //publish the new velocity
		// pub_vel_turtle.publish(desired_velocity);
		
		ros::spinOnce();
		rate.sleep();
  //       tiempo+=(1/rate_hz); 
    }
    return 0;
}
