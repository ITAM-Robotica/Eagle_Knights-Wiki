#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
//#include "turtlesim/Velocity.h>
//Global Funciton

void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg);
double getDistance(double x1, double y1, double x2, double y2);
// Global variables
// to hold stop flag, wait till first command given
bool STOP = true;
bool firstRead =false;
bool rotateB = false;
int safeLoop;
//total distance
float distance;
// to hold current pose
turtlesim::Pose CurPose;
// variable to hold desired pose
geometry_msgs::Pose2D DesPose;

// variable to hold desired pose
ros::Publisher pub;
int t;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "TurtlesimPositionController_pubsub");
    ros::NodeHandle nh;
    //time by argv, is not elegant but work
    if (argc == 2)
    {
        t = atoi( argv [1] );

    }else{
        printf("Error en el número de argumentos, sólo indicar t \n");
        ros::shutdown();
    }
    //Pub advertise type twist
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    // goal position
    ros::Subscriber ComPose_sub = nh.subscribe("/turtle1/PositionCommand", 5, ComPoseCallback);
    // get actual location
    ros::Subscriber CurPose_sub = nh.subscribe("/turtle1/pose", 5, CurPoseCallback);
   
    ros::Rate rate(10);
    float ErrorLin = 0;
    float ErrorAng = 0;

    ROS_INFO("Ready to send position commands");                        // let user know we are ready and good
    while (ros::ok() && nh.ok() )                                        // while ros and the node are ok
    {
        ros::spinOnce();
        if (STOP == false)                                              // and no stop command
        {
        	if (firstRead)
        	{
        		distance = getDistance(CurPose.x, CurPose.y, DesPose.x, DesPose.y);
        		firstRead=false;
        	}
            geometry_msgs::Twist msg;
            msg.linear.x = getDistance(CurPose.x, CurPose.y, DesPose.x, DesPose.y)*.5*distance/(t-1);
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 2*(atan2(DesPose.y - CurPose.y, DesPose.x - CurPose.x)-CurPose.theta);
            pub.publish(msg);
            if (getDistance(CurPose.x, CurPose.y, DesPose.x, DesPose.y)<.3)
            {
            	rotateB=true;
            	STOP=true;
            	safeLoop=0;

            }
        }
        if(rotateB){
            printf("rotating...\n");
            int distanceAngle =DesPose.theta-CurPose.theta;
                 	if ((atan2(DesPose.y - CurPose.y, DesPose.x - CurPose.x)-CurPose.theta)>1)
                 	{
			            geometry_msgs::Twist msg;
			            msg.linear.x = .0;
			            msg.linear.y = 0;
			            msg.linear.z = 0;
			            msg.angular.x = 0;
			            msg.angular.y = 0;
			            msg.angular.z = 1*(atan2(DesPose.y - CurPose.y, DesPose.x - CurPose.x)-CurPose.theta);
			            pub.publish(msg);
			            if (safeLoop<5)
			            {
			            	safeLoop++;
			            }else
			            	ros::shutdown();	
                 	}else{
                 		ros::shutdown();
                 		printf("shutdown...\n");
                 	}
        }
        rate.sleep();//Done
    }
}

//Euclidian distance
double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

// Get new pos from turtlebot
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    STOP = false;
    firstRead =true;
    DesPose.x = msg->x;
    DesPose.y = msg->y;
    DesPose.theta = msg->theta;
    return;
}

// Get actual position
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    CurPose.x = msg->x;
    CurPose.y = msg->y;
    CurPose.theta = msg->theta;
    return;
}