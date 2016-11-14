#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <cmath> 
#include <iostream> 

using namespace std;

//Objetos
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
		
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void Destino(turtlesim::Pose goal_pose, double distance_tolerance, double tiempo, bool flag);	//Esto mueve a la tortuga a su posicion final
double getDistance(double X1, double Y1, double X2, double Y2);
//Variables
double tiempo;
bool flag;
char x;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "proyecto1");
	ros::NodeHandle n;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000); 
											
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	ros::Rate loop_rate(0.5); 

	turtlesim::Pose goal_pose;
	goal_pose.x;
	goal_pose.y;
	goal_pose.theta;
	tiempo;

	//Lo que ve el usuario
	cout << "Ingrese un valor para x:      ";
	cin >> goal_pose.x;
	cout << "Ingrese un valor para y:      ";
	cin >> goal_pose.y;
	cout << "Ingrese un valor para theta:  ";
	cin >> goal_pose.theta;
	cout << "Ingrese el tiempo en el que desea que se complete la traycetoria:  ";
	cin >> tiempo;
	cout << "Quiere la tortuga realice todos los movimientos en el tiempo establecido(S) o solo la 			parte que sea lineal (N)? ";
	cin >> x;
	if (x == 'S' || x == 's')
		flag = true;
	else if (x == 'N' || x == 'n')
		flag = false;
	else 
		cout << "Lo siento, debe teclear 'S' o 'N'. Cierre el programa y trate de nuevo";

	Destino(goal_pose, 0.01, tiempo, flag);
	loop_rate.sleep();	
	
	ros::spin();

	return 0;
}


void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}


void Destino(turtlesim::Pose goal_pose, double distance_tolerance, double tiempo, bool flag){
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(10);

	if (!flag) {
		//Rota a la tortuga hacia el destino que se desea
		while((atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta) != 0) {
			//velocidad angular	
			vel_msg.angular.z = (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

			velocity_publisher.publish(vel_msg);

			ros::spinOnce();
			loop_rate.sleep();

		}
	
		//Movimiento en linea recta hacia el destino
		double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.x = (distance)/tiempo;
		double t0 = ros::Time::now().toSec();
		double t1 = t0;
		while (t1 - t0 <= tiempo) {
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			loop_rate.sleep();
			t1 = ros::Time::now().toSec();
		}

		vel_msg.linear.x = 0;
	
		//Rota hacia el angulo en el que se encuentra el destino
		t0 = ros::Time::now().toSec();
		t1 = t0;
		double angle = 1;
		double steering_angle;
		while(angle > 0.0001) {
			angle = turtlesim_pose.theta - goal_pose.theta;
			if (angle < 0)
				steering_angle = abs(angle);
			else
				steering_angle = (-1)*abs(angle);
			angle = abs(angle);
			vel_msg.angular.z = steering_angle;
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			loop_rate.sleep();
			t1 = ros::Time::now().toSec();
		}
		vel_msg.angular.z = 0;
	
		cout<<"Llego al destino"<<endl;
		vel_msg.angular.z = 0;
		velocity_publisher.publish(vel_msg);
	}
	if (flag) {
		
		double steering_angle;
		double angle;
		double t0 = ros::Time::now().toSec();
		double t1 = t0;
		while(t1 - t0 <= tiempo/3) {
			angle = (atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x) - turtlesim_pose.theta);
			if (angle < 0)
				steering_angle = (-9)*abs(angle);
			else
				steering_angle = 9*abs(angle);
			vel_msg.angular.z = (steering_angle)/tiempo;
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			loop_rate.sleep();
			t1 = ros::Time::now().toSec();
		}
		vel_msg.angular.z = 0;

		double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.x = (3*distance)/tiempo;
		t0 = ros::Time::now().toSec();
		t1 = t0;
		while (t1 - t0 <= tiempo/3) {
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			loop_rate.sleep();
			t1 = ros::Time::now().toSec();
		}

		vel_msg.linear.x = 0;
	
		//Rota al angulo en el que se encuentra el destino
		t0 = ros::Time::now().toSec();
		t1 = t0;
		while(t1 - t0 <= tiempo/3) {
			angle = turtlesim_pose.theta - goal_pose.theta;
			if (angle < 0)
				steering_angle = abs(angle);
			else
				steering_angle = (-1)*abs(angle);
			vel_msg.angular.z = steering_angle;
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			loop_rate.sleep();
			t1 = ros::Time::now().toSec();
		}

		vel_msg.angular.z = 0;
	
		cout<<"Llego al destino deseado"<<endl;
		vel_msg.angular.z = 0;
		velocity_publisher.publish(vel_msg);
	}
}

//Obtencion de la distancia euclidiana entre la posicion actual y la deseada
double getDistance(double X1, double Y1, double X2, double Y2){
	return sqrt(pow((X2-X1),2) + pow((Y2-Y1),2));
}

