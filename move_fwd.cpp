#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

/* Objetos */
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

/* Constantes */
const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

/* Metodos del programa */
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
double setDesiredOrientation (double desired_angle_radians, double time);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance, double time);
double getDistance(double x1, double y1, double x2, double y2);

/* Main */
int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "move_fwd");
  ros::NodeHandle n;

/* Variables locales */
  double speed;
  double distance;
  bool isForward;

  double angular_speed;
  double angle;
  bool clockwise;
  
  double pose_x;
  double pose_y;
  double time;

  bool nueva;

//Publisher object.
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

//Subscriber object.
  pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

// Loop at 10Hz until the node is shut down.
  ros::Rate loop_rate(10);

  do{

//Pide y lee pos en x
  cout<<"ingrese pose en x: ";
  cin>>pose_x;

//Pide y lee pos en y
  cout<<"ingrese pose en y: ";
  cin>>pose_y;

//Pide y lee el angulo de rotacion de la pose final respecto a z pero toma el eje x para empezar a leer en sentido antihorario
  cout<<"ingrese angulo de giro respecto a z a partir del eje x (en grados) : ";
  cin>>angle;

//Pide y lee el tiempo de ejecucion, el cual se va a dividir en dos mitades, una para la traslacion y otra para la rotacion
  cout<<"enter time: ";
  cin>>time;

//Le pasa las variables de pose desada a la variable goal_pose
  turtlesim::Pose goal_pose;
  goal_pose.x=pose_x;
  goal_pose.y=pose_y;
  goal_pose.theta=0;

//Mueve a la tortuga a la posicion deseada
  moveGoal(goal_pose, 0.01, time);
  loop_rate.sleep();
//Gira a la tortuga a la pose deseada
  setDesiredOrientation(angle*PI /180.0 ,time);
  loop_rate.sleep();

//Pregunta si se desea ingresar un nuevo punto
  cout<<"Desea continuar? (Si=1, No=0) : ";
  cin>>nueva;        

  }while(nueva);

//Let ROS take over.
  ros::spin();

//Exit
  return 0;


}


/**
 *  Rota a la tortuga con cierta velocidad un cierto angulo respecto a su posicion inicial
 *  Este metodo va a ser utilizado en el metodo setDesiredOrientation
 */


void rotate (double angular_speed, double relative_angle, bool clockwise){

//Instancia mensaje
	geometry_msgs::Twist vel_msg;
	   
//Velocidad lineal = 0
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   
//Velocidad angular = 0
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

//Actualizar el angulo
	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(10);
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  Convierte el angulo dado en grados a radianes
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/**
 *  Rota la tortuga en un angulo absoluto respecto al eje x sin importar la rotacion inicial
 */

double setDesiredOrientation (double desired_angle_radians, double time){

	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	//cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	//rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
        rotate (abs((2*relative_angle_radians)/time), abs(relative_angle_radians), clockwise);
}

/**
 *  Se mantiene actualizada la posiscion actual de la tortuga
 */

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

/**
 *  Calcula la distancia euclidiana entre dos puntos
 */


double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/**
 *  Dada una pose deseada la tortuga se acerca al punto deseado mediante un controlador proporcional con un tiempo de ejecucion de t/2. Al
 *  llegar al destino, se tardara otros t/2 segundos en rotar hasta la posicion deseada.
 */

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance, double time){

//Instancia mensaje
    geometry_msgs::Twist vel_msg;

//Loop at 10Hz until the node is shut down.    
    ros::Rate loop_rate(10);

//Se declara una variable de tiempo al inicio del proceso para poder normalizar el movimiento respecto al tiempo
    double inicio = ros::Time::now().toSec();
    do{

//Se declara otra variable de tiempo al inicio de cada ciclo para compararla con el tiempo inicial y sacar su diferencia
        double actual = ros::Time::now().toSec();

/**
 *  Se declara la velocidad lineal, y y z van a ser cero pues la tortuga solo avanzara hacia adelante y entre mas lejos se encuentre del punto 
 *  inicial mas rapido avanzara y conforme se vaya acercando, disminuira su velocidad. Esto a su vez es dividido entre la diferencia
 *  entre t/2 y el tiempo que ha transcurrido para normalizar el avance y que llegue en el tiempo de ejecucion deseado.
 */

/**
 *  Nota: el hecho de estar tomando en cuenta la diferencia en tiempo hace que si se le dan tiempos pequeños a distancias grandes, 
 *  la division va a resultar en un numero muy grande que va a descontrolar el comportamiento de la tortuga, por lo que para distancias
 *  mayores a 2 se les recomienda utilizar un tiempo de ejecucion de 12 o mas segundos. En caso de ser distancias pequeñas, 
 *  el programa si puede realizarlas en lapsos de alrededor de 4 segundos, no hay un limite fijo pues depende de la distancia 
 *  y del tiempo por lo que solo se deben evitar tiempos pequeños con distancias pequeñas.
 */

        vel_msg.linear.x = (1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y))/((time/2)-(actual-inicio));
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

//Se declara la velocidad angular para que vaya acercandose cada vez mas a estar en linea recta con el punto deseado.
//Esta no se normaliza para que su trayectoria sea lo mas recta posible.
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);
        velocity_publisher.publish(vel_msg);

//Wait until it's time for another iteration.
        ros::spinOnce();
        loop_rate.sleep();

    }while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
//Se detiene la tortuga al llegar a su objetivo
    cout<<"end move goal"<<endl;
    vel_msg.linear.x=0;
    vel_msg.angular.z=0;
    velocity_publisher.publish(vel_msg);

}

