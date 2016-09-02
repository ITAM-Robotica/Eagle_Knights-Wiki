# Tarea 01
# Materia: Temas Selectos de Robótica

Turtlesim program, Este programa permite desplazar a la tortuga de un punto a otro.

Para ejecutar se tienen que ejecutar los siguientes comandos
	
	$ rosrun tarea01 tarea01 time #parametro time es unidad de tiempo
	# establecer nuevo punto
	$ rostopic pub /turtle1/PositionCommand geometry_msgs/Pose2D "x: 1.0
y: 2
theta: 0.0" -1

*to do Generate launch file
