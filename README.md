# Hello turtle package
In this repository is the development of the firs laboratory guide for robotics class at Universidad Nacional de Colombia.
For this practice we use ROS(Rootic Operative System), Ubuntu 20.04 , MATLAB and PYTHON.

Primero buscamos trabajar MATLAB junto con ROS, para ello primero se abrio una terminal en la cual se escribio el comando: 

roscore------Para correr el nodo maestro

Luego se abrio otra terminal en la cual se ingreso el comando:

rosrun turtlesim turtlesim node

Comando que abre el nodo de la tortuga para luego  hacer la conexión con MATLAB, para ello dentro de un script de MATLAB se escribio el sigueinte codigo:

%Inicio del codigo
rosinit; 
velPub = rospublisher(’/turtle1/cmd_vel’,’geometry_msgs/Twist’); 
velMsg = rosmessage(velPub); 
velMsg.Linear.X = 1; 
send(velPub,velMsg); 
pause(1)
%Fin del codigo

En el codigo anterior se hace la conexion con el nodo maestro, luego se crea el publicador y el mensaje para posteriormente dar un valor al mensaje y luego enviarlo.

Luego se creo un script en MATLAB para suscribirse al tipico de pose se la simulación, para ello se hizo uso del comando:

subPose= rossubscriber(TOPICNAME,MESSAGETYPE)

Y para finalizar se creo un ultimo script de MATLAB para poder enviar los valores asociados a la pose de la tortuga:

rclt = rossvcclient("/turtle1/teleport_absolute")
waitForServer(rclt); 
% Creacion del mensaje:
rqtMsg = rosmessage(rclt) 
rqtMsg.X = 10; 
rqtMsg.Y = 5;
% se envia requerimiento y esperamos respuesta
response = call(rclt,rqtMsg) 


Ahora vamos usar PYTHON para realizar un procedimiento similar:

Primeramente se creo un script llamado myTeleopKey.py en el cual realizamos un codigo que nos permitiera operar la tortuga haciendo uso del teclado con la siguientes indicaciones:

-Adelante presionando la tecla W

-Atras presionando la tecla S

-Rotación sentido horario presionando la tecla D

-Rotación sentido antihorario presionando la tecla A

Para ello se uso la clase myKeyboard()  en la que se definieron las funiones que nos permiten que al presionar una tecla, la tortuga haga la accion determinada.
