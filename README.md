# Hello turtle package
In this repository is the development of the first laboratory guide for robotics subject at the Universidad Nacional Colombia.

### Requirements
* ROS Noetic
* Turtlesim package (Pre-installed with ROS)
* ROS toolbox for MATLAB
* Pynput libray for python (`pip install pynput` or `pip3 install pynput`)
* Numpy libray (optional)

> If you don't want to use *numpy*, change **np.pi** of [teleop file](scripts/myTeleopKey.py) to 3.1415...

## MATLAB-ROS connection
In the first part of the lab, the MATLAB ROS toolbox is used to become familiar with the connection between MATLAB and ROS. Initially a subscriber to the pose topic is created to view the current pose of the turtle (turtlesim package). Also, to implement MATLAB's connection to ROS services, the teleport_relative service is called to move the turtle to any position.

### Execution
Open two terminals and run the following commands:

#### First terminal
Run master node
```bash
roscore
```

#### Second terminal
Run turtlesim node
```bash
rosrun turtlesim turtlesim_node
```

And finally, run this in MATLAB command window
```MATLAB
run matlab/matlab_ros_connection.m
run matlab/matlab_rossubs.m
run matlab/matlab_rossrv.m
```

In the [matlab ros connector file](matlab/matlab_ros_connection.m) we created a script that connects MATLAB to ROS and create a publisher to send **Twist** message by the topic *cmd_vel*. The second script contains the code to create a subscriber to the topic *pose* and to get the current pose of the turtle(latest message). Finally, the third script contains the code to create a service client to call the service *teleport_absolute* and to send the message to move the turtle to a new position; in this script is the command *rosshutdown* to close the connection with ROS.

![rosgraph](https://user-images.githubusercontent.com/30636259/162367660-2223aec7-2e35-4fba-a196-95d00c1cc6dc.png)

## Python keyboard control
AIn order to practice the commands to create publishers, subscribers and services from python, a script was created to move the turtle of the turtlesim node using the keyboard. To achieve this we used the _pynput_ library and optionally the _numpy_ library to rotate the turtle exactly pi radians.

### Execution
Open three terminals and run the following commands:

#### First terminal
~~~bash
roscore
~~~

#### Second terminal
~~~bash
rosrun turtlesim turtlesim_node
~~~

#### Third terminal
~~~bash
rosrun hello_turtle myTeleopKey.py
~~~

> Note: Don't forget source devel/setup.bash

### Controls
To move the turtle from the terminal use the following keys:

    * w: move forward
    * a: rotate left
    * s: move backward
    * d: rotate right

    * Space: Rotate turtle
    * r: Reset position

> Note: You must have focused the terminal where the myTeleopKey.py node is running

### Roslaunch
To run all nodes with a only command we created a [launch file](launch/runKeyTeleop.launch). To run this launch file we used the following command:

```bash
roslaunch hello_turtle runKeyTeleop.launch 
```