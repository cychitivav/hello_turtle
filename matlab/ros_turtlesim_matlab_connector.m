%% Start connection
clc, clear, close all
rosinit; %ROS master connection
%% Publisher creation
velPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist');
velMsg = rosmessage(velPub); % Message creation
%% Edit and send message
velMsg.Linear.X = 1; % Linear velocity in x
send(velPub,velMsg); % Send message
pause(1)
%% Suscriber creation
poseSub = rossubscriber("/turtle1/pose","turtlesim/Pose")
poseSub.LatestMessage % Show latest message
%% Client creation
rclt = rossvcclient("/turtle1/teleport_absolute")
waitForServer(rclt); % Wait for server to be available

rqtMsg = rosmessage(rclt) % Message creation
rqtMsg.X = 7; % X coordinate
rqtMsg.Y = 7; % Y coordinate

response = call(rclt,rqtMsg) % Send request
%% Stop MATLAB node
rosshutdown;
