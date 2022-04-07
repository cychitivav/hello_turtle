%% Start connection
rosinit; %ROS master connection

%% Publisher creation
velPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist');
velMsg = rosmessage(velPub); % Message creation

%% Edit and send message
velMsg.Linear.X = 1; % Linear velocity in x
send(velPub,velMsg); % Send message
pause(1)