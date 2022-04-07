%% Client creation
rclt = rossvcclient("/turtle1/teleport_absolute")
waitForServer(rclt); % Wait for server to be available

rqtMsg = rosmessage(rclt) % Message creation
rqtMsg.X = 7; % X coordinate
rqtMsg.Y = 7; % Y coordinate

response = call(rclt,rqtMsg) % Send request and wait response (in this case, the response is empty)
%% Stop MATLAB node
rosshutdown;