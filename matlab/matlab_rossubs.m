%% Subscriber creation
poseSub = rossubscriber("/turtle1/pose","turtlesim/Pose")
poseSub.LatestMessage % Show latest message