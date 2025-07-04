% This code takes as input a Rosbag file (.bag), outputs end effector
% position and orientation using the base (world) and end effector
% (panda_EE=frames{1}) frames.
clear;
clc;
close all;

bagMsgs = rosbagreader("heart_trajectory_data.bag")

frames = bagMsgs.AvailableFrames

tf = getTransform(bagMsgs,'world',frames{1});

tfTime = rostime(bagMsgs.StartTime + 1);
times = bagMsgs.MessageList.Time;

n = length(times);
positions = zeros(n, 3);
orientations = zeros(n, 4); 
for i= 1:n
    if (canTransform(bagMsgs,'world',frames{1},times(i)))
        tf2 = getTransform(bagMsgs,'world',frames{1},times(i));
        positions(i,1) = tf2.Transform.Translation.X;
        positions(i,2) = tf2.Transform.Translation.Y;
        positions(i,3) = tf2.Transform.Translation.Z;
        orientations(i,1) = tf2.Transform.Rotation.X;
        orientations(i,2) = tf2.Transform.Rotation.Y;
        orientations(i,3) = tf2.Transform.Rotation.Z;
        orientations(i,4) = tf2.Transform.Rotation.W;
    end
end


filename = 'EE_trajectory.csv'; 
writematrix(positions(4:1043,:), filename);