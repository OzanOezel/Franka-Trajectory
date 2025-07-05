%This code takes as input a Rosbag file (.bag), outputs joint position and
%velocities using the joint_states topic.
clear;
clc;
close all;

bag = rosbag('heart_trajectory_data.bag');

bag.AvailableTopics


%% Joint Positions & Velocities
jointBag = select(bag, 'Topic', '/joint_states');
jointMsgs = readMessages(jointBag, 'DataFormat', 'struct');

n = length(jointMsgs);
positions = zeros(n, length(jointMsgs{1}.Position));
velocities = zeros(n, length(jointMsgs{1}.Velocity));
timestamps = zeros(n,1);

for i = 1:n
    positions(i,:) = jointMsgs{i}.Position;
    velocities(i,:) = jointMsgs{i}.Velocity;
    timestamps(i) = jointMsgs{i}.Header.Stamp.Sec + ...
                    jointMsgs{i}.Header.Stamp.Nsec * 1e-9;
end


filename = 'joint_pos.csv'; 
writematrix(positions, filename);
filename = 'joint_vel.csv'; 
writematrix(velocities, filename);
