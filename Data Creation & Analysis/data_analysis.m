%This code takes as input the designed and executed trajectories.
%Calculates the minimum distance between them as error. Plots comparion
%between two trajectories and the error over time.
clear;
clc;
close all;


load('heart.csv');
load('EE_trajectory.csv');


D = pdist2(EE_trajectory, heart); %pairwise distance
error = min(D, [], 2); %mininmum value of each row
mean_error = mean(error);


figure('Name','');
hold on;
plot3(heart(:,1), heart(:,2), heart(:,3), 'r-');
plot3(EE_trajectory(:,1), EE_trajectory(:,2), EE_trajectory(:,3), 'k-')
title('Smoothed, Executable Heart Path');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Designed Trajectory', 'Executed Trajectory');
grid on;
%axis equal;


figure('Name', 'Tracking Error');
% Use the number of points in the executed path as a proxy for time
plot(1:size(EE_trajectory, 1), error, 'k', 'LineWidth', 1.5);
title('Tracking Error');
xlabel('Progress along path');
ylabel('Error (m)');
grid on;
% Add lines for the mean and max error for context
hold on;
yline(mean_error, '--r', 'Mean Error');
hold off;