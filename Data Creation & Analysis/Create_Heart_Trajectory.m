%This code creates a smoothed heart-shaped trajectory consisting of 100
%points.
clear
clc
close all

%% Creating the Heart shaped trajectory
scale = 0.1; 
n = 100;
t = linspace(0, 2*pi, n);
x_coords = scale * (16 * sin(t).^3);
y_coords = scale * (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t));

%Scaling, normalizing and changing the center
centerX = 0.5; 
centerY = 0.0; 
centerZ = 0.35;

x = x_coords / max(abs(x_coords)) * scale + centerX;
y = y_coords / max(abs(y_coords)) * scale + centerY;
z = ones(1, n) * centerZ;

raw_waypoints = [x', y', z'];

%Smoothing the data
waypoints = smoothdata(raw_waypoints, 'gaussian', 10);


%% Visualizing the Trajectory and comparing sharp and smooth shapes
figure;
hold on;
plot3(raw_waypoints(:,1), raw_waypoints(:,2), raw_waypoints(:,3), 'b-','LineWidth', 1);
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'r-', 'LineWidth', 2.5);

title('Heart Shaped Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Sharp Trajectory', 'Smoothed Trajectory for Robot');
grid on;

%% Writing to CSV
filename = 'heart.csv';
writematrix(waypoints, filename);

