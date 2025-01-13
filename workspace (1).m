% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2024

close all
clear all

% declare global variables
%   hPlot       plot widget
%   hFig        figure widget
%   c           counter
%   x           x coordinate of points
%   y           y coordinate of points
global hPlot hFig c x y 

% Create GUI
hFig = figure;

% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);

% Set up and initialise variables for real-time plotting
c = 0;
x = [];
y = [];

% TODO initialise geometry of 2-arm robotic system
%r1 = 111.064;
%r2 = 111.374;
r1 = 111.064;
r2 = 111.374;

% TODO Specify the angle resolution you want the workspace to be plotted at
resolution = 40;    % [1..50]
angle1_range = linspace(1,180,resolution);  % [0..180] degrees given the resolution
angle2_range = linspace(1,360, resolution);  % [0..360] degrees given the resolution

% TODO Iterate through the given resolution of both angles
for t1 = 1:resolution
    for t2 = 1:resolution
        % increase counter to save information 
        c = c+1;

        % TODO compute the homogeneous transformation via
        % forward_kinematics() function given the geometry and angles
        T = forward_kinematics(r1, r2, angle1_range(t1), angle2_range(t2));
        
        % TODO retrieve end effector position and save for plotting
        x(c) = T(1, 4);
        y(c) = T(2, 4);
    end
end

% real-time plotting
colour = linspace(1,10,length(x));
scatter(hPlot, x, y, 20, colour, 'filled');
xlim([-(r1+r2),r1+r2]); %Hard code this to 156 by 156 axes space
ylim([-(r1+r2),r1+r2]);
hold on

% Define the side length of the square (now L = 156)
L = 156;  % Square side length

% Define the corner coordinates for the square (centered at origin)
xSquare = [-L/2, L/2, L/2, -L/2, -L/2];
ySquare = [-L/2 + 102, -L/2 + 102, L/2 + 102, L/2 + 102, -L/2 + 102];

% Plot the square on top of the existing scatter plot
plot(xSquare, ySquare, 'k-', 'LineWidth', 2);  % 'k-' for black lines

% Alternatively, you can use rectangle function to draw the square:
% rectangle('Position', [-L/2, -L/2, L, L], 'EdgeColor', 'k', 'LineWidth', 2);

% Set axis limits to fit the square and robot workspace
xlim([-(r1+r2), r1+r2]);
ylim([-(r1+r2), r1+r2]);

% Add labels and title for better clarity
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
title('Workspace with Square of Known Length');

% Optionally, you can close the GUI when done:
% delete(hFig);  % Uncomment this line if you want the window to close after plotting

% Close GUI
%delete(hFig);