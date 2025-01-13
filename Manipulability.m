%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024
close all
clear all
clc
% Parameters for robot arm
r1 = 1.0;    % Length of first link
r2 = 1.0;    % Length of second link
t1 = -0.2;   % Initial angle of joint 1
t2 = -1;      % Initial angle of joint 2
% Desired position
x_d = [1.5; 1.0]; % Desired x and y coordinates of the end-effector
% Control parameters
damping_factor = 0.01;   % Damping factor for DLS
step_limit = 0.1;        % Maximum step size for target clamping
gain = 0.3;              % Gain for Jacobian transpose method
tolerance = 1e-3;        % Position error tolerance
max_iterations = 1000;   % Maximum number of iterations
dt = 0.01;               % Time step for plotting manipulability
ellipsoid_scale = 0.25;   % Scaling factor for ellipsoids
% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);
figure;
hold on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('X');
ylabel('Y');
title('Robot Arm Motion and Manipulability Ellipsoids');
for i = 1:max_iterations
    % Calculate forward kinematics to get the current end-effector position
    T = forward_kinematics3(r1, r2, t1, t2);
    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y];
    
    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;
    
    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) > step_limit
        error = (error / norm(error)) * step_limit; % task1
    end
    
    % Compute Jacobian for the current joint angles
    J = ik_jacobian(r1, r2, t1, t2);
    
    % Calculate manipulability as the square root of the determinant of (J*J')
    manipulability = sqrt ( det(J * (J')) ) ; %task 2 ;
    manipulability_values(i) = manipulability;
    
    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = (J' * J + damping_factor^2 * eye(size(J, 2)))\ J' ; % task 3;
    delta_q_dls = inv_J_damped * error;
    
    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain * J' * error; %task 4;
    
    % Select method: uncomment one of the two lines below
    %delta_q = delta_q_dls;     % Damped Least Squares method
    delta_q = delta_q_transpose; % Jacobian Transpose method
    
    % Update joint angles using calculated change
    t1 = t1 + delta_q(1);
    t2 = t2 + delta_q(2);
    
    % Plot current robot arm position
    plot([0, r1*cos(t1), r1*cos(t1) + r2*cos(t1 + t2)], ...
         [0, r1*sin(t1), r1*sin(t1) + r2*sin(t1 + t2)], '-o');
    
    % Plot the manipulability ellipsoid at the current configuration
    [U, S, V] = svd (J); %task 5;  % Singular Value Decomposition of the Jacobian
    theta = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * cos(theta) * S(1,1); %.....apply scaling;  
    ellipse_y = ellipsoid_scale * sin(theta) * S(2,2); %.....apply scaling;  
    ellipse = U * [ellipse_x; ellipse_y]; %Rotate ellipse based on current Jacobian ...* [ellipse_x; ellipse_y];
    
    % Plot the ellipsoid around the current end-effector position
    plot(ellipse(1, :) + x, ellipse(2, :) + y, 'm');
    
    pause(0.01);  % Pause for visualization
    
    % Check if the end-effector is within the desired tolerance
    if norm(error) < tolerance
        disp('Desired position reached!');
        break;
    end
end
% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');