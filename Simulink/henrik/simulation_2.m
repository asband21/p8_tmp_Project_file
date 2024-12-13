%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a simplified model of a boat in one dimension with a single thruster. i 
% The state space includes position and velocity. 
% The goal is to estimate the drag coefficient of the boat in one dimension.
% The estimated parameter is used in an extended Kalman filter, implemented 
% as an inner function, to see if we can accurately estimate the system parameters.
% Written by Asbjørn.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

N = 10000 % Number of simulation steps
time = zeros(1, N); 
ts = 0.1; % Time step size
time = ts:ts:N * ts;

% State variables: position, velocity, and drag coefficient
x = zeros(9, N);        
d = [-100,-1000,-2000];
x(:, 1) = [0;0;0;1;1;1;d(1);d(2);d(3)];    % Initial state [position, velocity, drag coefficient]


% Estimated state and covariance matrix initialization
x_est = zeros(9, N);         % Estimated state
x_est(:, 1) = [0;0;0;1;1;1;0;0;0];  % Initial guess for estimated state
P_est = eye(9) * 1e-3;       % Initial covariance matrix

% Input (control signal)
u = zeros(5, N);   
u(1, :) = -sign(sin((0:N-1)  * 0.070)) * 1000-1000;
u(2, :) = sign(sin((0:N-1)  * 0.050)) * 1000+1000;
u(3, :) = sign(sin((0:N-1)  * 0.030)) * 1000+1000;
u(4, :) = sign(sin((0:N-1)  * 0.005)) * 1000+1000;
u(5, :) = sign(sin((0:N-1)  * 0.130)) * 1000+1000;
% Loop through all time steps, calculate true dynamics, add measurement noise,
% and apply the extended Kalman filter.
for i = 2:N
    % Compute true system dynamics
    x(:, i) = dynamics(time(i), time(i-1), x(:, i-1), u(:, i-1), d);
    % Add noise to measurement
    noisy_measurement = x(:, i-1) + [1, 1, 1, 0, 0, 0, 0, 0, 0] * random('Normal', 0, 1, [9, 1]) * 2;
    % Apply extended Kalman filter
    [x_est(:, i), P_est] = extended_kalman_filter_sim_2(time(i), time(i-1), P_est, x_est(:, i-1), u(:, i-1), noisy_measurement);
end


figure;
subplot(3, 1, 1);
plot(time, x(1, :), 'b', time, x_est(1, :), 'r');
xlabel('Time (s)');
ylabel('X Position (m)');
legend('True X Position', 'Estimated X Position');
title('X-Axis Position Estimation');

subplot(3, 1, 2);
plot(time, x(4, :), 'b', time, x_est(4, :), 'r');
xlabel('Time (s)');
ylabel('X Velocity (m/s)');
legend('True X Velocity', 'Estimated X Velocity');
title('X-Axis Velocity Estimation');

subplot(3, 1, 3);
plot(time, x(7, :), 'b', time, -x_est(7, :), 'r');
xlabel('Time (s)');
ylabel('X Drag Coefficient (b_c)');
legend('True X b_c', 'Estimated X b_c');
title('X-Axis Drag Coefficient Estimation');

% Y-Axis: Position, Velocity, and Drag Coefficient
figure;
subplot(3, 1, 1);
plot(time, x(2, :), 'b', time, x_est(2, :), 'r');
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('True Y Position', 'Estimated Y Position');
title('Y-Axis Position Estimation');

subplot(3, 1, 2);
plot(time, x(5, :), 'b', time, x_est(5, :), 'r');
xlabel('Time (s)');
ylabel('Y Velocity (m/s)');
legend('True Y Velocity', 'Estimated Y Velocity');
title('Y-Axis Velocity Estimation');

subplot(3, 1, 3);
plot(time, x(8, :), 'b', time, -x_est(8, :), 'r');
xlabel('Time (s)');
ylabel('Y Drag Coefficient (b_c)');
legend('True Y b_c', 'Estimated Y b_c');
title('Y-Axis Drag Coefficient Estimation');

% theta-Axis: Position, Velocity, and Drag Coefficient
figure;
subplot(3, 1, 1);
plot(time, x(3, :), 'b', time, x_est(3, :), 'r');
xlabel('Time (s)');
ylabel('Angle Position (m)');
legend('True angle', 'Estimated angle');
title('Angle Estimation');

subplot(3, 1, 2);
plot(time, x(6, :), 'b', time, x_est(6, :), 'r');
xlabel('Time (s)');
ylabel('Angle Velocity (m/s)');
legend('True angle Velocity', 'Estimated angle Velocity');
title('Angle Estimation');

subplot(3, 1, 3);
plot(time, x(9, :), 'b', time, -x_est(9, :), 'r');
xlabel('Time (s)');
ylabel('Angle Drag Coefficient (b_c)');
legend('True Angle b_c', 'Estimated Angle b_c');
title('Angle Drag Coefficient Estimation');


% Function for true system dynamics
function [x_next] = dynamics(current_time, previous_time, x, u, b)
    td = current_time - previous_time;
    length_x = 2.74;
    length_y = 5.32;
    moter_left  = [-length_x/2, -length_y/2, 0, 0];
    moter_right = [ length_x/2, -length_y/2, 0, 0];
    moter_bow   = [ 0,           length_y,   0, 0];
    delta_t = current_time - previous_time; % Time step
    m = 1500; % Mass of the boat
    I = (length_y^2+length_x^2)*m*1/12;
    bx = b(1);
    by = b(2);
    bt = b(3);

    %moter 1
    t1x = moter_left(1);
    y1  = moter_left(2);
    %moter 2
    t2x = moter_right(1);
    y2  = moter_right(2);
    %moter 3
    t3x = moter_bow(1);
    y3  = moter_bow(2);
    f = @(xp_m1, u_m1) xp_m1+[0 0 0 1 0 0 0 0 0;
                              0 0 0 0 1 0 0 0 0;
                              0 0 0 0 0 1 0 0 0;
                              0 0 0 bx/m 0 0 0 0 0; 
                              0 0 0 0 by/m 0 0 0 0;
                              0 0 0 0 0 bt/I 0 0 0;
                              0 0 0 0 0 0 0 0 0;
                              0 0 0 0 0 0 0 0 0;
                              0 0 0 0 0 0 0 0 0]*xp_m1*td + [0,0,0,0,0;
                                                             0,0,0,0,0;
                                                             0,0,0,0,0;
                                                             1/m,0,1/m,0,1/m;
                                                             0,1/m,0,1/m,0;
                                                             -y1/I, t1x/I, -y2/I, t2x/I, -y3/I;
                                                             0,0,0,0,0;
                                                             0,0,0,0,0;
                                                             0,0,0,0,0]*td*u_m1;
    x_next = f(x,u);
end
