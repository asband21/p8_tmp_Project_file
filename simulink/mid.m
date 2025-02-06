clc; clear; close all;

% Function for state update
f_n = @(x, v) x + [cos(x(3)) -sin(x(3)) 0;
                    sin(x(3))  cos(x(3)) 0;
                    0          0         1] * v;

% Function to compute local velocity from previous and current state
v_n = @(x, x_prev) [cos(x_prev(3))  sin(x_prev(3))  0;
                    -sin(x_prev(3)) cos(x_prev(3))  0;
                     0              0             1] * (x - x_prev);

% Initialization
num_steps = 101;
x = [0; 0; 0];  % Initial state
x_prev = x;     % Previous state
trajectory = zeros(3, num_steps);        % Global trajectory
velocity_local = zeros(3, num_steps);    % Local velocity storage
trajectory_local = zeros(3, num_steps);  % Local trajectory storage

% Simulation loop
for i = 1:num_steps
    x_prev = x;
    x = f_n(x, [1; 0.1; 0.1]); % Apply velocity update
    
    % Store values
    trajectory(:, i) = x; 
    velocity_local(:, i) = v_n(x, x_prev); 
    
    % Compute local trajectory iteratively
    if i > 1
        trajectory_local(:, i) = trajectory_local(:, i-1) + velocity_local(:, i);
    end
end

% Global trajectory plot
figure;
plot(trajectory(1, :), trajectory(2, :), 'b-');
xlabel('Global X Position');
ylabel('Global Y Position');
title('Global Trajectory over Time');
grid on;

% Local velocity plot
figure;
plot(velocity_local(1, :), velocity_local(3, :), 'r-');
xlabel('Local X Velocity');
ylabel('Local Rotation Velocity');
title('Local Velocity over Time');
grid on;
