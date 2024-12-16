% Extended Kalman Filter to Estimate bc in a Mass-Damper System

clear all; close all; clc;

%% System Parameters
m = 1;          % Mass (kg)
b = 2;          % Known damping coefficient (N·s/m)
bc_true = 5;    % True unknown damping coefficient (N·s/m)

%% Time Parameters
dt = 0.01;      % Time step (s)
T = 100;         % Total simulation time (s)
N = T/dt;       % Number of time steps
t = 0:dt:T-dt;  % Time vector

%% Input (Control Force)
u = zeros(1, N);          % Zero input force
u(100:200) = 10;          % Apply a force between t=1s and t=2s
u(500:700) = -20;
u(1200:1600) = 10;
%% Process and Measurement Noise
Q = diag([1e-5, 1e-5, 1e-7]);  % Process noise covariance
R = 1e-2;                      % Measurement noise covariance

%% True System Simulation
x_true = zeros(3, N);      % True state vector [p; p_d; bc]
x_true(:,1) = [0; 0; bc_true];  % Initial true state

for k = 1:N-1
    % True system dynamics
    p = x_true(1,k);
    p_d = x_true(2,k);
    bc = x_true(3,k);
    
    % State derivatives
    dp = p_d;
    dp_d = (-b*p_d - bc*p_d)/m + u(k)/m;
    dbc = 0;  % bc is constant
    
    % Update states
    x_true(:,k+1) = x_true(:,k) + dt*[dp; dp_d; dbc];
end

%% Measurements (Position with noise)
y = x_true(1,:) + sqrt(R)*randn(1, N);

%% Extended Kalman Filter Initialization
x_est = zeros(3, N);        % Estimated state vector [p; p_d; bc]
x_est(:,1) = [0; 0; 0];     % Iu(100:200) = 10;nitial estimate
P = eye(3);                 % Initial estimate error covariance

%% EKF Implementation
for k = 1:N-1
    % Prediction Step
    % Extract estimates
    p = x_est(1,k);
    p_d = x_est(2,k);
    bc = x_est(3,k);
    
    % State derivatives
    dp = p_d;
    dp_d = (-b*p_d - bc*p_d)/m + u(k)/m;
    dbc = 0;  % bc dynamics
    
    % Predict next state
    x_pred = x_est(:,k) + dt*[dp; dp_d; dbc];
    
    % Jacobian of the system dynamics w.r.t. states
    F = [1, dt, 0;
         0, 1 - dt*(b + bc)/m, -dt*p_d/m;
         0, 0, 1];
     
    % Predict error covariance
    P = F*P*F' + Q;
    
    % Measurement Prediction
    y_pred = x_pred(1);  % Predicted measurement
    
    % Measurement Jacobian
    H = [1, 0, 0];
    
    % Innovation covariance
    S = H*P*H' + R;
    
    % Kalman Gain
    K = P*H'/S;
    
    % Measurement Update
    x_est(:,k+1) = x_pred + K*(y(k+1) - y_pred);
    
    % Update error covariance
    P = (eye(3) - K*H)*P;
end

%% Plot Results
figure;
subplot(3,1,1);
plot(t, x_true(1,:), 'b', t, x_est(1,:), 'r--');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True Position', 'Estimated Position');
title('Position Estimation');

subplot(3,1,2);
plot(t, x_true(2,:), 'b', t, x_est(2,:), 'r--');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('True Velocity', 'Estimated Velocity');
title('Velocity Estimation');

subplot(3,1,3);
plot(t, x_true(3,:), 'b', t, x_est(3,:), 'r--');
xlabel('Time (s)');
ylabel('bc Estimate (N·s/m)');
legend('True bc', 'Estimated bc');
title('Parameter Estimation of bc');

sgtitle('Extended Kalman Filter Estimation Results');

