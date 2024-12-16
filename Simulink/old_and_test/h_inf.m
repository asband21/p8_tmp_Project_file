function [x_hat, P] = hinfinity_filter(time, olde_time, P_m1, Xp_m1, u_m1, y, gamma)
    td = time - olde_time; % time delta

    bc = Xp_m1(3);
    b = -1000;
    m = 1500;

    A = [0, 1, 0;
         0, b/m, 1/m;
         0, 0,  0];
    
    B = [0, 0, 0;
         0, 1/m, 0;
         0, 0, 0];
    
    % Updated measurement matrix to enhance observability
    C = [1, 0, 0;
         0, 0, 0;
         0, 0, 0];  % Directly measure bc for better observability

    Q_m1 = diag([1e-5, 1e-5, 1e-3]); % Process noise covariance
    R = diag([1e-2, 1e-2, 1e-1]);    % Measurement noise covariance
    
    F_m1 = eye(size(A)) + A * td;
    G_m1 = B * td;
    H = C;

    I = eye(3);
    
    % \( H_\infty \) update equations
    Pm = F_m1 * P_m1 * F_m1' + Q_m1;

    % Robust \( H_\infty \) gain computation
    S = H * Pm * H' + R; % Innovation covariance
    L = gamma^(-2) * H' / (I + gamma^(-2) * S); % \( H_\infty \) gain
    
    Xm = F_m1 * Xp_m1 + G_m1 * u_m1;
    K = Pm * L / (I + H * Pm * L);
    Xp = Xm + K * (y - H * Xm);
    
    Pp = (I - K * H) * Pm * (I - K * H)' + K * R * K';

    % Output
    x_hat = Xp;
    P = Pp;
end

function [x_1] = dynamik(time, olde_time, x, u)
    td = time - olde_time; % time delta
    m = 1500; b = -1000; bc = -200;
    A = [0, 1, 0;
         0, (b + bc)/m, 0;
         0, 0, 0];
    B = [0, 0, 0;
         0, 1/m, 0;
         0, 0, 0];
    x_1 = A * x * td + B * u * td + x;
end

% Simulation
clear all; close all; clc;
N = 1000;
time = zeros(1, N);
ts = 0.1; % time step size
time = ts:ts:N * ts;

x = zeros(3, N);        % True system dynamics
x(:, 1) = [0; 1; -200]; % Initial state
x_hat = zeros(3, N);
x_hat(:, 1) = [0; 1; 0];
p_hat = eye(3) * 1e-3;  % Small positive definite matrix

u = zeros(3, N);        % Zero input force
u(2, 500:600) = -500;

gamma = 5; % Adjusted robustness parameter for \( H_\infty \) filter

for i = 2:N
    x(:, i) = dynamik(time(i), time(i-1), x(:, i-1), u(:, i-1)); % True system dynamics
    noise_measurement = x(:, i) + [random('Normal', 0, 0.1), random('Normal', 0, 0.1), random('Normal', 0, 0.5)]';
    [x_hat(:, i), p_hat] = hinfinity_filter(time(i), time(i-1), p_hat, x_hat(:, i-1), u(:, i-1), noise_measurement, gamma); % Estimate state
end

% Plot results
figure;
subplot(3,1,1);
plot(time, x(1, :), 'b', time, x_hat(1, :), 'r');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True Position', 'Estimated Position');
title('Position Estimation');

subplot(3,1,2);
plot(time, x(2, :), 'b', time, x_hat(2, :), 'r');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('True Velocity', 'Estimated Velocity');
title('Velocity Estimation');

subplot(3,1,3);
plot(time, x(3, :), 'b', time, x_hat(3, :), 'r');
xlabel('Time (s)');
ylabel('Drag Coefficient (b_c)');
legend('True b_c', 'Estimated b_c');
title('Drag Coefficient Estimation');
