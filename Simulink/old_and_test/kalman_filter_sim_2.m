function [x_hat, P] = ekf_update(time, olde_time, P_m1, Xp_m1, u_m1, y)
    td = time - olde_time; % Time step
    
    % System Parameters
    b = -1000;
    m = 1500;
    x1 = Xp_m1(1); x2 = Xp_m1(2); bc = Xp_m1(3);

    % Non-linear State Transition
    f = [
        x2;
        (b + bc)/m * x2 + 1/m * u_m1;
        0
    ];

    % Linearized State Transition Jacobian
    F_k = [
        0, 1, 0;
        0, (b + bc)/m, x2/m;
        0, 0, 0
    ];

    % Measurement Model
    h = [x1; x2];

    % Measurement Jacobian
    H_k = [
        1, 0, 0;
        0, 1, 0
    ];

    % Process and Measurement Noise
    Q_k = diag([1e-5, 1e-5, 1e-2]);
    R_k = diag([1e-2, 1e-2]);

    % Prediction Step
    x_pred = Xp_m1 + f * td;                % Predict state
    P_pred = F_k * P_m1 * F_k' + Q_k;       % Predict error covariance

    % Kalman Gain
    K_k = P_pred * H_k' / (H_k * P_pred * H_k' + R_k);

    % Update Step
    x_hat = x_pred + K_k * (y - h);         % Update state
    P = (eye(3) - K_k * H_k) * P_pred;     % Update covariance
end


function [x_1] = dynamik(time, olde_time, x, u)
    td = time - olde_time; % Time delta
    m = 1500; b = -1000; bc = x(3);  % Use current estimate of b_c
    A = [0, 1, 0 ;
         0, (b + bc)/m, 0 ;
         0, 0, 0];
    B = [0; 1/m; 0];
    x_1 = A * x * td + B * u * td + x;
end

clear all; close all; clc;

N = 1000;         % Number of time steps
ts = 0.1;         % Time step size
time = ts:ts:N*ts;

% True system states
x = zeros(3, N);  
x(:, 1) = [0; 1; -200];  % Initial state: [position; velocity; drag coefficient]

% Estimated states
x_hat = zeros(3, N);  
x_hat(:, 1) = [0; 1; 0];  % Initial estimate
p_hat = eye(3) * 1e-3;    % Small positive definite matrix

% Control input
u = zeros(1, N);
u(500:600) = -500;

% Kalman Filter Loop
for i = 2:N
    % True system dynamics
    x(:, i) = dynamik(time(i), time(i-1), x(:, i-1), u(i-1));
    noise_measurement = x(1:2, i-1) + random('Normal', 0, 1, [2, 1]);
    [x_hat(:, i), p_hat] = ekf_update(time(i), time(i-1), p_hat, x_hat(:, i-1), u(i-1), noise_measurement);

end

% Plot Results
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

