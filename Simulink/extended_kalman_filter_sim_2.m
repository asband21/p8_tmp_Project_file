function [x_hat, P] = extended_kalman_filter_sim(time, olde_time, P_m1, Xp_m1, u_m1, y)
    % Time step
    td = time - olde_time;

    % Known parameters
    b = 800;
    m = 1500;

    % Noise covariances
    Q_m1 = diag([1e-5, 1e-5, 1e-4]); % Smaller process noise for bc
    R = diag([1e-2, 1e-3, 1e-1]);    % Smaller measurement noise for velocity

    % State transition model
    f = @(xp_m1, u_m1) [xp_m1(1) + td * xp_m1(2);
                        xp_m1(2) + td * ((-b - xp_m1(3)) / m) * xp_m1(2) + td * u_m1(2) / m;
                        xp_m1(3)]; % bc remains constant

    % Measurement model
    h = @(Xp_m1) [Xp_m1(1); Xp_m1(2); 0]; % Measure position, velocity, no direct bc

    % Jacobian of state transition model
    F_m1 = @(Xp_m1) [1, td, 0;                           % ∂f/∂x1
                     0, 1 + td * (-b - Xp_m1(3)) / m, ... % ∂f/∂x2
                     td * Xp_m1(2) / m;                  % ∂f/∂bc
                     0, 0, 1];                           % ∂f/∂bc (constant)

    % Jacobian of measurement model
    H_f = @(Xp_m1) [1, 0, 0;   % Sensitivity of position
                    0, 1, 1/m; % Sensitivity of velocity (including bc)
                    0, 0, 0];  % No direct measurement for bc

    % Process noise Jacobian
    L_m1 = eye(3);

    % Identity matrix for Kalman gain calculation
    I = eye(3);

    % Prediction step
    Pm = F_m1(Xp_m1) * P_m1 * F_m1(Xp_m1)' + L_m1 * Q_m1 * L_m1';
    Xm = f(Xp_m1, u_m1);

    % Update step
    H = H_f(Xm);
    K = Pm * H' / (H * Pm * H' + R); % Kalman gain
    bc_update = K(3, :) * (y - h(Xm)); % Update bc selectively
    Xp = Xm + K * (y - h(Xm)); % State update
    Xp(3) = Xm(3) + 1 * bc_update; % Dampen bc updates
    Pp = (I - K * H) * Pm;           % Covariance update

    % Outputs
    x_hat = Xp; % Estimated state
    P = Pp;     % Updated covariance
end
