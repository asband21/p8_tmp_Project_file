function [x_hat,P] = kalman_filter_sim_l(time, olde_time,P_m1, x_mes, x_hat_old)
    td = time - olde_time; %time delta
    
    bc = -1000;
    m = 1000;
    
    A = [0, 1,    0 ;
         0, bc/m, 0 ;
         0, 0,    0];
    
    B = [0, 0, 0;
         0, 1, 0;
         0, 0, 0];
    
    C = [1, 0, 0;
         0, 1, 0;
         0, 0, 1];
    
    Q = diag([1e-5, 1e-5, 1e-7]);  % Process noise covariance
    R = 1e-2;                      % Measurement noise covariance
    
    I = eye(3)
    P = (I-k*H)*P_m1
    %x_hat = A*x_hat_old*td+x_hat_old;
end

function [x_1] = dynamik(time, olde_time, x)
    td = time - olde_time; %time delta
    A = [0, 1,    0 ; 0, -1200/1500, 0 ;0, 0,    0];
    x_1 = A*x*td+x;
end

N = 100;
time = zeros(1, N);
ts = 0.1 %time step size.
time = ts:ts:N*ts;

x = zeros(3, N);        % True system dynamics.
x(:, 1) = [0; 1; 0];    % Initial state
x_hat = zeros(3, N);
x_hat(:, 1) = [0; 1; 0];
p = [1 0 0;
     0 1 0;
     0 0 1;] %estimate error covariance

u = zeros(1, N);        % Zero input force
u(100:200) = 10; 

for i = 2:N
    x(:,i)      = dynamik(time(i), time(i-1), x(:,i-1)); % True system dynamics.
    x_hat(:,i)  = kalman_filter_sim_l(time(i), time(i-1),p , x(:,i-1)+random('Normal',0,1), x_hat(:,i-1)); % Estes Medt state.
end


figure;
subplot(2,1,1);
plot(time, x(1, :), 'b', time, x_hat(1, :), 'r');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True Position', 'Estimated Position');
title('Position Estimation');
subplot(2,1,2);
plot(time, x(1, :), 'b', time, x_hat(1, :), 'r');
xlabel('Time (s)');
ylabel('velocity. (m/s)');
legend('True velocity', 'Estimated velocity');
title('velocity Estimation');