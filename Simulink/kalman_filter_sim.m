function [x_hat,P] = kalman_filter_sim_l(time, olde_time, P_m1, Xp_m1,u_m1,y)
    td = time - olde_time; %time delta
    
    bc = Xp_m1(3);
    b = -800;
    m = 1500;
    
    A = [0, 1, 0 ;
         0, (b+bc)/m, 0;
         0, 0,  0];
    
    B = [0, 0, 0;
         0, 1/m, 0;
         0, 0, 0];
    
    C = [1, 0, 0;
         0, 0, 0;
         0, 0, 0];

    Q_m1 = diag([1e-5, 1e-5, 1e-3])
    %Q_m1 = diag([1e-5, 1e-5, 1e-7]);  % Process noise covariance
    R = diag([1e-2, 1e-2, 1e-1]);                      % Measurement noise covariance
    
    F_m1 = eye(size(A)) + A * td;
    G_m1 = B* td;
    H = C;
    
    I = eye(3)
    Pm = F_m1*P_m1*F_m1'+Q_m1
    K  = Pm*H'*inv(H*Pm*H'+R)
    Xm = F_m1*Xp_m1+G_m1*u_m1
    Xp = Xm + K*(y-H*Xm)
    Pp = (I - K*H)*Pm*(I - K*H)' + K*R*K'

    x_hat = Xp
    P = Pp
end

function [x_1] = dynamik(time, olde_time, x,u)
    td = time - olde_time; %time delta
    m = 1500; %b = -1000;
    A = [0, 1, 0 ; 0, x(3)/m, 0 ;0, 0,    0];
        B = [0, 0, 0; 0, 1/m, 0; 0, 0, 0];
    x_1 = A*x*td+B*u*td+x;
end


clear all; close all; clc;
N = 1000;
time = zeros(1, N);
ts = 0.1 %time step size.
time = ts:ts:N*ts;

x = zeros(3, N);        % True system dynamics.
x(:, 1) = [0; -10; -1000];    % Initial state
x_hat = zeros(3, N);
x_hat(:, 1) = [0; -10; -900];
p_hat = eye(3) * 1e-3;  % Small positive definite matrix

u = zeros(3, N);   % Zero input force
for i = 500:N; u(2,i) = sin(i*i*0.00006)*100000; end
u(2,100:200) = 5000;
u(2,400:750) = -700;

for i = 2:N
    x(:,i)      = dynamik(time(i), time(i-1), x(:,i-1), u(:,i-1)); % True system dynamics.
    noise_meserment = x(:,i-1) + [1,1,0]*random('Normal', 0, 1, [3, 1])*1;
    %[x_hat(:,i), p_hat] = kalman_filter_sim_l(time(i), time(i-1), p_hat ,x_hat(:,i-1), u(:,i-1), noise_meserment); % Estes Medt state.
    [x_hat(:,i), p_hat] = extended_kalman_filter_sim(time(i), time(i-1), p_hat ,x_hat(:,i-1), u(:,i-1), noise_meserment);
end


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
ylabel('velocity. (m/s)');
legend('True velocity', 'Estimated velocity');
title('velocity Estimation');

subplot(3,1,3);
plot(time, x(3, :), 'b', time, x_hat(3, :), 'r');
xlabel('Time (s)');
ylabel('Drag Coefficient (b_c)');
legend('True b_c', 'Estimated b_c');
title('Drag Coefficient Estimation');