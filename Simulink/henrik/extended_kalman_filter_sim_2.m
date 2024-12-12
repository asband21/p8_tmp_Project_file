function [x_hat, P] = extended_kalman_filter_sim(time, olde_time, P_m1, Xp_m1, u_m1, y)
    % Time step
    td = time - olde_time;
    length_x = 2.74;
    length_y = 5.32;
    c = -1;
    moter_left  = [-length_x/2, -length_y/2+c, 0, 0];
    moter_right = [ length_x/2, -length_y/2+c, 0, 0];
    moter_bow   = [ 0,           length_y+c  , 0, 0];

    m = 1500; % Mass of the boat
    I = (length_y^2+length_x^2)*m*1/12;
    % Known parameters
    bx = -0;
    by = -0;
    bt = -0;
    t1x = moter_left(1);
    y1  = moter_left(2);
    %moter 2
    t2x = moter_right(1);
    y2  = moter_right(2);
    %moter 3
    t3x = moter_bow(1);
    y3  = moter_bow(2);


    % Noise covariances
    Q_m1 = diag([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4]); % Smaller process noise for bc
    R    = diag([1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-1, 1e-1, 1e-1]);    % Smaller measurement noise for velocity

    % State transition model
    A = [0 0 0 1 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 bx/m-Xp_m1(7)/m 0 0 0 0 0; 0 0 0 0 by/m-Xp_m1(8)/m 0 0 0 0; 0 0 0 0 0 bt/I-Xp_m1(9)/I 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0];
    c = diag([1; 1; 1; 0; 0; 0; 0; 0; 0]);
    f = @(xp_m1, u_m1) xp_m1+[0 0 0 1 0 0 0 0 0;
                              0 0 0 0 1 0 0 0 0;
                              0 0 0 0 0 1 0 0 0;
                              0 0 0 bx/m-xp_m1(7)/m 0 0 0 0 0; 
                              0 0 0 0 by/m-xp_m1(8)/m 0 0 0 0;
                              0 0 0 0 0 bt/I-xp_m1(9)/I 0 0 0;
                              0 0 0 0 0 0 0 0 0;
                              0 0 0 0 0 0 0 0 0;
                              0 0 0 0 0 0 0 0 0]*xp_m1*td + [0,0,0,0,0;
                                                             0,0,0,0,0;
                                                             0,0,0,0,0;
                                                             1,0,1,0,1;
                                                             0,1,0,1,0;
                                                             -y1/m, t1x/m, -y2/m, t2x/m, -y3/m;
                                                             0,0,0,0,0;
                                                             0,0,0,0,0;
                                                             0,0,0,0,0]*td*u_m1;
    % syms x1 x2 x3 x4 x5 x6 x7 x8 x9 bx by bt m I y1 y2 y3 t1x t2x td
    % xp_m1 = [x1; x2; x3; x4; x5; x6; x7; x8; x9]
    % f = ...
    % jacobian(f(xp_m1, [1;1;1;1;1]),xp_m1)
    F_m1 = @(x) [1, 0, 0,                   td,                    0,                    0,            0,            0,            0;
                 0, 1, 0,                    0,                   td,                    0,            0,            0,            0;
                 0, 0, 1,                    0,                    0,                   td,            0,            0,            0;
                 0, 0, 0, td*(bx/m-x(7)/m) + 1,                    0,                    0, -(td*x(4))/m,            0,            0;
                 0, 0, 0,                    0, td*(by/m-x(8)/m) + 1,                    0,            0, -(td*x(5))/m,            0;
                 0, 0, 0,                    0,                    0, td*(bt/I-x(9)/I) + 1,            0,            0, -(td*x(6))/I;
                 0, 0, 0,                    0,                    0,                    0,            1,            0,            0;
                 0, 0, 0,                    0,                    0,                    0,            0,            1,            0;
                 0, 0, 0,                    0,                    0,                    0,            0,            0,            1];
    % Measurement model
    h = @(xp_m1) [xp_m1(1); xp_m1(2); xp_m1(3); 0; 0; 0; 0; 0; 0];
    L_m1 = @(w) eye(9);
    H_f = @(Xp_m1) diag([1; 1; 1; 0; 0; 0; 0; 0; 0]);% C matrix
    M_f = @(Xp_m1) eye(9);

    H = H_f(Xp_m1);
    M = M_f(Xp_m1);
    I = eye(9);

    Pm = F_m1(Xp_m1)*P_m1*F_m1(Xp_m1)' + L_m1(Xp_m1)*Q_m1*L_m1(Xp_m1)';
    Xm = f(Xp_m1,u_m1);
    
    rank(obsv(F_m1(Xp_m1),H));
    cond(obsv(F_m1(Xp_m1),H));

    K = Pm * H' / (H * Pm * H' + M * R * M');
    Xp = Xm + K*(y-h(Xp_m1));
    Pp = (I - K*H)*Pm*(I - K*H)' + K*R*K';
    P = Pp;
    x_hat = Xp;   % Updated covariance
end