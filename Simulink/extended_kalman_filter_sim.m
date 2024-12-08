function [x_hat,P] = extended_kalman_filter_sim(time, olde_time, P_m1, Xp_m1, u_m1, y)
    td = time - olde_time;
    b = -800;
    m = 1500;
    bc = Xp_m1(3);
    
    f = @(xp_m1, u_m1,w) = eye(size(A))+[0, 1, 0 ;0, (b+bc)/m, 0;0, 0,  0]*xp_m1*td + [0, 0, 0;0, 1/m, 0;0, 0, 0]*td*u_m1 + eye(size(A))*w*td;
    F_m1 = @(Xp_m1)  [0, 1,   0  ;
                      0, 1/m, 1/m;
                      0, 1,   0  ] * Xp_m1;
    L_m1 = @(w) = [1, 0, 0;
                   0, 1, 0;
                   0, 0, 1]*w
    H_f = @(Xp_m1)[1, 0, 0; % C matrix
                0, 0, 0;
                0, 0, 0]*Xp_m1;
    H = H_f(Xp_m1);
    
    I = eye(3)
    Pm = F_m1(Xp_m1)*P_m1*F_m1(Xp_m1)'+L_m1(Xp_m1)*Q_m1*L_m1(Xp_m1)'
    Xm = f(Xp_m1,u_m1,0)

    H = H_f(Xp_m1);
    K  = Pm*H'*inv(H*Pm*H'+R)
    Xp = Xm + K*(y-H*Xm)
    Pp = (I - K*H)*Pm*(I - K*H)' + K*R*K'

end