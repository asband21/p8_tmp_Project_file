function [x_hat,P] = extended_kalman_filter_sim(time, olde_time, P_m1, Xp_m1, u_m1, y)
    td = time - olde_time;
    b = -800;
    m = 1500;

    f = @(x, u, w, t)  [0, 1, 0 ;0, (b+bc)/m, 0;0, 0,  0]*x*t + [0, 0, 0;0, 1/m, 0;0, 0, 0]*u*t + w


end