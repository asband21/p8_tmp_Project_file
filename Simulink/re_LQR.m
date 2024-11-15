function [K, B, thuster_angle] = re_LQR()

%loading, and model parameters
[moter_left, moter_right, moter_bow, A ,mass, boat_dimensions, friction, inertia] = get_parameters()

%x_d = A*x + B*u;
%x = [x, y , angle]
%u = [moter_left, moter_right, moter_bow]

%setting up the functions for optimization.
%theta = [moter_left, moter_right]
B_fuc = @(theta) [cos(theta(1)),                             cos(theta(2))                               ,1                              ;
                  sin(theta(1)),                             sin(theta(1))                               ,0                              ;
                  sin(moter_left(3)+theta(1))*moter_left(4), sin(moter_right(3)+theta(2))*moter_right(4) ,sin(moter_bow(3))*moter_bow(4)];

min_b = @(theta) cond(B_fuc(theta));


% Starting optimization process
initial_guess = [0.3, 1.23];

%show a graph over the optimization uncommon the following line.
%options = optimset('PlotFcns',@optimplotfval);

thuster_angle = fminsearch(min_b, initial_guess) %,options)

B   = B_fuc(thuster_angle)
cot = min_b(thuster_angle)

thuster_angle = [thuster_angle, 0]

Q = [100 0   0   ;
     0   100 0   ;
     0   0   100];

R = [1 0 0;
     0 1 0;
     0 0 1];

[K,S,P] = lqr(A,B,Q,R)
end