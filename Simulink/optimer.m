bx = -100;
by = -1000;
br = -2000;
m = 3000;
len = 10;
I = (1/12)*m*len*len;
%x = [x_d; y_d; theta_d];

A = [bx/m 0 0;
     0 by/m 0;
     0 0 br/m];

theta_1 = 1;
moter_1_x = -2;
moter_1_y = -5;
len_origen_1 = sqrt(moter_1_y*moter_1_y + moter_1_x*moter_1_x);
vector_angel_1 = atan2(moter_1_y, moter_1_y);

theta_2 = 1;
moter_2_x = 2;
moter_2_y = -5;
len_origen_2 = sqrt(moter_1_y*moter_1_y + moter_1_x*moter_1_x);
vector_angel_2 = atan2(moter_1_y, moter_1_y);

theta_3 = 1;
moter_1_3 = 0;
moter_1_3 = 4;
len_origen_3 = sqrt(moter_1_y*moter_1_y + moter_1_x*moter_1_x);
vector_angel_3 = atan2(moter_1_y, moter_1_y);


b_min = @(theta) cond([cos(theta(1)+vector_angel_1), cos(theta(2)+vector_angel_2) 0;
                         sin(theta(1)+vector_angel_1) sin(theta(2)+vector_angel_2) 1;
                         sin(theta(1))*len_origen_1 sin(theta(2))*len_origen_2 sin(theta_3)*len_origen_3])

tt0 = b_min([0,0])
initial_guess = [0.3, 1.23];
% men graf over optimering;
%options = optimset('PlotFcns',@optimplotfval);
minimum = fminsearch(b_min, initial_guess) %,options)

