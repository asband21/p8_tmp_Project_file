function [moter_left, moter_right, moter_bow, A ,mass, boat_dimensions, friction, inertia, wind_gain] = get_parameters()
%
% The boat is pointing in the y direction
% With the angle measured from x to x axis

%             [x,   y, angle, distance from origin]
moter_left  = [-1, -3, 0, 0];
moter_right = [ 1, -3, 0, 0];
moter_bow   = [ 0,  2, 0, 0];

n_l = norm(moter_left)
n_r = norm(moter_right)
n_b = norm(moter_bow)

% calculating the reference of the angle.
moter_left(3)  = atan2( moter_left(2), moter_left(1) );
moter_right(3) = atan2(moter_right(2), moter_right(1));
moter_bow(3)   = atan2(  moter_bow(2), moter_bow(1)  );

% calculating the distance from origin
moter_left(4)  = sqrt(moter_left(1)^2  + moter_left(2)^2 )
moter_right(4) = sqrt(moter_right(1)^2 + moter_right(2)^2)
moter_bow(4)   = sqrt(moter_bow(1)^2   + moter_bow(2)^2  )


% x axet/width 2.74 m;
length_x = 2.74;
% y axet/length 5.32 m;
length_y = 5.32
% z axet/height 1.50 m
length_z = 1.50;
% z axet above the water level/height 1.50 m
length_z_to_water = 0.50;
boat_dimensions = [length_x, length_y ,length_z];

mass = 3000;

%inertia = [(length_y^2+length_z^2)*mass*1/12,  0,                                  0                                 ;
%           0                                   (length_x^2+length_z^2)*mass*1/12 , 0                                 ;
%           0                                   0                                   (length_y^2+length_x^2)*mass*1/12];
inertia = (length_y^2+length_x^2)*mass*1/12;

% friction in the following direction.moter_left
b_x = -100;
b_y = -1000;
b_t = -2000;
friction = [b_x, b_y, b_t];

A = [b_x/mass 0        0        ;
     0        b_y/mass 0        ;
     0        0        b_t/mass];

wind_gain = 2;

end