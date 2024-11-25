
speed = [1;1;1]

friction = [-2, -1, -2];
pos = [1, 1, pi];


firk = [friction(1) 0           0           ;
        0           friction(2) 0           ;
        0           0           friction(3)]

glo_to_lok = [cos(-pos(3)) -sin(-pos(3)) 0 ;
              sin(-pos(3)) cos(-pos(3))  0 ;
              0           0              1]

globla_fik = glo_to_lok*firk*speed
