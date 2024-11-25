import numpy as np
import math

moter_1_x = -2;
moter_1_y = -5;
len_origin_1 = math.sqrt(moter_1_y*moter_1_y + moter_1_x*moter_1_x);
vector_angle_1 = math.atan2(moter_1_y, moter_1_y);

moter_2_x = 2;
moter_2_y = -5;
len_origin_2 = math.sqrt(moter_1_y*moter_1_y + moter_1_x*moter_1_x);
vector_angle_2 = math.atan2(moter_1_y, moter_1_y);

moter_1_3 = 0;
moter_1_3 = 4;
len_origin_3 = math.sqrt(moter_1_y*moter_1_y + moter_1_x*moter_1_x);

steb = 0.3

theta_1 = 0
while theta_1 < 3.14*2: 
    theta_2 = 0
    while theta_2 < 3.14*2: 
        B = np.array([[np.cos(theta_1 + vector_angle_1), np.cos(theta_2 + vector_angle_2), 0],
              [np.sin(theta_1 + vector_angle_1), np.sin(theta_2 + vector_angle_2), 1],
              [np.sin(theta_1) * len_origin_1, np.sin(theta_2) * len_origin_2, len_origin_3]])
        cond_number = np.linalg.cond(B)
        print(f"Condition Number({theta_1:.3f}, {theta_2:.3f}) :{cond_number}")
        theta_2 = theta_2 + steb;
    theta_1 = theta_1 + steb;

