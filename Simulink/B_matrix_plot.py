import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Motor positions
moter_1_x, moter_1_y = -2, -5
moter_2_x, moter_2_y = 2, -5
moter_3_x, moter_3_y = 0, 4

# Lengths and angles for vectors
len_origin_1 = math.sqrt(moter_1_x**2 + moter_1_y**2)
len_origin_2 = math.sqrt(moter_2_x**2 + moter_2_y**2)
len_origin_3 = math.sqrt(moter_3_x**2 + moter_3_y**2)

vector_angle_1 = math.atan2(moter_1_y, moter_1_x)
vector_angle_2 = math.atan2(moter_2_y, moter_2_x)

# Step size for angles
steb = 0.1

# Lists to store data for plotting
theta_1_vals = []
theta_2_vals = []
cond_numbers = []

# Loop through angles
theta_1 = 0
while theta_1 < 2 * math.pi:
    theta_2 = 0
    while theta_2 < 2 * math.pi:
        # Construct matrix B
        B = np.array([
            [np.cos(theta_1 + vector_angle_1), np.cos(theta_2 + vector_angle_2), 0],
            [np.sin(theta_1 + vector_angle_1), np.sin(theta_2 + vector_angle_2), 1],
            [np.sin(theta_1) * len_origin_1, np.sin(theta_2) * len_origin_2, len_origin_3]
        ])
        
        # Calculate condition number
        cond_number = np.linalg.cond(B)
        if cond_number > 1000:
            cond_number = 1000
        # Append values for plotting
        theta_1_vals.append(theta_1)
        theta_2_vals.append(theta_2)
        cond_numbers.append(cond_number)
        
        # Increment theta_2
        theta_2 += steb
    
    # Increment theta_1
    theta_1 += steb

# Convert to numpy arrays for plotting
theta_1_vals = np.array(theta_1_vals)
theta_2_vals = np.array(theta_2_vals)
cond_numbers = np.array(cond_numbers)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(theta_1_vals, theta_2_vals, cond_numbers, cmap='viridis', edgecolor='none')
ax.set_xlabel('Theta 1')
ax.set_ylabel('Theta 2')
ax.set_zlabel('Condition Number')
ax.set_zlim(0, 1000)
ax.set_title('Condition Number as a Function of Theta 1 and Theta 2')

plt.show()
