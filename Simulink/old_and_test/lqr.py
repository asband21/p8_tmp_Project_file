import numpy as np
from scipy.linalg import solve_continuous_are

# Define matrices A, B, Q, and R
A = np.array([[1, 2, 3],
              [4, 5, 6],
              [7, 8, 9]])
B = np.array([[1, 0, 1],
              [0, -1, 2],
              [3, 0, -1]])
Q = np.diag([10, 10, 10])  # Q is a 3x3 matrix with higher penalties on state deviations
R = np.diag([1, 1, 1])     # R is a 3x3 identity matrix with a moderate control penalty
print(A)
print(B)
print(Q)
print(R)

# Solve the continuous-time Algebraic Riccati Equation (ARE)
P = solve_continuous_are(A, B, Q, R)

# Compute the gain matrix K
K = np.linalg.inv(R) @ B.T @ P

# Return the matrices P and K for detailed insight
print(P)
print(K)

