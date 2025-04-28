import numpy as np
from scipy.signal import place_poles

# Gravity constant
g = 9.81

# System matrices
A = np.array([[0, 1, 0, 0], [0, 0, g, 0], [0, 0, 0, 1], [0, 0, 0, 0]])

B = np.array([[0], [0], [0], [1]])

# Desired closed-loop poles
desired_poles = np.array([-1.0, -1.5, -2.0, -2.5])

# Place the poles
place_obj = place_poles(A, B, desired_poles)

# Extract gain matrix K
K = place_obj.gain_matrix

print("Calculated K matrix:", K)
