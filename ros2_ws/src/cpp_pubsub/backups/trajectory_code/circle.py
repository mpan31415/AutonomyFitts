import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

################################################### NOTE: Define trajectory in [meters] ###################################################

ax = plt.figure().add_subplot(projection='3d')

# Spherical coordinates
r = 0.1  # [meters]
t = np.linspace(0, 2 * np.pi, 100)

# In original 3D plane
old_x = r * np.cos(t)
old_y = r * np.sin(t)
old_z = np.zeros(100)
old_points = np.stack([old_x, old_y, old_z])

# Transformation matrix into new coordinate frame
alpha = np.pi / 4
tran = [[1, 0, 0], [0, cos(alpha), sin(alpha)], [0, -sin(alpha), cos(alpha)]]
T = np.matrix(tran)

# Prepare arrays x, y, z
new_points = np.asarray(T * old_points)

x = new_points[0]
y = new_points[1]
z = new_points[2]

# Do plotting
ax.plot(x, y, z, label='parametric curve')
ax.legend()

plt.show()