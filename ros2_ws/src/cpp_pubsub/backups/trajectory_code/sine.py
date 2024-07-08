import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos


ax = plt.figure().add_subplot(projection='3d')

# define the sine curve parameters
a = 2

# Sine curve in non-base frame
old_x = np.linspace(0, 4 * np.pi, 100)
old_y = a * np.sin(old_x)
# old_z = a * np.sin(old_x*4)
old_z = np.zeros(100)
old_points = np.stack([old_x, old_y, old_z])

# Transformation matrix into base frame
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