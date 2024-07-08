import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
import time

################################################### NOTE: Define trajectory in [meters] ###################################################


####################### calculating the trajectory coordinates #######################

NUM_POINTS = 50
TOTAL_TIME = 1   # in seconds

# Spherical coordinates
r = 0.1  # [meters]
t = np.linspace(0, 2 * np.pi, NUM_POINTS)

# In original 3D plane
old_x = r * np.cos(t)
old_y = r * np.sin(t)
old_z = np.zeros(NUM_POINTS)
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


####################### live plotting the trajectory #######################

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# ax.set_xlim(-0.5, 0.5)
# ax.set_ylim(-0.5, 0.5)

count = 0
plotting = True

while plotting:

    xs = x[count]
    ys = y[count]
    zs = z[count]

    # Do plotting
    ax.scatter(xs, ys, zs)
    plt.draw()

    plt.pause(TOTAL_TIME / (NUM_POINTS - 1))

    count += 1

    if count >= NUM_POINTS:
        plotting = False
    


plt.ioff()
plt.show()