from math import pi, cos
import matplotlib.pyplot as plt


def cosineInterpolate(start, end, npoints):
    new_points = []
    for index in range(npoints):
        # angle = (start + i*interval) * pi
        angle = index / (npoints-1) * pi
        new_ratio = (1.0 - cos(angle)) * 0.5   # in [0, 1]
        new_point = start + new_ratio * (end - start)   # in [start, end]
        new_points.append(new_point)
    return new_points


res_list = cosineInterpolate(1, 5, 10)

print(res_list)

plt.plot(res_list)
plt.show()