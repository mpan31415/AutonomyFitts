from math import pi, sin, cos


N_TARGETS = 9
r = 10

target_positions = []


for i in range(N_TARGETS):
    theta = float(i/(N_TARGETS))*2*pi
    print(theta)
    tar_y = -r * sin(theta)
    tar_z = r * cos(theta)
    target_positions.append((tar_y, tar_z))

print(target_positions)