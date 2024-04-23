from numpy import matrix, pi, ndarray, linspace, sin, cos, stack, asarray, absolute, zeros
# from math import sin, cos
import matplotlib.pyplot as plt


####################################################################################
def get_sine_ref_points(n_points: int, a, b, c, s, h, height, width, depth, origin: list[float], use_depth: int):

    ampz = h * height

    theta = linspace(0, 2*pi, n_points)
    npx = zeros(n_points)
    if use_depth == 1:
        npx = absolute(asarray(theta-pi))/pi*depth - (depth/2)
    npy = theta/(2*pi)*width - (width/2)
    npz = ampz * (sin(a*(theta+s)) + sin(b*(theta+s)) + sin(c*(theta+s)))

    # extract each dimension vector and convert to python list
    x = npx.tolist()
    y = npy.tolist()
    z = npz.tolist()

    for i in range(n_points):
        x[i] += origin[0]
        y[i] += origin[1]
        z[i] += origin[2]

    return x, y, z



####################################################################################
def get_rotation_matrix(axis: str, angle: float) -> matrix:
    
    theta = angle / 180 * pi   # in radians
    
    match axis:
        case "x":
            tran = [[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]]
        case "y":
            tran = [[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]]
        case "z":
            tran = [[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]]
    
    return matrix(tran)


####################################################################################
def get_spiral_ref_points(n_points: int, r: float, h: float, axis: str, angle: float, origin: list[float]):

    theta = linspace(0, 2 * pi, n_points)
    old_x = r * sin(theta * 2)
    old_y = r * cos(theta * 2)
    old_z = linspace(-h/2, h/2, n_points)
    old_points = stack([old_x, old_y, old_z])

    # get transformation matrix
    T = get_rotation_matrix(axis, angle)

    # Prepare arrays x, y, z
    new_points = asarray(T * old_points)

    # extract each dimension vector and convert to python list
    x = new_points[0].tolist()
    y = new_points[1].tolist()
    z = new_points[2].tolist()

    for i in range(n_points):
        x[i] += origin[0]
        y[i] += origin[1]
        z[i] += origin[2]

    return x, y, z


####################################################################################
def plot_with_scale(x: ndarray, y: ndarray, z: ndarray, margin: float, label: str):
    
    # get bounds for each dimension
    x_min = x.min()
    x_max = x.max()
    y_min = y.min()
    y_max = y.max()
    z_min = z.min()
    z_max = z.max()
    
    # get limits
    axes_min = min(x_min, y_min, z_min) - margin
    axes_max = max(x_max, y_max, z_max) + margin
    
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim([axes_min, axes_max])
    ax.set_ylim([axes_min, axes_max])
    ax.set_zlim([axes_min, axes_max])
    
    ax.plot(x, y, z, label=label)
    ax.legend()
    
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    plt.show()