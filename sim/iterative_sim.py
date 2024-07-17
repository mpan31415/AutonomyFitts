from numpy.linalg import solve
from numpy import array
import matplotlib.pyplot as plt
from math import log2

##########################################################################
def solve_coeffs(T, D, v0, a0):
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    A = array([[T3, T4, T5], [3*T2, 4*T3, 5*T4], [6*T, 12*T2, 20*T3]])
    v = array([D-v0*T-a0*T2/2, -v0-a0*T, -a0])
    sol = solve(A, v)
    # return float(sol[0]), float(sol[1]), float(sol[2])
    return sol[0], sol[1], sol[2]

##########################################################################
def compute_dist_vel_accel(b0, b1, b2, b3, b4, b5, t):
    t2, t3, t4, t5 = t**2, t**3, t**4, t**5
    dist = b0 + b1*t + b2*t2 + b3*t3 + b4*t4 + b5*t5
    vel = b1 + 2*b2*t + 3*b3*t2 + 4*b4*t3 + 5*b5*t4
    accel = 2*b2 + 6*b3*t + 12*b4*t2 + 20*b5*t3
    return dist, vel, accel

##########################################################################
def compute_fitts_time(a, b, A, W):
    return a + b * log2(A/W + 1)

##########################################################################
def simulate_trajectory(a, b, A, W, alpha, vr, num_steps):
    
    ###### Compute total time & length of simulation timestep ######
    total_time = compute_fitts_time(a, b, A, W)
    delta = total_time / num_steps
    
    ###### trajectory recording ######
    t_vec = [0.0]
    x_vec = [0.0]
    v_vec = [0.0]
    a_vec = [0.0]

    ###### variables (initialized here, these change throughout the simulation) ######
    T = total_time      # required time window to reach target
    D = A               # remaining distance to target
    v0 = 0              # initial velocity of each time step
    a0 = 0              # initial acceleration of each time step

    ###### simulate until arrive at target ######
    while D > W/2:
        
        # estimate remaining time using Fitts' Law
        T = compute_fitts_time(a, b, D, W)
        if T < 0:
            break
        
        print("Computing trajectory for T = ", T)
        # compute polynomial coefficients
        b0, b1, b2 = 0, v0, a0/2
        b3, b4, b5 = solve_coeffs(T, D, v0, a0)
        
        # update remaining distance to target, initial velocity and acceleration at next time step
        dist_h, vel, accel = compute_dist_vel_accel(b0, b1, b2, b3, b4, b5, delta)
        dist_r = alpha * vr * delta
        dist = dist_h + dist_r
        D -= dist
        v0, a0 = vel, accel
        
        # add data to trajectory
        t_vec.append(t_vec[-1]+delta)
        x_vec.append(x_vec[-1]+dist)
        v_vec.append(v0)
        a_vec.append(a0)
        
        print("D = %.3f" % D)
        
    print("Final distance to target = %.3f" % D)
        
    return t_vec, x_vec, v_vec, a_vec


##########################################################################
def main():
    
    ################## Task Parameters ##################
    a = -0.4                        # Fitts' intercept
    b = 0.47                        # Fitts' slope
    A = 0.236
    W = 0.01
    alpha = 0.8
    vr = A / 1.2
    
    steps = 100

    ### simulate the trajectory ###
    t_vec, x_vec, v_vec, a_vec = simulate_trajectory(a, b, A, W, alpha, vr, steps)
    
    ### plot the trajectory ###
    plt.subplot(3, 1, 1)
    plt.plot(t_vec, x_vec, "r-o")
    plt.legend(["position"])
    
    plt.subplot(3, 1, 2)
    plt.plot(t_vec, v_vec, "g-o")
    plt.legend(["velocity"])
    
    plt.subplot(3, 1, 3)
    plt.plot(t_vec, a_vec, "b-o")
    plt.legend(["acceleration"])
    
    plt.show()



if __name__ == "__main__":
    main()
