from numpy.linalg import solve
from numpy import array, roots, real, imag
import matplotlib.pyplot as plt
from math import log2

# ##########################################################################
# def solve_coeffs(T, D, v0, a0):
#     T2, T3, T4, T5 = T**2, T**3, T**4, T**5
#     A = array([[T3, T4, T5], [3*T2, 4*T3, 5*T4], [6*T, 12*T2, 20*T3]])
#     v = array([D-v0*T-a0*T2/2, -v0-a0*T, -a0])
#     sol = solve(A, v)
#     # return float(sol[0]), float(sol[1]), float(sol[2])
#     return sol[0], sol[1], sol[2]

##########################################################################
def solve_coeffs(T, D):
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    A = array([[T3, T4, T5], [3*T2, 4*T3, 5*T4], [6*T, 12*T2, 20*T3]])
    v = array([D, 0, 0])
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
def get_real_root(coeffs):
    rts = roots(coeffs)
    real_rts = []
    for rt in rts:
        if imag(rt) == 0:
            real_rts.append(real(rt))
    if len(real_rts) > 1:
        print("MORE THAN 1 REAL ROOT!")
        raise TypeError
    else:
        return real_rts[0]


##########################################################################
def simulate(a, b, A, W, alpha, use_w=True, use_alpha=True):
    
    # {a, b} are the Fitts' linear regression parameters
    # {A, W} are the ring parameters for amplitude and width respectively
    # {alpha} is the autonomy level
    
    ###### 1. estimate required time using task parameters and Fitts' Law ######
    T = compute_fitts_time(a, b, A, W)
    # print("Fitts time = %.3f" % T)
    
    ###### 2. calculate quintic polynomial fit for the position trajectory ######
    b0, b1, b2 = 0, 0, 0
    # b3, b4, b5 = solve_coeffs(T, A, 0, 0)
    b3, b4, b5 = solve_coeffs(T, A)
    
    ###### 3. solve for (t) when the human position reaches the desired distance D ######
    D = A
    if use_alpha:
        D *= (1.0-alpha)
    if use_w:
        D -= (W/2)
        
    coeffs = [b5, b4, b3, b2, b1, b0-D]
    t = get_real_root(coeffs)
    
    return t



##########################################################################
def main():
    
    ################## Task & Fitts Parameters ##################
    a = -0.4                        # Fitts' intercept
    b = 0.47                        # Fitts' slope
    A = 0.236
    W = 0.01
    
    alpha = 0.8
    
    
    ###### simluate ######
    t = simulate(a, b, A, W, alpha)
    print("The calculated time is ", t)
    
    


if __name__ == "__main__":
    main()
