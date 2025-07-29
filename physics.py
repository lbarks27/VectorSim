import numpy as np
import quaternion as quat
import solvers_updated as so


####################################################################################################################################################################################################################################################################################################################


def euler_step(f, t, y, h):

    return f(t, y) * h

def rk4_step(f, t, y, h): #returns kwmean + h, must use v(n+1) = v(n) + kwmean +h

    k1 = f(t, y)
    k2 = f(t + h * 0.5, y + (k1 * h * 0.5))
    k3 = f(t + h * 0.5, y + (k2 * h * 0.5))
    k4 = f(t + h, y + (k3 * h))
    return y + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


####################################################################################################################################################################################################################################################################################################################


#ODE rotational motion

def second_order_attitude_ode(t, attitude_state):

    broadcasted_output = np.array([(attitude_state[0] / attitude_state[1]) - np.cross(attitude_state[2], (attitude_state[1] * attitude_state[2]))], dtype = np.float64)

    return broadcasted_output