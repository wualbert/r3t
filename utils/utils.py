import numpy as np

def wrap_angle(theta):
    theta %= 2*np.pi
    if theta>=np.pi:
        return theta-2*np.pi
    else:
        return theta

def angle_diff(theta1, theta2):
    cw = (theta2-theta1)%(2*np.pi)
    cw = wrap_angle(cw)
    ccw = (theta1-theta2)%(2*np.pi)
    ccw = wrap_angle(ccw)
    if abs(cw)>abs(ccw):
        return ccw
    else:
        return cw
