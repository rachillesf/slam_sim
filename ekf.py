
from math import *
import numpy as np

class ExtendedKalmanFilter:

    @staticmethod
    def g(state, control):

        [v,w,dt] = control

        #member variables for state
        x = state[0]
        y = state[1]
        theta = state[2]

        if w == 0:
            new_x = x + cos(theta) * dt
            new_y = y + sin(theta) * dt
            new_theta = theta
        else:
            new_x = x - (v/w)*sin(theta) + (v/w) * sin(theta + (w*dt))
            new_y = y + (v/w)*cos(theta) - (v/w) * cos(theta + (w*dt))
            new_theta = (theta + w*dt) % (2*pi)

        return np.array([new_x, new_y, new_theta])
