# testing code borrow from clauss brenner slam course

from math import sin, cos, pi
from numpy import *

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

        return array([new_x, new_y, new_theta])
    @staticmethod
    def dg_dstate(state, control):
        v,w,dt = control
        x,y,theta = state
        w += 10e-6
        if w != v:
            q = v/w
            g1 =  q*cos(theta + (w*dt)) - q*cos(theta)
            g2 = -q*sin(theta) +q*sin(theta + (w*dt))

            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])
        else:
            g1 = -dt*v*sin(theta)
            g2 = dt*v*cos(theta)

            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])
        return m


if __name__ == '__main__':
    # If the partial derivative with respect to x, y and theta (the state)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    v = 5.0
    w = 5.0
    dt = 0.5
    control = array([v,w,dt])

    # Compute derivative numerically.
    print "Numeric differentiation dx, dy, dtheta:"
    delta = 1e-7
    state_x = array([x + delta, y, theta])
    state_y = array([x, y + delta, theta])
    state_theta = array([x, y, theta + delta])

    dg_dx = (ExtendedKalmanFilter.g(state_x, control) -\
             ExtendedKalmanFilter.g(state, control)) / delta
    dg_dy = (ExtendedKalmanFilter.g(state_y, control) -\
             ExtendedKalmanFilter.g(state, control)) / delta
    dg_dtheta = (ExtendedKalmanFilter.g(state_theta, control) -\
                 ExtendedKalmanFilter.g(state, control)) / delta
    dg_dstate_numeric = column_stack([dg_dx, dg_dy, dg_dtheta])
    print dg_dstate_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dx, dy, dtheta:"
    dg_dstate_analytic = ExtendedKalmanFilter.dg_dstate(state, control)
    print dg_dstate_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dstate_numeric - dg_dstate_analytic
    print "Seems correct:", allclose(dg_dstate_numeric, dg_dstate_analytic)
