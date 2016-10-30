# This adds the derivative of g, this time with respect to the control
# (left and right motor movement).
#
# slam_07_c_control_derivative
# Claus Brenner, 11.12.2012

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
    def dg_dcontrol(state, control):
        v,w,dt = control
        x,y,theta = state
        if w==0: w+=0.0001

        q1 = (-sin(w*dt + theta) + sin(theta))/w
        g1dv = -q1
        g1dw = (v/w)*( q1 + dt*cos(dt*w + theta))
        g1dt = v * cos(dt*w + theta)

        q2 = (cos(theta) - cos(dt*w + theta))/w
        g2dv = q2
        g2dw = (v/w)*(-q2 + dt*sin(dt*w + theta))
        g2dt = v*sin(dt*w + theta)

        m = array([[g1dv, g1dw ,g1dt], [g2dv, g2dw,g2dt], [0, dt,w]])  # Remove this.

        return m


if __name__ == '__main__':
    # If the partial derivative with respect to l and r (the control)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    v = 10.0
    w = 4.32
    dt = 0.15
    control = array([v, w,dt])

    # Compute derivative numerically.
    print "Numeric differentiation dl, dr"
    delta = 10e-7

    control_v = array([v + delta,w, dt])
    control_w = array([v, w + delta, dt])
    control_dt =  array([v, w , dt + delta])

    dg_dv = (ExtendedKalmanFilter.g(state, control_v) -\
             ExtendedKalmanFilter.g(state, control)) / delta
    dg_dw = (ExtendedKalmanFilter.g(state, control_w) -\
             ExtendedKalmanFilter.g(state, control)) / delta
    dg_dt = (ExtendedKalmanFilter.g(state, control_dt) -\
             ExtendedKalmanFilter.g(state, control)) / delta

    dg_dcontrol_numeric = column_stack([dg_dv, dg_dw, dg_dt])

    print dg_dcontrol_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dl, dr:"
    dg_dcontrol_analytic = ExtendedKalmanFilter.dg_dcontrol(state, control)
    print dg_dcontrol_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dcontrol_numeric - dg_dcontrol_analytic
    print "Seems correct:", allclose(dg_dcontrol_numeric, dg_dcontrol_analytic)
