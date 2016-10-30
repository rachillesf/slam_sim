from math import sin, cos, pi, atan2
import numpy as np


class ExtendedKalmanFilter:
    def __init__(self,state):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = np.diag([1.5**2, 1.5**2, (10.0 / 180.0 * pi) ** 2])

        # Some constants.
        self.control_motion_factor = 0.35  # Error in motor control.
        self.control_turn_factor = 0.6  # Additional error due to slip when turning.


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

    @staticmethod
    def dg_dstate(state, control):
        v,w,dt = control
        x,y,theta = state
        w += 10e-6
        if w != v:
            q = v/w
            g1 =  q*cos(theta + (w*dt)) - q*cos(theta)
            g2 = -q*sin(theta) +q*sin(theta + (w*dt))

            m = np.array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])
        else:
            g1 = -dt*v*sin(theta)
            g2 = dt*v*cos(theta)

            m = np.array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])
        return m

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

        m = np.array([[g1dv, g1dw ,g1dt], [g2dv, g2dw,g2dt], [0, dt,w]])  # Remove this.

        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = np.linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, np.sqrt(eigenvals[0]), np.sqrt(eigenvals[1]))

    def predict(self, control):
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        v,w,dt = control

        alpha_1 = self.control_motion_factor
        alpha_2 = self.control_turn_factor

        g2v = (alpha_1 * v)**2
        g2w = (alpha_1 * v)**2 + (alpha_2 * (w))**2
        g2t = 0.001
        Sigma_control = np.diag([g2v,g2w,g2t])

        Vt = self.dg_dcontrol( self.state,control)#!!
        VtT = Vt.T

        Sigma_covariance = self.covariance
        Gt = self.dg_dstate( self.state,control )#!!
        GtT = Gt.T

        self.covariance = np.dot(np.dot(Gt,Sigma_covariance),GtT) + np.dot(np.dot(Vt,Sigma_control),VtT)

        self.state = self.g(self.state,control)
