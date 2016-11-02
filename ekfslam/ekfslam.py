# EKF SLAM - prediction step.
#
# slam_09_a_slam_prediction
# Claus Brenner, 20 JAN 13
from math import sin, cos, pi, atan2, sqrt
import numpy as np

class ExtendedKalmanFilterSLAM:
    def __init__(self, state):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = np.diag([0.2**2, 0.2**2, (10.0 / 180.0 * pi) ** 2])

        # Some constants.
        self.scanner_displacement = 0.0
        self.control_motion_factor = 0.35  # Error in motor control.
        self.control_turn_factor = 0.6  # Additional error due to slip when turning.
        self.measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
        self.measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

        # Currently, the number of landmarks is zero.
        self.number_of_landmarks = 0

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
        x = state[0]
        y = state[1]
        theta = state[2]

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
        x = state[0]
        y = state[1]
        theta = state[2]
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

    def predict(self, control):
        """The prediction step of the Kalman filter."""

        #jacobian of g in respect to control: dg(x,y,theta,v,w,dt)/d(v,w,dt)
        G3 = self.dg_dstate(self.state, control)
        v,w,dt = control

        # Covariance in control space, depends on move distance.
        alpha_1 = self.control_motion_factor
        alpha_2 = self.control_turn_factor
        g2v = (alpha_1 * v)**2 #variance of linear velocity
        g2w = (alpha_1 * v)**2 + (alpha_2 * (w))**2 #variance of angular velocity
        g2t = 0.001 #variance of time command
        control_covariance = np.diag([g2v,g2w,g2t])

        # jacobian of g in respect to control: dg(x,y,theta,v,w,dt)/d(v,w,dt)
        V = self.dg_dcontrol(self.state, control)

        # R = V * (covariance in control space) * VT.
        R3 = np.dot(V, np.dot(control_covariance, V.T))

        #size of the matrix in accordance with the number of landmarks
        N = self.number_of_landmarks
        N2 = N*2

        G = np.zeros((3+N2,3+N2)) #G matrix of size (3 + 2*N)x(3 + 2*N)
        G[0:3,0:3] = G3     # insert jacobian  dg/d(x,y,theta)
        G[3:,3:] = np.eye(N2) #complete diagonals with ones

        R = np.zeros((3+N2,3+N2)) # #R matrix of size (3 + 2*N)x(3 + 2*N)
        R[0:3,0:3] = R3 # insert motion noise matrix R3 into R

        # covariance update:
        #covariance' = G * covariance * GT + R
        self.covariance = np.dot(G, np.dot(self.covariance, G.T)) + R

        # state update:
        # state' = g(state, control)

        new_state = self.g(self.state[:3], control)

        self.state = np.hstack((new_state[:],self.state[3:]))



    def add_landmark_to_state(self, initial_coords):
        """Enlarge the current state and covariance matrix to include one more
           landmark, which is given by its initial_coords (an (x, y) tuple).
           Returns the index of the newly added landmark."""


        index = self.number_of_landmarks
        self.number_of_landmarks +=1

        x1, y1 = initial_coords
        new_landmark = np.array([x1,y1])

        dim_x,dim_y = self.covariance.shape
        new_matrix = np.zeros((dim_x+2,dim_y+2))

        new_matrix[0:dim_x,0:dim_y] = self.covariance
        new_matrix[dim_x:,dim_y:] = np.diag([50,50])

        self.covariance = new_matrix

        new_state = np.hstack((self.state[:], new_landmark[:]))
        self.state = new_state


        return index  # Replace this.



    def h(self,state, landmark):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        scanner_displacement = self.scanner_displacement
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return np.array([r, alpha])



    def dh_dstate(self,state, landmark):
        scanner_displacement = self.scanner_displacement

        theta = state[2]
        cost, sint = cos(theta), sin(theta)
        dx = landmark[0] - (state[0] + scanner_displacement * cost)
        dy = landmark[1] - (state[1] + scanner_displacement * sint)
        q = dx * dx + dy * dy
        sqrtq = sqrt(q)
        drdx = -dx / sqrtq
        drdy = -dy / sqrtq
        drdtheta = (dx * sint - dy * cost) * scanner_displacement / sqrtq
        dalphadx =  dy / q
        dalphady = -dx / q
        dalphadtheta = -1 - scanner_displacement / q * (dx * cost + dy * sint)

        return np.array([[drdx, drdy, drdtheta],
                      [dalphadx, dalphady, dalphadtheta]])

    def correct(self, measurement, landmark_index):
        """The correction step of the Kalman filter."""
        # Get (x_m, y_m) of the landmark from the state vector.
        landmark = self.state[3+2*landmark_index : 3+2*landmark_index+2]
        H3 = self.dh_dstate(self.state, landmark)

        # --->>> Add your code here to set up the full H matrix.
        N = self.number_of_landmarks
        new_H = np.zeros((2,3+2*N))
        new_H[0:2,0:3] = H3
        new_H[0:2, 3+2*landmark_index : 3+2*landmark_index+2] = new_H[0:2,0:2]*-1

        H = new_H

        # Update step
        Q = np.diag([self.measurement_distance_stddev**2,
                  self.measurement_angle_stddev**2])
        K = np.dot(self.covariance,
                np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(self.covariance, H.T)) + Q)))
        innovation = np.array(measurement) -\
                     self.h(self.state, landmark)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        self.state = self.state + np.dot(K, innovation)
        self.covariance = np.dot(np.eye(np.size(self.state)) - np.dot(K, H),
                              self.covariance)



    def get_landmarks(self):
        """Returns a list of (x, y) tuples of all landmark positions."""

        return ([(self.state[3+2*j], self.state[3+2*j+1])
                 for j in xrange(self.number_of_landmarks)])
    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = np.linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))

    def get_landmark_error_ellipses(self):
        """Returns a list of all error ellipses, one for each landmark."""
        ellipses = []
        for i in xrange(self.number_of_landmarks):
            j = 3 + 2 * i
            ellipses.append(self.get_error_ellipse(
                self.covariance[j:j+2, j:j+2]))
        return ellipses

    def associate_landmark(self,measure):
        """ Given a measure in global coordinates, find if
        it corresponds to any landmark in state"""
        x,y = measure
        landmarks = self.get_landmarks()
        for i in xrange(len(landmarks)):
            d = sqrt((x - landmarks[i][0])**2 + (y - landmarks[i][1])**2)
            print "saved landmarks", landmarks[i]
            print "measure", measure
            print "error",d
            #raw_input()
            if d < 0.5:
                print "returning index"
                return i

        print "adding landmark"
        return -1
