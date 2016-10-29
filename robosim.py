#TODO:
# make automatic update, without key press
# 


import matplotlib.pyplot as plt
from math import *
import numpy as np
import random
import time
plt.ion()


class Robot:
    # simulate a robot model that can sense landmarks and move
    # state is represented in form of (x(m),y(m),theta(rad))
    # control is given by (linear_vel(m/s), ang_vel(rad/s), time_var(s))
    # the sensor are able to locate landmarks in a range given by the "sensor_maxrange"
    # and "sensor_opening"

    # init:
    #    creates robot and initializes location/orientation
    def __init__(self,init_state,map):
        self.map = map
        self.USE_NOISE = True
        self.state = [0.0,0.0,0.0]
        self.states_list = [init_state] #store all visited states
        self.sensor_opening = 3*pi/4
        self.sensor_maxrange = 3
        self.landmarks = []  #current visible landmarks
        self.landmarks_list = [] #store all landmarks observed
        self.measure_noise = [0.05,0.04]
        self.control_noise = [0.1,-0.2,0]


    # set_state:
    #       set the robot to a desired state
    def set_state(self,state):
        #set state
        self.state = state

    #apply_control_noise
    #   apply a sistematic and a random noise given in self.control_noise
    #   return: control with noise
    def apply_control_noise(self, control):

        rand = np.random.rand(2)
        rand[0]*= 0.03*control[0]
        rand[1]*= 0.03*control[1]

        v = control[0] + self.control_noise[0]*control[2] + rand[0]
        w = control[1] + self.control_noise[1]*control[2] + rand[1]
        print v,w,control[2]
        return [v,w,control[2]]

    # move
    #       execute a control instructios in the form of (v,w,dt)
    def move(self,control):

        #member variable of control with error
        if(self.USE_NOISE):
            [v,w,dt] = self.apply_control_noise(control)
        else:
            [v,w,dt] = control

        #member variables for state
        x = self.state[0]
        y = self.state[1]
        theta = self.state[2]

        if w == 0:
            new_x = x + cos(theta) * dt
            new_y = y + sin(theta) * dt
            new_theta = theta
        else:
            new_x = x - (v/w)*sin(theta) + (v/w) * sin(theta + (w*dt))
            new_y = y + (v/w)*cos(theta) - (v/w) * cos(theta + (w*dt))
            new_theta = (theta + w*dt) % (2*pi)

        #update state and append it to states list
        self.state =[float("{0:.4f}".format(new_x)),
                    float("{0:.4f}".format(new_y)),
                    float("{0:.4f}".format(new_theta))]

        self.states_list.append(self.state)

        self.search_landmarks(self.map.global_landmark_list)

    #apply_measurement_noise
    #   apply a sistematic and a random noise given in self.measure_noise
    #   return: ladmark with noise
    def apply_measurement_noise(self, landmark):
        dx = landmark[0] - self.state[0]
        dy = landmark[1] - self.state[1]

        alpha = (atan2(dy, dx) - self.state[2] + pi) % (2*pi) - pi
        r = sqrt(dx * dx + dy * dy)
        #limits the random noise to 2% of the landmark radius
        rand = np.random.rand(2)*0.02*r

        lx = landmark[0] + self.measure_noise[0]*r + rand[0]
        ly = landmark[1] + self.measure_noise[1]*r + rand[1]
        return [lx,ly]

    #is_landmark_visible
    #   check if a landmarks is visible to the sensor given the min/max angle
    #   and the max range.
    #   returns True or False
    def is_landmark_visible(self,landmark):
        dx = landmark[0] - self.state[0]
        dy = landmark[1] - self.state[1]

        max_alpha = self.state[2] + self.sensor_opening
        min_alpha = self.state[2] - self.sensor_opening
        alpha = (atan2(dy, dx) - self.state[2] + pi) % (2*pi) - pi

        r = sqrt(dx * dx + dy * dy)

        if alpha >= min_alpha and alpha <= max_alpha and r < self.sensor_maxrange:
            return True
        else: return False


    #search_landmarks
    #   given the coordinates of all the landmarks in the system, updates the
    #   self.landmark_list with the visible ones
    def search_landmarks(self,global_landmark_list):
        landmarks = []
        for landmark in global_landmark_list:
            if(self.is_landmark_visible(landmark)):
                dx = landmark[0] - self.state[0]
                dy = landmark[1] - self.state[1]
                r = sqrt(dx * dx + dy * dy)
                alpha = (atan2(dy, dx)) % (2*pi) - pi
                landmarks.append(self.apply_measurement_noise(landmark))

        self.landmarks = landmarks
        self.landmarks_list.append(landmarks)


    #__repr__
    #   allows printing the robot state using print function
    def __repr__(self):
        return '[x=%s y=%s theta=%s]' % (str(self.state[0]), str(self.state[1]),
                                                str(self.state[2]))


class Map:
    def __init__(self):
        self.size = 6 #assuming square map
        self.global_landmark_list = [[1.0,1.0],
                                     [2.0,2.0],
                                     [3.0,3.0],
                                     [4.0,4.0],
                                     [5.0,5.0]]
        self.IDEAL = False

    #move_in_list
    #   execute a list of control instructions in order
    def move_in_list(self,robot,control_list):
        for control in control_list:
            for time_step in range(5):
                dt = control[2]/5
                robot.move([control[0],control[1],dt])
                self.show_map(robot)
                plt.show()
                raw_input()
                plt.clf()

    def plot_global_landmarks(self):
        #plot all landmarks
        for ldm in self.global_landmark_list:
            plt.plot(ldm[0], ldm[1],'bo')
        plt.axis([-1,self.size, -1,self.size])

    def plot_robot_path(self,robot):
        #plot all robot states as lines
        for state in robot.states_list:
            x = state[0]
            y = state[1]
            theta = state[2]
            x1 = x + (0.1 * cos(theta))
            y1 = y + (0.1 * sin(theta))
            plt.plot([x,x1],[y,y1],color='b')
        #show robot final pose as a circle
        plt.gca().add_patch(plt.Circle((x,y),radius=0.1,fc='b'))

    def plot_observed_landmarks(self,robot):
        #plot all observed landmarks
        for obs_landmark in robot.landmarks:
            plt.plot(obs_landmark[0],obs_landmark[1],'go')
        plt.axis([-1,self.size, -1,self.size])

    def show_map(self,robot):
        self.plot_global_landmarks()
        self.plot_robot_path(robot)
        self.plot_observed_landmarks(robot)





if __name__ == "__main__":

    map = Map()
    control_list = [[0.0,0.0,1.0],
                    [1.0,0.0,1.0],
                    [1.0,pi/2,1.0],
                    [1.0,0.0,1.0],
                    [1.0,-pi/2,1.0]]

    robot = Robot([0.0,0.0,0.0],map)
    map.move_in_list(robot,control_list)
