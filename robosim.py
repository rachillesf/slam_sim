import numpy as np
import random
from math import *
import matplotlib.pyplot as plt

#TODO:
# add noise to  measurement



class Robot:
    # simulate a robot model that can sense landmarks and move
    # state is represented in form of (x(m),y(m),theta(rad))
    # control is given by (linear_vel(m/s), ang_vel(rad/s), time_var(s))
    # the sensor are able to locate landmarks in a range given by the "sensor_maxrange"
    # and "sensor_opening"

    # init:
    #    creates robot and initializes location/orientation
    def __init__(self,init_state,map):
        self.USE_NOISE = True
        self.state = [0.0,0.0,0.0]
        self.states_list = [init_state] #store all visited states
        self.sensor_opening = 3*pi/4
        self.sensor_maxrange = 3
        self.landmarks = []  #current visible landmarks
        self.landmarks_list = [] #store all landmarks observed
        self.map = map


    # set_state:
    #       set the robot to a desired state
    def set_state(self,state):
        #set state
        self.state = state


    # move
    #       execute a control instructios in the form of (v,w,dt)
    def move(self,control):

        #member variable of control with error
        if(self.USE_NOISE):
            v = control[0]  + 10e-3 + 0.042*control[0] + 0.065*control[1]
            w = control[1]  + 10e-3 + 0.06*control[0] + 0.032*control[1]
        else:
            v = control[0]  + 10e-3
            w = control[1]  + 10e-3

        delta_t = control[2]
        for time_step in range(5):
            dt = delta_t/5.0

            #member variables for state
            x = self.state[0]
            y = self.state[1]
            theta = self.state[2]

            #state update equations
            new_x = x - (v/w)*sin(theta) + (v/w) * sin(theta + (w*dt))
            new_y = y + (v/w)*cos(theta) - (v/w) * cos(theta + (w*dt))
            new_theta = (theta + w*dt) % (2*pi)

            #update state and append it to states list
            self.state =[float("{0:.4f}".format(new_x)),
                        float("{0:.4f}".format(new_y)),
                        float("{0:.4f}".format(new_theta))]

            self.states_list.append(self.state)

        self.search_landmarks(self.map.global_landmark_list)



    #move_in_list
    #   execute a list of control instructions in order
    def move_in_list(self,control_list):
        for control in control_list:
            self.move(control)


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
                landmark[0] += 0.2*r
                landmark[1] += 0.2*r
                landmarks.append(landmark)

        self.landmarks = landmarks
        self.landmarks_list.append(landmarks)
        print "visible_landmarks"
        print self.landmarks


    #__repr__
    #   allows printing the robot state using print function
    def __repr__(self):
        return '[x=%s y=%s theta=%s]' % (str(self.state[0]), str(self.state[1]),
                                                str(self.state[2]))


class Map:
    def __init__(self):
        self.size = 6 #assuming square map
        self.global_landmark_list = [[1.0,2.0],
                                     [3.0,3.5],
                                     [4.0,5.0],
                                     [5.5,3.0]]
        self.IDEAL = False

    def show_map(self,robot):

        #if is plotting the ideal robot only show the states
        if self.IDEAL:
            #plot all robot states as lines
            for state in robot.states_list:
                x = state[0]
                y = state[1]
                theta = state[2]
                x1 = x + (0.1 * cos(theta))
                y1 = y + (0.1 * sin(theta))
                plt.plot([x,x1],[y,y1],color='r')
            #show robot final pose as a circle
            plt.gca().add_patch(plt.Circle((x,y),radius=0.1,fc='r'))


        else:
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

            #plot all landmarks
            for landmark in self.global_landmark_list:
                plt.plot(landmark[0],landmark[1],'ro')

            #plot all observed landmarks
            for obs_landmark in robot.landmarks_list[-1]:
                plt.plot(obs_landmark[0],obs_landmark[1],'go')


        plt.axis([-1,self.size, -1,self.size])




if __name__ == "__main__":

    map = Map()
    control_list = [[0.0,0.0,1.0],
                    [1.0,0.0,1.0],
                    [1.0,pi/2,1.0],
                    [1.0,0.0,1.0],
                    [1.0,-pi/2,1.0]]

    robot = Robot([0.0,0.0,0.0],map)
    robot.move_in_list(control_list)
    map.show_map(robot)

    map2 = Map()
    map2.IDEAL = True
    robot2 = Robot([0.0,0.0,0.0],map)
    robot2.USE_NOISE = False
    robot2.move_in_list(control_list)
    map2.show_map(robot2)

    plt.show()
