import matplotlib.pyplot as plt
from matplotlib import patches
from math import *
import numpy as np
import random
import time
from robot import *
from ekf import *
plt.ion()

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
                plt.clf()
                dt = control[2]/5
                robot.move([control[0],control[1],dt])
                robot.kalman_predict([control[0],control[1],dt])
                self.show_map(robot)
                plt.show()
                plt.pause(0.01)

        raw_input("press any key to close...")


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

        plt.gca().add_patch(plt.Circle((x,y),radius=0.1,fc='b'))
        x = state[0]
        y = state[1]
        theta = state[2]
        x1 = x + (0.1 * cos(theta))
        y1 = y + (0.1 * sin(theta))
        plt.plot([x,x1],[y,y1],color='r')

    def plot_observed_landmarks(self,robot):
        #plot all observed landmarks
        for obs_landmark in robot.landmarks:
            x = robot.state[0]
            y = robot.state[1]
            theta = robot.state[2]
            [r,alpha] = obs_landmark

            x1 = x + (r * cos(theta+alpha))
            y1 = y + (r * sin(theta+alpha))

            plt.plot(x1,y1,'go')
        plt.axis([-1,self.size, -1,self.size])
    def draw_elipse(self, robot):
        #get the error elipse for kalman filters
        [theta,val2,val1] = robot.ekf.get_error_ellipse(robot.ekf.covariance)

        pose = robot.odometry
        theta = (theta + pose[2])*180/pi
        #show robot final pose as a circle
        plt.gca().add_patch(patches.Ellipse((pose[0], pose[1]), val1, val2,
                                angle=theta, linewidth=1, fill=False, zorder=2))

    def plot_odometry(self,robot):
        #plot all robot states as lines
        for pose in robot.odometry_list:
            theta = pose[2]
            x1 = pose[0] + (0.1 * cos(theta))
            y1 = pose[1] + (0.1 * sin(theta))
            plt.plot([pose[0],x1],[pose[1],y1],color='b')

        self.draw_elipse(robot)
        #show robot final pose as a circle
        plt.gca().add_patch(plt.Circle((pose[0],pose[1]),radius=0.1,fc='r'))

    def show_map(self,robot):
        self.plot_global_landmarks()
        self.plot_robot_path(robot)
        self.plot_observed_landmarks(robot)
        self.plot_odometry(robot)
