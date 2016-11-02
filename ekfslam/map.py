import matplotlib.pyplot as plt
from matplotlib import patches
from math import *
import numpy as np
import random
import time
from robot import *
from ekfslam import *
plt.ion()

class Map:
    def __init__(self):
        self.size = 8 #assuming rectangular map
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

    def plot_ekfstate_landmarks(self,robot):
        lm = robot.ekf.get_landmarks()
        ellipses = robot.ekf.get_landmark_error_ellipses()

        for landmark in lm:
            plt.plot(landmark[0], landmark[1],'go')

        for i in range(len(lm)):
            self.draw_elipse([lm[i][0],lm[i][1],0],ellipses[i])
        plt.axis([-1,self.size, -1,self.size])

    def draw_elipse(self, pose, elipse):

        [theta,val2,val1] = elipse

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

        robot_elipse = robot.ekf.get_error_ellipse(robot.ekf.covariance)
        self.draw_elipse(robot.odometry,robot_elipse)
        #show robot final pose as a circle
        plt.gca().add_patch(plt.Circle((pose[0],pose[1]),radius=0.1,fc='r'))

    def show_map(self,robot):
        self.plot_global_landmarks()
        self.plot_robot_path(robot)
        self.plot_ekfstate_landmarks(robot)
        self.plot_odometry(robot)
