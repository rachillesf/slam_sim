import matplotlib.pyplot as plt
from math import *
import numpy as np
import random
import time
from robot import *
from map import *

plt.ion()


if __name__ == '__main__':

    control_list = [[0.0,0.0,1.0],
                    [0.0,0.0,1.0],
                [0.0,0.0,1.0],
                [1.0,0.0,1.0],
                [1.0,pi/9,1.0],
                [1.0,pi/9,1.0],
                [1.0,pi/9,1.0],
                [1.0,0.0,1.0],
                [1.0,pi/3,1.0],
                [1.0,0.0,1.0],
                [1,pi/4,1.0],
                [1,pi/4,1.0],
                [1,pi/4,1.0],
                [1,pi/6,1.0],
                [1,0.0,1.0],
                [1.0,-pi/4,1.0],
                [1.0,pi/5,1.0],
                [1.0,0.0,0.8]]

    map = Map()
    robot = Robot([0.0,0.0,0.0],map)
    #robot.ekf.add_landmark_to_state((1.0, 3.0))
    map.move_in_list(robot,control_list)
