#!/usr/bin/env python3

import rospy 
from typing import Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import time
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import cv2 
import cv_bridge
import sys
sys.path.append("./Turtle-C3BF")
sys.path.append("./Turtle-C3BF/bots")
sys.path.append("./Turtle-C3BF/controllers")
import matplotlib.pyplot as plt
from matplotlib import animation

t = []
x = []
y = []
obs_x = []
obs_y = []

fig, ax = plt.subplots()
ax.set_xlim(-1, 6)
ax.set_ylim(0, 4)
agend_line = ax.plot([], [], 'r', markersize=200)[0]
obs_line = ax.plot([], [], 'g', markersize=200)[0]


def live_plot(data) -> None:

    with open("./run_no.txt", "r") as f:
            l = f.readline()
            run_no = int(l)-1
    with open('./log/cbf{}.txt'.format(run_no), 'r') as file:
        lines = file.readlines()
        line = lines[-1].split()
        vars = list(map(float, line))

        t.append(vars[0])
        x.append(vars[1])
        y.append(vars[2])
        obs_x.append(vars[12])
        obs_y.append(vars[13])
        agend_line.set_data(x, y)
        obs_line.set_data(obs_x, obs_y)

        return agend_line, obs_line





def cb(data):
    x.append(data.pose.pose.position.x)
    y.append(data.pose.pose.position.y)


def live_plot2(data):
    global x_line
    global x, y
    print("updating")
    x_line.set_data(x, y)
    # plt.draw()
    # plt.pause(0.1)

    return x_line,



if __name__=="__main__":
    # log_reader = LogReader("./runs/log496.txt")
    # live_plot(log_reader)
    
    # rospy.init_node("temp0")
    # s = rospy.Subscriber("tb3_1/odom", Odometry, cb)
    
    ani = animation.FuncAnimation(fig, live_plot,
                               frames=None,
                               interval=50)
    plt.show(block = True)
   
    



