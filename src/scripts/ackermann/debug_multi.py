#!/usr/bin/env python3

import rospy
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
sys.path.append('/home/rao/Downloads/F1-C3BF')
sys.path.append('/home/rao/Downloads/F1-C3BF/controllers')
sys.path.append('/home/rao/Downloads/F1-C3BF/bots')
from bots.Bicycle import Bicycle
from controllers.QP_controller_bicycle_multi_num import QP_Controller_Bicycle
from gazebo_msgs.msg import ModelStates 
from tf2_msgs.msg import TFMessage
from pathlib import Path
from ackermann_msgs.msg import AckermannDrive
# Todo, simple pid controller that takes robots position and 
# Moves it along a straight path 


# Setup the QP for cbf 
qp = QP_Controller_Bicycle(1, obs_radius= 0.3)


# define the virtual bot for cbf computation 
bot1_config_file_path = '/home/rao/Downloads/F1-C3BF/bots/bot_config/bot1.json'
bot = Bicycle.from_JSON(bot1_config_file_path)


x,y = 0.2,0
v = 0.200000000004007215170614283263
theta  = -0.000000000001000689975138472679

w = 0.05

bot.update_state(np.array([x, y]), theta, v, w)
# [obs_x, obs_y], [obs_v_x, obs_v_y] = [3.9964726350047917, 0.005380904373468807], [-1.2760163448743201e-05, 4.365235173108283e-05]
[obs_x1, obs_y1], [obs_v_x1, obs_v_y1] = [4,0.3] ,[0,0]
[obs_x2, obs_y2], [obs_v_x2, obs_v_y2] = [4,-0.0] ,[0,0]

u_ref = np.array([2.14963766 ,0.00488573])
t1 = time.time()

qp.set_reference_control(u_ref)

qp.setup_QP(bot, [obs_x1, obs_y1], [obs_v_x1, obs_v_y1], [obs_x2, obs_y2], [obs_v_x2, obs_v_y2]) #  
t2 = time.time()


h = qp.solve_QP(bot)

print("inferece time", t2-t1)
# # # Bot Kinematics
# u_star = qp.get_optimal_control() 
# a,steer_angle = u_star
# # print(u_star)
# if u_star[0] != u_ref[0] or u_star[1] != u_ref[1 ]:
#     steer_angle = steer_angle
    
#     print("   ")
#     print("--------------------------------------")
#     print("CBF ACTIVE!!!")

# print(f"value of h: {h}")
# print("curr x and y ", x, y)
# # Simulation
# # Solve QP

# print(f"current velocity: {v} angular: {w}")
# # Printing bot params 
# print("cbf inputs ", [obs_x, obs_y], [obs_v_x, obs_v_y])

# print(f"botx : {bot.x} boty: {bot.y}, bot v: {bot.v}")
# # print(f"bot theta: {bot.theta  } bot w: {bot.w}")
# print("reference ", u_ref)
# print("cbf", u_star)
# print('botl', bot.L)
# print("--------------------------")
# print("  ")



