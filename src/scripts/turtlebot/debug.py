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
sys.path.append("./Turtle-C3BF")
sys.path.append("./Turtle-C3BF/bots")
sys.path.append("./Turtle-C3BF/controllers")


from bots.AC_unicycle import Unicycle
from controllers.QP_controller_unicycle import QP_Controller_Unicycle
from tf2_msgs.msg import TFMessage


# u_ref = np.array([-0.06034186 ,0.02669465])
abs_x, abs_y, obs_v_x, obs_v_y = [3, 0.3, -0,0]


Kp1 = 0.08
Kd1 = 0.3 
Ki1 = 0.0

t0 = time.time()

Kp2 = 0.5
Kd2 = 0.5
Ki2 = 0

goaly = 0 
theta = 0.5 
x_pos, y_pos, theta_pos, v_pos, w_pos = 1.4454858280513805, 0.31125610563815875, 0.24828269905947484, 0.22524242704520334, -0.012202423549507442




x_pos, y_pos, theta_pos, v_pos, w_pos  = 0.048177285121411305 , 0.006560170011079643 , -0.05883071219849109 , -9.082720249645505e-05 , -0.019835089965899857

x_neg, y_neg, theta_neg, v_neg, w_neg = 1.4454858280513805, -0.31125610563815875,-0.24828269905947484, 0.22524242704520334, 0.012202423549507442

v_des = 0.3

prev_vel_error = 0
prev_heading_error = 0 

dt = 0.05 

# def pid():

#     global goaly
#     global y 
#     global prev_heading_error
#     global prev_vel_error

#     delta_theta = (np.arctan2((goaly - y), 0.9)) - theta
#     #Error is delta_theta in degrees
#     e_new = delta_theta
    
#     e_dot = (e_new - prev_heading_error)/dt 
#     alpha = (Kp1*e_new) + (Kd1*e_dot)
#     prev_heading_error = e_new 
#     e_v = (v_des- v ) 
#     e_vdot = (e_v - prev_vel_error) / dt
#     prev_vel_error = e_v 
#     a = Kp2 * e_v  + Kd2 * (e_vdot) 
#     print("a, alpha", a, alpha) 

#     return np.array([a, alpha])

# u_ref = pid()

# u_ref_pos = np.array([-0.05644145091455314, -0.05464389913787823])
u_ref_pos = np.array([0.05506001, 0.0080606])
u_ref_neg = np.array([-0.05644145091455314,  0.05464389913787823])



qp = QP_Controller_Unicycle(1, obs_radius= 0.3)

bot1_config_file_path = './Turtle-C3BF/bots/bot_config/bot1.json'
bot_pos = Unicycle.from_JSON(bot1_config_file_path)
bot_neg = Unicycle.from_JSON(bot1_config_file_path)
bot_pos.x, bot_pos.y, bot_pos.theta, bot_pos.v, bot_pos.w = x_pos, y_pos, theta_pos, v_pos, w_pos
bot_neg.x, bot_neg.y, bot_neg.theta, bot_neg.v, bot_neg.w = x_neg, y_neg, theta_neg, v_neg, w_neg


# Doing for positive 
print("   ")
print("--------------------------------------")
print("Positive")
qp.set_reference_control(u_ref_pos)
qp.setup_QP(bot_pos, [abs_x, abs_y], [obs_v_x, obs_v_y]) #  
h  = qp.solve_QP(bot_pos)
u_star = qp.get_optimal_control() 
a, alpha = u_star

if u_star[0] != u_ref_pos[0] or u_star[1] != u_ref_pos[1]:
    active = 1
    
    print("CBF ACTIVE!!!")

print(f"current velocity: {bot_pos.v} angular: {bot_pos.w}")
# Printing bot params 
print("cbf inputs ", [abs_x, abs_y], [obs_v_x, obs_v_y])
print(f"botx : {bot_pos.x} boty: {bot_pos.y}, bot v: {bot_pos.v}")
print(f"bot theta: {bot_pos.theta  } bot w: {bot_pos.w}")

print("reference ", u_ref_pos)
print("cbf", u_star)

print("--------------------------")
print("  ")

# Doing for Negative 
print("   ")
print("--------------------------------------")
print("Negative")
qp.set_reference_control(u_ref_neg)
qp.setup_QP(bot_neg, [abs_x, abs_y], [obs_v_x, obs_v_y]) #  
h  = qp.solve_QP(bot_neg)
u_star = qp.get_optimal_control() 
a, alpha = u_star





if u_star[0] != u_ref_neg[0] or u_star[1] != u_ref_neg[1]:
    active = 1
    
    print("CBF ACTIVE!!!")

# print(f"current velocity: {bot_neg.v} angular: {bot_neg.w}")
# # Printing bot params 
# print("cbf inputs ", [abs_x, abs_y], [obs_v_x, obs_v_y])
# print(f"botx : {bot_neg.x} boty: {bot_neg.y}, bot v: {bot_neg.v}")
# print(f"bot theta: {bot_neg.theta  } bot w: {bot_neg.w}")
# print("reference ", u_ref_neg)
# print("cbf", u_star)

# print("--------------------------")
# print("  ")

