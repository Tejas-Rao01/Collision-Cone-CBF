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
from phasespace_msgs.msg import Markers
from bots.AC_unicycle import Unicycle
from controllers.QP_controller_unicycle import QP_Controller_Unicycle
from tf2_msgs.msg import TFMessage


# Todo, simple pid controller that takes robots position and 
# Moves it along a straight path 

class Controller():

    def __init__(self) -> None:
        
        print("initializing")
        
        self.init_vars()
        self.init_qp()
        self.reset_files()
        self.init_pubsub()

        
    def init_vars(self):
        self.t_prev = time.time()
        self.t0 = time.time()
        self.t_odom_prev = time.time()

        self.x = 0 
        self.y = 1.2        
        self.theta = 0 
        self.v = 0
        self.w = 0
        self.theta_list = []

        self.goalx, self.goaly = 0, 1.2
        
        #PID params

        self.Kp1 = 0.1
        self.Kd1 = 0.4
        self.Ki1 = 0.0

        self.Kp2 = 2
        self.Kd2 = 0.4
        self.Ki2 = 0

        self.v_des = 0.15
        
        self.prev_heading_error =0  
        self.prev_vel_error = 0
        self.prev_heading_error_obs =0  
        self.prev_vel_error_obs = 0

        self.obs_x = -30 
        self.obs_y = -30
        self.obs_v_x = 0
        self.obs_v_y = 0
        self.obs_v = 0 

        self.v_target = 0.15
        self.w_target = 0
        

    def init_pubsub(self):
        self.odom_sub = rospy.Subscriber('/phasespace/markers', Markers , self.odom_callback)
        self.control_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.obs_sub = rospy.Subscriber('/odom', Odometry, self.obstacle_sub)
    
        self.agent_sub = rospy.Subscriber('/tb3_1/odom', Odometry, self.agent_sub)

    def reset_files(self):
        with open("./run_no.txt", "r") as f:
            l = f.readline()
            self.no = int(l)
        open("./run_no.txt", "w").close()
        # self.no = 0
        with open("./run_no.txt", "w") as f:
            f.write(str(self.no + 1))
        
        self.file_path = f"./runs/cbf{self.no}.txt"


    def init_qp(self):
        self.qp = QP_Controller_Unicycle(1, obs_radius= 0.15)
        bot1_config_file_path = './Turtle-C3BF/bots/bot_config/bot1.json'
        self.bot = Unicycle.from_JSON(bot1_config_file_path)

    def pid(self) -> Tuple[float, float]:
        """
        Computes the alpha and a values using PID control.

        Returns:
            Tuple[float, float]: The computed alpha and a values.
        """
        self.dt = time.time() - self.t_prev 
        self.t_prev = time.time()

        delta_theta = np.arctan2(self.goaly - self.y, 0.9) - self.theta
        e_new = delta_theta
        e_dot = (e_new - self.prev_heading_error) / self.dt
        alpha = self.Kp1 * e_new + self.Kd1 * e_dot
        self.prev_heading_error = e_new

        e_v = self.v_des - self.v
        e_vdot = (e_v - self.prev_vel_error) / self.dt
        self.prev_vel_error = e_v
        a = self.Kp2 * e_v + self.Kd2 * e_vdot

        return a, alpha

    def publish(self) -> None:
        t = time.time()
        a, alpha = self.pid()
        
        u_ref = np.array([a, alpha])
        self.u_ref = u_ref
        a , alpha = np.array([a]), np.array([alpha])
        active = 0
        self.qp.set_reference_control(u_ref)
        self.qp.setup_QP(self.bot) #  
        
        H  = self.qp.solve_QP(self.bot,  [self.obs_x, self.obs_y], [self.obs_v_x, self.obs_v_y])
        # # Bot Kinematics
        u_star = self.qp.get_optimal_control() 
        self.u_star = u_star
        a, alpha = u_star

        if abs(u_star[0] - u_ref[0]) < 0.01 or abs(u_star[1] != u_ref[1]) < 0.01:
            active = 1

        self.print_vars(active)
        
        self.v_target =  self.v_target + a * self.dt 
        self.w_target =  self.w_target + alpha * self.dt 

        self.v_target = min(0.15, max(-0.15, self.v_target))
        self.w_target = min(1, max(-1, self.w_target))
        
        cmd_vel = Twist()
        cmd_vel.linear.x =  self.v_target
        cmd_vel.angular.z= self.w_target
        self.control_pub.publish(cmd_vel)

        if type(self.v_target) == float or type(self.v_target) == int:
            self.v_target = np.array([self.v_target])
        
        if type(self.w_target) == float or type(self.w_target) == int:
            self.w_target = np.array([self.w_target])
        vars = [t, self.bot.x, self.bot.y, self.bot.theta, self.bot.v, self.bot.w, u_ref[0], u_ref[1], u_star[0][0], u_star[1][0], self.v_target[0], self.w_target[0], self.obs_x, self.obs_y, self.obs_v_x , self.obs_v_y]
        self.log(vars)

    def log(self, vars):

        
        with open(self.file_path, "a") as f:
            s = " ".join(str(i) for i in vars)
            s += '\n'
            f.write(s)
            f.close()
        
    def print_vars(self, active):
        if type(self.v_target) == float:
            self.v_target = np.array([self.v_target])
    
        if type(self.w_target) == float:
            self.w_target = np.array([self.w_target])

        print("   ")
        print("--------------------------------------")
        if active == 1 : print("CBF ACTIVE!!!")
        print("curr x and y ", self.x, self.y)
        print(f"current velocity: {self.v} angular: {self.w}")
        # Printing bot params 
        print("cbf inputs ", [self.obs_x, self.obs_y], [self.obs_v_x, self.obs_v_y])
        print(f"botx : {self.bot.x} boty: {self.bot.y}, bot v: {self.bot.v}")
        print(f"bot theta: {self.bot.theta  } bot w: {self.bot.w}")
        print("reference ", self.u_ref)
        print("cbf", self.u_star)
        print("params", self.bot.x, ",",self.bot.y,",", self.bot.theta, ",",self.bot.v, ",", self.bot.w)
        print("--------------------------")
        print("  ")



    def obstacle_sub(self, data):
        print("obstacle callback")
        v = data.twist.twist.linear.x
        self.obs_v = v
        print("v ", v)

    def agent_sub(self, data):
        print("agent callback")
        v = data.twist.twist.linear.x
        
        self.bot.v = v
        self.v = v 
        self.bot.w = data.twist.twist.angular.z
        self.w = data.twist.twist.angular.z



    def odom_callback(self, data):
                # print("pid odom callback")
        

        if len(data.markers) == 0:
            print("no markers!!!!")
        t_odom = time.time()

        t = time.time()
        dt = t- self.t_odom_prev
        if dt > 0.1:
            self.t_odom_prev = t_odom
            x1 = data.markers[1].x
            y1 = data.markers[1].y

            x2 = data.markers[0].x
            y2 = data.markers[0].y
            
            x3 = data.markers[2].x
            y3 = data.markers[2].y

            x4 = data.markers[3].x
            y4 = data.markers[3].y

            if 0 in [x1,x2,x4,y1,y2,y4,x3,y3]:
                
                print()
                print()
                print("----------------")
                print("Mocap not working !!!")
                print("----------------")
                print()
                
                return

            x = x1* 0.5 + x2 *0.5 
            y = (y1 + y2)*0.5

            self.obs_v_x = ((x3 + x4) * 0.5 - self.obs_x) /dt 
            self.obs_v_y = ((y3 + y4)*0.5 - self.obs_y) /dt


            self.obs_x = (x3 + x4) * 0.5 
            self.obs_y = (y3 + y4)*0.5

            theta = np.arctan2((y2 - y), (x2 - x))
            self.obs_theta = np.arctan2((y4 - (y4+y3)/2), (x4 - (x3+x4)/2))


            theta1 = np.arctan2((y4 - (y4+y3)/2), (x4 - (x3+x4)/2))
            print("theta 1", theta1)



            # self.obs_v_x = self.obs_v * np.cos(theta1)
            # self.obs_v_y = self.obs_v * np.sin(theta1)

            print("obs v x ", self.obs_v_x, " obs v y ", self.obs_v_y)


            self.x = x 
            self.y = y 
            self.theta = theta 

            self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.w,   dt )

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()   


if __name__ == "__main__":
    rospy.init_node("pid")
    print("Initialised PID controller")
    pid = Controller()

    pid.run()


