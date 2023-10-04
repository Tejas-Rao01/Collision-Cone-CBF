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
from phasespace_msgs.msg import Markers

sys.path.append('/home/focaslab/Downloads/F1-C3BF')
sys.path.append('/home/focaslab/Downloads/F1-C3BF/controllers')
sys.path.append('/home/focaslab/Downloads/F1-C3BF/bots')

from bots.Bicycle import Bicycle
from controllers.QP_controller_bicycle import QP_Controller_Bicycle
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16, Float32, Bool

from ackermann_msgs.msg import AckermannDrive
# Todo, simple pid controller that takes robots position and 
# Moves it along a straight path 

class Controller():

    def __init__(self) -> None:
        # rospy.init_node("pid_controller")
        print()
        print("controller initialised !!!!")
        print()
        self.t_prev_odom = None 

        self.x_prev = None 
        self.y_prev = None 
        self.dir = 1 
        self.odom_sub = rospy.Subscriber('/phasespace/markers', Markers , self.odom_callback)
        self.obstacle_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size =1)
        # self.obs_sub = rospy.Subscriber('/tb3_1/odom', Odometry, self.obstacle_sub)
        self.steer_pub = rospy.Publisher("/steering", Float32,queue_size=1)
        self.pwm_pub = rospy.Publisher("/pwm_value", Int16,queue_size=1)
        self.dir_pub = rospy.Publisher("/dir", Bool,queue_size=1)
        # self.pose_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        # self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.x = 0
        self.y = 0
        self.theta = 0 
        self.theta_list = []

        self.t_prev_odom = rospy.get_time()

        self.goalx, self.goaly = 0, 0
        self.curr_goal = 1  
        self.goal_thresh = 0.2

        #PID params
        self.k = 0.4  
        self.ks = 0.4
        self.Kp2 = 0.7
        self.Kd2 = 0.5 
        self.Ki1 = 0.0
        
        self.obs_x, self.obs_y, self.obs_v_x, self.obs_v_y = 0,0,0,0

        self.v = 0
        self.w = 0
        self.prev_vel_error = 0
        self.steer  = 0 
        self.max_steer = 0.37   

        self.abs_x, self.abs_y, self.abs_theta = None, None, None
       
        self.v_obsx = -0.02
        self.v_obsy = 0 
        self.qp = QP_Controller_Bicycle(1, obs_radius= 0.15)
        self.t_prev = time.time()
        
        self.v_des = 3
        self.v_target = 3
        self.w_target = 0
        self.return_to_origin = False
        
        
        self.pwm = 40
        bot1_config_file_path = '/home/focaslab/Downloads/F1-C3BF/bots/bot_config/bot1.json'
        self.bot = Bicycle.from_JSON(bot1_config_file_path)

        
        with open("/home/focaslab/Downloads/run_no.txt", "r") as f:
            l = f.readline()
            self.no = int(l)
        open("/home/focaslab/Downloads/run_no.txt", "w").close()
        # self.no = 0
        with open("/home/focaslab/Downloads/run_no.txt", "w") as f:
            f.write(str(self.no + 1))
        
        self.file = f"/home/focaslab/Documents/log{self.no}.txt"
        # open(self.file, "w").close()
        self.file_path = f"/home/focaslab/Documents/obstaclepos{self.no}.txt"
        # open(self.file_path, "w").close()

        self.file_path2 = f"/home/focaslab/Documents/obstaclevel{self.no}.txt"
        # open(self.file_path2, "w").close()
        
        self.file_path3 = f"/home/focaslab/Documents/robotpos{self.no}.txt"
        # open(self.file_path3, "w").close()
        self.file_path4 = f"/home/focaslab/Documents/gt{self.no}.txt"
        self.dt_odom = 0.1


        #Internal variables to store the obstacle pos
        self.internal_obs_x = 1.5
        self.internal_obs_y = 0.0
        self.internal_obs_velx = 0
        self.internal_obs_vely = 0 
        # self.obs_offsetx, self.obs_offsety = 2.5, 0
        
    
    def pid(self):

        t_curr = time.time()
        self.dt = t_curr - self.t_prev 
        if self.dt == 0:
            self.dt += 0.2
        # print("dt:",self.dt)
        self.t_prev = t_curr 

        # Trajectory heading 
        psi = 0 - self.theta
        crosstrack = np.arctan2(self.k * (1.55- self.y)    , (self.ks + self.v) ) 
        delta = psi + crosstrack 
        # print("crosstrack", crosstrack)
        # print("psi", psi)
        # print("delta", delta)
        # print("selfvxvy", self.vx_self, self.vy_self)
        e_v = (self.v_des- self.v ) 
        # print("e_v", e_v)
        # print("self.v", self.v)
        e_vdot = (e_v - self.prev_vel_error) / self.dt
        self.prev_vel_error = e_v 
        # print(e_v, e_vdot)
        # print("e-v * kp", self.Kp2 * e_v)
        a = self.Kp2 * e_v  + self.Kd2 * (e_vdot) 
        # print("a, alpha", a, alpha) 
    
        return delta, a

    def kill(self):
        msg = Int16()
        msg.data = 0
        self.pwm_pub.publish(msg)

    def publish(self, obs_x, obs_y, obs_v_x, obs_v_y, tick):
        

        t = time.time()
        with open(self.file_path, 'a') as f:
            s = " ".join(str(i) for i in [t,obs_x, obs_y])
            s += '\n'
            f.write(s)
            f.close()

        with open(self.file_path2, 'a') as f:
            s = " ".join(str(i) for i in [t, obs_v_x, obs_v_y])
            s += '\n'
            f.write(s)
            f.close()
        # print()
        # print("--------------------")
        self.abs_x = obs_x 
        self.abs_y = obs_y
        self.obs_v_x = obs_v_x
        self.obs_v_y = obs_v_y 

        steer_angle, a= self.pid()
        # print("controller steer_angle", steer_angle)
        steer_angle = min(self.max_steer, max(-self.max_steer, steer_angle))
        # print("robotx, roboty, robott", self.x, self.y, self.theta)
        # print("robto vel",self.v)
        
        u_ref = np.array([ a, steer_angle])
        active = 0
        detection = 0
        value_of_h = 0
        if self.abs_x != None  and self.abs_y != None :
            
            self.qp.set_reference_control(u_ref)
            self.qp.setup_QP(self.bot, [self.abs_x, self.abs_y], [self.obs_v_x, self.obs_v_y]) #  
            
            
            # print("vrel ", self.vobs_x, self.vrely)
            
            value_of_h, _ = self.qp.solve_QP(self.bot)
            
            # # Bot Kinematics
            u_star = self.qp.get_optimal_control() 
            a, steer_angle = u_star

            if u_star[0] != u_ref[0] or u_star[1] != u_ref[1]:

                active = 1
                pass
            #     print("   ")
            #     print("--------------------------------------")
            #     print("CBF ACTIVE!!!")


            # print("curr x and y ", self.x, self.y)
            # # print("rel ", self.obs_x, self.rely)
            # # Simulation
            # # Solve QP

            # print(f"current velocity: {self.v} angular: {self.w}")
            # # Printing bot params 
            # print("cbf inputs ", [self.abs_x, self.abs_y], [self.v_obsx, self.v_obsy])

            # print(f"botx : {self.bot.x} boty: {self.bot.y}, bot v: {self.bot.v}")
            # # print(f"bot theta: {self.bot.theta  } bot w: {self.bot.w}")
            # print("reference ", u_ref)
            # print("cbf", u_star)

            # print("--------------------------")
            # print("  ")

        # print("updating bot state", np.array([self.x, self.y]), self.theta, self.v, self.dt )
        self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.dt )
        steer_angle = min(self.max_steer, max(-self.max_steer, steer_angle))
        # print("steer, a", steer_angle ,a)
        prev_target_v = self.v_target
        # print("self.dt", self.dt)
        self.v_target = a * self.dt   / 0.071+ prev_target_v
        # print("odl w target", prev_target_v)
        # print("new w target", self.v_target)
        self.v_target = min(0.5, max(0.1, self.v_target))

        pub_v = int(self.pwm_map(self.v_target * 0.071))
        print("v_target", self.v_target * 0.071)
        pwm_msg= Int16()

        if self.x > 4:
            pub_v = 0
        pwm_msg.data =0# pub_v
        self.pwm_pub.publish(pwm_msg)

        # steer_angle = np.arctan(2*np.tan(steer_angle))
        steer_angle = self.steer_map(steer_angle)
        steer_msg = Float32()
        steer_msg.data = steer_angle
        self.steer_pub.publish(steer_msg)

        # print("pwm", pub_v)
        # print("servo angle", steer_angle)

        if self.v_target < 0:
            self.dir = -1
        else:
            self.dir = 1

        bool_msg = Bool()
        if self.dir == 1:
            bool_msg.data = 0
        else:
            bool_msg.data = 1  
        self.dir_pub.publish(bool_msg)

        cmd_vel = Twist()
        cmd_vel.linear.x = -0.0    
        cmd_vel.angular.z = -0.0




        
        self.obstacle_pub.publish(cmd_vel)



    def steer_map(self, steer_angle):
        # servo = -20.853 * (steer_angle) **2 + 121.966 * steer_angle + 46.92
        servo = -123.021 * steer_angle +45
        return servo
    
    def pwm_map(self, vel):
        kp = 9
        # vel = 0.2 
        e = vel -self.v
        self.pwm = (e * kp) + self.pwm

        self.pwm = max(50, min(self.pwm, 100))
        # pwm = int(188.137 *X**2 +47.86* X +33.98)
        return self.pwm 
    


    def odom_callback(self, data):
        # print("pid odom callback")

        if not self.x_prev:
            self.x_prev = self.x 

        if not self.y_prev:
            self.y_prev = self.y

        t = time.time()
        dt = t- self.t_prev_odom 
        if dt > 0.1:
            dx = self.x - self.x_prev 
            dy = self.y - self.y_prev 
            vx = dx / dt 
            vy = dy / dt 
            self.x_prev, self.y_prev = self.x , self.y
            self.v = np.sqrt(vx **2 + vy **2 ) * self.dir
            self.t_prev_odom = t        

        x1 = data.markers[0].x
        y1 = data.markers[0].y

        x2 = data.markers[1].x
        y2 = data.markers[1].y
        

        self.x = x1* 0.5 + x2 *0.5 
        self.y = (y1 + y2)*0.5
        
        # print("xnew xold", self.x, self.x_prev)
        # print("dt", dt)

        theta = np.arctan2((y2 - y1), (x2 -x1)) 
        # print(theta)
        self.theta_list.append(theta)
        # print(self.theta_list)
        theta = np.unwrap(self.theta_list)[-1]
        self.theta_list[-1] = theta
        self.theta_list = self.theta_list[-5:-1]
        self.theta = theta

        
    def run(self):
        rate = rospy.Rate(10)
        tick = 0
        while not rospy.is_shutdown():
            self.publish(2,1.65,0,0, tick)
            # print("publishing")
            # if tick%10 ==0:
                # print("_-------------")
                # print(f"camera: relx: {self.relx}, rely: {self.rely}, vrelx {self.vrelx}, vrely: {self.vrely}")
                # print(f"camera: relx: {self.relx_est}, rely: {self.rely_est}, vrelx {self.vrelx_est}, vrely: {self.vrely_est}")
                # print("---------------")
            rate.sleep()    

            tick += 1 
            if tick == 10:
                tick = 0
if __name__ == "__main__":
    
    rospy.init_node("pid")
    pid = Controller()
    try:
        pid.run()
    except KeyboardInterrupt:
        print('bye')
        pid.kill()

    



