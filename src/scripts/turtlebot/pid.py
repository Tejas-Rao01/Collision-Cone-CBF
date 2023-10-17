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


# Todo, simple pid controller that takes robots position and 
# Moves it along a straight path 

class Controller():

    def __init__(self) -> None:
        

        print("blah 6")
        self.pose_sub = rospy.Subscriber('/tb3_0/odom', Odometry, self.odom_callback)
        self.control_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size =1)
        # self.obs_sub = rospy.Subscriber('/tb3_1/odom', Odometry, self.obstacle_sub)
        # self.pose_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.x = 0
        self.y = 0
        self.theta = 0 
        self.theta_list = []

        print("blah 7")
        self.goalx, self.goaly = 0, 0
        self.curr_goal = 1  
        self.goal_thresh = 0.2

        #PID params

        self.Kp1 = 0.08
        self.Kd1 = 0.3 
        self.Ki1 = 0.0
        
        # self.Kp1 = 0.05   
        # self.Kd1 = 0.4  
        # self.Ki1 = 0.0
        
        # self.Kp2 = 0.06
        # self.Kd2 = 0.015
        # self.Ki2 = 0

        self.t0 = time.time()

        self.Kp2 = 0.5
        self.Kd2 = 0.5
        self.Ki2 = 0

        self.v = 0
        self.w = 0
        self.prev_heading_error =0  
        self.prev_vel_error = 0

        self.vx_self = 0 
        self.vy_self = 0 

        self.obs_x = -30 
        self.obs_y = -30
        self.obs_v_x = 0 
        self.obs_v_y = 0
        self.abs_x, self.abs_y, self.abs_theta = None, None, 0

        self.relx_est, self.rely_est, self.vrelx_est, self.vrely_est = 0,0,0,0
        self.v_obsx = -0.00
        self.v_obsy = 0 
        self.qp = QP_Controller_Unicycle(1, obs_radius= 0.3)
        self.t_prev = rospy.get_time()
        self.t_prev_odom = None 
        self.v_des = 0.11
        self.v_target = 0.15
        self.w_target = 0
        self.v_target_old = 0.15
        self.w_target_old = 0
        self.return_to_origin = False

        bot1_config_file_path = './Turtle-C3BF/bots/bot_config/bot1.json'
        self.bot = Unicycle.from_JSON(bot1_config_file_path)


        self.reset_files()
        
        self.dt_odom = 0.05


        #Internal variables to store the obstacle pos
        self.internal_obs_x = 2
        self.internal_obs_y = 0.1
        self.internal_obs_velx = 0
        self.internal_obs_vely = 0 
        # self.obs_offsetx, self.obs_offsety = 2.5, 0



    def reset_files(self):
        with open("./run_no.txt", "r") as f:
            l = f.readline()
            self.no = int(l)
        open("./run_no.txt", "w").close()
        # self.no = 0
        with open("./run_no.txt", "w") as f:
            f.write(str(self.no + 1))
        
        self.file = f"./runs/log{self.no}.txt"
        # open(self.file, "w").close()
        self.file_path = f"./runs/obstaclepos{self.no}.txt"
        # open(self.file_path, "w").close()

        self.file_path2 = f"./runs/obstaclevel{self.no}.txt"
        # open(self.file_path2, "w").close(
        
        self.file_path3 = f"./runs/robotpos{self.no}.txt"
        # open(self.file_path3, "w").close()
        self.file_path4 = f"./runs/gt{self.no}.txt"

        self.file_path5 = f"./runs/cbf{self.no}.txt"


    def pid(self):

        t_curr = rospy.get_time()
        self.dt = t_curr - self.t_prev 
        
        print("dt:",self.dt)
        self.t_prev = t_curr 

        # if self.x > 3.2:
        #     self.goaly = 0.2

        delta_theta = (np.arctan2((self.goaly - self.y), 0.9)) - self.theta
        # delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
        # print("delta theta", delta_theta)
        # delta_theta = np.pi/2 - self.thet.a

        #Error is delta_theta in degrees
        e_new = delta_theta
        
        e_dot = (e_new - self.prev_heading_error)/self.dt 
        # print("enew", e_new)1
        # print("slef. theta", 11self.theta)
        # print("edot", e_dot)
        # print("etheta, edottheta"?, e_new, e_dot)
        alpha = (self.Kp1*e_new) + (self.Kd1*e_dot)
        self.prev_heading_error = e_new 
        
        # self.v = np.sqrt(self.vx_self **2 + self.vy_self ** 2 )

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
    
        return a, alpha


    def publish(self, obs_x, obs_y, obs_v_x, obs_v_y):
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

        self.abs_x = obs_x 
        self.abs_y = obs_y
        self.obs_v_x = obs_v_x
        self.obs_v_y = obs_v_y 
        a, alpha = self.pid()
        
        # print("A, ALPHA", a , alpha)
        # print("v", self.v)
        u_ref = np.array([a, alpha])
        a , alpha = np.array([a]), np.array([alpha])
        active = 0
        detection = 0
        value_of_h = 0
        if self.abs_x != None  and self.abs_y != None :
            detection = 1
            # print(" ..................................")
            # print()
            # print()
            # print()
            # print("detection")
            # print("x and y", self.abs_x, self.abs_y)
            self.qp.set_reference_control(u_ref)
            # print("setting up qp: xy vxvy",  [self.abs_x, self.abs_y], [self.obs_v_x, self.obs_v_y])
            self.qp.setup_QP(self.bot, [self.abs_x, self.abs_y], [self.obs_v_x, self.obs_v_y]) #  
            
            
            # print("vrel ", self.vobs_x, self.vrely)
            value_of_h  = self.qp.solve_QP(self.bot)
            # print("value of h", value_of_h)
            # # Bot Kinematics
            u_star = self.qp.get_optimal_control() 
            if t - self.t0 > 0:
                
                a, alpha = u_star

                if u_star[0] != u_ref[0] or u_star[1] != u_ref[1]:
                    active = 1
                    print("   ")
                    print("--------------------------------------")
                    print("CBF ACTIVE!!!")


            print("curr x and y ", self.x, self.y)
            # print("rel ", self.obs_x, self.rely)  
            # Simulation
            # Solve QP

            print(f"current velocity: {self.v} angular: {self.w}")
            # Printing bot params 
            print("cbf inputs ", [self.abs_x, self.abs_y], [self.obs_v_x, self.obs_v_y])
            print(f"botx : {self.bot.x} boty: {self.bot.y}, bot v: {self.bot.v}")
            print(f"bot theta: {self.bot.theta  } bot w: {self.bot.w}")
            print("reference ", u_ref)
            print("cbf", u_star)
            print("params", self.bot.x, ",",self.bot.y,",", self.bot.theta, ",",self.bot.v, ",", self.bot.w)
            print("--------------------------")
            print("  ")
        else:
            self.qp.set_reference_control(u_ref)
            self.qp.setup_QP(self.bot, [self.internal_obs_x, self.internal_obs_y], [self.internal_obs_velx, self.internal_obs_vely]) #            
            value_of_h   = self.qp.solve_QP(self.bot)
            print("value of h", value_of_h)



        self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.w,self.dt )
        with open(self.file_path3, 'a') as f:
            s = " ".join(str(i) for i in [t, self.x, self.y, active])
            s += '\n'
            f.write(s)
            f.close()
        
        with open(self.file_path4, 'a') as f:
            s = " ".join(str(i) for i in [t, self.internal_obs_x, self.internal_obs_y, self.internal_obs_velx, self.internal_obs_vely])
            s += '\n'
            f.write(s)
            f.close()

        self.internal_obs_x = .00 * self.dt +self.internal_obs_x
        self.internal_obs_velx = 0.00
        self.internal_obs_vely = 0
        
        self.v_target_old =  self.v_target_old + a * self.dt #(self.targetVelocity1 + self.targetVelocity2)* 0.033 / 2 
        self.w_target_old =  self.w_target_old + alpha * self.dt #(self.targetVelocity2 - self.targetVelocity1) * 0.033 / 0.15

        # self.v_target_old = max(-0.15, min(0.15, self.v_target_old))
        # self.w_target_old = max(-1.5, min(1.5, self.w_target_old))


        self.v_target = self.v_target_old
        self.w_target = self.w_target_old
        cmd_vel = Twist()
        cmd_vel.linear.x =  self.v_target
        cmd_vel.angular.z= self.w_target
        self.control_pub.publish(cmd_vel)


        cmd_vel.linear.x = -0.04
        cmd_vel.angular.z = -0.0
        self.obstacle_pub.publish(cmd_vel)


        t2 = time.time()
        print("inference time", t2-t)
        if type(self.v_target) == float:
            self.v_target = np.array([self.v_target])
        
        if type(self.w_target) == float:
            self.w_target = np.array([self.w_target])
        
        
        with open(self.file, "a") as f:
            l = [self.x, self.y, value_of_h, t, active, detection, self.v_target[0], self.w_target[0]]
            s = " ".join(str(i) for i in l)
            s += '\n'
            f.write(s)
            f.close()


        with open(self.file_path5, "a") as f:
            l = [self.bot.x, self.bot.y, self.bot.theta, self.bot.v, self.bot.w, t, value_of_h, active, u_ref[0], u_ref[1], u_star[0][0], u_star[1][0], self.v_target[0], self.w_target[0]]
            s = " ".join(str(i) for i in l)
            s += '\n'
            f.write(s)
            f.close()


    def obstacle_sub(self, data):
        print("obstacle callback")
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        # print("odom x, y {x}, {y}")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw

        # self.v_obsx = data.twist.twist.linear.x
        # self.v_obsy = data.twist.twist.linear.y

        self.internal_obs_x = -x + self.obs_offsetx
        self.internal_obs_x = -y + self.obs_offsety
        self.internal_obs_x = data.twist.twist.linear.x
        self.internal_obs_vely = 0

        print("gt" ,data.pose.pose.position.x, data.pose.pose.position.y,data.twist.twist.linear.x, data.twist.twist.linear.y)



    def odom_callback(self, data):
        # print("pid odom callback")
        
        if not self.t_prev_odom:
            self.t_prev_odom = data.header.stamp.secs + data.header.stamp.nsecs / 10**9
            pass
        t = data.header.stamp.secs + data.header.stamp.nsecs / 10**9
        dt = t - self.t_prev_odom
        self.dt_odom = dt
        # print("dt odom", dt)
        if dt == 0:
            dt = 0.1
        self.t_prev_odom = t 

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # print("odom x, y {x}, {y}")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw
        
        self.v = data.twist.twist.linear.x
        # print(self.v, "self.v")

        self.vx_self = (x - self.x) / dt  
        self.vy_self = (y - self.y)/ dt 
        self.w = data.twist.twist.angular.z
        self.theta_list.append(theta)
        self.theta_list = np.unwrap(self.theta_list).tolist()
        if len(self.theta_list) > 5:
            self.theta_list.pop(0)

        self.theta = self.theta_list[-1] 

        # self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.w,dt )
        self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.w,   dt )

        self.x = x 
        self.y = y



    def run(self):
        rate = rospy.Rate(20)
        tick = 0
        x = 2
        v = 0.00
        t0 = time.time()
        while not rospy.is_shutdown():
            self.publish(x,0.0,-v,0)
            t = time.time()
            dt = t - t0 
            t0 = t 
            x = x - v * dt
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
    print("blah 8")
    pid = Controller()

    pid.run()


