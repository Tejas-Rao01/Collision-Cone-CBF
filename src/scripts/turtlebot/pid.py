#!/usr/bin/env python3

import rospy 
from typing import Tuple, List
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


class Controller():

    def __init__(self) -> None:
        

        self.pose_sub = rospy.Subscriber('/tb3_0/odom', Odometry, self.odom_callback)
        self.control_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.obs_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size =1)
        self.obs_sub = rospy.Subscriber('/tb3_1/odom', Odometry, self.obstacle_sub)

        self.x = 0
        self.y = 0
        self.theta = 0 
        self.theta_list = []

        self.goalx, self.goaly = 0, 0
        self.curr_goal = 1  
        self.goal_thresh = 0.2

        #PID params

        self.Kp1 = 0.4
        self.Kd1 = 0.3 
        self.Ki1 = 0.0
        
        # self.Kp1 = 0.05   
        # self.Kd1 = 0.4  
        # self.Ki1 = 0.0
        
        # self.Kp2 = 0.06
        # self.Kd2 = 0.015
        # self.Ki2 = 0

        self.t0 = time.time()

        self.Kp2 = 2.5
        self.Kd2 = 0.4
        self.Ki2 = 0

        self.v = 0
        self.w = 0
        self.prev_heading_error =0  
        self.prev_vel_error = 0
        self.prev_heading_error_obs =0  
        self.prev_vel_error_obs = 0

        self.vx_self = 0 
        self.vy_self = 0

        self.obs_x = -30 
        self.obs_y = -30
        self.obs_v_x = -0.05 
        self.obs_v_y = 0
        self.abs_x, self.abs_y, self.abs_theta = None, None, 0

        self.relx_est, self.rely_est, self.vrelx_est, self.vrely_est = 0,0,0,0
        self.v_obsx = -0.00
        self.v_obsy = 0 
        self.qp = QP_Controller_Unicycle(1, obs_radius= 0.25)
        self.t_prev = rospy.get_time()
        self.t_prev_odom = None 
        self.v_des = 0.15
        self.v_target = 0.15
        self.w_target = 0
        self.v_target_old = 0.15
        self.w_target_old = 0

        self.obs_v = 0

        self.v_target_obs = 0.08 
        self.w_target_obs=  0

        self.theta_list_obs = []


        self.return_to_origin = False

        bot1_config_file_path = './Turtle-C3BF/bots/bot_config/bot1.json'
        self.bot = Unicycle.from_JSON(bot1_config_file_path)


        self.reset_files()
        
        self.dt_odom = 0.05


    def reset_files(self) -> None:
        """
        Resets the run number counter and creates a new run file.

        Returns:
            None
        """
        with open("./run_no.txt", "r") as f:
            l = f.readline()
            self.no = int(l)
        open("./run_no.txt", "w").close()
        # self.no = 0
        with open("./run_no.txt", "w") as f:
            f.write(str(self.no + 1))
        
        self.file_path = f"./runs/cbf{self.no}.txt"


    def pid(self) -> Tuple[float, float]:
        """
        Computes the alpha and a values using PID control.

        Returns:
            Tuple[float, float]: The computed alpha and a values.
        """
        self.dt = rospy.get_time() - self.t_prev 
        self.t_prev = rospy.get_time()

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
        

    def publish(self):
        t = time.time()
        a, alpha = self.pid()
        
        u_ref = np.array([a, alpha])
        a , alpha = np.array([a]), np.array([alpha])
        self.qp.set_reference_control(u_ref)
        self.qp.setup_QP(self.bot) #  
        
        
        h  = self.qp.solve_QP(self.bot,  [self.obs_x, self.obs_y], [self.obs_v_x, self.obs_v_y])
        # # Bot Kinematics
        u_star = self.qp.get_optimal_control() 
            
        a, alpha = u_star
        active = 0 
        if (u_star[0] - u_ref[0])  ** 2 +  (u_star[1] - u_ref[1]) ** 2 > 0.001:
            active = 1
        self.print_vars(active=active)
        self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.w,self.dt )
        
        self.v_target_old =  self.v_target_old + a * self.dt #(self.targetVelocity1 + self.targetVelocity2)* 0.033 / 2 
        self.w_target_old =  self.w_target_old + alpha * self.dt #(self.targetVelocity2 - self.targetVelocity1) * 0.033 / 0.15


        self.v_target = self.v_target_old
        self.w_target = self.w_target_old
        
        self.v_target = min(0.15, max(-0.15, self.v_target))
        self.w_target = min(1.5, max(-1.5, self.w_target))
        
        cmd_vel = Twist()
        cmd_vel.linear.x =  self.v_target
        cmd_vel.angular.z= self.w_target
        self.control_pub.publish(cmd_vel)


        if type(self.v_target) == float:
            self.v_target = np.array([self.v_target])
        
        if type(self.w_target) == float:
            self.w_target = np.array([self.w_target])

        vars = [t, self.bot.x, self.bot.y, self.bot.theta, self.bot.v, self.bot.w, u_ref[0], u_ref[1], u_star[0][0], u_star[1][0], self.v_target[0], self.w_target[0], self.obs_x, self.obs_y, self.obs_v_x , self.obs_v_y]
        self.log(vars)


        # Publish command for obstacle
        obs_cmdvel = Twist()
        obs_cmdvel.linear.x = 0.1
        self.obs_pub.publish(obs_cmdvel)
            
        

    def log(self, vars: List[float]) -> None:
        """
        Appends the given list of variables to the log file.
        
        Args:
            vars: List of floats containing the values to be logged.
        
        Returns:
            None
        """
        with open(self.file_path, "a") as f:
            s = " ".join(str(i) for i in vars)
            s += '\n'
            f.write(s)
            f.close()
    def print_vars(self, active: bool) -> None:
        """
        Prints the current state of the bot and the CBF

        Args:
            active (bool): Whether the CBF is active or not

        Returns:
            None
        """
        if type(self.v_target) == float:
            self.v_target = np.array([self.v_target])
        
        if type(self.w_target) == float:
            self.w_target = np.array([self.w_target])

        print("   ")
        print("--------------------------------------")
        if active: print("CBF ACTIVE!!!")
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

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        # print("odom x, y {x}, {y}")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw
        self.theta_list_obs.append(theta)
        theta = np.unwrap(np.array(self.theta_list_obs))[-1]

        if theta < 0:
            theta = 2 * np.pi - theta 
        if len(self.theta_list_obs )>5:
            self.theta_list_obs.pop(0)
        
        self.obs_theta = theta

        v = data.twist.twist.linear.x
        self.obs_v = v
        self.obs_x = x 
        self.obs_y = y 
        self.obs_v_x = v * np.cos(theta)
        self.obs_v_y = v * np.sin(theta)



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

        if dt > 0.2:
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
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()    

if __name__ == "__main__":
    rospy.init_node("pid")
    print("blah 8")
    pid = Controller()

    pid.run()


