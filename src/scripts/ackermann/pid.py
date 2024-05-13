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
sys.path.append('./F1-C3BF')
sys.path.append('./F1-C3BF/controllers')
sys.path.append('./F1-C3BF/bots')
from bots.Bicycle import Bicycle
from controllers.QP_controller_bicycle import QP_Controller_Bicycle
from gazebo_msgs.msg import ModelStates 
from tf2_msgs.msg import TFMessage
from pathlib import Path
from ackermann_msgs.msg import AckermannDrive
# Todo, simple pid controller that takes robots position and 
# Moves it along a straight path 

class Controller():

    def __init__(self) -> None:
        # rospy.init_node("pid_controller")
        self.t_prev_odom = time.time()
        self.t_prev = time.time()

        self.theta_list = []    
        # Init subscribers and publishers 
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.odom_callback)
        self.control_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size =1)
        
        self.obs_sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.obstacle_sub1)
        self.obs_sub2 = rospy.Subscriber('/tb3_0/odom', Odometry, self.obstacle_sub2)


        # Robot state variables 
        self.x = 0
        self.y = 0
        self.theta = 0 
        self.v = 0
        self.w = 0
        self.theta_list = [] # unwrap to account for discontinuities 
        
        self.goalx, self.goaly = 0, 0
        self.curr_goal = 1  
        self.goal_thresh = 0.2

        # Reference Controller Params 
        # Stanley control for steering 
        self.k = 0.2
        self.ks = 0.5

        #Pid velocity control 
        self.Kp2 = 2
        self.Kd2 = 0.4
        self.Ki1 = 0.0

        # Desired linear and angular velocities for the controller 
        self.v_des = 0.2
        self.w_target = np.array([0])
        self.v_target = np.array([0.25])
        
        # Obstacle Position and Absolute Velocity 
        self.obs_x, self.obs_y, self.obs_v_x, self.obs_v_y = 10, 10, 0, 0

        # Store the heading errors        
        self.prev_heading_error =0  
        self.prev_vel_error = 0

        # Setup the QP for cbf 
        self.qp = QP_Controller_Bicycle(1, obs_radius= 0.45)
        
            # define the virtual bot for cbf computation 
        bot1_config_file_path = '/home/rao/Downloads/F1-C3BF/bots/bot_config/bot1.json'
        self.bot = Bicycle.from_JSON(bot1_config_file_path)
        

        # Limit the maximum steering angle 
        self.steer  = 0 
        self.max_steer = 0.4

        # Init the folders for plotting 
        self.run_no = "./run_no.txt" # Obtain the current run number 
        self.init_files()

    def init_files(self):

        with open("./run_no.txt", "r") as f:
            l = f.readline()
            self.no = int(l)
        open("./run_no.txt", "w").close()
        # self.no = 0
        with open("./run_no.txt", "w") as f:
            f.write(str(self.no + 1))
        
        self.file_path = f"./log/cbf{self.no}.txt"

    def log(self, file_path, vars:list):
        with open(file_path, 'a') as f:
            s = " ".join(str(i) for i in vars)
            s += '\n'
            f.write(s)
            f.close()

    def pid(self):
        """
        Compute the alpha and a values using PID control.

        Returns:
            Tuple[float, float]: The computed alpha and a values.
        """

        # Obtain the time difference dt
        t_curr = time.time()
        self.dt = t_curr - self.t_prev 
        self.t_prev = t_curr 

        # Compute the trajectory heading delta and the crosstrack error
        #   psi is the angle between the robot's heading and the x-axis
        #   delta is the steering angle needed to follow the trajectory
        psi = 0 - self.theta
        crosstrack = np.arctan2(self.k * (0.2 - self.y), (self.ks + self.v) )
        delta = psi + crosstrack 

        # Compute the velocity error and its derivative
        e_v = self.v_des - self.v 
        e_vdot = (e_v - self.prev_vel_error) / self.dt
        self.prev_vel_error = e_v 

        # Compute the control input a using PID gains
        a = self.Kp2 * e_v + self.Kd2 * e_vdot 

        return delta, a


    def publish(self):
        t = time.time()

        # self.log(self.file_path, [t, self.obs_x, self.obs_y])
        # self.log(self.file_path2, [t, self.obs_v_x, self.obs_v_y])
        steer_angle, a = self.pid()

        # Limit the steering angle to physical limits
        steer_angle = min(self.max_steer, max(-self.max_steer, steer_angle))
        u_ref = np.array([ a, steer_angle])
        active = 0


        d1 = (self.obs_x1 - self.bot.x)**2 + (self.obs_y1 - self.bot.y)**2 
        d2 = (self.obs_x2 - self.bot.x)**2 + (self.obs_y2 - self.bot.y)**2

        if (d1 < d2):
            self.obs_x = self.obs_x1
            self.obs_y = self.obs_y1 
            self.obs_v_x = self.v_obsx1
            self.obs_v_y = self.v_obsy1

        else:
            self.obs_x = self.obs_x2
            self.obs_y = self.obs_y2 
            self.obs_v_x = self.v_obsx2
            self.obs_v_y = self.v_obsy2
        
        # self.bot.v = 0.1
        self.qp.set_reference_control(u_ref)
        self.qp.setup_QP(self.bot)
        
        h = self.qp.solve_QP(self.bot, [self.obs_x, self.obs_y], [self.obs_v_x, self.obs_v_y])
        
        # # Bot Kineprint(h.shape)matics
        u_star = self.qp.get_optimal_control() 
        a,steer_angle = u_star
        
        if u_star[0] != u_ref[0] or u_star[1] != u_ref[1 ]:
            # steer_angle = -steer_angle
            active = 1
            pass
            print("   ")
            print("--------------------------------------")
            print("CBF ACTIVE!!!")

        print(f"value of h: {h}")
        print("curr x and y ", self.x, self.y)
        # Simulation
        # Solve QP

        print(f"current velocity: {self.v} angular: {self.w}")
        # Printing bot params 
        print("cbf inputs ", [self.obs_x1, self.obs_y1], [self.v_obsx1, self.v_obsy1], [self.obs_x2, self.obs_y2], [self.v_obsx2, self.v_obsy2])

        print(f"botx : {self.bot.x} boty: {self.bot.y}, bot v: {self.bot.v}")
        # print(f"bot theta: {self.bot.theta  } bot w: {self.bot.w}")
        print("reference ", u_ref)
        print("cbf", u_star)

        print("--------------------------")
        print("  ")


        self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v, self.dt )
        print("v target", self.v_target)
        print("ustar", u_star.shape)
        print("uref", u_ref.shape)
        
        prev_target_v = self.v_target
        self.v_target = a * self.dt + prev_target_v
        self.v_target = np.array([min(0.2, max(-0.2, self.v_target.squeeze()))])

        print(u_ref[0], u_ref[1], u_star[0][0], u_star[1][0] , self.v_target)


        var = [t, self.bot.x, self.bot.y, self.bot.theta, self.bot.v,  u_ref[0], u_ref[1], u_star[0][0], u_star[1][0] , self.v_target[0], self.obs_x, self.obs_y, self.obs_v_x , self.obs_v_y]
        self.log(self.file_path,  vars=var )

        pub_v = self.v_target #* 0.033
        ack = AckermannDrive()
        ack.speed = pub_v

        steer_angle = steer_angle
        # cmd_vel.linear.x = -0.00    
        # cmd_vel.angular.z = -0.0
        # self.obstacle_pub.publish(cmd_vel)
        ack.steering_angle = steer_angle 
        self.control_pub.publish(ack)

        # cmd_vel = Twist()
        # cmd_vel.linear.x = -0.00    
        # cmd_vel.angular.z = -0.0
        # self.obstacle_pub.publish(cmd_vel)

    def obstacle_sub1(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # print("odom x, y {x}, {y}")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw
        v =  data.twist.twist.linear.x
        self.v_obsx1 = v * np.cos(theta)
        self.v_obsy1 = v * np.sin(theta)

        self.obs_x1 = x 
        self.obs_y1 = y 
        self.obs_theta1 = theta

    def obstacle_sub2(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # print("odom x, y {x}, {y}")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw

        v =  data.twist.twist.linear.x
        self.v_obsx2 =  v * np.cos(theta)
        self.v_obsy2 =  v * np.sin(theta)

        self.obs_x2 = x 
        self.obs_y2 = y 
        self.obs_theta2 = theta


    def odom_callback(self, data):
        # print("pid odom callback")
        # print(data.name)
        num = 0
        for id, i in enumerate(data.name):
            if i == 'ackermann_vehicle':
                num = id
                break
        
        
        t = time.time()
        dt = t - self.t_prev_odom
        self.t_prev_odom = t 

        # print("data.pose", data.pose[1])
        x = data.pose[num].position.x
        y = data.pose[num].position.y

        # print("odom x, y {x}, {y}")
        orientation_q = data.pose[1].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw
        
        self.v = data.twist[num].linear.x
        self.w = data.twist[num].angular.z
        self.theta_list.append(theta)
        self.theta_list[-1] = np.unwrap(self.theta_list)[-1]
        if len(self.theta_list) > 5:
            self.theta_list.pop(0)

        self.theta = self.theta_list[-1]
       

        self.x = x 
        self.y = y


    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            t1 = time.time()
            self.publish()
            t2 = time.time()

            print("--------------")
            print("inference time", t2-t1)
            print("--------------")
            rate.sleep()    

if __name__ == "__main__":
    
    rospy.init_node("pid")
    pid = Controller()
    time.sleep(0.1)
    pid.run()


