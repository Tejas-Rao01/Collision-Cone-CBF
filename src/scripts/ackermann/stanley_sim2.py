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
        
        # self.obs_sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.obstacle_sub1)
        # self.obs_sub2 = rospy.Subscriber('/tb3_0/odom', Odometry, self.obstacle_sub2)


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
        self.k = 0.3 
        self.ks = 0.4

        #Pid velocity control 
        self.Kp2 = 1
        self.Kd2 = 0.4
        self.Ki1 = 0.0

        # Desired linear and angular velocities for the controller 
        self.v_des = 0.15
        self.w_target = 0
        self.v_target = 0.25
        
        # Obstacle Position and Absolute Velocity 
        self.obs_x, self.obs_y, self.obs_v_x, self.obs_v_y = 10, 10, 0, 0

        # Store the heading errors        
        self.prev_heading_error =0  
        self.prev_vel_error = 0

        # Setup the QP for cbf 
        self.qp = QP_Controller_Bicycle(1, obs_radius= 0.2)
        
            # define the virtual bot for cbf computation 
        bot1_config_file_path = '/home/rao/Downloads/F1-C3BF/bots/bot_config/bot1.json'
        self.bot = Bicycle.from_JSON(bot1_config_file_path)
        

        # Limit the maximum steering angle 
        self.steer  = 0 
        self.max_steer = 0.4

        # Init the folders for plotting 
        self.run_no = "./run_no.txt" # Obtain the current run number 
        plt_folder_path = "./log" # Path to the folder to save data
        self.init_files(plt_folder_path)

    def init_files(self, folder_path):

        # Specify the path of the folder you want to create
        folder_path = Path(folder_path)

        # Check if the folder already exists
        if not folder_path.exists():
            # Create the folder
            folder_path.mkdir(0o755)
            print(f"Folder '{folder_path}' created successfully.")
        else:
            print(f"Folder '{folder_path}' already exists.")
        
        with open(self.run_no, "r") as f:
            l = f.readline()
            self.no = int(l)
        open(self.run_no).close()

        with open(self.run_no, "w") as f:
            f.write(str(self.no + 1))
        
        self.file =  str( folder_path / f"log{self.no}.txt")
        # open(self.file, "w").close()
        self.file_path = str(folder_path / f"obstaclepos{self.no}.txt")
        # open(self.file_path, "w").close()

        self.file_path2 = str(folder_path / f"obstaclevel{self.no}.txt")
        # open(self.file_path2, "w").close()
        
        self.file_path3 = str(folder_path / f"robotpos{self.no}.txt")
        
        # open(self.file_path3, "w").close()
        self.file_path4 = str(folder_path / f"gt{self.no}.txt")

    def log(self, file_path, vars:list):
        with open(file_path, 'a') as f:
            # print("filepath ", file_path)
            s = " ".join(str(i) for i in vars)
            s += '\n'
            f.write(s)
            f.close()


    def pid(self):
        
        # Obtain dt
        t_curr = time.time()
        self.dt = t_curr - self.t_prev 
        self.t_prev = t_curr 

        # Trajectory heading 
        psi = 0 - self.theta
        crosstrack = np.arctan2(self.k * (0-self.y), (self.ks + self.v) ) 
        delta = psi + crosstrack 
        e_v = (self.v_des- self.v ) 
        e_vdot = (e_v - self.prev_vel_error) / self.dt
        self.prev_vel_error = e_v 
        a = self.Kp2 * e_v  + self.Kd2 * (e_vdot) 


        print("psi", psi)
        print("cross track", crosstrack)
        print("num", self.k * (1-self.y))
        print("den",(self.ks + self.v) )
        print("delta", delta)
        
        return delta, a


    def publish(self):
        t = time.time()

        self.log(self.file_path, [t, self.obs_x, self.obs_y])
        self.log(self.file_path2, [t, self.obs_v_x, self.obs_v_y])
        steer_angle, a = self.pid()

        # Limit the steering angle to physical limits
        steer_angle = min(self.max_steer, max(-self.max_steer, steer_angle))
        u_ref = np.array([ a, steer_angle])
        active = 0


        print("cbf inputs ", [self.obs_x1, self.obs_y1], [self.v_obsx1, self.v_obsy1], [self.obs_x2, self.obs_y2], [self.v_obsx2, self.v_obsy2])

        print(f"botx : {self.bot.x} boty: {self.bot.y}, bot v: {self.bot.v}")
        print(f"bot theta: {self.bot.theta  }")
        if 0 == 0: # self.obs_x != None  and self.obs_y != None :
            # self.bot.v = 0.1
            self.qp.set_reference_control(u_ref)
            self.qp.setup_QP(self.bot, [self.obs_x1, self.obs_y1], [self.v_obsx1, self.v_obsy1], [self.obs_x2, self.obs_y2], [self.v_obsx2, self.v_obsy2]) #  
            
            h = self.qp.solve_QP(self.bot)
            
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
            
        self.log(self.file_path3, [t, self.x, self.y, active])
        self.log(self.file,  [self.x, self.y, h, t, active, 0] )

        prev_target_v = self.v_target
        self.v_target = a * self.dt + prev_target_v
        pub_v = self.v_target #* 0.033
        ack = AckermannDrive()
        ack.speed = pub_v

        steer_angle = steer_angle
        ack.steering_angle = steer_angle 
        self.control_pub.publish(ack)

        cmd_vel = Twist()
        cmd_vel.linear.x = -0.00    
        cmd_vel.angular.z = -0.0
        self.obstacle_pub.publish(cmd_vel)

    def obstacle_sub1(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # print("odom x, y {x}, {y}")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw

        self.v_obsx1 = data.twist.twist.linear.x
        self.v_obsy1 = data.twist.twist.linear.y

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

        self.v_obsx2 = data.twist.twist.linear.x
        self.v_obsy2 = data.twist.twist.linear.y

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
        # time.sleep(0.3)
        while not rospy.is_shutdown():
            t1 = time.time()
            self.publish()
            t2 = time.time()

            print("--------------")
            print("imferemce time", t2-t1)
            rate.sleep()    

if __name__ == "__main__":
    
    rospy.init_node("pid")
    pid = Controller()
    time.sleep(0.1)
    pid.run()


