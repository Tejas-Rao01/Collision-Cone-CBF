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

import pid 
# Todo, simple pid controller that takes robots position and 
# Moves it along a straight path 

class Controller():

    def __init__(self) -> None:

        self.iter_ptr = 0 
        self.obs_pos, self.obs_vel = [],[]
        
        self.reset_files()
        self.pid = pid.Controller()
        print("blah 6")
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

        self.Kp2 = 1
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
        self.obs_v_x = -0.05 
        self.obs_v_y = 0
        self.abs_x, self.abs_y, self.abs_theta = None, None, 0

        self.relx_est, self.rely_est, self.vrelx_est, self.vrely_est = 0,0,0,0
        self.v_obsx = -0.00
        self.v_obsy = 0 
        self.qp = QP_Controller_Unicycle(1, obs_radius= 0.3)
        self.t_prev = rospy.get_time()
        self.t_prev_odom = None 
        self.v_des = 0.05
        self.v_target = 0.15
        self.w_target = 0
        self.v_target_old = 0.15
        self.w_target_old = 0
        self.return_to_origin = False

        bot1_config_file_path = './Turtle-C3BF/bots/bot_config/bot1.json'
        self.bot = Unicycle.from_JSON(bot1_config_file_path)


        
        
        self.dt_odom = 0.05


        #Internal variables to store the obstacle pos
        self.internal_obs_x = 2
        self.internal_obs_y = 0.1
        self.internal_obs_velx = 0
        self.internal_obs_vely = 0 
        # self.obs_offsetx, self.obs_offsety = 2.5, 0


    def reset_files(self):
        # with open("./run_no.txt", "r") as f:
        #     l = f.readline()
        #     self.no = int(l)
        # open("./run_no.txt", "w").close()
        # # self.no = 0
        # with open("./run_no.txt", "w") as f:
        #     f.write(str(self.no + 1))
        
        self.no = 467


        self.file = f"./runs/log{self.no}.txt"

        
        # open(self.file, "w").close()
        self.file_path = f"./runs/obstaclepos{self.no}.txt"
        # open(self.file_path, "w").close()


        with open(self.file_path, 'r') as f:
            lines = f.readlines()

            for line in lines:

                if line.find("None") == -1:
                    vals = list(map(float, line.split()))
                    self.obs_pos.append(vals)

                else:
                    vals = [float(line.split()[0]), None, None]
                    # vals = list(map(float, line.split()))
                    self.obs_pos.append(vals)   
            f.close()

        self.t_log0 = self.obs_pos[0][0]
        self.file_path2 = f"./runs/obstaclevel{self.no}.txt"
        # open(self.file_path2, "w").close(
        with open(self.file_path2, 'r') as f:
            lines = f.readlines()

            for line in lines:
                if line.find("None") == -1:
                    vals = list(map(float, line.split()))
                    self.obs_vel.append(vals)

                else:
                    vals = [float(line.split()[0]), None, None]
                    # vals = list(map(float, line.split()))
                    self.obs_vel.append(vals)
            f.close()
    
        
        self.file_path3 = f"./runs/robotpos{self.no}.txt"
        # open(self.file_path3, "w").close()
        self.file_path4 = f"./runs/gt{self.no}.txt"

        self.file_path5 = f"./runs/cbf{self.no}.txt"




    def publish(self):
        t = time.time()

         
        t_log = self.obs_pos[self.iter_ptr][0] - self.t_log0


        obsx, obsy, obs_vx, obs_vy = [None for i in range(4)]
        
        if (t - self.t0) >= t_log:
            
            obsx, obsy = self.obs_pos[self.iter_ptr][1], self.obs_pos[self.iter_ptr][2]
            obs_vx, obs_vy = self.obs_vel[self.iter_ptr][1], self.obs_vel[self.iter_ptr][2]
            self.iter_ptr += 1  
        
        
        return obsx, obsy, obs_vx, obs_vy


    def run(self):
        rate = rospy.Rate(20)
        tick = 0
        x = 2
        v = 0.00
        time.sleep(0.2)
        t0 = time.time()


        # self.pid.run()

        while not rospy.is_shutdown():
            t = time.time()
            obsx, obsy, obs_vx, obs_vy = self.publish()
            self.pid.publish(obsx, obsy, obs_vx, obs_vy)
            t2 = time.time()
            print("inference time", t2 - t)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pid")
    print("blah 8")
    pid = Controller()

    pid.run()


