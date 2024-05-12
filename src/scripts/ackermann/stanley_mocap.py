#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

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

sys.path.append('/home/rao/Downloads/F1-C3BF')
sys.path.append('/home/rao/Downloads/F1-C3BF/controllers')
sys.path.append('/home/rao/Downloads/F1-C3BF/bots/bot_config')

from bots.Bicycle import Bicycle
from controllers.QP_controller_bicycle import QP_Controller_Bicycle
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16, Float32, Bool

from ackermann_msgs.msg import AckermannDrive
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
        self.t_odom_prev = time.time()
        #PID params
        self.k = 2.5  
        self.ks = 1.5
        self.Kp2 = 0.7
        self.Kd2 = 0.5 
        self.Ki1 = 0.0

        self.x = 0
        self.y = 0
        self.theta = 0 
        self.theta_list = []
        self.v = 0
        self.w = 0

        self.obs_x, self.obs_y, self.obs_v_x, self.obs_v_y = 0,0,0,0

        self.goalx, self.goaly = 0, 1.07

        self.v_des = 3
        self.v_target = 3

        
        self.prev_vel_error = 0
        self.steer  = 0 
        self.max_steer = 1 



    def init_qp(self):
        self.qp = QP_Controller_Bicycle(1, obs_radius= 0.15)
        bot1_config_file_path = '/home/rao/Downloads/F1-C3BF/bots/bot_config/bot1.json'
        self.bot = Bicycle.from_JSON(bot1_config_file_path)

    def init_pubsub(self):
        self.odom_sub = rospy.Subscriber('/phasespace/markers', Markers , self.odom_callback)
        self.steer_pub = rospy.Publisher("/steering", Float32,queue_size=1)
        # self.obs_sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.obstacle_sub1)
        self.throttle_pub = rospy.Publisher("/tb3_2/cmd_vel", Twist,queue_size=1)
        self.obs_pub = rospy.Publisher("/tb3_1/cmd_vel", Twist,queue_size=1)
        self.agent_sub = rospy.Subscriber('tb3_2/odom', Odometry, self.agent_callback)
    def reset_files(self):
        with open("./run_no.txt", "r") as f:
            l = f.readline()
            self.no = int(l)

        with open("./run_no.txt", "w") as f:
            f.write(str(self.no + 1))   
    
    
        self.file_path = f"./log/cbf{self.no}.txt"
    
    def pid(self):
        

        t_curr = time.time()
        print("===============================")
        print("Starting PID calculation...")
        print("Current time: ", t_curr)
        print("Previous time: ", self.t_prev)
        print("Time difference: ", t_curr - self.t_prev)
        self.dt = t_curr - self.t_prev 
        print("Time difference: ", self.dt)
        self.t_prev = t_curr 
        print("Updated previous time: ", self.t_prev)

        psi = 0 - self.theta
        print("Angle: ", psi)

        if self.x > 1.5: 
            desy = 1.6
        else:
            desy = 1.5
        crosstrack = np.arctan2(self.k * (desy- self.y)    , (self.ks + 0.1) ) 
        print("Crosstrack: ", crosstrack)
        delta = psi + crosstrack 
        print("Steering angle: ", delta)

        e_v = (self.v_des- self.v ) 
        print("Error in velocity: ", e_v)
        e_vdot = (e_v - self.prev_vel_error) / self.dt
        print("Error in velocity dot: ", e_vdot)
        self.prev_vel_error = e_v 
        print("Updated previous error in velocity: ", self.prev_vel_error)

        a = self.Kp2 * e_v  + self.Kd2 * (e_vdot) 
        print("Acceleration: ", a)

        print("Finished PID calculation.")
        print("===============================")

        return a, delta


    def publish(self):
    
        a, steer_angle= self.pid()

        u_ref = np.array([ a, steer_angle])
        self.u_ref = u_ref 
        self.qp.set_reference_control(u_ref)
        self.qp.setup_QP(self.bot) #  
        
        H = self.qp.solve_QP(self.bot, [self.obs_x, self.obs_y], [self.obs_v_x, self.obs_v_y])
        
        # # Bot Kinematics
        u_star = self.qp.get_optimal_control() 
        a, steer_angle = u_star
        self.u_star = u_star

        active = 0 
        if abs(u_star[0] - u_ref[0]) < 0.01 or abs(u_star[1] != u_ref[1]) < 0.01:
            active = 1

        self.print_vars(active)

        steer_angle = min(self.max_steer, max(-self.max_steer, steer_angle))
        self.v_target =  self.v_target + a * self.dt 
        self.v_target = min(0.5, max(-0.1, self.v_target))
        steer_angle = min(self.max_steer, max(-self.max_steer, steer_angle))


        print("v target: ", self.v_target)
        throttle_msg = Twist()
        throttle_msg.linear.x = -self.v_target
        throttle_msg.angular.z = steer_angle
        
        steer_msg = Float32()
        steer_msg.data = steer_angle

        self.steer_pub.publish(steer_msg)
        self.throttle_pub.publish(throttle_msg)


        if type(self.v_target) == float:
            self.v_target = np.array([self.v_target])
        
        vars = [time.time(), self.bot.x, self.bot.y, self.bot.theta, self.bot.v,  u_ref[0], u_ref[1], u_star[0][0], u_star[1][0] , self.v_target[0], self.obs_x, self.obs_y, self.obs_v_x , self.obs_v_y]
        self.log(vars )

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.05
        self.obs_pub.publish(cmd_vel)

    def log(self, vars):
        
        with open(self.file_path, "a") as f:
            s = " ".join(str(i) for i in vars)
            s += '\n'
            f.write(s)
            f.close()


    def print_vars(self, active):
        
        print("   ")
        print("--------------------------------------")
        if active == 1 : print("CBF ACTIVE!!!")
        print("curr x and y ", self.x, self.y)
        print(f"current velocity: {self.v}")
        # Printing bot params 
        print("cbf inputs ", [self.obs_x, self.obs_y], [self.obs_v_x, self.obs_v_y])
        print(f"botx : {self.bot.x} boty: {self.bot.y}, bot v: {self.bot.v}")
        print(f"bot theta: {self.bot.theta  } ")
        print("reference ", self.u_ref)
        print("cbf", self.u_star)
        print("params", self.bot.x, ",",self.bot.y,",", self.bot.theta, ",",self.bot.v, )
        print("--------------------------")
        print("  ")


    def agent_callback(self, data):
        v = -data.twist.twist.linear.x 
        self.v = v 
        self.bot.v = v 

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

            vx = (x - self.obs_x) / dt 
            vy = (y - self.obs_y) / dt

            self.v = np.sqrt(vx**2 + vy**2)

            self.obs_v_x = ((x3 + x4) * 0.5 - self.obs_x) / dt
            self.obs_v_y = ((y3 + y4)*0.5 - self.obs_y) / dt

            self.obs_x = (x3 + x4) * 0.5 
            self.obs_y = (y3 + y4)*0.5

            

            theta = np.arctan2((y2 - y), (x2 - x))
            self.obs_theta = np.arctan2((y4 - (y4+y3)/2), (x4 - (x3+x4)/2))


            theta1 = np.arctan2((y4 - (y4+y3)/2), (x4 - (x3+x4)/2))

            
            self.x = x 
            self.y = y 
            self.theta = theta 

            self.bot.update_state(np.array([self.x, self.y]), self.theta, self.v,   dt )
    

        
    def run(self):
        rate = rospy.Rate(10)
        tick = 0
        while not rospy.is_shutdown():
            self.publish(   )
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

    



