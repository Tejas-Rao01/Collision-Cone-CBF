#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt


class Controller():
    def __init__(self):
        rospy.init_node('Controller')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_sub = rospy.Subscriber('/robot', Odometry, self.target_callback)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.etheta_list = []

        # Defining Parameters
        self.tau = 0.1
        self.eps_theta = 0.1
        self.omega_max = 0.8
        self.v_max = 0.5
        self.w_max = 1
        self.eta_v = 5
        self.eta_w = 5
        self.t_prev = 0
        self.thetad_prev = 0
        self.T_sim = 100
        self.t_start = rospy.get_time()
        self.x_t = 0
        self.y_t = 0

        self.robot_xs = []
        self.robot_ys = []
        self.target_xs = []
        self.target_ys = []
        self.ex = []
        self.ey = []
        self.etheta = []

        self.flag = True

    def publisher(self):

        t_curr = rospy.get_time() - self.t_start
        dt = t_curr - self.t_prev
        self.t_prev = t_curr

        x_t, y_t = self.get_traj_point()

        ex = -self.x + x_t
        ey = -self.y + y_t
        theta_d = np.arctan2(ey, ex)
        thetad_dot = (theta_d - self.thetad_prev) / dt
        e_theta = -self.theta + theta_d
        self.etheta_list.append(e_theta)
        e_theta = np.unwrap(self.etheta_list)[-1]
        self.thetad_prev = theta_d

        D = np.sqrt(ex ** 2 + ey ** 2)
        v = self.v_max * np.cos(e_theta) * self.eta_v * D / (self.eta_v * D + 1)
        w = (self.w_max - self.omega_max) * np.tanh(self.eta_w * e_theta) + thetad_dot

        pub_msg = Twist()
        pub_msg.linear.x = v
        pub_msg.angular.z = w

        self.pub.publish(pub_msg)



        # self.robot_xs.append(self.x)
        # self.robot_ys.append(self.y)
        # self.target_xs.append(x_t)
        # self.target_ys.append(y_t)
        # self.ex.append(ex)
        # self.ey.append(ey)
        # self.etheta.append(e_theta)

        # if t_curr > 50 and self.flag == True:
        #     arr = np.zeros(shape=(2006, 7))
        #     for i in range(len(self.ex)):
        #         l = [self.robot_xs[i], self.robot_ys[i], self.target_xs[i], self.target_ys[i], self.ex[i], self.ey[i],
        #              self.etheta[i]]
        #         arr[i, :] = l
        #     print('saving')
        #     np.save("run2.npy", arr)
        #     self.flag = False

        print(f"publishing, {t_curr}")

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.theta = yaw
        time = rospy.get_time()

    def target_callback(self, data):
        self.x_t = data.pose.pose.position.x
        self.y_t = data.pose.pose.position.y

        self.publisher()
        
    def get_traj_point(self):
        return self.x_t, self.y_t

if __name__ == "__main__":
    controller = Controller()
    rospy.spin()
    

