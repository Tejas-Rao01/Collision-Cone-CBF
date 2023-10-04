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
        self.target_pub = rospy.Publisher('/targetPose', Odometry)
        
        self.x = 0
        self.y = 0
        self.theta = 0
        self.etheta_list = []

        # Defining Parameters
        self.tau = 0.1
        self.eps_theta = 0.1
        self.omega_max = 2
        self.v_max = 0.11
        self.w_max = 2.7
        self.eta_v = 10
        self.eta_w = 5
        self.t_prev = 0
        self.thetad_prev = 0
        self.T_sim = 200
        self.t_start = rospy.get_time()

        self.robot_xs = []
        self.robot_ys = []
        self.target_xs = []
        self.target_ys= []
        self.ex = []
        self.ey = []
        self.etheta = []

        self.flag = True
    def run(self):
        rospy.spin()

    def publisher(self):

        t_curr = rospy.get_time() - self.t_start
        dt = t_curr - self.t_prev
        self.t_prev = t_curr

        x_t, y_t = self.get_traj_point(t_curr, self.T_sim)

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

        target = Odometry()
        target.pose.pose.position.x = x_t
        target.pose.pose.position.y = y_t
        self.target_pub.publish(target)

        self.robot_xs.append(self.x)
        self.robot_ys.append(self.y)
        self.target_xs.append(x_t)
        self.target_ys.append(y_t)
        self.ex.append(ex)
        self.ey.append(ey)
        self.etheta.append(e_theta)

        if t_curr >50 and self.flag == True:
            arr=  np.zeros(shape=(2006, 7))
            for i in range(len(self.ex)):
                l = [self.robot_xs[i], self.robot_ys[i], self.target_xs[i], self.target_ys[i], self.ex[i], self.ey[i], self.etheta[i]]
                arr[i, :] = l 
            print('saving')
            np.save("run2.npy", arr)
            self.flag = False    
                    

        print(f"publishing, {t_curr}")
       

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.theta = yaw
        time = rospy.get_time()



       

    def get_traj_point(self,t, T_sim):
        x_t = 3 * np.cos(2 * np.pi * t / T_sim)
        y_t = 2 * np.sin(2 * np.pi * t / T_sim)
        return x_t, y_t


if __name__ == "__main__":
    controller = Controller()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        controller.publisher()
        plt.show(block=True)
        rate.sleep()

