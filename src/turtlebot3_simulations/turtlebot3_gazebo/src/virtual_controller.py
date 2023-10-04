#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt


class LeaderController():
    def __init__(self):
        rospy.init_node('LeaderController')
        self.pub = rospy.Publisher('/leaderPose', Odometry, queue_size=10)
        self.target_pub = rospy.Publisher('/targetPose', Odometry,queue_size=10)

        self.robot1_pub = rospy.Publisher('tb3_0/robot', Odometry, queue_size=10)
        self.robot2_pub = rospy.Publisher('tb3_1/robot', Odometry, queue_size=10)
        self.robot3_pub = rospy.Publisher('tb3_2/robot', Odometry, queue_size=10)
        self.robot4_pub = rospy.Publisher('tb3_3/robot', Odometry, queue_size=10)
        self.robot5_pub = rospy.Publisher('tb3_4/robot', Odometry, queue_size=10)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.etheta_list = []
        self.angle_desired_list = []

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
        self.T_sim = 200
        self.t_start = rospy.get_time()

        self.robot_xs = []
        self.robot_ys = []
        self.target_xs = []
        self.target_ys = []
        self.ex = []
        self.ey = []
        self.etheta = []

        self.v = 0
        self.w = 0
        self.flag = True


    def update_leader_state(self, dt):
        x = self.x
        y = self.y
        theta = self.theta

        dtheta = dt * self.w
        #
        print(f'V W: {self.v}, {self.w}')
        print(f'x, y, theta old: {x},{y},{theta}')
        x = x + self.v * np.cos(theta + dtheta/2) * dt
        y = y + self.v * np.sin(theta + dtheta / 2) * dt
        theta = theta + dtheta
        print(f'x, y, theta new: {x},{y},{theta}')
        self.x = x
        self.y = y
        self.theta = theta

    def publish_robot_target(self, robot_no):
        pub_list = [self.robot1_pub, self.robot2_pub,self.robot3_pub, self.robot4_pub, self.robot5_pub ]
        x = self.x + 2 * np.cos(np.pi * 2 * robot_no/ 5 + self.theta)
        y = self.y + 2 * np.sin(np.pi * 2 * robot_no/ 5 + self.theta)
        theta = self.theta

        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = theta

        pub = pub_list[robot_no-1]
        pub.publish(msg)
    
        print(f"publishing for robot {robot_no}")

    def publisher(self):

        t_curr = rospy.get_time() - self.t_start
        dt = t_curr - self.t_prev
        if (dt < 0.06):
            time.sleep(0.1)
            print("sleeping", dt)
            t_curr = rospy.get_time() - self.t_start
            dt = t_curr - self.t_prev
        self.t_prev = t_curr

        x_t, y_t = self.get_traj_point(t_curr, self.T_sim)

        ex = -self.x + x_t
        ey = -self.y + y_t

        self.update_leader_state(dt)

        theta_d = np.arctan2(ey, ex)
        self.angle_desired_list.append(theta_d)
        theta_d = np.unwrap(self.angle_desired_list)[-1]
        self.angle_desired_list[-1] = theta_d

        e_theta = -self.theta + theta_d
        self.etheta_list.append(e_theta)
        e_theta = np.unwrap(self.etheta_list)[-1]
        self.etheta_list[-1] = e_theta

        thetad_dot = (theta_d - self.thetad_prev) / dt
        self.thetad_prev = theta_d

        D = np.sqrt(ex ** 2 + ey ** 2)
        v = self.v_max * np.cos(e_theta) * self.eta_v * D / (self.eta_v * D + 1)
        w = (self.w_max - self.omega_max) * np.tanh(self.eta_w * e_theta) + thetad_dot


        self.v = v
        self.w = w
        pub_msg = Odometry()
        pub_msg.pose.pose.position.x = self.x
        pub_msg.pose.pose.position.y = self.y
        pub_msg.pose.pose.position.z = self.theta

        self.pub.publish(pub_msg)

        target = Odometry()
        target.pose.pose.position.x = x_t
        target.pose.pose.position.y = y_t
        self.target_pub.publish(target)

        self.publish_robot_target(1)
        self.publish_robot_target(2)
        self.publish_robot_target(3)
        self.publish_robot_target(4)
        self.publish_robot_target(5)



        self.robot_xs.append(self.x)
        self.robot_ys.append(self.y)
        self.target_xs.append(x_t)
        self.target_ys.append(y_t)
        self.ex.append(ex)
        self.ey.append(ey)
        self.etheta.append(e_theta)

        if t_curr > 30 and self.flag == True:
            arr = np.zeros(shape=(2006, 7))
            for i in range(len(self.ex)):
                l = [self.robot_xs[i], self.robot_ys[i], self.target_xs[i], self.target_ys[i], self.ex[i], self.ey[i],
                     self.etheta[i]]
                arr[i, :] = l
            print('saving')
            np.save("run2.npy", arr)
            self.flag = False

        print(f"publishing, {t_curr}")

    def get_traj_point(self, t, T_sim):
        x_t = 10 * np.cos(2 * np.pi * t / T_sim) #(5 + np.sin(6 * np.pi * t / T_sim)) * np.cos(2 * np.pi * t / T_sim)
        y_t = 6 * np.sin(2 * np.pi * t / T_sim)  #(5 + np.sin(6 * np.pi * t / T_sim)) * np.sin(2 * np.pi * t / T_sim)
        return x_t, y_t


if __name__ == "__main__":
    controller = LeaderController()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        controller.publisher()
        rate.sleep()













