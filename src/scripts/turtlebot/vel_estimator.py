#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import torch 
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from gazebo_msgs.msg import ModelStates
import sys 
sys.path.append("/home/stochlab/Documents")
import pid
import pandas as pd

#code to predict obstacle location based on odom position and estimate velocity and store for plotting later on 

class VelocityEstimator():

    def __init__(self):
        


        self.load_model()

        # Subscribers 
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.img_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.gt_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)#, self.odom_callback)   s
        
        # Parameters for image 
        self.depth_flag = False
        self.depth1 = None  
        self.rgb1 = None
        self.bridge = CvBridge()

        # File Paths for storing data
        self.file_path = "/home/rao/perception_stack/src/turtlebot3_simulations/turtlebot3_gazebo/src/data.txt"
        self.gt_path = "/home/rao/perception_stack/src/turtlebot3_simulations/turtlebot3_gazebo/src/gt.txt"
        open(self.file_path, 'w').close()
        open(self.gt_path, "w").close()


        # Parameters for vel estimation 
        self.num_samples_optical_flow = 3000 
        self.image_height = 480
        self.image_width = 640
        # self.fx = 462.1379699707031
        self.fx = 913.2835693359375

        # Init the values for computing velocity 
        self.obs_x_prev = 0 
        self.obs_y_prev = 0 

        self.t_prev = None 
        self.obs_v_x = 0
        self.obs_v_y = 0

        self.detect_flag = 0 

        self.x = 0 
        self.y = 0
        self.theta = 0

        self.t_prev_ctrl = 0


        
        self.controller = pid.Controller()
        self.robot_vel_x, self.robot_vel_y = 0,0


        self.obs_x = -30
        self.obs_y = -30

        print("initialised")


    def load_model(self):
        weights_path = "/home/rao/torch_models/yolov5/best2.pt"
        model_path = "/home/rao/torch_models/yolov5"
        self.model = torch.hub.load(model_path, 'custom', path=weights_path, source='local', force_reload=True)

        # Used to store the model
        self.df = pd.DataFrame()


    def img_callback(self, rgb):
        x = self.x 
        y = self.y 
        theta = self.theta


        # print("................")
        # print("image callback")
        # print("xyt", x,y , theta)

        rgb_time= rgb.header.stamp.secs + rgb.header.stamp.nsecs / 10**9
        # depth_time = depth.header.stamp.secs + depth.header.stamp.nsecs / 10**9

        # Obtain rgb image 
        try:
            # Convert your ROS Image message to OpenCV2
            rgb_img = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            
        except CvBridgeError as e:
            print("cvbridge error rgb", e)
            pass    
        

        cv.imwrite("/home/rao/img.png", rgb_img)
        self.rgb1 = rgb_img
        if self.depth_flag:
            self.compute_pos_and_vel((rgb_time), x, y , theta)



    def depth_callback(self, depth):
        self.depth_flag = True
        try:
            # print(msg.header.stamp.secs,msg.header.stamp.nsecs )
            # Convert your ROS Image message to OpenCV2
            depth.encoding = "mono16"
            depth_img = self.bridge.imgmsg_to_cv2(depth, "mono16")

        except CvBridgeError as e:
            print("cvbridge error depth", e)
            pass
        
        self.depth1 = depth_img 
        np.save("/home/rao/depth.npy", depth_img)





    def odom_callback(self, msg):
        odom_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 10**9
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y 

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.theta = yaw 


        # print("selfxyt", self.x, self.y, self.theta)


    def image_callback(self, rgb, depth):
        # print('.....................')
        # print("image callback ")
        

        x = self.x 
        y = self.y 
        theta = self.theta

        # print(f"xytheta: {x}, {y},, {theta}")

        rgb_time= rgb.header.stamp.secs + rgb.header.stamp.nsecs / 10**9
        depth_time = depth.header.stamp.secs + depth.header.stamp.nsecs / 10**9

        # Obtain rgb image 
        try:
            # Convert your ROS Image message to OpenCV2
            rgb_img = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            
        except CvBridgeError as e:
            print("cvbridge error rgb", e)
            pass    

        # Obtain depth image 
        try:
            # print(msg.header.stamp.secs,msg.header.stamp.nsecs )
            # Convert your ROS Image message to OpenCV2
            depth.encoding = "mono16"
            depth_img = self.bridge.imgmsg_to_cv2(depth, "mono16")

        except CvBridgeError as e:
            print("cvbridge error depth", e)
            pass
        
        self.rgb1 = rgb_img
        self.depth1 = depth_img
        out_vars= self.compute_pos_and_vel((rgb_time + depth_time)/2, x, y , theta)
        # self.log(t, x, y, x_rel, y_rel, obs_x, obs_y, obs_v_x, obs_v_y)

        print(f"image time: {(rgb_time)}, {depth_time}")

        

    def compute_pos_and_vel(self, t, x, y, theta):
        
        if not self.t_prev:
            self.t_prev = t - 0.2 
        
        dt = t - self.t_prev

        camrelx, camrely = self.get_cam_rel_pos_no_of(t)
        print(f"camrelx: {camrelx}, camerly: {camrely}")
        if camrelx != -1 and camrely != -1:
            

            # Predict absolute pos

            # Camera frame rotated  by pi/2 clockwise wrt robot frame
            angle = theta - np.pi/2
            rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            rotated_pos = np.matmul(rot, np.array([camrelx,camrely]).reshape([2,1]))

            x_rel = rotated_pos[0][0]
            y_rel = rotated_pos[1][0]

            # print(f"x_rel {x_rel} y_rel {y_rel}")

            self.obs_x = rotated_pos[0][0] + x 
            self.obs_y = rotated_pos[1][0] + y 

            if not self.obs_x_prev:
                self.obs_x_prev = x_rel
            if not self.obs_y_prev:
                self.obs_y_prev = y_rel


            self.obs_v_x = (self.obs_x- self.obs_x_prev) / dt 
            self.obs_v_y = (self.obs_y - self.obs_y_prev) / dt 



            # print("_------- ---------")
            # print(f"x and y {x}, {y}")
            # print(f"absx absy: {self.obs_x} {self.obs_y}")
            # print("------------------")
            


            self.obs_x_prev = self.obs_x 
            self.obs_y_prev = self.obs_y
            self.t_prev = t
        
        else:
            self.obs_x = -30 
            self.obs_y = -30
            self.obs_x_prev = None 
            self.obs_y_prev = None 
            self.obs_v_x = 0 
            self.obs_v_y = 0 




        
    def get_cam_rel_pos_no_of(self, t):
        t1 = time.time()
        
        results = self.model([self.rgb1, self.rgb1])
        df1, _ = results.pandas().xyxy[0], results.pandas().xyxy[1]
        
        if len(df1) == 0 :
            return [-1, -1]
        print("  -----------------------00")
        print("")
        print("")
        print("detected object!!1!")
        print("")
        print("")
        print("-------------------------")

        sx1, sy1, ex1, ey1  = df1.iloc[0, 0:4]
        sx1 = min(max(sx1, 0), self.image_width)
        sy1 = min(max(sy1, 0), self.image_height)
        ex1 = min(max(ex1, 0), self.image_width)
        ey1 = min(max(ey1, 0), self.image_height)
        
        t2 = time.time()
        num_samples = self.num_samples_optical_flow
        p0 = np.array([np.random.randint(sx1, ex1, size=(num_samples, 1)), np.random.randint(sy1, ey1, size=(num_samples, 1))])
        p0 = p0.transpose((1,2,0)).astype(np.float32)

        t3 = time.time()


        depth_obstacle = []
        hor_obstacle = []

        for i in range(self.num_samples_optical_flow):
            
        
            depth_obstacle.append(self.depth1[int(p0[i, 0 ][1]), int(p0[i, 0][0])])
        
        med_depth_obstacle = np.median(depth_obstacle) /1000


        for i in range(p0.shape[0]):
            depth = self.depth1[int(p0[i, 0 ][1]), int(p0[i, 0][0])] / 1000
            if np.abs(depth - med_depth_obstacle) <= 0.06:
                hor_obstacle.append(depth * (p0[i, 0][0]-self.image_width/2) /self.fx)

        med_hor_obstacle = np.median(hor_obstacle)

        return med_hor_obstacle, med_depth_obstacle

# Returns the X and Y positions wrt camera frame. X is in horizontal direction and Y is positive in depth 


    def log(self, time, x, y, relx, rely, absx, absy, velx, vely):
        with open(self.file_path, "a") as f:
            print("logging")

            l = [time, x, y, relx[0], rely[0], absx[0], absy[0], velx[0], vely[0], self.robot_vel_x, self.robot_vel_y]
            out_str = ','.join(str(i) for i in l)
            out_str += '\n'
            f.write(out_str)
            f.close()
    

    def get_obs_params(self):
        
        t = time.time()
        # dt = t - self.t_prev_ctrl
        # self.obs_x = self.obs_x  + self.obs_v_x * dt 
        # self.obs_y = self.obs_y  + self.obs_v_y * dt
        # self.t_prev_ctrl = t 
        print("----------------------------" )
        print()
        print("obs x y vx vy", self.obs_x, self.obs_y, self.obs_v_x, self.obs_v_y)
        print()
        print()
        print("-------------------------------")
        return (self.obs_x, self.obs_y, self.obs_v_x, self.obs_v_y)



if __name__ == "__main__":  

    rospy.init_node('Velocity_Estimator')
    vel_estimator = VelocityEstimator()
    controller = pid.Controller()
    rate = rospy.Rate(10)
    tick = 0
    
    while not rospy.is_shutdown():
        obs_x, obs_y, obs_v_x, obs_v_y = vel_estimator.get_obs_params()
        controller.publish(obs_x, obs_y, obs_v_x, obs_v_y, tick)

        tick += 1 
        if tick == 10:
            tick = 0

        rate.sleep()


