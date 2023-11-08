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
import pid
import pandas as pd


bridge = CvBridge()
t = time.time()
save_path = "/home/focaslab/dataset/"
img_no = 0 
def cb(rgb):
    global img_no
    global bridge
    global t
    global save_path
    rgb_time= rgb.header.stamp.secs + rgb.header.stamp.nsecs / 10**9
    # depth_time = depth.header.stamp.secs + depth.header.stamp.nsecs / 10**9

    # Obtain rgb image 
    try:
        # Convert your ROS Image message to OpenCV2
        rgb_img = bridge.imgmsg_to_cv2(rgb, 'bgr8')
        
    except CvBridgeError as e:
        print("cvbridge error rgb", e)
        pass    
    
    t2  = time.time()
    if (t2 -t > 0.5):
        path = save_path + f"{img_no}.png"
        cv.imwrite(path , rgb_img)



if __name__ == "__main__":
    rospy.init_node("saver")
    sub = rospy.Subscriber("/camera/color/image_raw", callback=cb, queue_size=3)
    rospy.spin()

