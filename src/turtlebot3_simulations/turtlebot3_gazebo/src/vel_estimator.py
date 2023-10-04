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

class VelocityEstimator():
    def __init__(self):
        
        weights_path = "/home/stochlab/torch_models/yolov5/best.pt"
        model_path = "/home/stochlab/torch_models"
        self.model = torch.hub.load(model_path, 'custom', path=weights_path, source='local', force_reload=True)

        rospy.init_node('Velocity_Estimator')

        self.pub = rospy.Publisher('/obstacle_vel', Odometry, queue_size=10)
        self.rgb_sub = rospy.Subscriber('/tb3_0/d435/color/image_rect', Image, self.img_callback)
        self.depth_sub = rospy.Subscriber('/tb3_0/d435/depth/image_rect', Image, self.img_callback)
        
        self.depth1 = None 
        self.depth2 = None 
        self.rgb1 = None
        self.rgb2 = None 
        self.bridge = CvBridge()

        pub_flag = False

        self.t_depth1, self.t_depth2, self.t_rgb1, self.t_rgb2 = 0,0,0,0


    def img_callback(self, msg):
        try:
            # print(msg.header.stamp.secs,msg.header.stamp.nsecs )
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.depth = cv2_img

            
            # cv2.imwrite(f'/home/stochlab/turtlebot3_dataset/scene3/image/image_{img_no}.jpeg', cv2_img)
            # with open('/home/stochlab/turtlebot3_dataset/scene3/image/timestamps.txt', 'a') as f:
            #     time_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs /(10**9) 
            #     string = str(img_no)+ " " +  str(time_stamp) + '\n'
            #     f.write(string)
            #     f.close()
  
        except CvBridgeError:
            print("cvbridge error rgb")
    def depth_callback(self, msg):

        msg.encoding = 'mono16'
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "mono16")
            self.depth = cv2_img
            # print(msg.header.stamp.secs,msg.header.stamp.nsecs )

            # np.save(f'/home/stochlab/turtlebot3_dataset/scene3/depth/image_{img_no}.npy',cv2_img)
            # with open('/home/stochlab/turtlebot3_dataset/scene3/depth/timestamps.txt', 'a') as f:
            #     time_stamp = msg.header.stamp.secs + msg.header.stamp.nsecs /(10**9)
            #     string = str(img_no2)+ " " + str(time_stamp) + '\n'  
            #     f.write(string)
            #     f.close()
            
        except CvBridgeError as e:
            print(e)



    def compute_vel(self):
        
        # Todo save 1 and 2 of the images + synchronize
        
        results = self.model([self.rgb1, self.rgb2])
        df1, df2 = results.pandas().xyxy[0]
        sx1, sy1, ex1, ey1  = df1.iloc[0, 0:4]
        sx2, sy2, ex2, ey2 = df2.iloc[0, 0:4]

        sx = int(min(sx1, sx2))
        sy = int(min(sy1, sy2))
        ex = int(max(ex1,ex2))
        ey = int(max(ey1, ey2))
        
        prev_gray = cv.cvtColor(self.image1, cv.COLOR_BGR2GRAY)
        gray = cv.cvtColor(self.image2, cv.COLOR_BGR2GRAY)


        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize = (45, 45),
                        maxLevel = 4,
                        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT,
                                    10, 0.03))


        num_samples = 4000
        p0 = np.array([np.random.randint(sx, ex, size=(num_samples, 1)), np.random.randint(sy, ey, size=(num_samples, 1))])
        p0 = p0.transpose((1,2,0)).astype(np.float32)


        p1, st, err = cv.calcOpticalFlowPyrLK(prev_gray,
                                                gray,
                                                p0, None,
                                                **lk_params)

        good_new = p1[st == 1]
        good_old = p0[st == 1]


        rgb1 = np.zeros(shape=(self.image1.shape))
        rgb2 = np.zeros(shape=(self.image1.shape))

        deltax = []
        deltay = []

        for i, (new, old) in enumerate(zip(good_new,
                                            good_old)):

            col,row  = old
            row, col = int(row), int(col)

            colnew,rownew = new
            rownew, colnew = int(rownew), int(colnew)

            fx = colnew - col
            fy = rownew - row

            # fx, fy = flowx[row, col], flowy[row,col]

            if col < sx1 or col > ex1 or row < sy1 or row > ey1:
                continue

            if row + fy >= min(480, ey2) or col + fx >=640 or row + fy < 0 or col + fx < max(0, sx2):# or max(abs(fx), abs(fy)) > 60 :
                continue
            # if col < sx1 or col >ex1 or row < sy1 or row > ey1:
            #   continue

            if rownew >= 480 or colnew >=640 or rownew < 0 or colnew <0: # or max(abs(rownew - row), abs(colnew - col)) > 60 :
                continue
            if np.sqrt(fx**2 + fy**2) > 1 and self.depth1[row, col] < 9000 and self.depth2[int(row + fy), int(col + fx) ] < 9000:
                rgb1[row, col] = self.image1[row, col]
                rgb2[int(row+ fy), int(col+ fx)] = self.image2[int(row + fy), int(col + fx)]
                d1 = self.depth1[row, col] /1000
                d2 = self.depth2[int(row+ fy), int(col+ fx)] / 1000


            x1 = d1 * (col-320) /462
            x2 = d2 * (col + fx - 320) /462

            deltax.append(x2 - x1)
            deltay.append(d2-d1)
        
        
        dt = (self.t_depth2 + self.t_rgb2 - self.t_depth1 - self.t_rgb1)/2

        velx = np.median(deltax) / dt 
        vely = np.median(deltay) / dt
        

if __name__ == "__main__":

    est_node = VelocityEstimator()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        est_node.publish()
        rate.sleep()




