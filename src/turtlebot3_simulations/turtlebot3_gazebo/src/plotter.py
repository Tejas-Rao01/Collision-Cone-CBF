#!/usr/bin/env python3
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x=[]
robot_position_y=[]
robot_orientation = []


target_position_x = []
target_position_y = []

#defining two axes and figures for plotting 
fig, ax1 = plt.subplots()


X_cor = [0.0, 0.0]
Y_cor = [0.0, 0.0]



ax1.set(xlim=(-10,10), ylim=(-10,10))
r_traj, = ax1.plot([0, 0], [0, 0], 'blue')
t_traj, = ax1.plot([0, 0], [0, 0], 'red')

heading, = ax1.plot([0, 0], [0, 0], 'black')

robotX = 0 
robotY = -1.5
robotTheta = 0.5


plt.axis('equal')
target_plt = ax1.scatter(0, 0,s=30,marker='^',c='r')
robot_plt = ax1.scatter(0, 0, s=40, marker= 'x' , c = 'b')

    

    
def target_callback(data):
    

    global t_traj
    global target_plt
    global target_position_x
    global target_position_y


    x = data.pose.pose.position.x 
    y = data.pose.pose.position.y
    print(f'x, y : {x}, {y}')
        
    target_position_x.append(x)
    target_position_y.append(y)
    
    t_traj.set_data(target_position_x, target_position_y)
    
    target_plt.set_offsets([x, y])
    
    time.sleep(0.4)
    
    print('plotting target')

def robot_callback(data):
    

    global r_traj
    global heading
    global robot_plt
    global robot_position_y
    global robot_position_x

    x = data.pose.pose.position.x 
    y = data.pose.pose.position.y
    print(f'x, y : {x}, {y}')
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    theta = yaw

    robot_position_x.append(x)
    robot_position_y.append(y)
    
    print(f"robot pos x, {robot_position_x}")
    r_traj.set_data(robot_position_x, robot_position_y)
    
    
    robot_plt.set_offsets([x, y])


    x1, y1 = x + 0.5 * np.cos(theta) , y + 0.5 * np.sin(theta)
    heading.set_xdata([x, x1])
    heading.set_ydata([y, y1])
    time.sleep(0.1)
    
    print('plotting robot')




if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)
    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/targetPose", Odometry, target_callback,queue_size=1)
    # rospy.Subscriber("/odom", Odometry,  robot_callback, queue_size=1)
    plt.xlim(-10, 10)
    plt.ylim(-10,10)
    plt.axis('equal')
    plt.show(block=True)
    print('blah')
    rospy.spin()
        
        
       