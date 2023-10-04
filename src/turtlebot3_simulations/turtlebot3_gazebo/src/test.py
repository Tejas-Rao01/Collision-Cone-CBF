#!/usr/bin/env python3

import rospy 
from std_msgs.msg import String

class simple():

    def __init__(self) -> None:
        
        rospy.init_node('tesst')
        self.pub = rospy.Publisher('/blah', String, queue_size=1)
        self.sub = rospy.Subscriber('/blah', String, self.callback, queue_size =1 )
        self.tick = 1
    
    def callback(self,msg):
        print("callback: " ,msg.data)


    def publish(self):
        msg = String()
        msg.data = f"test{self.tick}"
        self.pub.publish(msg)
        self.tick += 1

if __name__ =="__main__":

    node = simple()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()
