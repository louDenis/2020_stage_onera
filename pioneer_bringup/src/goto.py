#!/usr/bin/env python
from __future__ import print_function
import numpy as np 
import yaml
import sys
import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class goto:
    def __init__(self):
    
        self.sub_marker= rospy.Subscriber("debug_marker", Marker, self.publish_cmd, queue_size=10)
        self.pub_vel= rospy.Publisher("/cmd_vel_auto", Twist, queue_size = 10)

    def publish_cmd(self, marker):
            vel_x = Twist()
            if marker.pose.position.x > 1.40:
                vel_x.linear.x = +0.05
            if marker.pose.position.x < 1.10:   
                vel_x.linear.x = -0.05
            self.pub_vel.publish(vel_x)



  
def main(args):
    rospy.init_node('goto', anonymous=True)
    ic = goto()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
