#!/usr/bin/env python
from __future__ import print_function
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class vel_synchronizer:
    
    def __init__(self):
        self.activation_auto_sub= Subscriber("/activation", Bool)
        self.cmd_vel_auto=Subscriber("/cmd_vel_auto", Twist)
        self.cmd_vel= rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ts = ApproximateTimeSynchronizer([self.activation_auto_sub, self.cmd_vel_auto], queue_size =10 , slop =0.5, allow_headerless=True)
        self.ts.registerCallback(self.callback)
    
    def callback(self, a , cmd):
        self.cmd_vel.publish(cmd)
 

def main(args):
  rospy.init_node('vel_synchronizer', anonymous=True)
  ic = vel_synchronizer()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
