#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from sensor_msgs.msg import Image

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/front_camera/image_raw",Image, queue_size = 10)
    self.count=0
    self.sub=10
    self.image_sub = rospy.Subscriber("/front_camera/image_orig",Image,self.callback)


  def callback(self,data):
    self.count+=1
    if self.count%self.sub == 0:
        self.image_pub.publish(data)




def main(args):
  rospy.init_node('temp_sub_sampling', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 

if __name__ == '__main__':
    main(sys.argv)
