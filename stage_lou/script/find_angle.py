#!/usr/bin/env python
from __future__ import print_function
import numpy as np 
import yaml
from vision_msgs.msg import Detection2DArray
import sys
import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist

class calibration:
    def __init__(self):
        self.is_init= False
        self.pub_vel= rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.sub_calib= rospy.Subscriber("/front_camera/camera_info",CameraInfo, self.callback)
        self.sub_detection= rospy.Subscriber("/detectnet/detections",Detection2DArray, self.callback_detect )
    
    def callback(self, calib):
        self.cx= calib.K[2]
        self.fx= calib.K[0]
        self.is_init= True

    def callback_detect(self, detection):
        if self.is_init:
            
            
            angle_min=1000
            for i in range(0, len(detection.detections)):
                centre=detection.detections[i].bbox.center.x
                deviation= centre-self.cx
                angle= np.rad2deg(np.arctan2(deviation,self.fx))
                if np.abs(angle) < angle_min:
                    angle_min=angle
            self.publish_cmd(angle_min)

            rospy.loginfo("angle = %f"%(angle_min))
            
    def publish_cmd(self, angle):
            vel = Twist()
            if np.abs(angle) > 10:
                if angle > 0: 
                   vel.angular.z = -0.2
                else:
                   vel.angular.z = 0.2
            self.pub_vel.publish(vel)



  
def main(args):
    rospy.init_node('calibration', anonymous=True)
    ic = calibration()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
