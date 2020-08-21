#!/usr/bin/env python

import rospy 
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import yaml
from sensor_msgs.msg import CameraInfo

def redad_yaml(yaml_fname):
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = "plumb_bob"
    return camera_info_msg
 
class ImageSplitter:
    def __init__(self,yaml_dirname, subsample_t = 10, subsample_space = 1, queue_size = 1):

        self.info_left =  redad_yaml(yaml_dirname+'/left.yaml')
        self.info_right =   redad_yaml(yaml_dirname+'/right.yaml')
        self.subsample_t = subsample_t
        self.count = 0
        self.subsample_space = subsample_space
        self.bridge = CvBridge()
        self.pub_l = rospy.Publisher('left/image_raw', Image, queue_size = 1)
        self.pub_r = rospy.Publisher('right/image_raw', Image, queue_size = 1)
        self.pub_info_left = rospy.Publisher("left/camera_info", CameraInfo, queue_size=10)
        self.pub_info_right = rospy.Publisher("right/camera_info", CameraInfo, queue_size=10)
        sub = rospy.Subscriber('image_raw', Image, self.callback, queue_size = queue_size)

    def callback(self, msg):
        try:
            self.count += 1
            #rospy.loginfo(self.count)
            if self.count == self.subsample_t:
                I = self.bridge.imgmsg_to_cv2(msg)
                cols = I.shape[1]
                R = I[:,:cols/2,:]
                L = I[:,cols/2:,:]
                if self.subsample_space > 1:
                    L = L[::self.subsample_space, ::self.subsample_space, :]
                    R = R[::self.subsample_space, ::self.subsample_space, :]
                msg_l = self.bridge.cv2_to_imgmsg(L, 'rgb8')
                msg_l.header = msg.header
                msg_l.header.frame_id = 'zed_left'
                msg_r = self.bridge.cv2_to_imgmsg(R, 'rgb8') 
                msg_r.header = msg.header
                msg_r.header.frame_id = 'zed_right'
                #rospy.loginfo('publish')        
                self.pub_l.publish(msg_l)
                self.pub_r.publish(msg_r)
                self.info_left.header = msg_l.header
                self.info_right.header = msg_r.header
                self.pub_info_left.publish(self.info_left)
                self.pub_info_right.publish(self.info_right)
                self.count = 0
        except  CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('zed_splitter', anonymous=True)

    filename = rospy.get_param('~filename', '')
    node = ImageSplitter(filename)
    rospy.spin()


