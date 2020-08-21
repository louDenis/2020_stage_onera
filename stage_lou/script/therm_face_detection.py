#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from sensor_msgs.msg import Image 
from std_msgs.msg import Float64 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_therm:
    def __init__(self, fname):
        rospy.loginfo(fname)
        self.face_cascade = cv2.CascadeClassifier(fname)
        self.pub_temperature= rospy.Publisher("temperature", Float64, queue_size = 10)
        self.image_pub = rospy.Publisher("therm_image",Image, queue_size = 10)
        self.image_debug = rospy.Publisher("debug2_image",Image, queue_size = 10)
        
        image_sub = rospy.Subscriber("/openmv/image_raw", Image, self.callback2, queue_size= 10)
        self.bridge = CvBridge()
    
    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        temperature = (gray.astype(np.float32) /255) * (50-20) + 20
        gray = (255*(temperature- 35) /(40-35)).clip(0,255).astype(np.uint8)
        faces= self.face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))
            self.image_debug.publish(self.bridge.cv2_to_imgmsg(gray,"mono8"))
        except CvBridgeError as e:
            print(e)
    def callback2(self, data):

        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #gray = cv2.equalizeHist(gray)
        #faces= self.face_cascade.detectMultiScale(gray, 1.1, 4)
        temperature = (gray.astype(np.float32) /255) * (50-20) + 20
        binary = np.zeros(temperature.shape, dtype= np.uint8)
        binary[temperature >37] = 255
        for i in range(2):
            eroded= cv2.erode(binary, (16,16))
            binary= cv2.dilate(eroded, (16,16))
            
        
        ret, labels = cv2.connectedComponents(binary)
        for label in range(1,ret):
            mask = np.array(labels, dtype=np.uint8)
            mask[labels == label] = 25
           
        
        rospy.loginfo(ret)
        # getting ROIs with findContours
        #contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        #for cnt in contours:
        #    (x,y,w,h) = cv2.boundingRect(cnt)
        #    ROI = image[y:y+h,x:x+w]
        itete = 1
        selection_tete = (labels == itete).astype(np.uint8)
        print(selection_tete)
        temp= np.sum(temperature*selection_tete)/ np.sum(selection_tete)
        rospy.loginfo("temp = %f"%(temp                                         ))
        (x,y,w,h) = cv2.boundingRect(selection_tete)
        #for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))
            self.image_debug.publish(self.bridge.cv2_to_imgmsg(eroded,"mono8"))
            self.pub_temperature.publish(temp)
        except CvBridgeError as e:
            print(e)


def main(args):
  rospy.init_node('therm_detect', anonymous=True)
  filename = rospy.get_param('~filename', '')
  ic = image_therm(filename)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
