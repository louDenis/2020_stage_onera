#!/usr/bin/env python
from __future__ import print_function
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import Detection2DArray

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 10)

    self.bridge = CvBridge()
    image_sub = Subscriber("/front_camera/image_raw",Image)
    detect_sub = Subscriber("/detectnet/detections",Detection2DArray)
    self.ts = ApproximateTimeSynchronizer([image_sub, detect_sub], queue_size = 10 , slop = 0.5)
    self.ts.registerCallback(self.callback)

  def callback(self,data, detection):
    rospy.loginfo(len(detection.detections))
    if len(detection.detections) >= 1:
        rospy.loginfo(detection.detections[0].bbox.center.x)
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.rectangle(cv_image, (50,50), (65,65), (255,0,0), 2)
      cv_image= print_rectangle(self, data, detection, cv_image)
      #la partie a custom
      #cv2.circle(cv_image, (50,50), 10, 255)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def print_rectangle(self, data, detection, cv_image):
    for i in range (0,len(detection.detections)):
        xp1=detection.detections[i].bbox.center.x-(0.5*detection.detections[i].bbox.size_x)
        yp1=detection.detections[i].bbox.center.y-(0.5*detection.detections[i].bbox.size_y)
        xp2=detection.detections[i].bbox.center.x+(0.5*detection.detections[i].bbox.size_x)
        yp2=detection.detections[i].bbox.center.y+(0.5*detection.detections[i].bbox.size_y)
        cv_image= cv2.rectangle(cv_image, (int(xp1),int(yp1)), (int(xp2),int(yp2)), (255,0,0), 2)
    return cv_image



def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
