#!/usr/bin/env python
from __future__ import print_function
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 10)
    self.image_debug_pub = rospy.Publisher("debug_image",Image, queue_size = 10)
    self.image_debug_2_pub = rospy.Publisher("debug_image_2",Image, queue_size = 10)
    self.image_debug_3_pub = rospy.Publisher("debug_image_3",Image, queue_size = 10)
    self.image_marker_pub = rospy.Publisher("debug_marker",Marker, queue_size = 10)
    self.bridge = CvBridge()
    image_sub_left = Subscriber("left/image_raw",Image)
    image_sub_right = Subscriber("right/image_raw",Image)
    detect_sub = Subscriber("/detectnet/detections",Detection2DArray)
    self.ts = ApproximateTimeSynchronizer([image_sub_left, image_sub_right,              detect_sub],            
    queue_size = 10 , slop = 0.5)
    self.ts.registerCallback(self.callback)

  def callback(self,msg_left, msg_right,detection):
    rospy.loginfo(len(detection.detections))
    if len(detection.detections) >= 1:
        rospy.loginfo(detection.detections[0].bbox.center.x)
    try:
      L = self.bridge.imgmsg_to_cv2(msg_left, "bgr8")
      R = self.bridge.imgmsg_to_cv2(msg_right, "bgr8")
    except CvBridgeError as e:
      print(e)
    if len(detection.detections)>0:

    #COORDONNEES RECTANGLE LEFT

        i=0
        xp1L=int(detection.detections[i].bbox.center.x-(0.5*detection.detections[i].bbox.size_x))
        yp1L=int(detection.detections[i].bbox.center.y-(0.5*detection.detections[i].bbox.size_y))
        xp2L=int(detection.detections[i].bbox.center.x+(0.5*detection.detections[i].bbox.size_x))
        yp2L=int(detection.detections[i].bbox.center.y+(0.5*detection.detections[i].bbox.size_y))
      
      
       #CREATION DE CROP_LEF ET DU BANDEAU
 
        crop_lef = cv2.cvtColor(L[yp1L:yp2L,xp1L:xp2L,:], cv2.COLOR_BGR2GRAY)
        bandeau= cv2.cvtColor(R[yp1L:yp2L, :, :], cv2.COLOR_BGR2GRAY)
        res= cv2.matchTemplate(crop_lef, bandeau, cv2.TM_CCOEFF_NORMED)
        b, a, min_loc, max_loc = cv2.minMaxLoc(res)
        res = ((res - b)/ (a-b))*255
        rospy.loginfo(res.shape)
        rospy.loginfo(max_loc)
        #bandeau= R[yp1L:yp2L, :, :]
        #bandeau = cv2.circle(bandeau, (int(max_loc[0]+crop_lef.shape[1]/2), int(bandeau.shape[0]/2)), 5, (0, 255, 0) )


        #CALCUL PROFONDEUR
        #b= baseline= ecart entre les 2 cameras, f= distance focale, disparite= ecart entre x left et x right tout ca pour trouver z la profondeur
        fb= 80.42162
        f= 688.39679
        x_left = detection.detections[0].bbox.center.x
        x_right= max_loc[0]+crop_lef.shape[1]/2
        d= np.abs(x_left - x_right)
        z= (fb)/d
        rospy.loginfo("z = %f"%(z))

        #COORDONNEES RECTANGLE RIGHT
        xp1R=int(x_right-(0.5*detection.detections[i].bbox.size_x))
        yp1R=int(detection.detections[i].bbox.center.y-(0.5*detection.detections[i].bbox.size_y))
        xp2R=int(x_right+(0.5*detection.detections[i].bbox.size_x))
        yp2R=int(detection.detections[i].bbox.center.y+(0.5*detection.detections[i].bbox.size_y))       

        #AFFICHAGE RECTANGLES
        L=print_rectangle(xp1L,xp2L,yp1L,yp2L, L)
        R=print_rectangle(xp1R,xp2R,yp1R,yp2R, R)
        
        #AFFICHAGE DES CYLINDRES DANS RVIZ
        fx= 688.39679
        fy= 688.39679
        cx= 655.16699
        cy= 369.89593
        u= (detection.detections[i].bbox.center.x)
        v= (detection.detections[i].bbox.center.y)
        x= z*(u-cx)/fx
        y= z*(v-cy)/fy
        marker_= Marker()
        marker_.header.frame_id= "base_link"
        marker_.type= marker_.CYLINDER
        marker_.pose.position.x= z
        marker_.pose.position.y= -x
        marker_.pose.position.z= -y
        marker_.pose.orientation.w =0 #0.7071
        marker_.pose.orientation.x =0
        marker_.pose.orientation.y =0#0.7071
        marker_.pose.orientation.z =0
        marker_.scale.x= 0.5
        marker_.scale.y= 0.5
        marker_.scale.z= 2
        marker_.color.r=1
        marker_.color.a=1
        #marker_.lifetime= rospy.Duration.from_sec(lifetime_)
        #marker_array_.markers.append(marker_)

    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      #self.image_debug_pub.publish(self.bridge.cv2_to_imgmsg(crop_lef, "mono8"))
      self.image_debug_2_pub.publish(self.bridge.cv2_to_imgmsg(L, "bgr8"))
      self.image_debug_3_pub.publish(self.bridge.cv2_to_imgmsg(R, "bgr8"))
      self.image_marker_pub.publish(marker_)
    
    except CvBridgeError as e:
      print(e)

def print_rectangle(x1,x2,y1,y2, cv_image):
 
    cv_image= cv2.rectangle(cv_image, (int(x1),int(y1)), (int(x2),int(y2)), (255,0,0), 2)
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
