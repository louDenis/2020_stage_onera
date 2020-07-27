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
