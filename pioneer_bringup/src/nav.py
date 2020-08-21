#!/usr/bin/env python
from __future__ import print_function
import numpy as np 
import yaml
from nav_msgs.msg import OccupancyGrid
import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
from geometry_msgs.msg import PoseStamped
import math


def seek_spot(I, i, j, candidats):	
		if I[i][j+1] == -1:
			candidats.append((i, j+1))
		if I[i][j-1]  == -1:
			candidats.append((i, j-1))
		if I[i-1][j] == -1:
			candidats.append((i-1, j))
		if I[i+1][j] == -1:
			candidats.append((i+1, j))
		if I[i-1][j+1] == -1:
			candidats.append((i-1, j+1))
		if I[i-1][j-1] == -1:
			candidats.append((i-1, j-1))
		if I[i+1][j-1] == -1:
			candidats.append((i+1, j-1))
		if I[i+1][j+1] == -1:
			candidats.append((i+1, j+1))



class navigation:
	def __init__(self):
		self.sub_grid= rospy.Subscriber("/map", OccupancyGrid, self.callback, queue_size= 10 )
		self.pub_img= rospy.Publisher("/image_nav", Image, queue_size= 10)
		self.bridge= CvBridge()
		self.listener= tf.TransformListener()
		self.pub_point= rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size= 10)

	def callback(self, msg):
		col =msg.info.width
		row =msg.info.height
		#data est sous forme de liste donc on la convertit en matrice
		data = np.array(msg.data, dtype=np.int8)
		img = np.zeros((row, col), dtype=np.uint8)
		I = data.reshape((row, col))
		
		Unexp = np.zeros((row, col), dtype=np.uint8)
		Libre =  np.zeros((row, col), dtype=np.uint8)
		Occ = np.zeros((row, col), dtype=np.uint8)
		Occ[I == 100] = 1
		Unexp[I == -1] = 1
		Libre[I == 0] = 1

		img[I == -1] = 128
		img[I == 0] = 255
		img[I == 100] = 0

		kernel = np.array([[0,1,0],[1,1,1],[0,1,0]], dtype= np.uint8)

		Libre = cv2.erode(Libre, np.ones((3,3), dtype= np.uint8))
		Libre = cv2.dilate(Libre, np.ones((3,3), dtype= np.uint8))
		dilation = cv2.dilate(Unexp,kernel,iterations = 1)
		good = np.logical_and(Libre == 1 ,dilation == Libre)
		idx = np.where(good)
		print(idx)
		rospy.loginfo("index dilatation")
		rospy.loginfo(len(idx[0]))
		
		#affichage des points de idx
		"""for i in range(0, len(idx[0])):
			img[idx[0][i], idx[1][i]]= 50"""
		
		#recentrer image
		xmin = np.min(idx[1])
		ymin = np.min(idx[0])
		xmax = np.max(idx[1])
		ymax = np.max(idx[0])
		
		#rospy.loginfo("img publiee")

		#position du robot
		try:
			(trans, rot)= self.listener.lookupTransform("/map","/base_link", rospy.Time(0))
			rospy.loginfo("translation:")
			rospy.loginfo(trans)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo('tf fail')
		#position en pixel
		resol= msg.info.resolution
		rospy.loginfo("resolution")
		rospy.loginfo(resol)
		mx= msg.info.origin.position.x
		my= msg.info.origin.position.y
		rx= trans[0]
		ry= trans[1]
		pixel_pos_x= (rx-mx)/resol
		pixel_pos_y= (ry-my)/resol
		img[int(pixel_pos_y)][int(pixel_pos_x)]= 160
		
		

		#calcul meilleure distance
		x1= pixel_pos_x
		y1= pixel_pos_y
		dmin= 1000000000
		for i in range(0, len(idx[0])):
			x2= idx[1][i]
			y2= idx[0][i]
			d= math.sqrt((x2-x1)**2  + (y2-y1)**2)
			if d < dmin:
				dmin= d
				res= i
		
		
	
		#trouver les regions ou se rendre
		binaire= np.zeros(img.shape, dtype=np.uint8)
		binaire[good] = 1
		ret, labels= cv2.connectedComponents(binaire)
		rospy.loginfo("ret")
		rospy.loginfo(ret)
		largest_region= 0 
		areas= []
		for i in range(0, ret):
			res= np.sum(labels == i)
			areas.append(res)
		sorted_idx= np.argsort(areas)
		i_largest= sorted_idx[len(sorted_idx)-2]
		#mettre en couleur la plus grosse zone inexploree
		img= cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
		goal_idx = np.where(labels==i_largest)
		for i in range(len(goal_idx[0])):
			img[goal_idx[0][i],goal_idx[1][i],:] = [255,0,0]
	
		cv2.circle(img,(idx[1][res], idx[0][res]), 5, [0,0,255] )
		cv2.circle(img,(int(pixel_pos_x), int(pixel_pos_y)), 5, [0,255,0] )
		
	
		#determiner le point ou se rendre (calculer le point median)
		sum_x= 0
		for i in range (0, len(goal_idx[1])):
			sum_x = sum_x + goal_idx[1][i]
		mean_x= int(sum_x / len(goal_idx[1]))
		sum_y= 0
		for i in range (0, len(goal_idx[0])):
			sum_y= sum_y + goal_idx[0][i]
		mean_y= int(sum_y / len(goal_idx[0]))
		cv2.circle(img,(mean_x, mean_y), 5, [255,0,0] )
		img= img[ymin:ymax, xmin:xmax, :]
		self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))



		msg_pts = PoseStamped()
		msg_pts.pose.position.x = mean_x * resol + mx
		msg_pts.pose.position.y = mean_y * resol + my
		msg_pts.pose.orientation.w= 1 
		msg_pts.header.frame_id = "map"
		self.pub_point.publish(msg_pts)


	
		rospy.loginfo("finito")
		"""nb = []
		for i in range (0, row):	
			for j in range (0, col):
				if I[i][j] not in nb:
					nb.append(I[i][j])
		#rospy.loginfo("res=")
		#rospy.loginfo(nb)	
		#rospy.loginfo(I)"""
	
		#recherche des endroits a explorer
		"""candidats= []
		for i in range (0, row):	
			for j in range (0, col):
		idx= np.where(I==0)
		rospy.loginfo(len(idx[0]))
		for i, j in zip(idx[0], idx[1]):
				seek_spot(I, i, j, candidats)
		#rospy.loginfo(len(candidats))"""
			
		
		
		
	

def main(args):
	rospy.init_node('navigation', anonymous=True)
	ic= navigation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

