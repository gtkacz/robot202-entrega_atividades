#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import sys

id_to_find=11
marker_size=25

calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

font = cv2.FONT_HERSHEY_PLAIN

bridge = CvBridge() #converte a msg do ROS para OpenCV
cv_image = None
scan_dist = 0

def scaneou(dado):
	global scan_dist 
	scan_dist = dado.ranges[0]*100
	return scan_dist


def idTag(cv_image):
	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	if ids is not None:
		ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
		rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
		aruco.drawDetectedMarkers(cv_image, corners, ids) 
		aruco.drawAxis(cv_image, camera_matrix, camera_distortion, rvec, tvec, 1)

		str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
		cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)
		cv2.line(cv_image, (cv_image.shape[1]/2,cv_image.shape[0]/2), ((cv_image.shape[1]/2 + 50),(cv_image.shape[0]/2)), (0,0,255), 5) 
		cv2.line(cv_image, (cv_image.shape[1]/2,cv_image.shape[0]/2), (cv_image.shape[1]/2,(cv_image.shape[0]/2 + 50)), (0,255,0), 5) 			
		m = marker_size/2
		pts = np.float32([[-m,m,m], [-m,-m,m], [m,-m,m], [m,m,m],[-m,m,0], [-m,-m,0], [m,-m,0], [m,m,0]])
		imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, camera_distortion)
		imgpts = np.int32(imgpts).reshape(-1,2)
		cv_image = cv2.drawContours(cv_image, [imgpts[:4]],-1,(0,0,255),4)
		for i,j in zip(range(4),range(4,8)):
            cv_image = cv2.line(cv_image, tuple(imgpts[i]), tuple(imgpts[j]),(0,0,255),4)
		cv_image = cv2.drawContours(cv_image, [imgpts[4:]],-1,(0,0,255),4)
		ids = ids.tolist()
	if ids is None:
		ids = [[0]]	
		
	return ids