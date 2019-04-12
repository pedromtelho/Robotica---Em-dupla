#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from math import pi

centro = None
bridge = CvBridge()
cv_image = None
dif = None
margem = 30


def cam_data(imagem):
	global centro
	global bridge
	global cv_image
	global dif
	cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	media, centro, area =  cormodule.identifica_cor(cv_image, margem)
	# print(media[0]-centro[0])
	dif = media[0]-centro[1]

	


if __name__=="__main__":

		rospy.init_node("follow_red")

		vel_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
		recebe_scan = rospy.Subscriber("/kamera", CompressedImage, cam_data)

			# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
		while not rospy.is_shutdown():
			vel_frente = Twist(Vector3(0.13,0,0), Vector3(0,0,0))
			vel_traz = Twist(Vector3(-0.13,0,0), Vector3(0,0,0))
			vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
			vel_left = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.3))
			vel_right = Twist(Vector3(0.1,0,0), Vector3(0,0,0.3))

			if dif < -margem:
				print('right')
				vel_saida.publish(vel_right)

			elif dif > margem:
				print('left')
				vel_saida.publish(vel_left)

			else:
				print('foward')
				vel_saida.publish(vel_frente)
			

			rospy.sleep(0.5)



			