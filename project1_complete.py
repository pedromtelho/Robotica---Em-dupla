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
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import cormodule
from math import pi

centro = (0,0)
bridge = CvBridge()
cv_image = None
dif = None
bump = None
laser_dist = []
laser_dist_back = []
laser_dist_right = []
laser_dist_left = []
margem = 30
atraso = 1.5E9
area_blue = None
media_blue = (0,0)
margem_vertical = 60
angle_for_list = None
near_blue = False

def cam_data(imagem):
	global centro
	global bridge
	global cv_image
	global dif
	global media_blue	
	
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		centro_red, centro, media_blue, area_blue =  cormodule.identifica_cor(cv_image, margem, margem_vertical)
		dif = centro_red[0]-centro[1]

	except CvBridgeError as e:
		print('ex', e)


def bumper_data(dado):
	global bump

	bump = dado.data


def laser_data(dado):
	global laser_dist
	global laser_dist_back
	global laser_dist_right
	global laser_dist_left
	global angle_for_list
	global near_blue

	dist_list = dado.ranges

	laser_dist.append(round(dist_list[1],2))
	laser_dist.append(round(dist_list[0],2))
	laser_dist.append(round(dist_list[-1],2))


	laser_dist_left.append(round(dist_list[89],2))
	laser_dist_left.append(round(dist_list[90],2))
	laser_dist_left.append(round(dist_list[91],2))


	laser_dist_back.append(round(dist_list[179],2))
	laser_dist_back.append(round(dist_list[180],2))
	laser_dist_back.append(round(dist_list[181],2))


	laser_dist_right.append(round(dist_list[269],2))
	laser_dist_right.append(round(dist_list[270],2))
	laser_dist_right.append(round(dist_list[271],2))

	if angle_for_list is not None:
		print(dist_list[angle_for_list])
		if dist_list[angle_for_list] < 0.30 and dist_list[angle_for_list] != 0:
			near_blue = True



if __name__=="__main__":

		rospy.init_node("follow_red")

		vel_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
		see_image = rospy.Subscriber("/kamera", CompressedImage, cam_data)
		see_bumper = rospy.Subscriber("/bumper", UInt8, bumper_data)
		see_laser = rospy.Subscriber("/scan", LaserScan, laser_data)

		false_positive = True
# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera

		laser_list_reset_timer = 0


		while not rospy.is_shutdown():
			vel_foward = Twist(Vector3(0.9,0,0), Vector3(0,0,0))
			vel_foward_fast = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			vel_tras = Twist(Vector3(-0.13,0,0), Vector3(0,0,0))
			vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
			vel_left_following = Twist(Vector3(0.1,0,0), Vector3(0,0,0.2))
			vel_right_following = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.2))
			vel_right_avoid = Twist(Vector3(0,0,0), Vector3(0,0,-pi/8))
			vel_left_avoid = Twist(Vector3(0,0,0), Vector3(0,0,pi/8))
			vel_near_blue = Twist(Vector3(0,0,0), Vector3(0,0,3))

			if not false_positive:
				if dif < -margem:
					vel_saida.publish(vel_left_following)
				
				elif dif > margem:
					vel_saida.publish(vel_right_following)
				
				else:
					vel_saida.publish(vel_foward)

				if bump == 1:
					bump = None
					vel_saida.publish(vel_tras)
					rospy.sleep(1)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_tras)
					rospy.sleep(1)
					vel_saida.publish(vel_left_avoid)
					rospy.sleep(4)

				if bump == 2:
					bump = None
					vel_saida.publish(vel_tras)
					rospy.sleep(1)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_tras)
					rospy.sleep(1)
					vel_saida.publish(vel_left_avoid)
					rospy.sleep(4)

				if bump == 3 :
					bump = None
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)

				if bump == 4 :
					bump = None
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)

				if near_blue:
					vel_saida.publish(vel_near_blue)
					rospy.sleep(4)
					near_blue = False
					vel_saida.publish(vel_parado)
					rospy.sleep(0.2)

				if media_blue[1] > (centro[0] - 2*margem_vertical) and media_blue[1] < (centro[0] - 40):
					angle = media_blue[0]/10
					angle_for_list = -(angle - 32)

				for i in laser_dist:
					if i < 0.4 and i != 0:
						print('frente')
						laser_dist = []
						vel_saida.publish(vel_right_avoid)
						rospy.sleep(1)
						vel_saida.publish(vel_parado)
						break

				for i in laser_dist_left:
					if i < 0.3 and i != 0:
						print('left')
						laser_dist_left = []
						vel_saida.publish(vel_left_avoid)
						rospy.sleep(1)
						vel_saida.publish(vel_parado)
						break

				for i in laser_dist_right:
					if i < 0.3 and i != 0:
						print('right')
						laser_dist_right = []
						vel_saida.publish(vel_right_avoid)
						rospy.sleep(1)
						vel_saida.publish(vel_parado)
						break

				for i in laser_dist_back:
					if i < 0.15 and i != 0:
						print('back')
						laser_dist_back = []
						vel_saida.publish(vel_foward_fast)
						rospy.sleep(2)
						vel_saida.publish(vel_parado)
						break

				rospy.sleep(0.1)


			if false_positive and bump != 0:
				false_positive = False
				bump = None

			laser_list_reset_timer += 1
			if laser_list_reset_timer == 5:
				laser_dist = []
				laser_dist_back = []
				laser_dist_right = []
				laser_dist_left = []
				laser_list_reset_timer = 0
			if laser_list_reset_timer == 2 or laser_list_reset_timer == 4:
				bump = None


			angle_for_list = None
			angle = None



			
