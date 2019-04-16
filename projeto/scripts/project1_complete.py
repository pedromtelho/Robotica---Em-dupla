#!/usr/bin/env python
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
from math import pi
import visao_module
import cormodule
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import numpy as np
import argparse
import cv2


# off = True
off = False


centro = (0,0)
bridge = CvBridge()
cv_image = None
dif = None
bump = None
laser_dist = []
laser_dist_back = []
laser_dist_right = []
laser_dist_left = []
margem = 100
atraso = 1.5E9
area_blue = None
media_blue = (0,0)
margem_vertical = 60
angle_for_list = None
near_blue = False
see_bottle = False
tracking_bottle = False
consecutive_frame = 0
xy0 = (0,0)
xy1 = (0,0)
fps = None
initBB = ((0,0),(0,0))
v = 0

coisa = raw_input("escolha o objeto: ")

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

# extract the OpenCV version info
(major, minor) = cv2.__version__.split(".")[:2]

# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())

# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:
	# initialize a dictionary that maps strings to their corresponding
	# OpenCV object tracker implementations
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}

	# grab the appropriate object tracker using our dictionary of
	# OpenCV object tracker objects
	tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()


def cam_data(imagem):
	global centro
	global bridge
	global cv_image
	global dif
	global media_blue
	global see_bottle
	global tracking_bottle
	global consecutive_frame
	global xy1
	global xy0
	global fps
	global initBB

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		centro_red, centro, media_blue, area_blue =  cormodule.identifica_cor(cv_image, margem, margem_vertical)
		# print(consecutive_frame)
		if not tracking_bottle:
			centro, cv_image, results =  visao_module.processa(cv_image)
			for e in results:
				if e[0]== coisa:
					xy0 = e[2]
					xy1 = e[3]
					x0, y0 = xy0
					x1, y1 = xy1
					dif = None
					saw_bottle = True
					consecutive_frame += 1
			if consecutive_frame >= 10:
				tracking_bottle = True
				initBB = (x0,y0,x1-x0,y1-y0)
		if tracking_bottle:
			tracker.init(cv_image, initBB)
			fps = FPS().start()
			(success, box) = tracker.update(cv_image)
			if success:
				(x, y, w, h) = [int(v) for v in box]
				cv2.rectangle(cv_image, (x, y), (x + w, y + h),
					(0, 255, 0), 2)
				centro_bottle = (x+w/2, y+h/2)
				dif = centro_bottle[0]-centro[1]
			if not success:
				tracking_bottle = False
				consecutive_frame = 0
			fps.update()
			fps.stop()

		cv2.imshow("frame", cv_image)



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
	try:
		if angle_for_list is not None:
			print(dist_list[angle_for_list])
			print(angle_for_list)
			print('')
			if dist_list[angle_for_list] < 1 and dist_list[angle_for_list] != 0:
				near_blue = True
	except:''


if __name__=="__main__":

		rospy.init_node("project1_complete")

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
			vel_near_blue = Twist(Vector3(0,0,0), Vector3(0,0,pi/2))
			vel_left_avoid_tapao = Twist(Vector3(0.2,0,0), Vector3(0,0,-1))
			vel_right_avoid_tapao = Twist(Vector3(0.2,0,0), Vector3(0,0,1))


			# if cv2.waitKey(1) & 0xFF == ord('q'):
			# 	off = False


			if not false_positive and not off:
				if bump == 3:
					print('bump 3')
					bump = None
					vel_saida.publish(vel_foward)
					rospy.sleep(1)
					vel_saida.publish(vel_parado)


				if bump == 4:
					print('bump 4')
					bump = None
					vel_saida.publish(vel_foward)
					rospy.sleep(1)
					vel_saida.publish(vel_parado)


				if bump == 1:
					bump = None
					print("bump 1")
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_left_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_parado)

				if bump == 2:
					bump = None
					print('bump 2')
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_right_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_tras)
					rospy.sleep(2)
					vel_saida.publish(vel_left_avoid)
					rospy.sleep(4)
					vel_saida.publish(vel_parado)


				if near_blue:
					print("perto de azul")
					vel_saida.publish(vel_near_blue)
					rospy.sleep(2)
					near_blue = False
					vel_saida.publish(vel_parado)
					rospy.sleep(0.01)

				if media_blue[1] > (centro[0] - 2*margem_vertical) and media_blue[1] < (centro[0] - 40):
					angle = media_blue[0]/10
					angle_for_list = -(angle-32)

				for i in laser_dist:
					v = round((0.25/2.75)*(i-3)+0.25,3)
					vel_following = Twist(Vector3(v,0,0), Vector3(0,0,0))
					if i < 0.3 and i != 0:
						print('lida: obstaculo a frente')
						laser_dist = []
						vel_saida.publish(vel_tras)
						rospy.sleep(1)
						vel_saida.publish(vel_parado)
						break
				print("V: " + str(v))
				for i in laser_dist_left:
					if i < 0.3 and i != 0:
						print('lida: obstaculo esquerda')
						laser_dist_left = []
						vel_saida.publish(vel_left_avoid_tapao)
						rospy.sleep(1)
						vel_saida.publish(vel_parado)
						break

				for i in laser_dist_right:
					if i < 0.3 and i != 0:
						print('lida: obstaculo a direita')
						laser_dist_right = []
						vel_saida.publish(vel_right_avoid_tapao)
						rospy.sleep(1)
						vel_saida.publish(vel_parado)
						break

				for i in laser_dist_back:
					if i < 0.15 and i != 0:
						print('lida: obstaculo a traz')
						laser_dist_back = []
						vel_saida.publish(vel_foward_fast)
						rospy.sleep(2)
						vel_saida.publish(vel_parado)
						break

				if tracking_bottle:
					if dif == None:
						vel_saida.publish(vel_parado)
					if dif < -margem:
						# print('vemrlho para a esquerda')
						vel_saida.publish(vel_left_following)
					
					elif dif > margem:
						# print('vermelho para a direita')
						vel_saida.publish(vel_right_following)
				
					else:
						vel_saida.publish(vel_following)
						
				if not tracking_bottle:
					vel_saida.publish(vel_parado)

				rospy.sleep(0.01)


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
			see_bottle = False
