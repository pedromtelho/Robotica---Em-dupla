#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import pi

dist = None

def scaneou(dado):
	global dist
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	# print("Leituras:")
	# print(np.array(dado.ranges).round(decimals=2))
	dist = (np.array(dado.ranges))[0]
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

		rospy.init_node("le_scan")

		velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
		recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

		while not rospy.is_shutdown():
			vel_frente = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			vel_traz = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
			vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0))

			print(dist)

			if dist <= 1:
				velocidade_saida.publish(vel_traz)
				print("back")

			elif dist > 1.06:
				velocidade_saida.publish(vel_frente)
				print('foward')

			rospy.sleep(1)



			