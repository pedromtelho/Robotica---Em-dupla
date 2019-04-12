#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from math import pi

bumpers = None

def bumper(dado):
	global bumpers

	bumpers = dado.data
	print(bumpers)


	


if __name__=="__main__":

		rospy.init_node("desvio")

		velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
		recebe_scan = rospy.Subscriber("/bumper", UInt8, bumper)


		bumpers = None
		while not rospy.is_shutdown():
			vel_frente = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			vel_frente2 = Twist(Vector3(1,0,0), Vector3(0,0,0))
			vel_tras = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
			parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
			vel_direita = Twist(Vector3(0,0,0), Vector3(0,0,-pi/8))
			vel_esquerda = Twist(Vector3(0,0,0), Vector3(0,0,pi/8))

			# print(bumpers)

			if bumpers == None:
				velocidade_saida.publish(vel_frente)
				rospy.sleep(0.5)

			if bumpers == 1 and bumpers == 2:
				velocidade_saida.publish(vel_tras)
				rospy.sleep(1)
				velocidade_saida.publish(vel_direita)
				rospy.sleep(4)
				velocidade_saida.publish(vel_tras)
				rospy.sleep(1)
				velocidade_saida.publish(vel_esquerda)
				rospy.sleep(4)

			if bumpers == 1 and bumpers != 2:
				velocidade_saida.publish(vel_tras)
				rospy.sleep(1)
				velocidade_saida.publish(vel_direita)
				rospy.sleep(4)
				velocidade_saida.publish(vel_tras)
				rospy.sleep(1)
				velocidade_saida.publish(vel_esquerda)
				rospy.sleep(4)
			

			if bumpers == 2 and bumpers != 1:
				velocidade_saida.publish(vel_tras)
				rospy.sleep(1)
				velocidade_saida.publish(vel_esquerda)
				rospy.sleep(4)
				velocidade_saida.publish(vel_tras)
				rospy.sleep(1)
				velocidade_saida.publish(vel_direita)
				rospy.sleep(4)



			if bumpers == 3 and bumpers != 4:
				velocidade_saida.publish(vel_frente2)
				rospy.sleep(2)
				velocidade_saida.publish(vel_direita)
				rospy.sleep(4)
				velocidade_saida.publish(vel_frente)
				rospy.sleep(1)
				velocidade_saida.publish(vel_esquerda)
				rospy.sleep(4)
			

			if bumpers == 4 and bumpers != 3:
				velocidade_saida.publish(vel_frente2)
				rospy.sleep(2)
				velocidade_saida.publish(vel_esquerda)
				rospy.sleep(4)
				velocidade_saida.publish(vel_frente)
				rospy.sleep(1)
				velocidade_saida.publish(vel_direita)
				rospy.sleep(4)

			if bumpers == 3 and bumpers == 4:
				velocidade_saida.publish(vel_frente2)
				rospy.sleep(2)
				velocidade_saida.publish(vel_direita)
				rospy.sleep(4)
				velocidade_saida.publish(vel_frente)
				rospy.sleep(1)
				velocidade_saida.publish(vel_esquerda)
				rospy.sleep(4)

			bumpers = None