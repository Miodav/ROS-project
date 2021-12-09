#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

global pub 
global object_twist 
global distance 

integrale_erreur = 0
erreur_precedente = 0

#ATTENTION A BIEN CHOISIR LES VALEURS A LA MAIN
#Ameliore la rapidité du systeme
Kp = 2
#Ameliore la precision du systeme 
Ki = 0.25
#Ameliore la stabilité du systeme
Kd = 0

def callback(msg):
	
	#AVEC PID
	global integrale_erreur,erreur_precedente

	#calcul de l'erreur 
	erreur = distance - msg.ranges[0]
	#calcul de l'integrale de l'erreur
    	integrale_erreur += erreur
	#calcul de la derivee de l'erreur
    	derivee_erreur = erreur - erreur_precedente
	#COMMANDE 
    	PID = Kp*erreur + Ki*integrale_erreur + Kd*derivee_erreur
	#On sauvegarde l'erreur precedente
    	erreur_precedente = erreur
	print("Obstacle a:",msg.ranges[0])
	#Envoie des vitesses
	object_twist.linear.x = -PID
	pub.publish(object_twist)

	"""
	#SANS PID
	vitesse_mur = 0.2 
	if msg.ranges[0] < distance : 
		print("Obstacle a:",msg.ranges[0])
		object_twist.linear.x = -vitesse_mur
		object_twist.linear.z = 0
	else: 
		print("Obstacle a:",msg.ranges[0])
		object_twist.linear.x = vitesse_mur
		object_twist.linear.z = 0
	pub.publish(object_twist)
	"""

if __name__ == '__main__':

	rospy.init_node('Obstacle2_node', anonymous=True)
	#Recupere le parametre de distance
	distance = rospy.get_param("d")
	#Recupere les donnees lasers
	sub = rospy.Subscriber('/scan',LaserScan,callback)
	#Publisher des vitesses
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	object_twist = Twist()
    	rate = rospy.Rate(2) 
	rospy.spin()

