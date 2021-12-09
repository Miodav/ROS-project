#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from math import pi
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState,ModelStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback_laser(msg):
	#Import des variables globales
	global angle_rad,yaw,pub,object_twist,integrale_erreur,erreur_precedente,distance,Kdl,Kil,Kdl,Kda,Kia,Kpa,integrale_angle,erreur_angle_p,orientation,iteration
	#Recupere l'indice du minimum (obstacle)
	angle = np.argmin( np.array(msg.ranges) )
	#Convertis en rad
	angle_rad = ( angle * pi )/ 180
	#calcul de l'erreur angulaire
	erreur = (angle_rad - 0)
	#calcul des variables pour PID angulaires
	integrale_angle += erreur
	derivee_angle = erreur - erreur_angle_p
	PID_angle = Kpa*erreur + Kia*integrale_angle + Kda*derivee_angle
	erreur_angle_p = erreur 

	#Sens trigonometrique CW
	if angle <= 180:
		if orientation == "cherche" :
			print("Erreur",erreur,"CW")
		object_twist.linear.x = 0
		object_twist.angular.z = PID_angle
	#Sens horaire CCW
	else:
		if orientation =="cherche" :
			print("Erreur",erreur,"CCW")
		object_twist.linear.x = 0
		object_twist.angular.z = -PID_angle	
	 
	#condition de changement d'etat
	if abs(erreur) < 0.2 and orientation == "cherche" and iteration > 10 :
		print("ORIENTATION TROUVEE")
		orientation = "trouve"	

	#calcul du PID lineaire	
	if orientation == "trouve":
		#calcul de la distance 
		erreur_distance = distance - msg.ranges[0]
		#calcul de l'integrale de l'erreur
	    	integrale_erreur += erreur_distance
		#calcul de la derivee de l'erreur
	    	derivee_erreur = erreur_distance - erreur_precedente
		#COMMANDE 
	    	PID = Kpl*erreur_distance + Kil*integrale_erreur + Kdl*derivee_erreur
		#On sauvegarde l'erreur precedente
	    	erreur_precedente = erreur_distance
		print("Obstacle a:",msg.ranges[0])
		#Envoie des vitesses
		object_twist.linear.x = -PID
		object_twist.angular.z = 0 

	pub.publish(object_twist)
	iteration += 1 

if __name__ == '__main__':

	rospy.init_node('Obstacle3_node', anonymous=True)
	#Nombre d'iteration
	iteration = 0	
	#etat du systeme d'orientation
	orientation = "cherche"
	#angle du mur
	angle_rad = 0
	#orientation du robot
	yaw = 0
	#variables pour PID angulaire
	integrale_angle = 0
	erreur_angle_p = 0 
	Kpa = 0.8
	Kia = 0.0001
	Kda = 0.001
	#variables pour PID lineaire
	integrale_erreur = 0
	erreur_precedente = 0 
	Kpl = 0.5
	Kil = 0.31
	Kdl = 0.2
	#parametre de distance a respecter
	distance = rospy.get_param('d') 
	#On recupere les donnees du laser
	sub_laser = rospy.Subscriber('/scan',LaserScan,callback_laser)
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	object_twist = Twist()
	rate = rospy.Rate(1) 		
	rospy.spin()

