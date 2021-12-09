#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2 

from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#Publisher
global pub_velocity ,pub_door 
#Recupere la distance de securité
distance = rospy.get_param('d')
#Instanciation d'objets
object_bridge = CvBridge()
object_twist = Twist()
object_bool = Bool()
#presence d'obstacle
obstacle = False
#etat du systeme
etat = "Avance"
#nombre d'obstacle rencontrée
compteur_obstacle = 0

def callback_camera(msg):
	global object_bridge,object_twist,object_bool
	global obstacle,compteur_obstacle,etat
	
	#Conversion from image to OCV image
	cv_image = object_bridge.imgmsg_to_cv2(msg,"bgr8")
	#Resize the dimension of the image 
	taille = 100 
	cv_image = cv2.resize(cv_image,(taille,taille))
	#Get new the dimensions 
	height , width , dim = cv_image.shape
	#Conversion to HSV mode
	hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
	#POUR AVOIR LES COULEURS EN HSV A PARTIR DE BGR 
	#color = np.uint8([[[B,G,R]]])
	#hsv_color = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
	#print(hsv_color)
	#HSV
	lower_yellow = np.array([20,245,245])
	upper_yellow = np.array([40,255,255])
	#Binarization of the image	
	mask = cv2.inRange(hsv_image,lower_yellow,upper_yellow)
	#Crop the image to get interest regions
	top = 3*height / 4
	bot = 3*height / 4 + 20 
	#Upper part 
	mask[0:top,0:width] = 0	
	#Lower part
	mask[bot:height,0:width] = 0
	#Apply the mask to the image
	filtered_image = cv2.bitwise_and(cv_image,cv_image,mask=mask)
	#Moment calculation to find centroid
	Moment = cv2.moments(mask,False)
	if Moment['m00'] > 0 : 
		cx = int(Moment['m10']/Moment['m00'])
		cy = int(Moment['m01']/Moment['m00'])
	else:
		cx = height /2
		cy = width /2
	#Draw a circle on the binary_image with radius 5 in white color (245,245,245) filled
	radius = 5 
	white = (245,245,245)
	thickness  = -1 #negative value --> filled circle
	#Image with yellow line and background black , white circle
	cv2.circle(filtered_image,(cx,cy),radius,white,thickness)
	cv2.namedWindow("Filtered_image",cv2.WINDOW_NORMAL)
	cv2.resizeWindow("Filtered_image",(taille,taille))
	cv2.imshow("Filtered_image",filtered_image)
	#calcul de l'erreur 
	erreur = cx - width/2
	#On regle la vitesse lineaire 
	#Si aucun d'obstacle on avance
	if obstacle == False and etat == "Avance":
		print("Pas d'obstacle",etat)
		object_twist.linear.x = 0.3
		etat = "Arret"
	#Si presence d'un obstacle on s'arret  
	elif obstacle == True and etat == "Arret":
		compteur_obstacle += 1
		print("Obstacle detectee",etat,"compteur",compteur_obstacle)
		object_twist.linear.x = 0
		#Si on detecte un deuxieme obstacle on lance un signal
		if compteur_obstacle == 2:
			object_bool.data = True
			pub_door.publish(object_bool)
		etat = "Avance"
	#On regle l'orientation 
	#Si presence d'obstacle 
	if etat == "Avance" : 
		object_twist.angular.z = 0
	#Si aucun d'obstacle on s'oriente
	elif etat == "Arret" :
		object_twist.angular.z = -float(erreur)/100
	#Une fois les vitesses mises a jour on publie 
	pub_velocity.publish(object_twist)
	cv2.waitKey(1)

def callback_laser(msg):

	global distance,obstacle 
	#Presence d'un obstacle ou non 
	if msg.ranges[0] < distance :
		obstacle = True
	else :
		obstacle = False

if __name__ == '__main__':

	rospy.init_node('Line_and_obstacle_node', anonymous=True)
	sub_camera = rospy.Subscriber('/camera/image_raw',Image,callback_camera)
	sub_laser = rospy.Subscriber('/scan',LaserScan,callback_laser)
	pub_velocity= rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	pub_door = rospy.Publisher('/Garage_Door_Opener',Bool,queue_size=1)
    	rate = rospy.Rate(2) 
	rospy.spin()
