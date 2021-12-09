#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
	
def callback(msg):

	global object_twist,distance,pub,etat
	#Recupere tant qu'on a pas detecter d'obstacle
	if msg.ranges[180] > distance and etat == "avance" :
		print("No obstacle")
		object_twist.linear.x = -0.4
		etat = "stop"  
	#Arret car d'ostacle detecter
	elif msg.ranges[180] < distance and etat == "stop":
		print("Obstacle detected")
		object_twist.linear.x = 0
		etat = "avance"
	pub.publish(object_twist)

if __name__ == '__main__':
	etat = "avance"
	rospy.init_node('Detect_obstacle_node', anonymous=True)	
	#Recupere la distance a laquelle le robot doit s'arreter
	distance = rospy.get_param('d')
	#Recupere les donnees lasers
	sub = rospy.Subscriber('/scan',LaserScan,callback)
	#On publie les vitesses
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	object_twist = Twist()
    	rate = rospy.Rate(1) 
	rospy.spin()

