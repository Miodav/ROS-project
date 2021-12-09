#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2 

from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

global pub  #publisher
global object_twist #msg to publish
global height , width, dim,taille #dimensions of the image to process
global flag_red,flag_yellow,flag_green #flag are "True" if the fctn get_centroid hasn't found any px of their color
global flag_scan #flag on true if the whole picture has been scanned and no px of any color has been found 
global flag_speed #flag qui dit de prendre un autre centroid plus éloigné si la vitesse est trop faible 
global rows_to_watch#number of rows of px to watch on the cropped picture


def callback(msg):    
    global flag_red,flag_yellow,flag_green
    global flag_scan
    global height , width, dim,taille
    global rows_to_watch    

    #global var are initialized in main 

    rospy.loginfo("\n\n\n\n")    
    rospy.loginfo("NEW DATA RECEIVED : entering callback fctn")
    #creating an hsv img
    hsv_img= img_format(msg)
    rospy.loginfo("displaying raw image")    
    cv2.imshow("raw img",hsv_img)    	
    cv2.namedWindow("raw img",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("raw img",(taille,taille))
    cv2.waitKey(1)    
        
    #we are going to scan the picture layer by layer of rows_to_watch px of heigth
    descenter  = np.arange(-height/2 , height/4 , rows_to_watch)
    #we retain only the quartes at the middle of the picture
    #descenter = np.where( descenter > height/4 , descenter , 0 )    
    rospy.loginfo("descenter = " + str(descenter))

        
    i = 0
    flag_red = flag_green = flag_yellow = True
    flag_scan = False
    while (flag_red and flag_yellow and flag_green) and i < len(descenter): 
        # scan the picture layer by layer 
        # while get_centroid doesn't find any centroid
        #and the whole picture hasn't been scanned
        
        #picture cropped 
        crop_img =  img_crop(hsv_img , descenter[i])        

        #get centroid of the picture 
        yellow_centroid = get_centroid(crop_img,"yellow")
        red_centroid = get_centroid(crop_img,"red")
        green_centroid = get_centroid(crop_img,"green")
                
        
        #display for debug 
        #rospy.loginfo("red = " + str(flag_red) + "\tyellow = " + str(flag_yellow) + "\tgreen = "+str(flag_green))    
        i +=1
    
        if  i == len(descenter) and flag_red and flag_yellow and flag_green :  
            #if  the whole picture has been scanned and no px has been found then
            # scan_flag is raised meaning that the bot will enter error mode in 
            #control_burger_bot function
            flag_scan = True
        else : 
            flag_scan = False
        
        
    

    if flag_scan : #on force le robot ç se mettre en mode erreur
        rospy.loginfo("leaving the while loop : no centroid found ")
        control_burger_bot(green_centroid , "green") 
        # as flagscan is raised the color of the centroid is choosen randomly         
        # the bot will enter error mode
        return 
        
    else : 
        rospy.loginfo("leaving the while loop : centroid found")
    
    # rectifying the y values :    
    #take in count that the picture has been cropped and that y axis is biased 
    yellow_centroid[1]  += height/2 + descenter[i-1]
    red_centroid[1]  += height/2 + descenter[i-1]
    green_centroid[1]  += height/2 + descenter[i-1]
       
           
    flag_arr = np.array([flag_red,flag_yellow,flag_green])                  #for debug    
    red_distance = euclid_distance(red_centroid)
    yellow_distance = euclid_distance(yellow_centroid)
    green_distance = euclid_distance(green_centroid)    

    dist_array = np.array([red_distance,yellow_distance,green_distance])
    rospy.loginfo("red\tyellow\tgreen")
    rospy.loginfo(flag_arr)
    rospy.loginfo(dist_array)                                               #for debug
    
    if red_distance <= yellow_distance and red_distance <= green_distance :
        control_burger_bot(red_centroid , "red")
    elif yellow_distance <= green_distance : 
        control_burger_bot(yellow_centroid , "yellow")
    else :
        control_burger_bot(green_centroid , "green")

    
    
    


        
"""____________________________________IMAGE FORMATING_________________________________"""
def img_format(raw_img):
    """fait le necessaire pour rednre l'image exploitable pour la suite de l'algorithme :
    extraction de ses dimensions
    conversion en HSV"""
    
    global height , width, dim, taille
    
    #Conversion from image to OCV image
    cv_img = bridge.imgmsg_to_cv2(raw_img,"bgr8")

    #Resize the dimension of the image 
    taille = 200 
    cv_img = cv2.resize(cv_img,(taille,taille))
    #Get new the dimensions 
    height , width , dim = cv_img.shape
        
    #Conversion to HSV mode 
    hsv_img= cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
    
    return hsv_img
    
    
    
    
    
    
    
"""_____________________________________IMAGE CROP_____________________________________________"""
def img_crop(img , descenter):
    """ 'Crop' the picture in order to have smaller amount of data to processs    
    and extract separately != centroids
    img : np.array[:,:,:]          image to crop
    descenter : scalar      center of the future cropped picture
    """
    #crop useless parts
    bottom = (height)/2 + descenter + rows_to_watch
    up =  (height)/2 + descenter 

    img = img[height - bottom : height - up , 0:width , : ] 
    
    return img
    
    
    
    
    
    
    
    
    
    

"""______________________EXTRACT CENTROID OF GIVEN COLOR FROM HSV IMG___________________________"""
def get_centroid(img , color):  
    """
   img : np.array[:,:,:] , img has to be converted in hsv before    
   color = str either "red","green" or "yellow"
   """    
    #Thresholding
    #POUR AVOIR LES COULEURS EN HSV A PARTIR DE BGR 
    #color = np.uint8([[[B,G,R]]])
    #hsv_color = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    #print(hsv_color)     0
    
    global height , width, dim
    global flag_red,flag_yellow,flag_green
    global flag_speed
    
    #choosing the color range of the mask
    if color == "yellow":
        lower_color = np.array([20,150,150])
        upper_color = np.array([40,255,255])
    if color == "red":
        lower_color = np.array([0,150,150])
        upper_color = np.array([20,255,255])
    if color == "green":
        lower_color = np.array([50,150,150])
        upper_color = np.array([70,255,255])

    #creating the mask
    mask = cv2.inRange(img,lower_color,upper_color)

    #Binarization of the picture	
    filtered_img = cv2.bitwise_and(img,img,mask=mask)     
    
    #if np px of given color found then the flag is raised / set to True
    if not np.any(filtered_img):
        if color == "yellow":
            flag_yellow = True
        if color == "red":
            flag_red = True
        if color == "green":
            flag_green = True
        #display of the result
        cv2.imshow(color + " Filtered_image",filtered_img)    	
        cv2.namedWindow(color + " Filtered_image",cv2.WINDOW_NORMAL)
        cv2.resizeWindow(color + "Filtered_image",(taille,taille))
        
        cv2.waitKey(1)
        return np.array([10000000,100000000]) #return high value in order to get a high distance
        
    #if a px has been found then flag is put down
    else :
        if color == "yellow":
            flag_yellow = False
        if color == "red":
            flag_red = False
        if color == "green":
            flag_green = False

    #Moment calculation of the binary_image to find the centroids
    gray_img  = np.array(filtered_img[:,:,2])
    _, thresh = cv2.threshold(gray_img , 200 , 255 , 0)                                                                                                #alternative
    contours, hierachy = cv2.findContours(thresh , cv2.RETR_TREE , cv2.CHAIN_APPROX_NONE) [-2:]      #cv2.RETR_CCOMP , cv2.CHAIN_APPROX_TC89_L1)
    rospy.loginfo("number of "+ color + " centroids==>" +  str(len(contours) ) )
    centroid_arr = np.zeros([ len(contours) , 2 ])
    distance = np.zeros([len(contours)])
    
    for i in range(len(contours)):       
        temp = contours[i]       
        moment = cv2.moments(temp)
        try : 
            cx = int(moment["m10"] / moment["m00"])
            cy = int(moment["m01"] / moment["m00"])
            
            # center the origin of x values :
            #we need to have the x = 0  at the center of the picture, not at the left-most part 
            cx -=  width/2
            #rospy.loginfo("cx = " + str(cx) + " cy = " + str(cy))
            centroid_arr[i , 0] = cx 
            centroid_arr[i , 1] = cy 
            
            distance[i] = euclid_distance( centroid_arr[i , :] )
        except ZeroDivisionError : 
            pass

    #si il n'y a qu'un centroid on le choisit sinon
    if len(contours) == 1 :
        centroid = centroid_arr[0]
        rospy.loginfo("un seul centroid disponible : " + str(centroid))
    else :
        rospy.loginfo("Liste des centroids " + str(centroid_arr))
        rospy.loginfo("Liste des distance " + str(distance))
    
        i_min = np.argmin(distance)
        centroid = centroid_arr[i_min , :]
        rospy.loginfo("i_min = " + str(i_min))
        
        #si le robot n'avance plus assez vite on le fait prendre un centroid plus loin 
        if flag_speed  and len(centroid_arr)> 1 :
            rospy.loginfo("FLAG_SPEED RAISED : taking another centroid\n UPDATING CENTROID LIST")            
            distance = np.delete(distance , i_min , 0)
            centroid_arr  = np.delete(centroid_arr , i_min , 0)       
            rospy.loginfo("Liste des centroids " + str(centroid_arr))
            rospy.loginfo("Liste des distance " + str(distance))
        
        i_min = np.argmin(distance)
        centroid = centroid_arr[i_min , :]
        rospy.loginfo("i_min = " + str(i_min))
        rospy.loginfo("centroid chosen : " + str(centroid) )    
        

        
    #Draw a circle on the binary_image with radius 10 in color a given color 
    radius = 4 
    #BGR
    if color == "yellow":
        circle_color = (245,245,0)
    if color == "red":
        circle_color = (0,0,250)
    if color == "green":
        circle_color = (0,245,0)
        
    thickness  = -1 #negative value --> filled circle
    
    #Image with yellow line and background black , white circle
    cv2.circle(filtered_img,(cx+width/2,cy),radius,circle_color,thickness)
    cv2.namedWindow(color + " Filtered_image",cv2.WINDOW_NORMAL)
    cv2.resizeWindow(color + "Filtered_image",(taille,taille))
    cv2.imshow(color + " Filtered_image",filtered_img)
    	
    cv2.waitKey(1)
    rospy.loginfo(color +" centroid found at : " + str(centroid))    
    return centroid










"""_____________________________________GIVE EUCLIDIAN DISTANCE_____________________________"""
def euclid_distance(centroid):
    """centroid : type np.array[:,:]"""
    return np.sqrt( np.sum(centroid ** 2) ) 










"""_____________________________________CONTROL BURGER BOT / PUBLISHER_____________________________"""
def control_burger_bot(centroid , color):
    """controle burger bot with a given centroid to follow and a color to weigth on its speed 
    centroid  = np.array centroid.shape = [1,2] 
    color = str either "red","green" or "yellow" """    
    global  flag_scan , flag_speed
    global max_speed
    global delta, derivate_delta, prev_delta
    
    if flag_scan :  
        #if the robot hasn't found anything he will enter error state and just turn around looking for a centroid
        object_twist.linear.x = 0
        object_twist.angular.z = 1
        pub.publish(object_twist)
        rospy.loginfo("No px found entering error mode")
      
        return

         
    rospy.loginfo(color + " centroid chosen")        

    #coef is the value that will adapt the bot speed in function of the color seen
    if color == "yellow":
        coef = 1
    if color == "red":
        coef = 0.8
    if color == "green":
        coef = 0
        
    #A Proportional controller
    cx = centroid[0]
    cy = centroid[1]
    
    #gains 
    K_linear = 0.020
    K_angular = 0.07
    #error calculation    
    delta =np.array([ cx , cy ])

    object_twist.linear.x = coef * max_speed
    object_twist.angular.z = - coef * K_angular * delta[0]
 
    # si le robot est trop lent alors on va le faire changer de centroid à suivre , voir fctn get_centroid l 263
    if  object_twist.linear.x < 0.002 :
        flag_speed = True
    else : 
        flag_speed = False
        
    pub.publish(object_twist)
    rospy.loginfo("linear speed : " + str(object_twist.linear.x))
    rospy.loginfo("angular speed : " + str(object_twist.angular.z) )
    rospy.loginfo("order published, exiting callback function")
    
    return     









"""_____________________________________MAIN_____________________________________________________"""
if __name__ == '__main__':
    global max_speed
    global rows_to_watch
    global flag_speed    
    #init values     
    max_speed = 0.29
    rows_to_watch = 20
    flag_speed = False
    #Create a CvBridge object 
    bridge = CvBridge()
    rospy.init_node('Following_Line_node', anonymous=True)
    sub = rospy.Subscriber('/camera/image_raw',Image,callback)
    
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    object_twist = Twist()
    rate = rospy.Rate(1) 
    rospy.spin()
