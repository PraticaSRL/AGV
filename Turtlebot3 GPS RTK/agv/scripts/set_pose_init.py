#!/usr/bin/env python

# ================================================================================================= 
#   PRATICA SRL - www.praticasrl.com                                                                    
#   Progetto: AGV - Veicolo a guida automatica per movimentazione pallet                                
#   Progetto cofinanziato dal Fondo POR FESR 14/20 Calabria                                             
#   Autori:                                                                                             
#   Duardo Domenico                                                                                     
#   Chila' Antonino                                                                                      
#   Scimonelli Mattia                                                                                   
# =================================================================================================

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from robot_localization.srv import SetPose


import tf2_ros
import tf2_geometry_msgs
import tf
import math
import numpy as np
import random


if __name__=="__main__":

	rospy.init_node('set_pose_init')

	#Leggo parametri
	full_param_name = rospy.search_param('p_robot_x')
	p_robot_x = rospy.get_param(full_param_name)
        full_param_name = rospy.search_param('p_robot_y')
	p_robot_y = rospy.get_param(full_param_name)
	full_param_name = rospy.search_param('p_robot_a')
	p_robot_a = rospy.get_param(full_param_name)

	#Creo messaggio
	my_mex = geometry_msgs.msg.PoseWithCovarianceStamped(header=rospy.Header(frame_id="map"))
        my_mex.header.stamp = rospy.Time.now()
	#Setto valori
	my_mex.pose.pose.position.x = p_robot_x
        my_mex.pose.pose.position.y = p_robot_y
	my_mex.pose.pose.position.z = 0
	q = tf.transformations.quaternion_from_euler(0, 0, p_robot_a)
	my_mex.pose.pose.orientation.x = q[0]
    	my_mex.pose.pose.orientation.y = q[1]
    	my_mex.pose.pose.orientation.z = q[2]
    	my_mex.pose.pose.orientation.w = q[3] 

	#Attendo il servizio e lo richiamo
	rospy.wait_for_service('/set_pose_map')
    	try:
        	set_pose = rospy.ServiceProxy('/set_pose_map', SetPose)
		resp = set_pose(my_mex)
		

    	except rospy.ServiceException, e:
        	rospy.loginfo("set_pose_init: ERRORE")


	rospy.loginfo("set_pose_init: Nodo attivato ed eseguito!")
	rospy.loginfo("set_pose_init: x=" + str(my_mex.pose.pose.position.x) + " y=" + str(my_mex.pose.pose.position.y) + " z=" + str(my_mex.pose.pose.position.z))
	rospy.loginfo("set_pose_init: w=" + str(my_mex.pose.pose.orientation.w) + " x=" + str(my_mex.pose.pose.orientation.x) + " y=" + str(my_mex.pose.pose.orientation.y) + " z=" + str(my_mex.pose.pose.orientation.z))



		
