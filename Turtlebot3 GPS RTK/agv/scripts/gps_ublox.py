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
import std_msgs.msg 
import geometry_msgs.msg 
import math
from ublox_msgs.msg import NavRELPOSNED
import numpy as np

def callbackGPS(data):
	
	#Questa callback e' relativa all'arrivo di un topic da parte del ROVER
	
	#Controllo che il rover sia fixed (23 = 10111)
	if data.flags == 23:
		#relPosX espresso in cm; relPosHPX espresso in 0.1 mm; il valore deve essere epresso in m
		RPN = float(data.relPosN)
		RHPN = float(data.relPosHPN)
		RPE = float(data.relPosE)
		RHPE = float(data.relPosHPE)
		x = (RPN + (RHPN / 100))/100
		y = (RPE + (RHPE / 100))/100

		#Preparo messaggio
		gps_mex.header.stamp = rospy.Time.now()
		gps_mex.pose.pose.position.x = x
		gps_mex.pose.pose.position.y = y
		gps_mex.pose.pose.position.z = 0
		gps_pub.publish(gps_mex)	
		rospy.loginfo("gps_ublox: X: " + str(x) + "y: " + str(y)) #Stampa di debug da cancellare
		

if __name__=="__main__":


	gps_pub = rospy.Publisher("/pose_gps", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)	

	rospy.init_node('gps_ublox')

	rospy.loginfo("gps_ublox: Nodo attivo! In attesa di dati da rover...")

	rate = rospy.Rate(10)

	#Preparo messaggio con dati fissi
	gps_mex = geometry_msgs.msg.PoseWithCovarianceStamped(header=rospy.Header(frame_id="gps"))
	gps_mex.pose.covariance = np.diag ([0.00025, 0.00025, 1e6, 1e6, 1e6, 1e6]).ravel() #COVARIANZA Pose x y z - Orientazione x y z  

	#Attendo Topic	
	rospy.Subscriber('/ROVER/navrelposned', NavRELPOSNED, callbackGPS)

	while not rospy.is_shutdown():
		pass


		
