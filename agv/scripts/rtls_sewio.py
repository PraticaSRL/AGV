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

import socket
import json
import rospy
import std_msgs.msg 
import geometry_msgs.msg 
import math
import numpy as np

#Impostazioni IP e Porta da dove riceve i dati (Server Sewio)
#UDP_IP = "10.0.0.5"    #IP pc fisso con RTLS Server in esecuzione
#UDP_PORT = 5300

UDP_IP = "192.168.1.19" #IP pc portatile con RTLS Server in esecuzione
UDP_PORT = 5300


#Apertura socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
sock.bind((UDP_IP, UDP_PORT))

if __name__=="__main__":


	rtls_pub = rospy.Publisher("/pose_rtls", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)	

	rospy.init_node('rtls_sewio')

	rospy.loginfo("rtls_sewio: Nodo attivo! In attesa di dati da server sewio...")

	#Preparo messaggio con dati fissi
	rtls_mex = geometry_msgs.msg.PoseWithCovarianceStamped(header=rospy.Header(frame_id="rtls"))
	rtls_mex.pose.covariance = np.diag ([20, 20, 1e6, 1e6, 1e6, 1e6]).ravel() #COVARIANZA Pose x y z - Orientazione x y z  

	while not rospy.is_shutdown():
		
		try:
			data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes	
			data_div = data.split("\"")
			x = float(data_div[17])
			y = float(data_div[29])
		except (IndexError):
			rospy.loginfo("rtls_sewio: Errore dati ")
			continue

		#Se dati ricevuti OK invio messaggio
		rtls_mex.header.stamp = rospy.Time.now()
		rtls_mex.pose.pose.position.x = x
		rtls_mex.pose.pose.position.y = y
		rtls_mex.pose.pose.position.z = 0
		rtls_pub.publish(rtls_mex)	
		rospy.loginfo("rtls_sewio: X: " + str(x) + "y: " + str(y)) #Stampa di debug da cancellare

		
