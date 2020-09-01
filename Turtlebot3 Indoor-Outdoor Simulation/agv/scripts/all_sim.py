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

import tf2_ros
import tf2_geometry_msgs
import tf
import math
import numpy as np
import random

rtls_freq=0
gps_freq=0

#__________________CONFIGURAZIONE__________________

#/pose_rtls
rtls_err=			0  	# 1 Errore Attivato 0 Errore Disattivato
rtls_sigma=			0.045 	# Deviazione standard su X e Y 
#rtls_covariance = np.diag	([0.002, 0.002, 1e6, 1e6, 1e6, 1e6]).ravel() # Pose x y z - Orientazione x y z  
rtls_covariance = np.diag	([1, 1, 1e6, 1e6, 1e6, 1e6]).ravel() # Pose x y z - Orientazione x y z  

#/pose_gps
gps_err=			0   	# 1 Errore Attivato 0 Errore Disattivato
gps_sigma=			0.0045 	# Deviazione standard su X e Y 
#gps_covariance = np.diag	([0.002, 0.002, 1e6, 1e6, 1e6, 1e6]).ravel() # Pose x y z - Orientazione x y z  
gps_covariance = np.diag	([1, 1, 1e6, 1e6, 1e6, 1e6]).ravel() # Pose x y z - Orientazione x y z 

#/odom
odom_err=			0 	# 1 Errore Attivato 0 Errore Disattivato
odom_sigma_linear_velocity=	0.005 	# Deviazione standard su: v_lineare x e y 
odom_sigma_angular_velocity=	0.002 	# Deviazione standard su: v_angolare z (yaw)
odom_covariance = np.diag	([0.0025, 0.0025, 1e6, 1e6, 1e6, 0.04]).ravel() # v_linear x y z - v_angular x y z (yaw)

#/imu
imu_err=					0 	# 1 Errore Attivato 0 Errore Disattivato
imu_sigma_orientation=				0.0005  	# Deviazione standard su: orientazione z
imu_sigma_angular_velocity=			0.0014  	# Deviazione standard su: velocita angolare z
imu_sigma_linear_acceleration=			0.002  	# Deviazione standard su: accelerazione lineare x
imu_orientation_covariance = np.diag		([0.0025, 0.0025, 0.0025]).ravel() # Orientazione x, y, z
imu_angular_velocity_covariance = np.diag	([0.02, 0.02, 0.02]).ravel() # velocita angolare x, y, z
imu_linear_acceleration_covariance = np.diag	([0.04, 0.04, 0.04]).ravel() # accelerazione lineare x, y, z

# interfaccia rtls gps (scegliere un valore) - zona intermedia di lunghezza 2*delta
delta=-0.5                                      # 0 attivo rtls e disattivo gps immediatamente
                                                # 0.5 zona intermedia no rtls no gps
                                                # -0.5 zona intermedia con rtls e gps

#_______________FINE CONFIGURAZIONE_______________

def odom_callback(msg):

	#Trasformo la pose di /odom sul frame map
	pose_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose, t_robot)
	
	#-----------------------------------------------------------------------
        #POSE REAL (XYZ e Orientamento)
	real_mex = geometry_msgs.msg.PoseWithCovarianceStamped(header=rospy.Header(frame_id="map"))
        real_mex.header.stamp = msg.header.stamp
	#Setto valori
	real_mex.pose.pose.position.x = pose_transformed.pose.position.x
        real_mex.pose.pose.position.y = pose_transformed.pose.position.y
	real_mex.pose.pose.position.z = pose_transformed.pose.position.z
	real_mex.pose.pose.orientation = pose_transformed.pose.orientation
        real_mex.pose.covariance = msg.pose.covariance
	real_pub.publish(real_mex)

	#-----------------------------------------------------------------------
	#POSE RTLS (X,Y,Z)
	if rtls== 1:
		global rtls_freq
		rtls_freq = rtls_freq + 1

		#Trasformo la pose sul frame map in pose su rtls
		pose_transformed_bis = tf2_geometry_msgs.do_transform_pose(pose_transformed, t_rtls)
		
		# Riduco frequenza messaggio
		if rtls_freq == 7:
			rtls_mex = geometry_msgs.msg.PoseWithCovarianceStamped(header=rospy.Header(frame_id="rtls"))
			rtls_mex.header.stamp = msg.header.stamp
			rtls_mex.pose.pose.position.x = pose_transformed_bis.pose.position.x
			rtls_mex.pose.pose.position.y = pose_transformed_bis.pose.position.y
			rtls_mex.pose.pose.position.z = pose_transformed_bis.pose.position.z
			rtls_mex.pose.covariance = rtls_covariance
			if bool(rtls_err)==True:
				s1=np.random.normal(pose_transformed_bis.pose.position.x, rtls_sigma, 100) #vettore 
			 	s2=np.random.normal(pose_transformed_bis.pose.position.y, rtls_sigma, 100) #vettore 
				rtls_mex.pose.pose.position.x = random.choice(s1)
			 	rtls_mex.pose.pose.position.y = random.choice(s2)
                        if (rtls_mex.pose.pose.position.x > -1+delta): 
			   rtls_pub.publish(rtls_mex)
			rtls_freq=0;

	#-----------------------------------------------------------------------
	#POSE GPS (X,Y,Z)
	if gps== 1:
		global gps_freq
		gps_freq = gps_freq + 1

		#Trasformo la pose sul frame map in pose su gps
		pose_transformed_bis = tf2_geometry_msgs.do_transform_pose(pose_transformed, t_gps)
		
		# Riduco frequenza messaggio
		if gps_freq == 14:
			gps_mex = geometry_msgs.msg.PoseWithCovarianceStamped(header=rospy.Header(frame_id="gps"))
			gps_mex.header.stamp = msg.header.stamp
			gps_mex.pose.pose.position.x = pose_transformed_bis.pose.position.x
			gps_mex.pose.pose.position.y = pose_transformed_bis.pose.position.y
			gps_mex.pose.pose.position.z = pose_transformed_bis.pose.position.z
			gps_mex.pose.covariance = gps_covariance
			if bool(gps_err)==True:
				s1=np.random.normal(pose_transformed_bis.pose.position.x, gps_sigma, 100) #vettore 
			 	s2=np.random.normal(pose_transformed_bis.pose.position.y, gps_sigma, 100) #vettore 
				gps_mex.pose.pose.position.x = random.choice(s1)
			 	gps_mex.pose.pose.position.y = random.choice(s2)
                        if(gps_mex.pose.pose.position.x<1-delta):
			   gps_pub.publish(gps_mex)
			gps_freq=0;

	#-----------------------------------------------------------------------
	#ODOMETRY
	#Introduco errore su Vx, Vy e Vyaw azzerando i valori non usati
	odom_msg = nav_msgs.msg.Odometry()
	odom_msg = msg #copio messaggio
	odom_msg.twist.covariance = odom_covariance
	if bool(odom_err)==True:
		s1=np.random.normal(msg.twist.twist.linear.x, odom_sigma_linear_velocity, 100) #vettore 
	 	s2=np.random.normal(msg.twist.twist.linear.y, odom_sigma_linear_velocity, 100) #vettore 
		s3=np.random.normal(msg.twist.twist.angular.z, odom_sigma_angular_velocity, 100) #vettore 
		odom_msg.twist.twist.linear.x = random.choice(s1)
	 	odom_msg.twist.twist.linear.y = random.choice(s2)
		odom_msg.twist.twist.angular.z = random.choice(s3)
	odom_pub.publish(odom_msg)
	

def imu_callback(msg):
        #IMU
	imu_msg= sensor_msgs.msg.Imu();
	imu_msg=msg; #copio messaggio
	imu_msg.orientation_covariance = imu_orientation_covariance
	imu_msg.angular_velocity_covariance = imu_angular_velocity_covariance
	imu_msg.linear_acceleration_covariance = imu_linear_acceleration_covariance
	if bool(imu_err)==True:
		explicit_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
		roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat) # euler[2] = yaw
		s1=np.random.normal(yaw, imu_sigma_orientation, 100) #vettore
         	s2=np.random.normal(msg.angular_velocity.z, imu_sigma_angular_velocity, 100) #vettore 
		s3=np.random.normal(msg.linear_acceleration.x, imu_sigma_linear_acceleration, 100) #vettore 
		yaw = random.choice(s1)
		x, y, z, w = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		imu_msg.orientation.x = x
		imu_msg.orientation.y = y
		imu_msg.orientation.z = z
		imu_msg.orientation.w = w
         	imu_msg.angular_velocity.z = random.choice(s2)
		imu_msg.linear_acceleration.x = random.choice(s3)
	imu_pub.publish(imu_msg)


if __name__=="__main__":

	#Topic che pubblico
	real_pub = rospy.Publisher("/pose_real", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
	rtls_pub = rospy.Publisher("/pose_rtls", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
	gps_pub = rospy.Publisher("/pose_gps", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
	odom_pub = rospy.Publisher("/odom", nav_msgs.msg.Odometry, queue_size=10)
	imu_pub = rospy.Publisher("/imu", sensor_msgs.msg.Imu, queue_size=10)

	rospy.init_node('all_sim')

	#Leggo parametri
	full_param_name = rospy.search_param('rtls')
	rtls = rospy.get_param(full_param_name)

	full_param_name = rospy.search_param('gps')
	gps = rospy.get_param(full_param_name)
        
	full_param_name = rospy.search_param('p_robot_x')
	p_robot_x = rospy.get_param(full_param_name)
        full_param_name = rospy.search_param('p_robot_y')
	p_robot_y = rospy.get_param(full_param_name)
	full_param_name = rospy.search_param('p_robot_a')
	p_robot_a = rospy.get_param(full_param_name)


	#Trasformata per passare da odom a map 
	#Sapendo la posizione di partenza del robot (map e' fisso, odom ha origine dove parte il robot)
	t_robot = geometry_msgs.msg.TransformStamped()
    	t_robot.header.stamp = rospy.Time.now()
	t_robot.header.frame_id = 'A'
    	t_robot.child_frame_id  = 'B'
    	t_robot.transform.translation.x = p_robot_x
    	t_robot.transform.translation.y = p_robot_y
    	t_robot.transform.translation.z = 0
    	q = tf.transformations.quaternion_from_euler(0, 0, p_robot_a)
    	t_robot.transform.rotation.x = q[0]
    	t_robot.transform.rotation.y = q[1]
    	t_robot.transform.rotation.z = q[2]
    	t_robot.transform.rotation.w = q[3]   

	#Trasformata per passare da map a rtls 
	#Fissa, pubblicata la leggo
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	t_rtls = tfBuffer.lookup_transform("rtls", "map", rospy.Time(), rospy.Duration(10.0))

	#Trasformata per passare da map a gps 
	#Fissa, pubblicata la leggo
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	t_gps = tfBuffer.lookup_transform("gps", "map", rospy.Time(), rospy.Duration(10.0))

	#Sottoscrivo il topic //odom_sim generato dal simulatore ideale
        rospy.Subscriber('/odom_sim', nav_msgs.msg.Odometry, odom_callback)
	rospy.Subscriber('/imu_sim', sensor_msgs.msg.Imu, imu_callback)

	rospy.loginfo("all_sim: Nodo attivo!")
	rospy.spin()

		
