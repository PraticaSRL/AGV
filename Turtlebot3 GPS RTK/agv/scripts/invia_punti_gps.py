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
import actionlib
import std_msgs.msg
from std_msgs.msg import Int8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf, math

import threading 
import time
import thread
import sys
import os
import signal

import select

def goal_pose(x, y, theta):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'

	goal_pose.target_pose.pose.position.x = x
	goal_pose.target_pose.pose.position.y = y

	quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
	goal_pose.target_pose.pose.orientation.x = quaternion[0]
	goal_pose.target_pose.pose.orientation.y = quaternion[1]
	goal_pose.target_pose.pose.orientation.z = quaternion[2]
	goal_pose.target_pose.pose.orientation.w = quaternion[3]
	return goal_pose

def signal_handler(sig, frame):
	print('Premuto Ctrl+C!')
	os.system('killall -INT rosmaster')
	sys.exit(0)

if __name__=="__main__":
	rospy.init_node('invia_punti_2')
	rate = rospy.Rate(10)
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	signal.signal(signal.SIGINT, signal_handler)

	print("Nodo <invia_punti> pronto!")

	fare=0

	print("Inserisci goal (A-B-C-D-O-S):")
	# x, y, angolo
	# Comandi prefissati: A, B, C, D, O 
        # Vertici di un rettangolo, dove A e' il vertice in basso a destra e si muovono in verso orario. 
        # O rappresenta l'origine. 
	# S per terminare

	while (1):
		

		#obiettivo = raw_input()
		obiettivo = None

		i, o, e = select.select( [sys.stdin], [], [], 3 )

		if (i):
  			obiettivo = sys.stdin.readline().strip()
		
		#print("Dato " + repr(rospy.is_shutdown()))
		if rospy.is_shutdown():
			sys.exit()

		#Goal raggiunto
		if fare==1:
			if client.get_state() == 3:
				print("Goal raggiunto!")
				fare=0

		if (obiettivo == None):
			continue
	
		#Divisione della stringa ottenuta dall'utente
		obiettivo = obiettivo.split(',', 3)
		control = str(obiettivo[0]).upper()

		#Condizione di terminazione
		if (control == 'S'):
			print("CHIUSURA")
			os.system('killall -INT rosmaster')
			break

		#condizioni pre impostate
		elif (control == 'A'):
			x = 1
			y = 0
			theta = math.pi/2

		elif (control == 'B'):
			x = 1
			y = 1
			theta = 0
		
		elif (control == 'C'):
			x = 3
			y = 1
			theta = -math.pi/2

		elif (control == 'D'):
			x = 3
			y = -1
			theta = -math.pi

		elif (control == 'E'):
			x = 1
			y = -1
			theta = math.pi/2

		elif (control == 'P'):
			x = 2.5
			y = 0
			theta = 0

		elif (control == 'O'):
			x = 0.0
			y = 0.0
			theta = 0.0

		#Messaggio personalizzato
		else:
			x = float(obiettivo[0])  #ok
			y = float(obiettivo[1])
			theta = float(obiettivo[2])

		#Invio goal al nav stack		
		goal = goal_pose(x, y, theta)
		print("Invio goal: " + repr(obiettivo))
		client.send_goal(goal)
		fare=1

		


		

		
