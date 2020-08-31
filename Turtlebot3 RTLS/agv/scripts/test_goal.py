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
	rospy.init_node('test_goal')
	rate = rospy.Rate(10)
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	signal.signal(signal.SIGINT, signal_handler)

	print("Nodo <test_goal> pronto!")

	pause=3
        total=0

	print("Invio di piu' goal in sequenza")

        time.sleep(20)

	while (1):
		                        

		#Invio goal al nav stack
                x = 1                               #A
                y = 0  
                theta = math.pi/2  		
		goal = goal_pose(x, y, theta)
                print("goal A inviato")
		client.send_goal_and_wait(goal)
                total=total+1
                print("goal A raggiunto! - totale raggiunti:"+str(total))
                time.sleep(pause)
                

	        x = 1                              #B
	        y = 0.5
                theta = 0
		goal = goal_pose(x, y, theta)
                print("goal B inviato")
		client.send_goal_and_wait(goal)
                total=total+1
                print("goal B raggiunto! - totale raggiunti:"+str(total))
                time.sleep(pause)
		

	        x = 2                                 #C
		y = 0.5
                theta = -math.pi/2
		goal = goal_pose(x, y, theta)
                print("goal C inviato")
		client.send_goal_and_wait(goal)
                total=total+1
                print("goal C raggiunto! - totale raggiunti:"+str(total))
                time.sleep(pause)


	        x = 2                                  #D
		y = -0.5
                theta = -math.pi
		goal = goal_pose(x, y, theta)
                print("goal D inviato")
		client.send_goal_and_wait(goal)
                total=total+1
                print("goal D raggiunto! - totale raggiunti:"+str(total))
                time.sleep(pause)


     	        x = 1                                   #E
		y = -0.5
                theta = math.pi/2
		goal = goal_pose(x, y, theta)
                print("goal E inviato")
		client.send_goal_and_wait(goal)
                total=total+1
                print("goal E raggiunto! - totale raggiunti:"+str(total))
                time.sleep(pause)





                






		


		

		
		
