#! /usr/bin/env python3
import rospy 		# if you dont know why we do this, why are you here tbh smh
import sys			# to get the arguments from the terminal from rosrun commands directly
import os			# why this? dont ask even idek
import math			# good to have some meth, why? cuz its intoxicating...
import numpy as np	# meth kar rahe the toh ise kyu chhode

from nav_msgs.msg import Odometry # Odometry to get Odometry :)
from geometry_msgs.msg import Point, Twist # Defining goal_point/s as ROS' Point class instnace/s
from nav_msgs.msg import OccupancyGrid # For getting the map from mapping as OccupancyGrid Docs: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html

#import service module and message file
import actionlib # for writing action client
import planner.msg # for action server getting ROS' action methods, these are created from .action files when you do catkin_make tutorial: http://wiki.ros.org/actionlib_tutorials/Tutorials
from planner.srv import * # getting the service messages


class RootClient(object): # OOP karlo friends
	def __init__(self):
		print("Initializing Subscriber/Publishers/Actions/Services/Variables")
		"""
		Initialize all ROS services, actions and Pub/Sub
		Services to be called --> astar_borrowed_planner
		Actions to be called --> initial_move, path_follow 
		Subscribed Topics --> /odom /map 
		Published Topics --> 
		Parameters Set -->
		"""

		# class variables initialized:

		# if goal reached -> True, else -> False
		self.reached = False

		# if stopped due to error/ failed planning of path -> True else False
		self.err = False

		# current position of the bot
		self.cur_x = 0.0
		self.cur_y = 0.0

		# final goal position of the bot
		

		# waypoints
		self.trajectory_x = []
		self.trajectory_y = []

		# current map of env
		self.obstacle_map = OccupancyGrid()

		# initial callback bools (details in the end)
		self.first_odom_clbk=False
		self.first_map_clbk=False


		# subscriber initialize
		self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_cb)
		self.map_sub = rospy.Subscriber('/map', OccupancyGrid , self.map_cb)

		# service initialize
		self.astar_service = rospy.ServiceProxy('astar_planner', astar_borrowed) # Defines a service proxy for calling the A* service
		try:
			print("Waiting for services") # I mean its a print statement dont worry
			rospy.wait_for_service('astar_planner') # this might worry you tbh, but lite, it just waits for servie till ist up and running (haa haa naam hi repeat kar raha hu thik hai)
			print("Done waiting for services")
		except rospy.ServiceException as e: # try catch block stuff, its called error handling, standard stuff, check this to get help: Docs: https://docs.python.org/3/tutorial/errors.html for video enthusiasts: https://www.youtube.com/watch?v=NIWwJbo-9_8
			rospy.logerr("Services could not be initialized")


		self.path_follow_client = actionlib.SimpleActionClient('path_follow', planner.msg.PathFollowAction) # initialize the action client 
		self.path_follow_client.wait_for_server() # wait for action server to be up and running i mean its in the name bro/sis/ whatever u prefer (we at Kratos are inclusive)
		print("Waiting for Subscriptions")

		while not(self.first_map_clbk and self.first_odom_clbk): # this is also how you can use the first callback bool
    			continue
		print("Done initializing Subscriber/Publishers/Actions/Services/Variables.")

		# a variable that will shift to aruco code
		self.searching=True
		

		# aruco part
		self.vel_pub=rospy.Publisher("/cmd_vel", Twist,queue_size=10)
		self.command=Twist()
		self.follow_tag_client = actionlib.SimpleActionClient('follow_tag', planner.msg.FollowTagAction) # initialize the action client 
		self.follow_tag_client.wait_for_server()
		self.rate=rospy.Rate(10)
		self.follow_gate_client = actionlib.SimpleActionClient('follow_gate', planner.msg.FollowGateAction)
		self.follow_gate_client.wait_for_server()
		print("Done initializing server!")
		self.spiral_search_service = rospy.ServiceProxy('spiral_search', Spiral_search)
		try:
			print("Waiting for aruco services")
			rospy.wait_for_service('spiral_search')
			print("Done waiting for aruco services")
			
		except rospy.ServiceException as e:
			print("Service call failed:" + str(e))
	def stop(self):
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0
		self.vel_pub.publish(self.command)
		print("am pun")

	def odom_cb(self, msg):
		# initialized as opposites to match co-ordinates in mapping (get your meth right using ReadMe_mapping.md)
		self.cur_x = -msg.pose.pose.position.y
		self.cur_y = msg.pose.pose.position.x
		self.first_odom_clbk=True # just some first callback things... No they not scary okay
	
	def map_cb(self, msg):
		self.obstacle_map = msg #just catching the map and storing and updating it in a script variable 
		self.first_map_clbk= True

	def move_fb(self,fb_data): # just like most corporations, we also ignore the feedbacks, because there is nothing you can do with them anyways
		pass
		# print("Current Position: ", fb_data.current_pos)

	def main(self, goal_x, goal_y,single_aruco_part,gate_aruco_part):

		"""
		Main controller function. 
		1. Call initial_move -> moves towards the goal while maping obstacles regularly, stops when obstacle detected in front.
		2. If reached goal -> stop.
		3. Else call astar_planner -> returns waypoints of the optimal path
		4. Call path_follow -> follows the waypoints while maping obstacles regularly, stops when obstacle detected in front.
		5. If reached goal -> stop/
		6. Else go to 5.

		To-DO:
		1. Store back-up points, nodes where the bot can return to if it gets stuck.
		"""
		self.goal_x = goal_x
		self.goal_y = goal_y
		self.single_aruco_part=single_aruco_part
		self.gate_aruco_part=gate_aruco_part

		
		
		goal_pos = Point() # initialising goal_position as instance of Point()...
		goal_pos.x = float(self.goal_x) # sys args come as strings you see, so floating them is necessry, then we just specify that the x atribute of the goal_pos we initialized in the line above is the self.goal_x
		goal_pos.y = float(self.goal_y) # we just specify that the x atribute of the goal_pos we initialized in the line above is the self.goal_x

		# Creates a goal to send to the action server.
		goal = planner.msg.InitialMoveGoal(goal=goal_pos)
		
		while(not self.reached and not rospy.is_shutdown()): # read it literally lol
			# Call astar service
			start_pos = Point() # we initialize the starting position if the bot as an instance of the Point() class message
			start_pos.x = self.cur_x # then we specify each of its attributes
			start_pos.y = self.cur_y

			'''
			call the astar service and get waypoints
			'''
			


			try:        
				response = self.astar_service(self.obstacle_map, start_pos, goal_pos) # Sending the map, start_pos and goal_pos to A* service, which will then return a path as a trajectory, ie a list of Point()s, IKR!!! Majik!!!
				print(response.ack) # A* service response, ack means acknowledge btw... its a bool variable which defines wheter A* was successful.
				if response.ack: # if A* is successful first we say YAY!!! and then assign its responses to lists defined above
					print("A* planner successfully generated the path to goal!!")
					self.trajectory_x = response.trajectory_x
					self.trajectory_y = response.trajectory_y
				else: # if it fails we say NOOO!! and raise up self.err and break out of the entire root controller (IK Sadge right)
					print("A* planner failed to generate the path to goal!!")
					self.err = True
					break
			except rospy.ServiceException as e: # Error handling again but this one lite, you can mostly do nothing abt it (unless if u are careless enough to devide by a 0 in the service you know... ofc not speaking from experience)
				print("Service call failed:" + str(e))
				self.err = True
				break

			'''
			call the path follower script to follow the waypoints
			'''
			# Creates a goal to send to the action server.
			goal = planner.msg.PathFollowGoal(trajectory_x = self.trajectory_x, trajectory_y = self.trajectory_y)

			# Sends the goal to the action server.
			self.path_follow_client.send_goal(goal, feedback_cb = self.move_fb)

			# Waits for the server to finish performing the action.
			self.path_follow_client.wait_for_result()

			# result of executing the action
			status = self.path_follow_client.get_result()  # ack
			if status.ack == False:
				# obstacle not detected in front, reached goal
				self.reached = True
		
		if self.reached and (not self.err): # if reach the goal sya YaY!!! else say NOOOO!!!!
			print("Successfully reached the goal point!")
		else:
			print("Service Call Failed/ Planner failed to generate the path.")

		while(self.single_aruco_part and not self.gate_aruco_part and not rospy.is_shutdown()):
			print(self.single_aruco_part)
			if self.searching:
				print("started spiraling")
				response = self.spiral_search_service(True,self.cur_x,self.cur_y)
				print("done spiralling")
				self.searching=response.searched
			else:
				self.stop()
				print("following tag now")
				self.follow_goal = planner.msg.FollowTagGoal(class_name='tag')
				self.follow_tag_client.send_goal(self.follow_goal)
				self.follow_tag_client.wait_for_result()
		
				if self.follow_tag_client.get_result().reached == False:
					print("Failed to reach tag within 2m spiralling again")
					try:        
						response = self.astar_service(self.obstacle_map, start_pos, goal_pos) # Sending the map, start_pos and goal_pos to A* service, which will then return a path as a trajectory, ie a list of Point()s, IKR!!! Majik!!!
						print(response.ack) # A* service response, ack means acknowledge btw... its a bool variable which defines wheter A* was successful.
						if response.ack: # if A* is successful first we say YAY!!! and then assign its responses to lists defined above
							print("A* planner successfully generated the path to goal!!")
							self.trajectory_x = response.trajectory_x
							self.trajectory_y = response.trajectory_y
						else: # if it fails we say NOOO!! and raise up self.err and break out of the entire root controller (IK Sadge right)
							print("A* planner failed to generate the path to goal!!")
							self.err = True
							break
					except rospy.ServiceException as e: # Error handling again but this one lite, you can mostly do nothing abt it (unless if u are careless enough to devide by a 0 in the service you know... ofc not speaking from experience)
						print("Service call failed:" + str(e))
						self.err = True
						break
				else:
					print("Reached!")
					self.stop()
					break
					
					rospy.sleep(15)
		while(self.gate_aruco_part and not self.single_aruco_part and not rospy.is_shutdown()):
			print(self.gate_aruco_part)
			if self.searching:
				print("started spiraling")
				response = self.spiral_search_service(True,self.cur_x,self.cur_y)
				print("done spiralling")
				self.searching=response.searched
			else:
				self.stop()
				print("following tag now")
				self.follow_goal = planner.msg.FollowGateGoal(class_name='gate')
				self.follow_gate_client.send_goal(self.follow_goal)
				self.follow_gate_client.wait_for_result()

				if self.follow_gate_client.get_result().reached == False:
					print("NOT passed the gate!")
					try:        
						response = self.astar_service(self.obstacle_map, start_pos, goal_pos) # Sending the map, start_pos and goal_pos to A* service, which will then return a path as a trajectory, ie a list of Point()s, IKR!!! Majik!!!
						print(response.ack) # A* service response, ack means acknowledge btw... its a bool variable which defines wheter A* was successful.
						if response.ack: # if A* is successful first we say YAY!!! and then assign its responses to lists defined above
							print("A* planner successfully generated the path to goal!!")
							self.trajectory_x = response.trajectory_x
							self.trajectory_y = response.trajectory_y
						else: # if it fails we say NOOO!! and raise up self.err and break out of the entire root controller (IK Sadge right)
							print("A* planner failed to generate the path to goal!!")
							self.err = True
							break
					except rospy.ServiceException as e: # Error handling again but this one lite, you can mostly do nothing abt it (unless if u are careless enough to devide by a 0 in the service you know... ofc not speaking from experience)
						print("Service call failed:" + str(e))
						self.err = True
						break
				else:
					print("Reached!")
					self.stop()
					break
					
					rospy.sleep(15)
				


		
'''	
Calling the RootClient class
'''
if __name__ == "__main__":

	rospy.init_node("controller_client")
	#This is just initialization of the program, basically take sys args in pairs of 2 and put them as goal points one after another  till you complete the course
	words=[]
	goal_x = []
	goal_y = []
	
	file=open("catkin_ws/src/planner-2022-Autonomous-rospy-/scripts/root_cont_coordinates.txt","r")
	line= file.readlines()[0]
	for word in line.split():
		words.append(word)
	print(words)
	file.close()
	for i in range(0,13,2):
		goal_x.append(words[i])
		goal_y.append(words[i+1])
	print(goal_x)
	print(goal_y)
	root = RootClient()
	for i in range(3):
		print(i)

		root.main(goal_x[i], goal_y[i],False,False)
	for i in range(3):
		print(i)

		root.main(goal_x[i+3], goal_y[i+3],True,False)


	root.main(goal_x[6], goal_y[6],False,True)
		
	rospy.spin()
	print(i)
	rospy.logwarn("Killing!")

	print("Reached gnss coordinate {} !".format(i+1))
	
