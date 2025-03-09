#! /usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import String
import random
import time
from math import pi

nodeid = str(sys.argv[1])
nodename = 'turtle'+nodeid

class Turtle3Listener:
	def __init__(self):
        #Start the ROS Node 
		rospy.init_node('turtle3_listener', anonymous=True)
		self.pub = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
		self.sub = rospy.Subscriber(nodename + '/pose', Pose, self.update_pose)
        # Make the subscriber for listen to message_to_turtle3 topic
		self.msg_sub = rospy.Subscriber('message_to_turtle3', String, self.message_process)
		self.pose = Pose()
		self.rate = rospy.Rate(10)  # 10 Hz
		self.received_message = None  # Message received from Turtle2 

	def update_pose(self, data):
		self.pose = data
	
	def message_process(self, msg):
        	# Print received message on terminal 
		rospy.loginfo(f"Turtle3 received message: {msg.data}")
		self.received_message = msg.data
		random_message_3 = String()
		random_message_3.data = str(random.randint(0, 1))  # Converting to the string form
		msg3 = msg.data + random_message_3.data
		self.turtlemovement(msg3)
		rospy.loginfo(f"Turtle3 produced message: {msg3}")
		
		

	def run(self):
	  # Node is running and listening message infinitely
		rospy.spin()
		
	def rotatetask(self, speed, clockwise, angle):

		rospy.loginfo("Rotating the Robot")
		angularspeed = speed *(pi/180)
		angleinradian = angle *(pi/180)
		vel_msg = Twist()
		vel_msg.angular.z = clockwise *angularspeed
		t0=rospy.Time.now().to_sec()
		curr_angle = 0
		while curr_angle < angleinradian:
			self.pub.publish(vel_msg)
			t1=rospy.Time.now().to_sec()
			curr_angle = angularspeed * (t1-t0)
			self.rate.sleep()
		#Stop the Robot	
		vel_msg.angular.z = 0
		#vel_msg.linear.x = 0
		self.pub.publish(vel_msg)
			
	def movedistancetask(self, distance, speed): 
		vel_msg = Twist()
		vel_msg.linear.x = speed
		vel_msg.angular.z = 0
		t0=rospy.Time.now().to_sec()
		curr_distance = 0
		while curr_distance < distance:
			self.pub.publish(vel_msg)
			t1=rospy.Time.now().to_sec()
			curr_distance = speed * (t1-t0)
			self.rate.sleep()
		

	def turtlemovement(self, msg3):
		self.rotatetask(10, -1, 90)
		command = msg3
		
		for char in command:
			self.movedistancetask(1,1)
			if char == "1":
				self.rotatetask(10, -1, 90)
				self.rate.sleep()
				self.movedistancetask(1.5, 1)
				self.rate.sleep()
				self.rotatetask(10, 1, 90)
				self.rate.sleep()
				
	
			else:
				self.rotatetask(10, 1, 90)
				self.rate.sleep()
				self.movedistancetask(1.5, 1)
				self.rate.sleep()
				self.rotatetask(10, -1, 90)
				self.rate.sleep()
				
				
		
if __name__ == "__main__":
	try:
		final_turtle = Turtle3Listener()
		final_turtle.run()
		
		
		
	except rospy.ROSInterruptException:
		rospy.loginfo("Turtle3 Listener ROS Node is ending.")
	
	
		
	
		
		
		
