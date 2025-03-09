#! /usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import String
from math import pow, atan2, sqrt, pi
import random

nodeid = str(sys.argv[1])
nodename = 'turtle'+nodeid

class Turtle2Listener:
	def __init__(self):
        #Start the ROS Node 
		rospy.init_node('turtle2_listener', anonymous=True)
		self.pub = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
		self.sub = rospy.Subscriber(nodename + '/pose', Pose, self.update_pose)
        # Make the subscriber for listen to message_to_turtle2 topic
		self.msg_sub = rospy.Subscriber('message_to_turtle2', String, self.message_process)
		self.msg_pub = rospy.Publisher('message_to_turtle3', String, queue_size=10)
		self.pose = Pose()
		self.rate = rospy.Rate(10)  # 10 Hz
		self.received_message = None  # Received message from Turtle

	def update_pose(self, data):
		self.pose = data
	
	def message_process(self, msg):
        	# Print received message on terminal 
		rospy.loginfo(f"Turtle2 received message: {msg.data}")
		self.received_message = msg.data
		random_message_2 = String()
		random_message_2.data = str(random.randint(0, 1))  # Converting to the string form
		msg2 = msg.data + random_message_2.data
		self.move2goal(msg2)

	def run(self):
        # Node is running and listening message infinitely
		rospy.spin()
	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.x - self.pose.x), 2)+pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		return constant *self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
	
		

	def angular_vel(self, goal_pose, constant=6):
		return constant *(self.steering_angle(goal_pose) - self.pose.theta)
		
	
	def move2goal(self, msg2):
		goal_pose = Pose()
		if nodeid == '2':
			goal_pose.x = 5.5
			goal_pose.y = 4.0
		
	
		dist_tolerance = 0.1
		
		vel_msg = Twist()
		
		while self.euclidean_distance(goal_pose) >= dist_tolerance:
			
			vel_msg.linear.x = self.linear_vel(goal_pose)
			vel_msg.angular.z = self.angular_vel(goal_pose)

			#print(str(self.pose.position.x), str(self.pose.position.y))
			self.pub.publish(vel_msg)
			self.rate.sleep()		
			
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.pub.publish(vel_msg)
		rospy.loginfo("Turtle2 has reached target, sending concatenated message")
		
		msg_to_publish = String()
		msg_to_publish.data = msg2
		self.msg_pub.publish(msg_to_publish)
		rospy.loginfo(f"Turtle2: Sended message to Turtle3: {msg2}")
		rospy.spin()
	
	


if __name__ == "__main__":
	try:
		
		listener = Turtle2Listener()
		listener.run()
		listener.move2goal()
	except rospy.ROSInterruptException:
		pass
