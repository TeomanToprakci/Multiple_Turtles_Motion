#! /usr/bin/env python3

#Make a python node executable
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/Turtle1move.py 

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


class TurtleSemiCircleDrawer:
	def __init__(self):
		rospy.init_node('semi_circle_drawer', anonymous=True)
		self.pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)
		#To take the current position of the turtle, subscribe to the turtle1/pose topic
		self.sub = rospy.Subscriber(nodename+'/pose', Pose, self.update_pose)
		#Holds the current position of the turtle,
		self.msg_pub = rospy.Publisher('message_to_turtle2', String, queue_size=10)
		self.pose = Pose()
		self.previous_pose = None 
		self.total_distance = 0.0
		self.rate = rospy.Rate(10)
		self.drawing_completed = False  #Checking if drawing completed

        	#During movement control updating position
		self.pause_pose_updates = False
	
	def angle_diff(self, target, current):
        #Normalise difference of between two angle with -pi and pi
		diff = target - current
		while diff > pi:
			diff -= 2 * pi
		while diff < -pi:
			diff += 2 * pi
		return diff

	def update_pose(self, data):
		# Updating the turtle's position and tracking completed range
		if self.previous_pose is not None:
		# Calculate the distance between previous position and current position
			distance = sqrt(pow((data.x - self.previous_pose.x), 2) + pow((data.y - self.previous_pose.y), 2))
			self.total_distance += distance

		self.previous_pose = Pose()
		self.previous_pose.x = data.x
		self.previous_pose.y = data.y
		self.previous_pose.theta = data.theta

		self.pose = data

	def reset_distance(self):
	# Reset the total distance
		self.total_distance = 0.0

	def draw_semi_circle(self, radius, speed):
		vel_msg = Twist()

        # Adjust the linear and angular speed
		vel_msg.linear.x = speed
		vel_msg.angular.z = speed / radius
		
		start_theta = self.pose.theta
		target_theta = start_theta + pi
		target_theta = (target_theta + pi) % (2 * pi) - pi # Normalise the target theta
		
		while True:
			current_theta = self.pose.theta
			diff = self.angle_diff(target_theta, current_theta)
			rospy.logdebug(f"Current theta: {current_theta}, Target theta: {target_theta}, Diff: {diff}")

			if abs(diff) < 0.05:  # 0.05 radian tolerance
				break

			self.pub.publish(vel_msg)
			self.rate.sleep()
        
        # Stop the turtle
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.pub.publish(vel_msg)

	def draw_multiple_semi_circles(self):
		initial_radius = 0.3  # Initial radius
		increment = 0.1      # radius increment value
		speed = 0.3         
		current_radius = initial_radius
		
		self.reset_distance()  #Before drawing, reset the total distance
		
		for i in range(6):  # 6 half circle
			rospy.loginfo(f"Drawing semi-circle {i + 1} with radius {current_radius}")
			self.draw_semi_circle(current_radius, speed)
			current_radius += increment  # Increase the radius
		self.drawing_completed = True


	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.x - self.pose.x), 2)+pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		return constant *self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
	
		

	def angular_vel(self, goal_pose, constant=6):
		return constant *(self.steering_angle(goal_pose) - self.pose.theta)
	def main(self):
		self.draw_multiple_semi_circles()
		
		if self.drawing_completed:
			self.move2goal()
		
	
	def move2goal(self):
		goal_pose = Pose()
		if nodeid == '1':
			goal_pose.x = 9.0
			goal_pose.y = 7.5
		else:

			goal_pose.x = 7.0
			goal_pose.y = 7.0
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
		rospy.loginfo("Turtle1 has reached the target, sending random message...")
		self.send_random_message_to_turtle2()
		rospy.spin()
	def send_random_message_to_turtle2(self):
    		#Producing a random message and sending to Turtle2
		random_message = String()
		random_message.data = str(random.randint(0, 1))  # Message converting into string form
		self.msg_pub.publish(random_message)
		rospy.loginfo(f"Turtle1 producing a random message: {random_message}")
		

if __name__ == "__main__":
	try:
		drawer = TurtleSemiCircleDrawer()
		drawer.main()
        
	except rospy.ROSInterruptException:
		pass
	
	
