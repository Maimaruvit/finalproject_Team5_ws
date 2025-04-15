#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist, Pose2D
import sys, select, termios, tty
import numpy as np

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D

class Coordinate_Control(Node):
	def __init__(self, name, number):
		super().__init__(name)
		self.pub = self.create_publisher(Twist,'cmd_vel', 1)
		subcriberString = '/robot'+str(number)+'/send_all_robot_pos'
		self.sub = self.create_subscription(Pose2D, subcriberString, self.listener_callback, 1) #Topic to receive coordinates from
		self.declare_parameter("linear_speed_limit", 1.0)
		self.declare_parameter("angular_speed_limit", 5.0)
		self.linear_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.rate = self.create_rate(2)
		self.settings = termios.tcgetattr(sys.stdin)
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)	
	def listener_callback(self, msg):
		self.get_logger().info("Sending Twist to Robot...")
		(speed, turn) = (0.2, 1.0)
		x = 0
		stop = False
		twist = Twist()
		Ks = .5
		Ka = .5
		
		new_msg = Pose2D()
		new_msg = msg
		goalx = new_msg.x
		goaly = new_msg.y
		
		(posx, posy) = (0,0)
		dt = .5
		
		try:
			print(self.vels(speed, turn))
			#Error between goal and current position
			speed = Ks*np.linalg.norm(np.array([posx, posy]) - np.array([goalx, goaly]))
			turn = np.arctan2((goaly - posy),(goalx - posx))
			x = np.cos(turn)
			y = np.sin(turn)
			
			#Compare speed to max paramters
			if speed > self.linear_speed_limit: 
				speed = self.linear_speed_limit
			if turn > self.angular_speed_limit: 
				turn = self.angular_speed_limit
			print(self.vels(speed, turn))
			
			#Calculate the next position based on speed and timestep - Open Loop control
			posx += dt*speed*x
			posy += dt*speed*y
			
			#Update the Twist message
			twist.linear.x = speed * x
			twist.linear.y = speed * y
			twist.angular.z = turn

			self.rate.sleep()
			
			self.pub.publish(twist)
			self.get_logger().info("x,y speed is " + str(twist.linear.x) + "," + str(twist.linear.y))
			self.rate.sleep()
				
		except Exception as e: print(e)
		#finally: self.pub.publish(Twist())
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN,self.settings)
		
def main():
	rclpy.init()
	number = input("Input robot number")
	yahboom_control = Coordinate_Control("yahboom_coordinate_ctrl", number)
	rclpy.spin(yahboom_control)
	yahboom_control.destroy_node()
	rclpy.shutdown()
