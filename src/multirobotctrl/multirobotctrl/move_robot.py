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
	def __init__(self,name):
		super().__init__(name)
		self.pub = self.create_publisher(Twist,'cmd_vel', 1)
		self.sub = self.create_subscription(Pose2D, '/send_all_robot_pos', 1) #Topic to receive coordinates from
		self.declare_parameter("linear_speed_limit", 1.0)
		self.declare_parameter("angular_speed_limit", 5.0)
		self.linear_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.rate = self.create_rate(2)
		self.settings = termios.tcgetattr(sys.stdin)
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)		
	
def main():
	rclpy.init()
	yahboom_control = Coordinate_Control("yahboom_coordinate_ctrl")
	(speed, turn) = (0.2, 1.0)
	x = 0
	stop = False
	twist = Twist()
	Ks = .5
	Ka = .5
	
	position = Pose2D()
	position = yahboom_control.sub.subscribe("/send_all_robot_pos")  #Use Pose2d
	goalx = position.x
	goaly = position.y
	
	(posx, posy) = (0,0)
	dt = 1/yahboom_control.rate
	
	try:
		print(yahboom_control.vels(speed, turn))
		while speed > .1 :
			#Error between goal and current position
			speed = Ks*np.linalg.norm(np.array(posx, posy) - np.array(goalx, goaly))
			turn = np.tan((goaly - posy)/(goalx - posx))
			x = np.cos(turn)
			y = np.sin(turn)
			
            #Compare speed to max paramters
			if speed > yahboom_control.linear_speed_limit: 
				speed = yahboom_control.linear_speed_limit
			if turn > yahboom_control.angular_speed_limit: 
				turn = yahboom_control.angular_speed_limit
			print(yahboom_control.vels(speed, turn))
			
            #Calculate the next position based on speed and timestep - Open Loop control
			posx += dt*speed*x
			posy += dt*speed*y
			
			#Update the Twist message
			twist.linear.x = speed * x
			twist.linear.y = speed * y
			twist.angular.z = turn
			
			yahboom_control.pub.publish(twist)
			yahboom_control.rate.sleep()
			
	except Exception as e: print(e)
	finally: yahboom_control.pub.publish(Twist())
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, yahboom_control.settings)
	yahboom_control.destroy_node()
	rclpy.shutdown()
