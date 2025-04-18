#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist, Pose2D
import numpy as np

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D

class Robot():
    def __init__(self, label, x, y):
        self.label = label
        self.x = float(x)
        self.y = float(y)

class Coordinate_Control(Node):
	def __init__(self, name, number, robot):
		super().__init__(name)
		self.pub = self.create_publisher(Twist,'/cmd_vel', 1)
		subcriberString = '/robot'+str(number)+'/send_all_robot_pos'
		self.sub = self.create_subscription(Pose2D, subcriberString, self.listener_callback, 1) #Topic to receive coordinates from
		self.declare_parameter("linear_speed_limit", 1.0)
		self.declare_parameter("angular_speed_limit", 5.0)
		self.linear_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.rate = self.create_rate(2)
		self.robot = robot
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)	
	def listener_callback(self, msg, robot=Robot):
		self.get_logger().info("Sending Twist to Robot...")
		twist = Twist()
		Ks = 1
		Ka = 1
		new_msg = Pose2D()
		new_msg = msg
		
		goalx = float(new_msg.x)
		goaly = float(new_msg.y)
		
		(posx, posy) = (self.robot.x,self.robot.y)
		dt =1/181
		
		try:
			#Error between goal and current position
			xchange = goalx-posx
			ychange = goaly-posy
			
			#Compare speed to max paramters
			if xchange > self.linear_speed_limit: 
				xchange = self.linear_speed_limit
			if xchange < -self.linear_speed_limit: 
				xchange = -self.linear_speed_limit
			if ychange > self.linear_speed_limit: 
				ychange = self.linear_speed_limit
			if ychange < -self.linear_speed_limit: 
				ychange = -self.linear_speed_limit
			
			#Calculate the next position based on speed and timestep - Open Loop control
			posx += dt*xchange
			posy += dt*ychange

			self.robot.x = posx
			self.robot.y = posy
			
			#Update the Twist message
			twist.linear.x = xchange
			twist.linear.y = ychange
			
			distance = np.linalg.norm(np.array([posx, posy]) - np.array([goalx, goaly]))

			#Check if by goal:
			if distance < 0.75:
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
				twist.angular.x = 0.0
				twist.angular.y = 0.0
				twist.angular.z = 0.0
				self.get_logger().info("At Goal")
			
			self.pub.publish(twist)
			self.get_logger().info("Current Goal:" + str(goalx) + " " + str(goaly))
			self.get_logger().info("x,y speed is " + str(twist.linear.x) + "," + str(twist.linear.y))
				
		except Exception as e: print(e)
		#finally: self.pub.publish(Twist())
		
def main():
	rclpy.init()
	robot = Robot("robot", 0.0,0.0)
	number = 2
	yahboom_control = Coordinate_Control("yahboom_coordinate_ctrl", number, robot)
	rclpy.spin(yahboom_control)
	yahboom_control.destroy_node()
	rclpy.shutdown()
