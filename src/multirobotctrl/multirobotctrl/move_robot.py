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
		self.pub = self.create_publisher(Twist,'cmd_vel', 1)
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
	def listener_callback(self, msg):
		self.get_logger().info("Sending Twist to Robot...")
		x = 0
		stop = False
		twist = Twist()
		Ks = .5
		Ka = .5
		new_msg = Pose2D()
		new_msg = msg
		
		goalx = float(new_msg.x)
		goaly = float(new_msg.y)
		
		(posx, posy) = (self.robot.x,self.robot.y)
		dt = .5
		
		try:
			
			#Error between goal and current position
			speed = Ks*np.linalg.norm(np.array([posx, posy]) - np.array([goalx, goaly]))
			turn = np.arctan2((goaly - posy),(goalx - posx))
			x = float(np.cos(turn))
			y = float(np.sin(turn))
			
			#Compare speed to max paramters
			if speed > self.linear_speed_limit: 
				speed = self.linear_speed_limit
			if turn > self.angular_speed_limit: 
				turn = self.angular_speed_limit

			print(self.vels(speed, turn))
			
			#Calculate the next position based on speed and timestep - Open Loop control
			posx += dt*speed*x
			posy += dt*speed*y

			self.robot.x = posx
			self.robot.y = posy
			
			#Update the Twist message
			twist.linear.x = speed * x
			twist.linear.y = speed * y
			twist.angular.z = turn

			#Check if by goal:
			if speed < .1:
				twist.linear.x = 0
				twist.linear.y = 0
				twist.linear.z = 0
				twist.angular.x = 0
				twist.angular.y = 0
				twist.angular.z = 0
				self.pub.publish(twist)
				self.get_logger().info("At Goal")
			
			self.pub.publish(twist)
			self.get_logger().info("x,y speed is " + str(twist.linear.x) + "," + str(twist.linear.y))
				
		except Exception as e: print(e)
		#finally: self.pub.publish(Twist())
		
def main():
	rclpy.init()
	robot = Robot("robot1", 0.0,0.0)
	number = input("Input robot number")
	yahboom_control = Coordinate_Control("yahboom_coordinate_ctrl", number, robot)
	rclpy.spin(yahboom_control)
	yahboom_control.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
    main()
