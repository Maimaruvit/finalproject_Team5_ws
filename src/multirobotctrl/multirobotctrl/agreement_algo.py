#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist, Pose2D
import sys, select, termios, tty
import numpy as np

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt

class Robot():
    def __init__(self, label, color, x, y, goalx, goaly):
        self.label = label
        self.color = color
        self.x = x
        self.y = y
        self.goalx = goalx
        self.goaly = goaly
        self.connections = []
		
class RobotControl(Node):
	def __init__(self,name):
		super().__init__(name)
		self.pub = self.create_publisher(Pose2D,'/send_all_robot_pos', 1)
		self.rate = self.create_rate(2)
		self.settings = termios.tcgetattr(sys.stdin)
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)	


def main():
	rclpy.init()

	yahboom_control = RobotControl("yahboom_coordinate_ctrl")
	dt = .5
	iterations = 3000
	robots = []
	goals = [np.array([10,20]), np.array([10,30]), np.array([10,40])]
	starts = [np.array([5,0]), np.array([10,0]), np.array([15,0])]
	labels = ["robot1", "robot2", "robot3"]

	#Create robots, assign start positions and goals
	for (i,g,s,l) in zip(range(0,3), goals,starts,labels) :
		if i < 6:
			robots.append(Robot(i, l, s[0], s[1], g[0], g[1]))

	#Assign Connections
	robots[0].connections.extend((robots[1], robots[2]))
	robots[1].connections.extend((robots[0], robots[2]))
	robots[2].connections.extend((robots[0], robots[1]))

	try:
		#Shape based formation control
		for i in range(0, iterations):
			for robot in robots:
				connectx = []
				connecty = []
				connectgoalx = []
				connectgoaly = []
				for unit in robot.connections:
					connectx.append(unit.x)
					connecty.append(unit.y)
					connectgoalx.append(unit.goalx)
					connectgoaly.append(unit.goaly)
				relposx = 0
				relposy = 0
				relgoalx = 0
				relgoaly = 0
				for i in range(len(robot.connections)):
					relposx += (connectx[i] - robot.x)
					relposy += (connecty[i] - robot.y)
					relgoalx += (connectgoalx[i] - robot.goalx)
					relgoaly += (connectgoaly[i] - robot.goaly)

				robot.x = robot.x + dt*(relposx - relgoalx)
				robot.y = robot.y + dt*(relposy - relgoaly)

				#Define msg to send
				msg = Pose2D()
				msg.x = robot.x
				msg.y = robot.y

				#Publish position to each robot
				yahboom_control.pub.publish(msg)
				yahboom_control.get_logger().info(str(i))
				yahboom_control.get_logger().info(str(msg.x) + str(msg.y))
				yahboom_control.rate.sleep()
			
	except Exception as e: print(e)
	#finally: yahboom_control.pub.publish(Pose2D())
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, yahboom_control.settings)
	yahboom_control.destroy_node()
	rclpy.shutdown()
