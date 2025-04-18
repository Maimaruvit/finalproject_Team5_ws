#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist, Pose2D
import numpy as np

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt

class Robot():
    def __init__(self, label, x, y, goalx, goaly):
        self.label = label
        self.x = x
        self.y = y
        self.goalx = goalx
        self.goaly = goaly
        self.connections = []
		
class RobotControl(Node):
	def __init__(self,name):
		super().__init__(name)
		self.pub1 = self.create_publisher(Pose2D,'/robot1/send_all_robot_pos', 1)
		self.pub2 = self.create_publisher(Pose2D,'/robot2/send_all_robot_pos', 1)
		self.pub3 = self.create_publisher(Pose2D,'/robot3/send_all_robot_pos', 1)
		self.rate = self.create_rate(2)
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)	

def main():
	rclpy.init()
	yahboom_control = RobotControl("yahboom_coordinate_ctrl")
	dt = .5
	iterations = 250
	robots = []
	goals = [np.array([0,-1]), np.array([0,0]), np.array([0,1])]
	starts = [np.array([1,0]), np.array([-1,-1]), np.array([-1,0])]
	labels = ["robot1", "robot2", "robot3"]

	#Create robots, assign start positions and goals
	for (i,g,s,l) in zip(range(0,3), goals,starts,labels) :
		if i < 6:
			robots.append(Robot(label=l, x=s[0], y=s[1], goalx=g[0], goaly=g[1]))

	#Assign Connections
	robots[0].connections.extend((robots[1], robots[2]))
	robots[1].connections.extend((robots[0], robots[2]))
	robots[2].connections.extend((robots[0], robots[1]))

	try:
		#Shape based formation control
		for i in range(0, iterations):
			yahboom_control.get_logger().info("Iteration:" + str(i))
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

				yahboom_control.get_logger().info("Robot x:"+str(robot.x)+"Robot y:"+str(robot.y))

				#Define msg to send
				msg = Pose2D()
				msg.x = robot.x
				msg.y = robot.y

				#Publish position to each robot
				if robot.label == "robot1":
					yahboom_control.pub1.publish(msg)
				elif robot.label =="robot2":
					yahboom_control.pub2.publish(msg)
				elif robot.label == "robot3":
					yahboom_control.pub3.publish(msg)
				else:
					yahboom_control.get_logger().info(":(")

				
			
	except Exception as e: print(e)
	#finally: yahboom_control.pub.publish(Pose2D())
	yahboom_control.destroy_node()
	rclpy.shutdown()
