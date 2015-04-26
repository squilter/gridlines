#!/usr/bin/env python

import rospy
import numpy as np

import serial

from uav_msgs.msg import IMUSample
from uav_msgs.msg import OptFlowSample
from uav_msgs.msg import UavCmd
from uav_msgs.msg import SimpleUavCmd

from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
#mavros_offboard_attctrl_test.py

import mavlink.mavlink as mavlink

class Pixhawk:
	# UAV start time (in ros time) -- used for synchronization
	uav_time_start = None
	uav_latency = None

	# stream rate counters
	cmd_stream_t = None
	cmd_stream_dt = 0

	# ROS-facing interface
	# NOTE: Define additional publishers/subscribers here
	cmd_pub = None
	alg_sub = None
	locpos_sub = None

	# current setpoint
	last_x = 0
	last_y = 0
	last_z = 0

	# last position command send time
	last_cmd_send_time = None

	rate = None

	local_positionx = 0
	local_positiony = 0
	local_positionz = 0

	def __init__(self):		
		self.uav_time_start = None
		self.uav_latency = None

		rospy.loginfo('Initializing ROS-facing interface...')
		# NOTE: Here is where you initialize any additional ROS publishers
		#self.imu_pub = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=5)
		#self.of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)
		self.cmd_pub = rospy.Publisher('uav_cmds/cmd', UavCmd, queue_size=5)

		# NOTE: And here you initialize ROS subscribers and link them to their callback functions		
		self.alg_sub = rospy.Subscriber('simple_uav_cmd',SimpleUavCmd,self.next_cmd)
		self.locpos_sub = rospy.Subscriber('/mavros/local_position/local', PoseStamped, self.locpos_received)

		#/mavros/local_position/local TODO figure out topic type and import it
		return

	def next_cmd(self, a):#format of these coordinates with respect to gps layout not certain, are we assuming drone has established 	 interface so apiconnect is needed?
		#if not a instanceof WayPointPost:
		#	print('bad parameter')
		#	return

		if (a.x_dir == None or a.x_dir == 0):
			print "no destination"
			return

		sendPositionMsg(a.x_dir,a.y_dir,a.z_dir)
		return


	def locpos_received(self,m):
		self.local_positionx = m.pose.position.x
		self.local_positiony = m.pose.position.y
		self.local_positionz = m.pose.position.z

		#This represents an orientation in free space in quaternion form
		#x,y,z,w
		self.local_orientation = m.pose.orientation 


	def sendPositionMsg(self,x,y,z):
		cmd = UavCmd()
		cmd.type = 1 #POS_TARGET
		cmd.pos_x = x
		cmd.pos_y = y
		cmd.pos_z = z # z points down in ROS
		cmd.haste = 0.5 
		# Coefficient describing how quickly or how carefully to move: proportion of maximum haste [0,1]
		rospy.loginfo(cmd)
		self.cmd_pub.publish(cmd)
		self.last_x = x
		self.last_y = y
		self.last_z = z
		return



	def check(self):
		# check if theres a new cmd_sub
		self.sendPosCmd()
		return

	def spin(self):
		rospy.spin()

	def sendTakeoff(self, height):
		self.sendPositionMsg(0,0,height)
		last_x = 0
		last_y = 0
		last_z = height
		return

	def sendPosCmd(self):
		self.sendPositionMsg(self.last_x,self.last_y,self.last_z)
		return

	def sendPath(self):
		positions = (	# NED
			(0, 0, 0), #ground
			(0, 0, -1), #takeoff
			(1, 1, -1), #front right
			(1, -1, -1), #bottom right
			(-1, -1, -1), #bottom left
			(1, 1, -1)) #front right

		for i in range(0, len(positions)):
			self.reach_position(positions[i][0], positions[i][1], positions[i][2], 120)

	def is_at_position(self, x, y, z, offset):
		rospy.logdebug("current position %f, %f, %f" % (self.local_positionx, self.local_positiony, self.local_positionz))
		desired = np.array((x, y, z))
		pos = np.array((self.local_positionx, self.local_positiony, self.local_positionz))
		return linalg.norm(desired - pos) < offset

	def reach_position(self, x, y, z, timeout):
		# set a position setpoint
		self.sendPositionMsg(x,y,z)

		# does it reach the position in X seconds?
		count = 0
		while count < timeout:
			if self.is_at_position(pos.x, pos.y, pos.z, 0.5):
				break
			count = count + 1
			self.rate.sleep()

		if (count < timeout):
			rospy.loginfo("took too long to get to position")

	def setRate(self,r):
		self.rate = r
		return

rospy.init_node('comm_node')
pixhawk = Pixhawk()
r = rospy.Rate(100)
pixhawk.setRate(r)

# send empty position command
#pixhawk.sendPosCmd()
# takeoff 1 meter
pixhawk.sendTakeoff(1)

rospy.loginfo('Commanded takeoff through mavros')


while not rospy.is_shutdown():
	pixhawk.check()
	r.sleep()






