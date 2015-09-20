#!/usr/bin/env python
# MIT UAV Team
# sources:
# mavros_offboard_attctrl_test.py
# setpoint_demo.py
#
# To run this script:
# On one tab, start the core service of ROS:
# roscore
# On another tab, connect to the pixhawk via mavros
# roslaunch mavros px4.launch 
# OR, custom to comm team,
# roslaunch mavros comm/launch/comm_node.launch 
# roslaunch mavros comm/launch/px4_radio.launch
# Then start the script
# python testPosition.py
# Note: behavior can be specified below in the testFlying() method, 
# or other similar ones

# ssh odroid@[odroidipaddress] pw: odroid
# use visual display and ipconfig to find 
# roslaunch comm px4.launch
# odroid@18.111.105.66

import rospy
import numpy as np
import thread
import threading
import time
import serial
from math import *

from uav_msgs.msg import IMUSample
from uav_msgs.msg import OptFlowSample
from uav_msgs.msg import UavCmd
from uav_msgs.msg import SimpleUavCmd

from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros.utils import *

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
	nav_pub = None #publishes to quadcopter to tell it where to go
	alg_sub = None
	locpos_sub = None
	vicon_pub = None #publishes to vicon

	# current setpoint
	x = 0
	y = 0
	z = 0

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
		self.nav_pub =  rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.vicon_pub = rospy.Publisher('/mavros/vicon_pose/pose',PoseStamped,queue_size=5)
		# NOTE: And here you initialize ROS subscribers and link them to their callback functions		
		self.alg_sub = rospy.Subscriber('simple_uav_cmd',SimpleUavCmd,self.next_cmd)
		self.locpos_sub = rospy.Subscriber('mavros/local_position/local', PoseStamped, self.locpos_received)
		self.vicon_sub = rospy.Subscriber('/Batman/pose', PoseStamped, self.sendViconData)

		# TODO subscribe to node that publishes VICON and send it over with self.sendViconData
		try:
			thread.start_new_thread( self.navigate, () )
		except:
			print "Error: Unable to start offboard command sending thread"

	def next_cmd(self, a):
		# a is the message that defines the local x,y,z of where the drone goes

		# format of these coordinates with respect to gps layout 
		# not certain, are we assuming drone has established 	
		# interface so apiconnect is needed?
		#if not a instanceof WayPointPost:
		#	print('bad parameter')
		#	return

		if (a.x_dir == None or a.x_dir == 0):
			print "no destination"
			return

		self.sendPositionMsg(a.x_dir,a.y_dir,a.z_dir)
		return


	def locpos_received(self,m):
		rospy.loginfo('received')
		self.local_positionx = m.pose.position.x
		self.local_positiony = m.pose.position.y
		self.local_positionz = m.pose.position.z

		#This represents an orientation in free space in quaternion form
		#x,y,z,w
		self.local_orientation = m.pose.orientation 


	def sendPositionMsg(self,x,y,z,delay=0,wait=False):
		cmd = UavCmd()
		cmd.type = 1 #POS_TARGET
		cmd.pos_x = x
		cmd.pos_y = y
		cmd.pos_z = z # z points up, but ROS will convert it to down
		cmd.haste = 0.5 
		# Coefficient describing how quickly or how carefully to move: proportion of maximum haste [0,1]
		# rospy.loginfo(cmd)
		self.cmd_pub.publish(cmd)
		self.x = x
		self.y = y
		self.z = z

		if wait:
			rate = rospy.Rate(5)
			while not self.is_at_position(self.x, self.y, self.z, 0.5):
				rate.sleep()

		time.sleep(delay)
		return



	def check(self):
		# check if theres a new cmd_sub
		self.sendPositionMsg(self.x,self.y,self.z,delay=0,wait=False)
		return

	def spin(self):
		rospy.spin()

	def sendTakeoff(self, height):
		self.sendPositionMsg(0,0,height)
		self.x = 0
		self.y = 0
		self.z = height
		return

	def sendPath(self):
		positions = (	# NED
			(0, 0, 0), #ground
			(0, 0, 1), #takeoff
			(1, 1, 1), #front right
			(1, -1, 1), #bottom right
			(-1, -1, 1), #bottom left
			(1, 1, 1)) #front right

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

	def flyCircle(self,radius=20):
		offset_x = 0.0
		offset_y = 0.0
		height = 10.0
		sides = 360    	
		self.sendPositionMsg(0.0, 0.0, height, 3)   # Climb to the starting height first
		i = 0
		while not rospy.is_shutdown():
			x = radius * cos(i*2*pi/sides) + offset_x   
			y = radius * sin(i*2*pi/sides) + offset_y
			z = height

			wait = False
			delay = 0
			if (i == 0 or i == sides):
				# Let it reach the setpoint.
				wait = True
				delay = 5

			setpoint.set(x, y, z, delay, wait)

			i = i + 1 
			rate.sleep()

			if (i > sides):
				print "Fly home"
				setpoint.set(0.0, 0.0, height, 5)
				break

	# used to send position commands to the quadcopter
	def navigate(self):
		offboard_rate = rospy.Rate(10) # 10hz

		msg = PoseStamped()
		msg.header = Header() 
		msg.header.frame_id = "base_footprint"
		msg.header.stamp = rospy.Time.now()

		while 1:
			msg.pose.position.x = self.x
			msg.pose.position.y = self.y
			msg.pose.position.z = self.z

			# we will lock yaw/heading to north.
			yaw_degrees = 0  # North
			yaw = radians(yaw_degrees)
			quaternion = quaternion_from_euler(0, 0, yaw)
			msg.pose.orientation = Quaternion(*quaternion)

			self.nav_pub.publish(msg)
			offboard_rate.sleep()

	def sendViconData(self,m):
		self.vicon_pub.publish(m)	

def testFlying():
	rospy.init_node('comm_node',anonymous=True)
	pixhawk = Pixhawk()
	r = rospy.Rate(10) #changed from 100 to 10
	pixhawk.setRate(r)

	rospy.loginfo("Commanding Climb through mavros")
	pixhawk.sendPositionMsg(0,0,2,delay=0)

	# takeoff 1 meter
	rospy.loginfo("Commanding Sink through mavros")
	pixhawk.sendPositionMsg(0,0,1,delay=5)

	
	#pixhawk.sendPath()
	#pixhawk.circle()
	while not rospy.is_shutdown():
		pixhawk.check()
		r.sleep()

if __name__ == '__main__':
	try:
		testFlying()
	except rospy.ROSInterruptException:
		pass