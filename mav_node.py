#!/usr/bin/env python

import rospy
import numpy as np

import serial

from uav_msgs.msg import IMUSample
from uav_msgs.msg import OptFlowSample
#from uav_msgs.msg import UavCmd

import mavlink.mavlink as mavlink

class MavNode:

	# UAV start time (in ros time) -- used for synchronization
	uav_time_start = None
	uav_latency = None

	# Mavlink-facing interface
	port = None
	serial_name = None
	mav = None 		# MAVLink object
	sys_id = 81#37
	comp_id = 50#1

	# ROS-facing interface
	# NOTE: Define additional publishers/subscribers here
	imu_pub = None
	of_pub = None
	cmd_sub = None

	def __init__(self, serial_name, baud_rate=None):

		self.uav_time_start = None
		self.uav_latency = None

		rospy.loginfo('Opening serial port.')
		self.serial_name = serial_name
		self.port = serial.Serial(port=self.serial_name, timeout=0)
		self.mav = mavlink.MAVLink(self.port, self.sys_id, self.comp_id)
		if (baud_rate != None):
			self.port.baudrate = baud_rate

		rospy.loginfo('Initializing ROS-facing interface...')
		# NOTE: Here is where you initialize any additional ROS publishers
		self.imu_pub = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=5)
		self.of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)
		# NOTE: And here you initialize ROS subscribers and link them to their callback functions
		#self.cmd_sub = rospy.Subscriber('uav_cmds/cmd', UavCmd, self.rosUavCmd)
		
		return

	# maintains running estimate of conversion from uav_time to rostime
	def uavTimeToRosTime(self, mav_time_ms, msg_receive_rostime):
		if not isinstance(msg_receive_rostime, rospy.Time):
			raise Exception('Invalid argument type received for msg_receive_rostime.')

		# if clock synchronization not initialized yet
		if self.uav_time_start is None or self.uav_latency is None:
			return msg_receive_rostime

		# otherwise perform estimation
		else:
			# update start time estimate with crude LPF
			dt = rospy.Duration.from_ms(mav_time_ms)
			t0 = msg_receive_rostime - self.uav_latency - dt
			
			correction = 0.1*(t0 - self.uav_time_start)
			self.uav_time_start += correction

			return self.uav_time_start + dt

	def spin(self):
		while True:
			self.mavCheck()
		return

	def mavCheck(self):
		rx_time = rospy.Time.now()

		count = self.port.inWaiting()
		if count == 0:
			return
		in_chars = self.port.read(count)

		msgs = self.mav.parse_buffer(in_chars)
		try:
			for m in msgs:
				self.mavHandler(m, rx_time)
		except TypeError:
			pass
		return

	## incoming ROS message handlers
	def rosUavCmd(self, uav_cmd):
		# do something with incoming command message from ROS
		# probably send Mission Item message to UAV
		pass

	## incoming MAVLink message handlers
	def mavHandler(self, msg, rx_time_ros):
		mid = msg.get_msgId()
		#print mid

		# NOTE: Here add if clauses to catch additional Mavlink message types
		if mid == 0: #mavlink.MAVLINK_MSG_ID_HEARTBEAT:
			pass
		elif mid == 1: #mavlink.MAVLINK_MSG_ID_SYSTEM_STATUS:
			pass
		elif mid == mavlink.MAVLINK_MSG_ID_HIGHRES_IMU: #HIGHRES_IMU #105
			self.mavHighResImu(msg, rx_time_ros)
		elif mid == mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD #106
			self.mavOpticalFlow(msg, rx_time_ros)
		elif mid == 74: #VFR_HUD
			pass
		elif mid == 32: #LOCAL_POSITION_NED
			pass
		elif mid == 87: #POSITION_TARGET_GLOBAL_INT
			pass
		elif mid == 83: #ATTITUDE_TARGET
			pass
		elif mid == 30: #ATTITUDE
			pass
		elif mid == 22: #PARAM_VALUE
			pass
		else:
			rospy.logwarn('Received unrecognized message with id %d.', msg.get_msgId())

		return

	def mavHighResImu(self, msg, rx_time_ros):		
		imu = IMUSample()
		imu.gyro_x = msg.xgyro 	# [rad/s]
		imu.gyro_y = msg.ygyro
		imu.gyro_z = msg.zgyro
		imu.acc_x = msg.xacc	# [m/s^2]
		imu.acc_y = msg.yacc 
		imu.acc_z = msg.zacc 
		imu.mag_x = msg.xmag 	# to [Gauss]
		imu.mag_y = msg.ymag 
		imu.mag_z = msg.zmag 

		imu.timestamp = self.uavTimeToRosTime(msg.time_usec, rx_time_ros)

		self.imu_pub.publish(imu)
		rospy.loginfo('Published imu message')
		return

	def mavOpticalFlow(self, msg, rx_time_ros):
		of = OptFlowSample()	
		of.integrated_xgyro = msg.integrated_xgyro
		of.integrated_ygyro = msg.integrated_ygyro
		of.integrated_x = msg.integrated_x
		of.integrated_y = msg.integrated_y
		of.ground_distance = msg.distance
		of.temperature = msg.temperature
		of.integration_time_us = msg.integration_time_us

		of.time_delta_distance_us = msg.time_delta_distance_us
		of.quality = msg.quality

		of.timestamp = self.uavTimeToRosTime(msg.time_usec, rx_time_ros)

		self.of_pub.publish(of)
		rospy.loginfo('Published optical flow message')
		return

rospy.init_node('comm_node')

mav_node = MavNode(serial_name='/dev/ttyACM0', baud_rate=None)
mav_node.spin()