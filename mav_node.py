#!/usr/bin/env python

import rospy
import numpy as np

import serial

from uav_msgs.msg import IMUSample
from uav_msgs.msg import OptFlowSample

import mavlink.mavlink as mavlink

# mavlink message ids
MAV_ID_HEARTBEAT 			= 00
MAV_ID_SYS_STATUS			= 01
MAV_ID_SYSTEM_TIME			= 02
MAV_ID_PING					= 04
MAV_ID_SET_MODE				= 11
MAV_ID_SCALED_IMU 			= 26
MAV_ID_SCALED_PRESSURE 		= 29
MAV_ID_ATTITUDE 			= 30
MAV_ID_LOCAL_POSITION_NED	= 32
MAV_ID_MISSION_ITEM			= 39
MAV_ID_OPTICAL_FLOW			= 100
MAV_ID_BATTERY_STATUS 		= 147

class MavNode:

	# UAV start time (in ros time) -- used for synchronization
	uav_time_start = None
	uav_time_smpl_count = 0

	# Mavlink-facing interface
	port = None
	serial_name = None
	mav = None 		# MAVLink object
	sys_id = 37
	comp_id = 1

	# ROS-facing interface
	imu_pub = None
	of_pub = None

	def __init__(self, serial_name):

		uav_time_start = None
		uav_time_smpl_count = 0

		rospy.loginfo('Opening serial port.')
		self.serial_name = serial_name
		self.port = serial.Serial(port=self.serial_name, timeout=0)
		self.mav = mavlink.MAVLink(self.port, self.sys_id, self.comp_id)

		rospy.loginfo('Initializing ROS-facing interface...')
		self.imu_pub = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=5)
		self.of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)
		
		return

	# maintains running estimate of conversion from uav_time to rostime
	def uavTimeToRosTime(self, mav_time_ms, msg_receive_rostime):
		if not isinstance(msg_receive_rostime, rospy.Time):
			raise Exception('Invalid argument type received for msg_receive_rostime.')
		
		return msg_receive_rostime
		# TODO: actually do estimate
		
	def mavCheck(self):
		rx_time = rospy.Time.now()

		in_chars = self.port.read(1)
		msgs = self.mav.parse_buffer(in_chars)
		try:
			for m in msgs:
				self.mavHandler(m, rx_time)
		except TypeError:
			pass
		return

	def mavHandler(self, msg, rx_time_ros):
		print msg.get_type()
		if isinstance(msg, mavlink.MAVLink_heartbeat_message):
			pass
		#if isinstance(msg, mavlink.MAVLink_system_status_message):
		#	pass
		if isinstance(msg, mavlink.MAVLink_scaled_imu_message):
			self.mavScaledImu(msg, rx_time_ros)
		elif isinstance(msg, mavlink.MAVLink_optical_flow_message):
			self.mavOpticalFlow(msg, rx_time_ros)
		else:
			rospy.logwarn('Received unrecognized message with id %d.', msg.get_msgId())

		return

	def mavScaledImu(self, msg, rx_time_ros):
		imu = IMUSample()
		imu.gyro_x = msg.xgyro * 0.001			# to [rad/s]
		imu.gyro_y = msg.ygyro * 0.001
		imu.gyro_z = msg.zgyro * 0.001
		imu.acc_x = msg.xacc * 0.001 * 9.80665 	# to [m/s^2]
		imu.acc_y = msg.yacc * 0.001 * 9.80665
		imu.acc_z = msg.zacc * 0.001 * 9.80665
		imu.mag_x = msg.xmag * 0.001			# to [T]
		imu.mag_y = msg.ymag * 0.001
		imu.mag_z = msg.zmag * 0.001

		imu.timestamp = uavTimeToRosTime(msg.time_boot_ms, rx_time_ros)

		self.imu_pub.publish(imu)
		return

	def mavOpticalFlow(self, msg, rx_time_ros):
		of = OptFlowSample()
		of.x_vel = msg.flow_comp_m_x
		of.y_vel = msg.flow_comp_m_y
		of.ground_distance = msg.ground_distance
		of.quality = msg.quality

		of.timestamp = uavTimeToRosTime(msg.time_us, rx_time_ros)

		self.of_pub.publish(of)
		return

	def spin(self):
		while True:
			self.mavCheck()
		return

rospy.init_node('comm_node')

mav_node = MavNode('/dev/ttyACM0')
mav_node.spin()