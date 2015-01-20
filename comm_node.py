#!/usr/bin/env python

import rospy
import numpy as np

from uav_msgs.msg import IMUSample
from uav_msgs.msg import OptFlowSample

# use MavRos bridge
# must sudo apt-get install ros-indigo-mavros
from mavros.msg import Mavlink

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


class MavCommNode:

	# UAV start time (in Ros time) -- used for synchronization
	uav_time_start = None
	uav_time_smpl_count = 0

	# Mavlink-facing interface
	mav_pub = None
	mav_sub = None

	# Ground-facing interface
	imu_pub = None
	of_pub = None

	def __init__(self):

		rospy.loginfo('Initializing GC-facing interface...')
		self.imu_pub = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=5)
		self.of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)

		rospy.loginfo('Initializing MavRos-facing interface...')
		self.mav_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=5)
		self.mav_sub = rospy.Subscriber('mavlink/from', Mavlink, self.mavCallback)

		return

	# maintains running estimate of conversion from uav_time to rostime
	def uavTimeToRosTime(self, mav_time_ms, msg_receive_rostime):
		if not isinstance(msg_receive_rostime, rospy.Time):
			raise Exception('Invalid argument type received for msg_receive_rostime.')
		
		return msg_receive_rostime
		# TODO: actually do estimate
		

	def mavCallback(self, msg):
		# funnel message to specific handler based on msgid
		if msg.msgid == MAV_ID_SCALED_IMU:
			self.mavScaledImu(msg)
		elif msg.msgid == MAV_ID_OPTICAL_FLOW:
			self.mavOpticalFlow(msg)
		else:
			rospy.logwarn('Unrecognized Mavlink message received with id %d.', msg.msgid)

		return

	def mavScaledImu(self, msg):
		data = msg.payload64
		rx_time_ros = msg.header.stamp

		# extract from uint64 data buffer
		time_boot_ms	= np.uint32(data[0]&0xFFFFFFFF)
		xacc		= np.int16((data[0]>>32)&0xFFFF) # milli g
		yacc		= np.int16((data[0]>>48)&0xFFFF)
		zacc		= np.int16((data[1]>>00)&0xFFFF)
		xgyro		= np.int16((data[1]>>16)&0xFFFF) # milli rad/s
		ygyro		= np.int16((data[1]>>32)&0xFFFF)
		zgyro		= np.int16((data[1]>>48)&0xFFFF)
		xmag		= np.int16((data[2]>>00)&0xFFFF) # milli T
		ymag		= np.int16((data[2]>>16)&0xFFFF)
		zmag 		= np.int16((data[2]>>32)&0xFFFF)

		imu = IMUSample()
		imu.gyro_x = xgyro * 0.001			# to [rad/s]
		imu.gyro_y = ygyro * 0.001
		imu.gyro_z = zgyro * 0.001
		imu.acc_x = xacc * 0.001 * 9.80665 	# to [m/s^2]
		imu.acc_y = yacc * 0.001 * 9.80665
		imu.acc_z = zacc * 0.001 * 9.80665
		imu.mag_x = xmag * 0.001			# to [T]
		imu.mag_y = ymag * 0.001
		imu.mag_z = zmag * 0.001

		imu.timestamp = uavTimeToRosTime(time_boot_ms, rx_time_ros)

rospy.init_node('comm_node')
node = MavCommNode()

rospy.spin()