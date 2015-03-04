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

	# stream rate counters
	imu_stream_t = None
	imu_stream_dt = 0
	of_stream_t = None
	of_stream_dt = 0

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

	def __init__(self, serial_name, baud_rate):

		self.uav_time_start = None
		self.uav_latency = None

		rospy.loginfo('Opening serial port.')
		self.serial_name = serial_name
		self.port = serial.Serial(port=self.serial_name, timeout=0)
		self.port.baudrate = baud_rate

		self.port.flushInput()
		self.port.flushOutput()

		self.mav = mavlink.MAVLink(self.port, self.sys_id, self.comp_id)

		rospy.loginfo('Initializing ROS-facing interface...')
		# NOTE: Here is where you initialize any additional ROS publishers
		self.imu_pub = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=5)
		self.of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)
		# NOTE: And here you initialize ROS subscribers and link them to their callback functions
		#self.cmd_sub = rospy.Subscriber('uav_cmds/cmd', UavCmd, self.rosUavCmd)
		
		return

	def sendPx4PortOpenCmd(self):
		self.port.flushOutput()
		self.port.flushInput()

		for i in range(3):
			self.port.write(chr(0x0d))
		
		self.port.write('mavlink stop-all\n')
		for i in range(3):
			self.port.write(chr(0x0d))
		self.port.write('mavlink start -d /dev/ttyACM0 -b 921600 -x\n')
		self.port.flush()
		# wait a second
		rospy.sleep(1.0)
		self.port.flushInput()
		self.port.baudrate = 921600
		
		for i in range(3):
			self.port.write(chr(0x0d))
		self.port.write('mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 100\n')
		for i in range(3):
			self.port.write(chr(0x0d))
		self.port.write('mavlink stream -d /dev/ttyACM0 -s OPTICAL_FLOW_RAD -r 10\n')
		for i in range(3):
			self.port.write(chr(0x0d))
		# self.port.write('exit\n')
		for i in range(3):
			self.port.write(chr(0x0d))

		for i in range(3):
			self.port.write(chr(0x0d))
		self.port.write(chr(0))
		self.port.flush()
		self.port.flushInput()

		rospy.sleep(1.0)
		self.port.flushInput()

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
		try:
			msgs = self.mav.parse_buffer(in_chars)
		except mavlink.MAVError as e:
			print "Mavlink error"
			print e.message
			return
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
		elif mid == mavlink.MAVLINK_MSG_ID_HIGHRES_IMU:    # HIGHRES_IMU #105
			self.mavHighResImu(msg, rx_time_ros)
		elif mid == mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: #106
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
		now = rospy.get_rostime()

		if self.imu_stream_t == None:
			self.imu_stream_t = now
		else:
			dt = (now - self.imu_stream_t).to_sec()
			self.imu_stream_t = now
			self.imu_stream_dt += 0.01*(dt - self.imu_stream_dt)
			rate = 1.0/self.imu_stream_dt
			rospy.loginfo('Recieved imu msg. Rate = %f', rate)

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
		return

	def mavOpticalFlow(self, msg, rx_time_ros):
		now = rospy.get_rostime()

		if self.of_stream_t == None:
			self.of_stream_t = now
		else:
			dt = (now - self.of_stream_t).to_sec()
			self.of_stream_t = now
			self.of_stream_dt += 0.1*(dt - self.of_stream_dt)
			rate = 1.0/self.of_stream_dt
			rospy.loginfo('Recieved of msg. Rate = %f', rate)

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
		return

rospy.init_node('comm_node')

mav_node = MavNode(serial_name='/dev/ttyACM0', baud_rate=57600)
mav_node.sendPx4PortOpenCmd()

rospy.loginfo('Initialized Mavlink stream over USB.')

r = rospy.Rate(10)

while not rospy.is_shutdown():
	mav_node.mavCheck()
	r.sleep()
