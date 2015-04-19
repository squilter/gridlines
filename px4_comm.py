#!/usr/bin/env python

import rospy
import numpy as np
import struct

from uav_msgs.msg import HighResImu
from uav_msgs.msg import OpticalFlowRad
from uav_msgs.msg import UavCmd

from mavros.msg import Mavlink # generic Mavros mavlink message container
from mavros.srv import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

import mavlink.mavlink as mv

ODROID_SYS_ID = 1
ODROID_COMP_ID = 50

def str_to_dw_array(string):
	l = []	
	# now convert str to list of uint64t
	dw = 0
	i = 0
	for c in payload_str:
		c = np.uint8(c)
		dw += c<<(8*i)
		i += 1
		if i == 8:
			l.append(dw)
			dw = 0
			i = 0
	return l

def unpack_payload(format, payload, l):
	l_dw = len(payload)
	b = bytearray()
	for dw in payload:
		for i in range(8):
			b.append( ( dw>>(8*i) ) & 0xFF)
	# now remove padding and convert to str
	s = str(b[0:l])
	return struct.unpack(format, s)

class OffboardController:

	latest_pose_target_sent = None
	setp_ned = None
	start_time = None
	seq = 0

	in_guided_mode = None

	imu_pub = None
	of_pub = None

	mav_pub = None
	mav_sub = None
	mavros_setp_pub = None


	def __init__(self):

		# Initialize mavros publishers/clients/subscribers
		#self.mav_pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=2)
		#self.mav_sub = rospy.Subscriber('/mavlink/from', Mavlink, self.mav_h_Mavlink)
		self.mavros_setp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

		# Initialize ros publishers
		self.imu_pub = rospy.Publisher('/uav_telemetry/imu', HighResImu, queue_size=5)
		self.of_pub = rospy.Publisher('/uav_telemetry/opt_flow', OpticalFlowRad, queue_size=5)

		self.start_time = rospy.get_rostime()
		self.setp_ned = Point(0,0,0)
		self.latest_pose_target_sent = None

		self.seq = 0
		self.in_guided_mode = False
		self.armed = False

		return

	# subscriber handles
	def ros_h_UavCmd(self, uav_cmd):
		rospy.loginfo('Sending new setpoint at [%f, %f, %f].', uav_cmd.x, uav_cmd.y, uav_cmd.z)
		self.setp_ned.x = uav_cmd.x
		self.setp_ned.y = uav_cmd.y
		self.setp_ned.z = uav_cmd.z

		self.sendLocalPosTargetNed(self.setp_ned)
		return

	def sendLocalPosTargetNed(self, setp_ned):
		self.seq += 1
		self.latest_pose_target_sent = rospy.get_rostime()

		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = self.latest_pose_target_sent
		pose_stamped.header.seq = self.seq
		pose_stamped.header.frame_id = 'local NED'
		pose_stamped.pose.position = setp_ned

		# publish to mavros
		self.mavros_setp_pub.publish(pose_stamped)
		return

	def enableGPSFailCircuitBreaker(self):
		rospy.loginfo('Enabling GPSFAIL circuit breaker.')

		param_pull_client = rospy.ServiceProxy('mavros/param/pull', ParamPull)
		param_get_client = rospy.ServiceProxy('mavros/param/get', ParamGet)
		param_push_client = rospy.ServiceProxy('mavros/param/push', ParamPush)
		param_set_client = rospy.ServiceProxy('mavros/param/set', ParamSet)

		# pull parameters
		r = param_pull_client(True)
		if not r.success:
			rospy.logerr('Failed to pull parameters.')
			return
		
		# get CBRK_GPSFAIL param
		r = param_get_client('CBRK_GPSFAIL')
		if r.success:
			rospy.loginfo('CBRK_GPSFAIL originally set to %l', r.integer)

		r = param_set_client('CBRK_GPSFAIL', 240024, 0.0)
		if not r.success:
			rospy.logerr('Unable to set CBRK_GPSFAIL.')
			return

		r = param_push_client() # push updated parameter list
		if not r.success:
			rospy.logerr('Unable to push new parameter list.')
			return
		
		return

	def mav_h_Mavlink(self, mav):
		if mav.sysid != ODROID_SYS_ID:
			rospy.logwarn('wrong sysid! %d:%d', mav.sysid, mav.compid) 
	
		#rospy.loginfo('Received mavlink message.')
		if mav.msgid == mv.MAVLINK_MSG_ID_HIGHRES_IMU:
			rospy.loginfo('HIGHRES_IMU')
			fields = unpack_payload('<QfffffffffffffH', mav.payload64, mav.len)

			imu = HighResImu()
			imu.time_usec = fields[0]
			imu.xacc = fields[1] 	# [m/s^2]
			imu.yacc = fields[2]
			imu.zacc = fields[3]
			imu.xgyro = fields[4]	# [rad/s]
			imu.ygyro = fields[5]
			imu.zgyro = fields[6]
			imu.xmag = fields[7] 	# [Gauss]
			imu.ymag = fields[8]
			imu.zmag = fields[9]
			imu.abs_pressure = fields[10]	# [mBar]
			imu.diff_pressure = fields[11]
			imu.pressure_alt = fields[12]	# [m]
			imu.temperature = fields[13]	# [Celsius]
			imu.fields_updated = fields[14]	# bitmask for which fields have been updated

			if imu.fields_updated > 0:
				self.imu_pub.publish(imu)


		elif mav.msgid == mv.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
			rospy.loginfo('OPTICAL_FLOW_RAD.')
			fields = unpack_payload('<QBLfffffhBLf', mav.payload64, mav.len)

			of = OpticalFlowRad()
			of.time_usec = fields[0] # convert usec to sec
			of.sensor_id = fields[1]
			of.integration_time_us = fields[2] # [usec] 
			of.integrated_x	= fields[3]			# [flow (some ephereal unit)]
			of.integrated_y = fields[4]
			of.integrated_xgyro = fields[5]		# [rad]
			of.integrated_ygyro = fields[6]
			of.integrated_zgyro = fields[7]
			of.temperature = fields[8]			# [x100 = Celsius]
			of.quality = fields[9]				# [0-255 worst-best]
			of.time_delta_distance_us = fields[10] # [usec]
			if fields[11] > 0:
				of.distance = fields[11] # [m]

			self.of_pub.publish(of)

		else:
			# unrecognized mavlink message received
			pass
		

		return

	def spin(self):
		latest_guided_mode_request = None
		latest_arming_request = None
		mode_client = rospy.ServiceProxy('mavros/cmd/guided_enable', CommandBool)
		arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

		while not rospy.is_shutdown():
			now = rospy.get_rostime()

			# update position setpoint
			if self.latest_pose_target_sent == None or (now - self.latest_pose_target_sent).to_sec() > 0.1:
				self.sendLocalPosTargetNed(self.setp_ned)

			# make sure we're in offboard mode
			if self.in_guided_mode != True and (latest_guided_mode_request == None or (now - latest_guided_mode_request).to_sec() > 0.5):
				rospy.loginfo('Attempting to enable offboard control.')
				latest_guided_mode_request = rospy.get_rostime()
				r = mode_client(True)
				if r.success:
					rospy.loginfo('Successfully entered guided mode.')
					self.in_guided_mode = True
				else:
					rospy.logwarn('Unable to enter guided mode.')

			# now try arming
			if self.armed != True and (latest_arming_request == None or (now - latest_arming_request).to_sec() > 1.0):
				rospy.loginfo('Attempting to arm.')
				latest_arming_request = now
				r = arming_client(True)
				if r.success:
					rospy.loginfo('Successfully ARMED.')
					self.armed = True
		return

rospy.init_node('px4_comm')

controller = OffboardController()
rospy.loginfo('Starting up OffboardController.')

controller.spin()
