#px4_comm.py

import rospy
import numpy as np
import struct

from uav_msgs import IMUSample
from uav_msgs import OptFlowSample
from uav_msgs import UavCmd

from mavros import Mavlink # Mavros mavlink message container

import mavlink.mavlink as mv

# Mavlink system id and component id
ODROID_SYS_ID = 5
ODROID_COMP_ID = 1
PX4_SYS_ID = 1
PX4_COMP_ID = 50

seq = 0
# Initialize publishers
mav_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=2)
imu_pub = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=5)
of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)

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

# subscriber handles
def ros_h_UavCmd(uav_cmd=None):
	time_boot_ms = 0
	target_system = PX4_SYS_ID
	target_component = PX4_COMP_ID
	coordinate_frame = mv.MAV_FRAME_LOCAL_NED
	type_mask = 0
	x = 0.0
	y = 0.0
	z = 0.0
	if uav_cmd not None:
		type_mask = 0x7 # meaningful x,y,z
		x = uav_cmd.pos_x
		y = uav_cmd.pos_y
		z = uav_cmd.pos_z

	mav = Mavlink()
	mav.sysid = ODROID_SYS_ID
	mav.compid = ODROID_COMP_ID
	mav.fromlcm = False
	mav.seq = seq
	mav.msgid = 84 # SET_POSITION_TARGET_LOCAL_NED
	mav.len = 53/8 + 1 # 53 bytes / (8 bytes per 64 bits)

	mav.payload = str_to_dw_array( struct.pack('<IBHfffffffffff', time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x, y, z, 0,0,0, 0,0,0, 0,0) )

	# publish message
	mav_pub.publish(mav)
	seq += 1
	return

def mav_h_Mavlink(mav):
	if mav.sysid != ODROID_SYS_ID:
		rospy.logwarn('wrong sysid! %d:%d', mav.sysid, mav.compid) 
	
	# get rostime
	now = rospy.get_rostime()

	if mav.msgid == mv.MAVLINK_MSG_ID_HIGHRES_IMU:
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













rospy.init_node('px4_comm')


