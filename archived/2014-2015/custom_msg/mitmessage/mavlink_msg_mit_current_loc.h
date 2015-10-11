// MESSAGE MIT_CURRENT_LOC PACKING

#define MAVLINK_MSG_ID_MIT_CURRENT_LOC 222

typedef struct __mavlink_mit_current_loc_t
{
 uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 int32_t lat; ///< Latitude in 1E7 degrees
 int32_t lon; ///< Longitude in 1E7 degrees
 int32_t alt; ///< Altitude in 1E3 meters (millimeters)
 uint8_t quality; ///< Current position quality / confidence. 0: bad, 255: maximum quality.
} mavlink_mit_current_loc_t;

#define MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN 21
#define MAVLINK_MSG_ID_222_LEN 21

#define MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC 61
#define MAVLINK_MSG_ID_222_CRC 61



#define MAVLINK_MESSAGE_INFO_MIT_CURRENT_LOC { \
	"MIT_CURRENT_LOC", \
	5, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mit_current_loc_t, usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mit_current_loc_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_mit_current_loc_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_mit_current_loc_t, alt) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mit_current_loc_t, quality) }, \
         } \
}


/**
 * @brief Pack a mit_current_loc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param quality Current position quality / confidence. 0: bad, 255: maximum quality.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mit_current_loc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int32_t lat, int32_t lon, int32_t alt, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint8_t(buf, 20, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#else
	mavlink_mit_current_loc_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MIT_CURRENT_LOC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN, MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
}

/**
 * @brief Pack a mit_current_loc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param quality Current position quality / confidence. 0: bad, 255: maximum quality.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mit_current_loc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int32_t lat,int32_t lon,int32_t alt,uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint8_t(buf, 20, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#else
	mavlink_mit_current_loc_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MIT_CURRENT_LOC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN, MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
}

/**
 * @brief Encode a mit_current_loc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mit_current_loc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mit_current_loc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mit_current_loc_t* mit_current_loc)
{
	return mavlink_msg_mit_current_loc_pack(system_id, component_id, msg, mit_current_loc->usec, mit_current_loc->lat, mit_current_loc->lon, mit_current_loc->alt, mit_current_loc->quality);
}

/**
 * @brief Encode a mit_current_loc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mit_current_loc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mit_current_loc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mit_current_loc_t* mit_current_loc)
{
	return mavlink_msg_mit_current_loc_pack_chan(system_id, component_id, chan, msg, mit_current_loc->usec, mit_current_loc->lat, mit_current_loc->lon, mit_current_loc->alt, mit_current_loc->quality);
}

/**
 * @brief Send a mit_current_loc message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param quality Current position quality / confidence. 0: bad, 255: maximum quality.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mit_current_loc_send(mavlink_channel_t chan, uint64_t usec, int32_t lat, int32_t lon, int32_t alt, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint8_t(buf, 20, quality);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, buf, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN, MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, buf, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
#else
	mavlink_mit_current_loc_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.quality = quality;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, (const char *)&packet, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN, MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, (const char *)&packet, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mit_current_loc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, int32_t lat, int32_t lon, int32_t alt, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint8_t(buf, 20, quality);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, buf, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN, MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, buf, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
#else
	mavlink_mit_current_loc_t *packet = (mavlink_mit_current_loc_t *)msgbuf;
	packet->usec = usec;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->quality = quality;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, (const char *)packet, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN, MAVLINK_MSG_ID_MIT_CURRENT_LOC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MIT_CURRENT_LOC, (const char *)packet, MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MIT_CURRENT_LOC UNPACKING


/**
 * @brief Get field usec from mit_current_loc message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_mit_current_loc_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from mit_current_loc message
 *
 * @return Latitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_mit_current_loc_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from mit_current_loc message
 *
 * @return Longitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_mit_current_loc_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from mit_current_loc message
 *
 * @return Altitude in 1E3 meters (millimeters)
 */
static inline int32_t mavlink_msg_mit_current_loc_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field quality from mit_current_loc message
 *
 * @return Current position quality / confidence. 0: bad, 255: maximum quality.
 */
static inline uint8_t mavlink_msg_mit_current_loc_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a mit_current_loc message into a struct
 *
 * @param msg The message to decode
 * @param mit_current_loc C-struct to decode the message contents into
 */
static inline void mavlink_msg_mit_current_loc_decode(const mavlink_message_t* msg, mavlink_mit_current_loc_t* mit_current_loc)
{
#if MAVLINK_NEED_BYTE_SWAP
	mit_current_loc->usec = mavlink_msg_mit_current_loc_get_usec(msg);
	mit_current_loc->lat = mavlink_msg_mit_current_loc_get_lat(msg);
	mit_current_loc->lon = mavlink_msg_mit_current_loc_get_lon(msg);
	mit_current_loc->alt = mavlink_msg_mit_current_loc_get_alt(msg);
	mit_current_loc->quality = mavlink_msg_mit_current_loc_get_quality(msg);
#else
	memcpy(mit_current_loc, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MIT_CURRENT_LOC_LEN);
#endif
}
