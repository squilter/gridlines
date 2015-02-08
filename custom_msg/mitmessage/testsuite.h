/** @file
 *	@brief MAVLink comm protocol testsuite generated from mitmessage.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef MITMESSAGE_TESTSUITE_H
#define MITMESSAGE_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_mitmessage(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_mitmessage(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_mit_current_loc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mit_current_loc_t packet_in = {
		93372036854775807ULL,963497880,963498088,963498296,65
    };
	mavlink_mit_current_loc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.quality = packet_in.quality;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mit_current_loc_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mit_current_loc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mit_current_loc_pack(system_id, component_id, &msg , packet1.usec , packet1.lat , packet1.lon , packet1.alt , packet1.quality );
	mavlink_msg_mit_current_loc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mit_current_loc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.lat , packet1.lon , packet1.alt , packet1.quality );
	mavlink_msg_mit_current_loc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mit_current_loc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mit_current_loc_send(MAVLINK_COMM_1 , packet1.usec , packet1.lat , packet1.lon , packet1.alt , packet1.quality );
	mavlink_msg_mit_current_loc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mitmessage(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_mit_current_loc(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MITMESSAGE_TESTSUITE_H
