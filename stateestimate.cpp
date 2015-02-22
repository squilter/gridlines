#include <uORB/uORB.h>
#include <uORB/topics/estimator_status.h>
#include <mavlink/mavlink_log.h>

static int mavlink_fd;
mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

static int topic_handle; 
int init()
{
	/* subscribe to the topic */
	topic_handle = orb_subscribe(ORB_ID(estimator_status));
}

void check_topic()
{
	bool updated;
	struct estimator_status_data ed;
 
	/* check to see whether the topic has updated since the last time we read it */
	orb_check(topic_handle, &updated);
 
	if (updated) {
		/* make a local copy of the updated data structure */
		orb_copy(ORB_ID(estimator_status), topic_handle, &ed);
		mavlink_log_info(mavlink_fd, "ESD: "+ed.x+", "+ed.y+", "+ed.z+", "+ed.thrust);
	}
}




