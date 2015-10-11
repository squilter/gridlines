#include <nuttx/config.h>
#include <uORB/uORB.h>
#include <uORB/topics/estimator_status.h>
#include <mavlink/mavlink_log.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
//#include <string> not sure

static int mavlink_fd;
mavlink_fd = get_mavlink_fd(); //open(MAVLINK_LOG_DEVICE, 0);

static int topic_handle; 
int init()
{
	/* subscribe to the topic */
	topic_handle = orb_subscribe(ORB_ID(estimator_status));
}

void check_topic()
{
	bool updated;
	struct estimator_status_report esr; // estimator_status_data
 
	/* check to see whether the topic has updated since the last time we read it */
	orb_check(topic_handle, &updated);
 
	if (updated) {
		/* make a local copy of the updated data structure */
		printf("Topic has been updated since last time we read it.\n")
		orb_copy(ORB_ID(estimator_status), topic_handle, &esr);
//		std::string hostname;	not sure
		mavlink_log_info(mavlink_fd, "ESR("+esr.n_states+"): "+esr.timestamp+", "+esr.states[0]+", "+esr.states[1]+", "+esr.states[2]+", "+esr.states[3]);
		printf("Logged ESR info.\n")
	}
}

__EXPORT int px4_state_estimate_main(int argc, char *argv[]);
 
int px4_state_estimate_main(int argc, char *argv[])
{
	printf("Hello mit uav team!\n");
	
	init();
	while (true) {
		check_topic();
	}
	return 0;
}



