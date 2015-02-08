#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
 
#include <uORB/uORB.h>
#include <uORB/topics/MIT_CURRENT_LOC.h>
 
__EXPORT int px4_simple_app_main(int argc, char *argv[]);
 
int px4_simple_app_main(int argc, char *argv[])
{
	printf("MIT message\n");
 
	/* subscribe to sensor_combined topic */
	int MIT_message = orb_subscribe(ORB_ID(MIT_CURRENT_LOC));
 
	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = MIT_CURRENT_LOC,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
 
	int error_counter = 0;
 
	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);
 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[px4_simple_app] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[px4_simple_app] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct MIT_CURRENT_LOC raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(MIT_CURRENT_LOC), MIT_message, &raw);
				printf("[px4_simple_app] Current_Pos:\t%8.4f\t%8.4f\t%8.4f\n",
					(double)raw.alt,
					(double)raw.lat,
					(double)raw.long);
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}
 
	return 0;
}

