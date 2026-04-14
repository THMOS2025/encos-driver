/* socket_can_driver.c
 * SocketCAN implementation providing the PCAN-like interfaces.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

// For SocketCAN
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// For netlink
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/can/netlink.h>

#include "log.h"
#include "socket_can_driver.h" 
#include "constant.h"


const char IP_LINK_UP_COMMAND[] = "sudo ip link set %s up type can "
                                  "bitrate 1000000 loopback off 2>/dev/null";


static int can_socket_fd[CHANNEL_COUNT] = { -1, -1, -1, -1 };


int
initialize_can(const uint8_t channel)
{
    int ret = 0;

    /* Check for invalid channel index */
    if (channel >= CHANNEL_COUNT) {
        ret = -1;
        log_error("%hu is greater than CHANNEL_COUNT at compile time.", channel);
        goto FAILED_NONE;
    }

    /* Check if already initialized */
    if (can_socket_fd[channel] != -1) {
        log_warn("Channel %hu is already initialized", channel);
        return 1;
    }

    /* Configure netlink intnerface */
    char buf[80];
    sprintf(buf, IP_LINK_UP_COMMAND, CHANNEL_NAME[channel]);
    if((ret = system(buf)) < 0) {
        log_warn("Can not link up channel %hu\n command: %s", channel, buf);
        return 1;
    }

    /* Create the SocketCAN socket */
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(s < 0) {
        log_warn("Can not create socket on channel %hu, reason: %s", \
                channel, strerror(s));
        goto FAILED_NONE;
    }

    /* Set the socket to NON-BLOCKING mode */
    if((ret = fcntl(s, F_SETFL, O_NONBLOCK)) < 0) {
        log_warn("Can not set channel %hu to non-blocking mode, reason: %s", \
                channel, strerror(errno));
        goto FAILED_OPENED;
    }

    /* Resolve the interface name to an index */
    struct ifreq ifr;
    strcpy(ifr.ifr_name, CHANNEL_NAME[channel]);
    if((ret = ioctl(s, SIOCGIFINDEX, &ifr)) < 0) {
        log_warn("Can not open channel %hu, reason: %s", channel, strerror(errno));
        goto FAILED_OPENED;
    }

    /* Bind the socket to the interface */
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if((ret = bind(s, (struct sockaddr *)&addr, sizeof(addr))) < 0) {
        log_warn("Can not bind to channel %hu, reason: %s", channel, strerror(errno));
        goto FAILED_OPENED;
    }

    /* Success: Store the file descriptor */
    can_socket_fd[channel] = s;
    return ret;

FAILED_OPENED:
    close(s);
FAILED_NONE:
    return ret;
}


int
uninitialize_can(const uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return -1;

    int fd = can_socket_fd[channel];
    if (fd == -1) return 0;

    if (close(fd) < 0) return -1;

    can_socket_fd[channel] = -1;
    return 0;
}


int
write_can_message(const uint8_t channel,
                  const uint16_t id,
                  const size_t len,
                  const uint64_t data)
{
    if (channel >= CHANNEL_COUNT || can_socket_fd[channel] == -1) {
        log_warn("Channel %hu is inavailable", channel);
        return -1;
    }
        
    log_trace("Channel %hu: host -> motor 0x%04x: len=%llu 0x%016llx", \
            channel, id, len, data);

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    if (id > 0x7FF) {
        frame.can_id = id | CAN_EFF_FLAG; 
    }
    else {
        frame.can_id = id & 0x7FF;
    }
    frame.can_dlc = (uint8_t)len;

    for(int i = 0; i < len && i < 8; ++i)
        frame.data[i] = (uint8_t)((data >> (56 - 8 * i)) & 0xff);

    if (write(can_socket_fd[channel], &frame, sizeof(struct can_frame))\
            != sizeof(struct can_frame))
        return -1;

    return 0;
}


int
read_can_message(const uint8_t channel,
                 uint16_t *id,
                 size_t *len,
                 uint64_t *data)
{
    if (channel >= CHANNEL_COUNT || can_socket_fd[channel] == -1) {
        log_warn("Channel %hu is inavailable", channel);
        return -1;
    }

    struct can_frame frame;

    if(read(can_socket_fd[channel], &frame, sizeof(struct can_frame)) \
            != sizeof(struct can_frame)) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            log_trace("Channel %hu is empty, all message handled", channel);
            return 1; // Receive queue is empty.
        }
        else {
            log_warn("Channel %hu read failed: %s", \
                    channel, strerror(errno));
            return -1;
        }
    }

    *id = frame.can_id;
    *len = frame.can_dlc;
    *data = 0;
    for (int i = 0; i < frame.can_dlc; i++)
        *data |= ((uint64_t)frame.data[i] << (56ull - 8ull * i));
    log_trace("Channel %hu: motor 0x%04x -> host: len=%llu 0x%016llx", \
            channel, *id, *len, *data);
    return 0;
}

#undef CAN_MAX_CHANNEL
#undef NLMSG_TAIL
