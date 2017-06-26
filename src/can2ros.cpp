/*
 * candump.c
 *
 * Copyright (c) 2002-2009 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "terminal.h"
#include "lib.h"
#include "ros/ros.h"
#include "ros_lawicel_canusb/CanMessageStamped.h"

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define BOLD    ATTBOLD
#define RED     ATTBOLD FGRED
#define GREEN   ATTBOLD FGGREEN
#define YELLOW  ATTBOLD FGYELLOW
#define BLUE    ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN    ATTBOLD FGCYAN

const char col_off [] = ATTRESET;

static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 
const int canfd_on = 1;


extern int optind, opterr, optopt;

static volatile int running = 1;



void sigterm(int signo)
{
	running = 0;
}

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}

int main(int argc, char **argv)
{

        ros::init(argc, argv, "can2ros");
	ros::NodeHandle n;

	ros::Publisher canpub = n.advertise<ros_lawicel_canusb::CanMessageStamped>("can_rx",2);
	ros_lawicel_canusb::CanMessageStamped can_frame;
	
	fd_set rdfs;
	int s[MAXSOCK];
	unsigned char down_causes_exit = 1;
	unsigned char view = 0;
	int opt, ret;
	int currmax;
	char *ptr;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct canfd_frame frame;
	int nbytes, i, maxdlen;
	struct ifreq ifr;
	struct timeval tv, last_tv;
	struct timeval timeout, timeout_config = { 0, 0 };

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	last_tv.tv_sec  = 0;
	last_tv.tv_usec = 0;

	while ((opt = getopt(argc, argv, "t:ciaSs:b:B:u:lDdxLn:r:heT:?")) != -1);
	currmax = argc - optind; /* find real number of CAN devices */

	if (currmax > MAXSOCK) 
	{
		fprintf(stderr, "More than %d CAN devices given on commandline!\n", MAXSOCK);
		return 1;
	}

	if (optind == argc) 
	{
		currmax = 1;	
	}

	for (i=0; i < currmax; i++) 
	{
	    ptr = argv[optind+i];
	    s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	    if (s[i] < 0) 
	    {
		perror("socket");
		return 1;
	    }
	    nbytes = strlen(ptr); /* no ',' found => no filter definitions */

	    if (nbytes > max_devname_len)
		max_devname_len = nbytes; /* for nice printing */

	    addr.can_family = AF_CAN;

	    memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	    strncpy(ifr.ifr_name, ptr, nbytes);

	    if (strcmp(ANYDEV, ifr.ifr_name)) 
	    {
		if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) 
		{
		    perror("SIOCGIFINDEX");
		    exit(1);
		}
		addr.can_ifindex = ifr.ifr_ifindex;
	    } 
	    else
		addr.can_ifindex = 0; /* any can interface */


	    /* try to switch the socket into CAN FD mode */
	    setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));


	    if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) 
	    {
		perror("bind");
		return 1;
	    }
	}

	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	int count = 0;
	while (running) 
	{   
////////////////////////////////////


////////////////////////////////////
	    FD_ZERO(&rdfs);
	    for (i=0; i<currmax; i++)
		FD_SET(s[i], &rdfs);

	    if ((ret = select(s[currmax-1]+1, &rdfs, NULL, NULL, NULL)) <= 0) 
	    {
		//perror("select");
		running = 0;
		continue;
	    }

	    for (i=0; i<currmax; i++) 
	    {  
		/* check all CAN RAW sockets */

		if (FD_ISSET(s[i], &rdfs)) 
		{
		    int idx;
		    /* these settings may be modified by recvmsg() */
		    iov.iov_len = sizeof(frame);
	    	    msg.msg_namelen = sizeof(addr);
		    msg.msg_controllen = sizeof(ctrlmsg);  
		    msg.msg_flags = 0;

		    nbytes = recvmsg(s[i], &msg, 0);
		    idx = idx2dindex(addr.can_ifindex, s[i]);

		    if (nbytes < 0) 
		    {
			if ((errno == ENETDOWN) && !down_causes_exit) 
			{
			    fprintf(stderr, "%s: interface down\n", devname[idx]);
			    continue;
		    	}
		    	perror("read");
		    	return 1;
		    }

		    if ((size_t)nbytes == CAN_MTU)
		    	maxdlen = CAN_MAX_DLEN;
		    else if ((size_t)nbytes == CANFD_MTU)
		     	maxdlen = CANFD_MAX_DLEN;
		    else 
			{
		    	    fprintf(stderr, "read: incomplete CAN frame\n");
		    	    return 1;
			}
		    
		    for (cmsg = CMSG_FIRSTHDR(&msg); cmsg && (cmsg->cmsg_level == SOL_SOCKET);cmsg = CMSG_NXTHDR(&msg,cmsg)) 
		    {
		    	if (cmsg->cmsg_type == SO_TIMESTAMP)
			    tv = *(struct timeval *)CMSG_DATA(cmsg);
		    	else if (cmsg->cmsg_type == SO_RXQ_OVFL)
			    dropcnt[i] = *(__u32 *)CMSG_DATA(cmsg);
		    }

		    /* check for (unlikely) dropped frames on this specific socket */
		    if (dropcnt[i] != last_dropcnt[i]) 
			{
		     	    __u32 frames = dropcnt[i] - last_dropcnt[i];
		    	    last_dropcnt[i] = dropcnt[i];
			}

		    /* once we detected a EFF frame indent SFF frames accordingly */
//		    if (frame.can_id & CAN_EFF_FLAG)
//		        view |= CANLIB_VIEW_INDENT_SFF;

//		    fprint_long_canframe(stdout, &frame, NULL, view, maxdlen);
//		    printf("\n");


		    can_frame.msg.id = (uint32_t)(frame.can_id & 0x1FFFFFFF);
		    can_frame.msg.dlc = (uint8_t)frame.len;
		    //We have a bug here and left it as an Easter-egg. Good Luck!
		    can_frame.msg.extended = (bool)(frame.can_id>>31);
		    for(size_t i = 0; i != 8; ++i)
    		    {
		    	can_frame.msg.data[i] = frame.data[i];
    		    }
		    can_frame.header.seq = count;
    		    can_frame.header.stamp = ros::Time::now();
		    can_frame.header.frame_id = "can2ros";
		    canpub.publish(can_frame);
				
	    	}

	    	out_fflush:
	    	fflush(stdout);
	    }
    	}

    	for (i=0; i<currmax; i++)
	    close(s[i]);
        return 0;
}
