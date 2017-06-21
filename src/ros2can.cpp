

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"
#include "data2msg.h"
#include "coms_msgs/CanMessage.h"

int can_id = 0;
uint8_t can_data[8] = {0};


void callback(const coms_msgs::CanMessage &can_frame)
{
    can_id = can_frame.id;
    for(int i=0; i<8; i++)
	can_data[i] = can_frame.data[i];
    data2msg(can_id,can_data);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros2can");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("can_tx",1000,callback);
    
    ros::spin();

    return 0;
}
