#include <stdio.h>
#include <stdint.h>
#include "canusbpub.h"
#include "data2msg.h"
#include "ros/ros.h"

int data2msg(int can_id, uint8_t data[])
{
    char s[50];
    char datashow[50];
    int n = 0;
    sprintf(s, "%08X#", can_id);
    sprintf(datashow, "can-id: %X  |  data: ", can_id);

    for(size_t i = 0; i != 8; ++i)
    {
	sprintf(s, "%s%02X", s,data[i]);
        sprintf(datashow, "%s%02X ",datashow,data[i]);
    }
    n = canusbpub(s);
    if (n != 1)
	ROS_INFO("%s",datashow);
    else
        ;
}
