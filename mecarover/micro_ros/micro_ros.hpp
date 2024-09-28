#pragma once

#include <mecarover/micro_ros/eth_transport.h>

#define MICRO_ROS_AGENT_IP "192.168.1.228"
#define MICRO_ROS_AGENT_PORT "8888"
#define ROS_DOMAIN_ID 42

extern "C"
{
void micro_ros(void* ct);
};
