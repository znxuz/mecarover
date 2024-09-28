#pragma once

#include <mecarover/micro_ros/eth_transport.hpp>

extern "C"
{

static inline constexpr unsigned int ROS_DOMAIN_ID = 42;
static eth_transport_params_t TRANSPORT_PARAMS
	= {{0, 0, 0}, {"192.168.1.228"}, {"8888"}};

void micro_ros(void* ct);
};
