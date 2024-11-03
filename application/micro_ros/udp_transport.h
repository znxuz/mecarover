// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  struct pollfd poll_fd;
  char ip[16];
  char port[5];
} udp_transport_params_t;

#define MICRO_ROS_FRAMING_REQUIRED false

bool udp_transport_open(struct uxrCustomTransport* transport);
bool udp_transport_close(struct uxrCustomTransport* transport);
size_t udp_transport_write(struct uxrCustomTransport* transport,
                           const uint8_t* buf, size_t len, uint8_t* err);
size_t udp_transport_read(struct uxrCustomTransport* transport, uint8_t* buf,
                          size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif
