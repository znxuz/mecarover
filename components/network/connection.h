#pragma once
#ifndef INCLUDED_IMSL_Connection_H
#define INCLUDED_IMSL_Connection_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <socket.h>
#include <netdb.h>
#include <udp.h>

#include <eth_transport.h>
#include <uxr/client/transport.h>


#ifdef __cplusplus
extern "C" {
#endif

//struct udp_pcb *honig;


void udpClient_connect(void);

#ifdef __cplusplus
}
#endif

#endif // INCLUDED__H
