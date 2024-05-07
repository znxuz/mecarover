/* 
 * this module provides a C interface to the C++ Arduino network classes WiFi and ETH
 */

#pragma once
#ifndef INCLUDED_IMSL_NETWORK_H
#define INCLUDED_IMSL_NETWORK_H

#include <stdbool.h> // for bool compatibility to C++
//namespace imsl {

#if __cplusplus
extern "C" {
#endif

void ethInit();
void wifiClientInit();
//void wifiAPInit(); // TODO
bool networkIsConnected();

#if __cplusplus
}
#endif

//}

#endif // INCLUDED__H
