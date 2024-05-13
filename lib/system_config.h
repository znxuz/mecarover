#pragma once
#ifndef SYSTEM_H_
#define SYSTEM_H_

// ----- ESP-IDF specific headers
#if __cplusplus
  // C++ specific headers
  #include <Eigen/Eigen.h>    // compatibility header for eigenlib
  #include "RTOS.h"     // Realtime OS C++ layer for FreeRTOS 
#endif

// floating point type
typedef float real_t; // FPU supports only single precision (float)
//typedef double real_t; 

#endif

