/*
 * retarget.h
 *
 *  Created on: Apr 12, 2022
 *      Author: robot
 */

// Allen Dank an Carmine Noviello für diesen Code
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/include/retarget/retarget.h
// Quelle: https://shawnhymel.com/1873/how-to-use-printf-on-stm32/


#ifndef RETARGET_H_
#define RETARGET_H_

//#include "stm32f7xx_hal_tim.h" //Für Kontrollausgaben bsp. __HAL_TIM_SET_Compare -> Pwm testen


#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f7xx.h>
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);


int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#ifdef __cplusplus
}
#endif


#endif /* RETARGET_H_ */
