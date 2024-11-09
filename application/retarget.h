#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f7xx.h>
#include <sys/stat.h>

void retarget_init(UART_HandleTypeDef *huart);

int _isatty(int fd);
int _write(int fd, char *ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char *ptr, int len);
int _fstat(int fd, struct stat *st);

#ifdef __cplusplus
}
#endif
