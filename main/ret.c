/*
 * ret.c
 *
 *  Created on: Apr 19, 2022
 *      Author: robot
 */

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "usart.h"
#include "retarget.h"
#include <stm32f7xx.h>
#include <stdbool.h>


#if !defined(OS_USE_SEMIHOSTING)

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

extern char _estack;  // see ld file
extern char _Min_Stack_Size;  // see ld file
extern int __io_getchar(void) __attribute__((weak));


char *__env[1] = { 0 };
char **environ = __env;


#define UART_SIZE 2048

static uint8_t buffer[UART_SIZE];
static uint8_t data;
static size_t head = 0, tail = 0;

UART_HandleTypeDef *gHuart;


//int _open(char *path, int flags, ...)
//{
//	HAL_UART_Receive_IT(gHuart, &data, 1);
//	return true;
//}
//
//int _close(int fd)
//{
//	 HAL_UART_Abort_IT(gHuart);
//	 return true;
//}

int _write(int fd, char *ptr, int len)
{

	  HAL_StatusTypeDef hstatus;

	  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
	    hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);

	    if (hstatus == HAL_OK)
	      return len;
	    else
	      return EIO;
	  }
	  errno = EBADF;
	  return -1;
//	HAL_StatusTypeDef r;
//	if(gHuart->gState == HAL_UART_STATE_READY){
//		r = HAL_UART_Transmit_IT(gHuart, (uint8_t *) ptr, len);
////		while(r == HAL_OK && gHuart->gState != HAL_UART_STATE_READY){
////			osDelay(1);
////		}
//		return (r == HAL_OK) ? len : 0;
//	}else{
//		return 0;
//	}
}

//int _read(int file, char *ptr, int len)
//{
//	size_t wrote = 0;
//	while((head != tail) && (wrote < len)){
//		ptr[wrote] = buffer[head];
//		head = (head + 1) % UART_SIZE;
//		wrote++;
//	}
//
//	return wrote;
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//    HAL_StatusTypeDef ret;
//ret = HAL_UART_GetState(huart);
//
//
//}
//
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
//int error;
//error = HAL_UART_GetError(huart);
//
//HAL_StatusTypeDef ret;
//ret = HAL_UART_GetState(huart);
//
//}


void RetargetInit(UART_HandleTypeDef *huart) {
  gHuart = huart;

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}

int _fstat(int fd, struct stat *st)
{
	  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
	    st->st_mode = S_IFCHR;
	    return 0;
	  }

	  errno = EBADF;
	  return 0;
}

int _isatty(int fd)
{
	  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
	    return 1;

	  errno = EBADF;
	  return 0;}

int _lseek(int fd, int ptr, int dir)
{
	  (void) fd;
	  (void) ptr;
	  (void) dir;

	  errno = EBADF;
	  return -1;
}


int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}

caddr_t _sbrk(int incr) {
    extern char __heap_start__ asm("end");  // Defined by the linker.
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL) heap_end = &__heap_start__;

    prev_heap_end = heap_end;

    if (heap_end + incr > &_estack - _Min_Stack_Size) {
            __asm("BKPT #0\n");
        errno = ENOMEM;
        return (caddr_t)-1;

    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;

}

PUTCHAR_PROTOTYPE{
HAL_UART_Transmit_IT(gHuart, (uint8_t *) &ch, 1);
return ch;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)

