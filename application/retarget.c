#include "retarget.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f7xx.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <time.h>

#if !defined(OS_USE_SEMIHOSTING)
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

/* Variables */
// #undef errno
extern int errno;
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

register char *stack_ptr asm("sp");

char *__env[1] = {0};
char **environ = __env;

extern char _estack;          // see ld file
extern char _Min_Stack_Size;  // see ld file

volatile int txComplete = 1;
volatile uint8_t txBuffer[200];

UART_HandleTypeDef *gHuart;

void retarget_init(UART_HandleTypeDef *huart) {
  gHuart = huart;

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  // setvbuf(stdout, NULL, _IONBF, 0);
}

int _getpid(void) { return 1; }

int _kill(int pid, int sig) {
  errno = EINVAL;
  return -1;
}

void _exit(int status) {
  _kill(status, -1);
  while (1) {
  } /* Make sure we hang here */
}

int _read(int file, char *ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (file == STDIN_FILENO) {
    hstatus = HAL_UART_Receive(gHuart, (uint8_t *)ptr, 1, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return 1;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _write(int file, char *ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (file == STDOUT_FILENO || file == STDERR_FILENO) {
    hstatus = HAL_UART_Transmit(gHuart, (uint8_t *)ptr, len, HAL_MAX_DELAY);

    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) return 0;

  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat *st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}

int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) return 1;

  errno = EBADF;
  return 0;
}

int _lseek(int fd, int ptr, int dir) {
  (void)fd;
  (void)ptr;
  (void)dir;

  errno = EBADF;
  return -1;
}

int _open(char *path, int flags, ...) {
  /* Pretend like we always fail */
  return -1;
}

int _wait(int *status) {
  errno = ECHILD;
  return -1;
}

int _unlink(char *name) {
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf) { return -1; }

int _stat(char *file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(char *old, char *new) {
  errno = EMLINK;
  return -1;
}

int _fork(void) {
  errno = EAGAIN;
  return -1;
}

int _execve(char *name, char **argv, char **env) {
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

#endif  // #if !defined(OS_USE_SEMIHOSTING)
