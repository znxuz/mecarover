#include "mrlogger.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <mecarover/mrtypes.h>
#include <mecarover/rtos_config.h>

#define MSGSIZE 150 // maximum size of a message
#define MAXMSG 10 // max number of messages

static bool use_colors = true;
static SemaphoreHandle_t msgmutex = NULL;
static QueueHandle_t msg_queue = NULL;

typedef struct {
	mr_logprio_t prio;
	char msg[MSGSIZE];
} mr_logger_msg;

static void logger_server_task(void* args)
{
	static mr_logger_msg msg;

	log_message(log_info, "Starting Logger");
	while (true) {
		BaseType_t ret = xQueueReceive(msg_queue, &msg, portMAX_DELAY);

		if (ret != pdTRUE) {
			perror("Can not receive new log message\n");
			continue;
		}
		if (use_colors)
			printf("\033[%dm", msg.prio);
		puts(msg.msg);
		if (use_colors)
			printf("\033[0m");

		fflush(stdout);
	}
}

void logger_use_colors(bool colors) { use_colors = colors; }

void log_message(mr_logprio_t prio, const char* format, ...)
{
	mr_logger_msg msg;

	static size_t counter = 0;
	char numbering[20];
	snprintf(numbering, sizeof(numbering), "%d. ", counter++);
	strcpy(msg.msg, numbering);

	msg.prio = prio;
	va_list arglist;
	va_start(arglist, format);
	int ret = vsnprintf(msg.msg + strlen(numbering), sizeof(msg.msg), format,
						arglist);
	va_end(arglist);

	if (ret <= 0) {
		perror("Error logging the message\n");
		return;
	}

	msg.msg[sizeof(msg.msg) - 1] = '\0';

	if (xSemaphoreTake(msgmutex, 1) == pdTRUE && msg_queue) {
		xQueueSend(msg_queue, &msg, 0);
		xSemaphoreGive(msgmutex);
	}
}

void logger_init()
{
	msgmutex = xSemaphoreCreateMutex();
	if (!msgmutex) {
		perror("Error: mutex can not be craeted\n");
		return;
	}

	msg_queue = xQueueCreate(MAXMSG, MSGSIZE);
	if (!msg_queue) {
		perror("Error: queue can not be craeted\n");
		return;
	}

	xTaskCreate(logger_server_task, "logger server", MAIN_TASK_STACK_SIZE, NULL,
				LOGGER_TASK_PRIORITY, NULL);
}
