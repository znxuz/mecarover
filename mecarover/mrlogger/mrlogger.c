#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include <stdarg.h> //  va_list, va_start, va_end
#include <stdio.h> // for vsnprintf()
#include <string.h> // strlen

#include <cmsis_os2.h>
#include <mecarover/mrtypes.h>
#include <mecarover/rtos_config.h>

#define MSGSIZE 100 // maximum size of a message
#define MAXMSG 10 // max number of messages

static mr_logprio_t log_level = log_debug;
static mr_logprio_t screen_level = log_debug;
static bool use_colors = true; // false;

static void logger_server_task(void* args); // server thread function
static TaskHandle_t logger_server = NULL; // thread for writng messages to terminal and syslogd
static SemaphoreHandle_t msgmutex = NULL;
static QueueHandle_t msgque = NULL;

static size_t counter = 0;

typedef struct {
	mr_logprio_t prio;
	char msg[MSGSIZE];
} mr_logger_msg;

// int queue, syslog and server thread
void logger_init()
{
	msgmutex = xSemaphoreCreateMutex();
	if (msgmutex == NULL) {
		printf("Error: mutex can not be craeted\n");
	}

	msgque = xQueueCreate(MAXMSG, MSGSIZE);
	if (msgque == NULL) {
		printf("Error: queue can not be craeted\n");
	}

	// create server thread
	BaseType_t retval = xTaskCreate(logger_server_task, "Logger Server", STACK_SIZE, NULL, (osPriority_t)LOGGER_TASK_PRIORITY, &logger_server);
	if (retval != pdPASS) {
		printf("Error: logger_init(), xTaskCreate()\n");
	}

	log_message(log_info, "Starting Logger");
}

void logger_cleanup()
{
	log_message(log_info, "Stopping Logger");
	vTaskDelete(logger_server);
	vQueueDelete(msgque);
	vSemaphoreDelete(msgmutex);
}

void logger_set_log_level(mr_logprio_t prio)
{
	log_level = prio;
}

void logger_set_screen_level(mr_logprio_t prio)
{
	screen_level = prio;
}

void logger_use_colors(bool colors)
{
	use_colors = colors;
}

static void logger_server_task(void* args)
{
	mr_logger_msg msg;

	if (use_colors)
		printf("\033[0m"); // clear colors

	while (true) {
		// Get the next message
		BaseType_t ret = xQueueReceive(msgque, &msg, portMAX_DELAY);

		if (ret != pdTRUE) {
			printf("Can not receive new log message\n");
			break;
		}

		if (msg.prio <= screen_level) {
			if (msg.prio <= log_emerg) {
				msg.prio = log_emerg;
			}

			if (msg.prio < log_warning) {
				if (use_colors)
					printf("\033[31m"); // red screen color for errors
				puts(msg.msg);
				if (use_colors)
					printf("\033[0m"); // clear colors
			} else if (msg.prio < log_info) {
				if (use_colors)
					printf("\033[33m"); // yellow screen color for warnings
				puts(msg.msg);
				if (use_colors)
					printf("\033[0m"); // clear colors
			} else if (msg.prio == log_info) {
				if (use_colors)
					printf("\033[32m"); // green screen color infos
				puts(msg.msg);
				if (use_colors)
					printf("\033[0m"); // clear colors
			} else {
				puts(msg.msg);
			}
			fflush(stdout);
		}

		if (msg.prio <= log_level) {
			ros_log_message(msg.prio, msg.msg); // log to ros
		}
	}
}

void log_message(mr_logprio_t prio, const char* format, ...)
{
	mr_logger_msg msg;

	char numbering[20];
	snprintf(numbering, sizeof(numbering), "%d. ", counter++);
	strcpy(msg.msg, numbering);

	msg.prio = prio;
	va_list arglist;
	va_start(arglist, format);
	int ret = vsnprintf(msg.msg + strlen(numbering), sizeof(msg.msg), format, arglist);
	va_end(arglist);

	// Abort if printf gave an error or no message is to be logged
	if (ret <= 0) {
		return;
	}

	// Terminate message if too long
	if (ret >= 0 && (unsigned int)ret >= sizeof(msg.msg)) {
		msg.msg[sizeof(msg.msg) - 1] = '\0';
	}

	if (xSemaphoreTake(msgmutex, 1) == pdTRUE) {
		if (msgque) {
			xQueueSend(msgque, &msg, 0);
		}
		xSemaphoreGive(msgmutex); // unlock
	}
}

void ros_log_message(mr_logprio_t prio, const char* msg)
{
	return; // do nothing, at the moment rosserial_python crashes on ros log messages
}
