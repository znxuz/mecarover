#pragma once

#include <stdbool.h>

#if __cplusplus
extern "C"
{
#endif

typedef enum mr_logprio_t
{
	log_emerg = 0,
	log_alert = 1,
	log_crit = 2,
	log_error = 3,
	log_warning = 4,
	log_notice = 5,
	log_info = 6,
	log_debug = 7,
} mr_logprio_t;

void log_message(mr_logprio_t prio, const char *fmt, ...) __attribute__ ((__format__ (__printf__, 2, 3)));
void logger_init();
void logger_cleanup();
void logger_set_log_level(mr_logprio_t prio);
void logger_set_screen_level(mr_logprio_t prio);
void logger_use_colors(bool use_colors);

#if __cplusplus
}
#endif
