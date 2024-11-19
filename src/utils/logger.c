#include "logger.h"
#include "stdio.h"
#include <stdarg.h>
#include "stdbool.h"

static enum LOG_LEVEL log_level;
static bool use_colors = true;

const char* log_list[] = {
    "! FATAL !",
    "ERROR",
    "WARN",
    "LOG",
    "INFO",
    "DEBUG",
    "NONE"
};

const char* log_colors[] = {
    "\033[1;31m",     // Bright Red
    "\033[0;31m",     // Red
    "\033[38;5;214m", // Orange
    "\033[0;33m",     // Yellow
    "\033[0;32m",     // Green
    "\033[0;36m",     // Blue
    ""                // Nada
};

void InitializeLogger(enum LOG_LEVEL level, bool enable_colors) {
    log_level = level;
    use_colors = enable_colors;
}

void _Log(enum LOG_LEVEL passed_log_level, const char* format, ...) {
    if (log_level < passed_log_level) {
        return;
    }

    const char* start_color = use_colors ? log_colors[passed_log_level] : "";
    const char* str_log_level = log_list[passed_log_level];

    printf("%s %s : ", start_color, str_log_level);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    if (use_colors) {
        printf("\033[0m"); // Reset color
    }
    fflush(stdout);
    fflush(stderr);
}

void DeitializeLogger();

