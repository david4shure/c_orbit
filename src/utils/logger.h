#pragma once

#ifndef ORBIT_LOGGER
#define ORBIT_LOGGER

#include "stdbool.h"

#define Fatal(...)   _Log(FATAL, __VA_ARGS__)
#define Error(...)   _Log(ERROR, __VA_ARGS__)
#define Warn(...)    _Log(WARN, __VA_ARGS__)
#define Log(...)     _Log(LOG, __VA_ARGS__)
#define Info(...)    _Log(INFO, __VA_ARGS__)
#define Debug(...)   _Log(DEBUG, __VA_ARGS__)

enum LOG_LEVEL{
    FATAL, // 0 
    ERROR, // 1
    WARN,  // 2
    LOG,   // 3
    INFO,  // 4
    DEBUG, // 5
    NONE,  // 6
};

void InitializeLogger(enum LOG_LEVEL level, bool enable_colors);
void DeitializeLogger();
void _Log(enum LOG_LEVEL,const char* format,...);
#endif // ORBIT_LOGGER

