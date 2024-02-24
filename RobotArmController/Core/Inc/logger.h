#ifndef LOGGER_H_
#define LOGGER_H_

#include "FreeRTOS.h"

#define logInfo(...) Logger_LogPrintf(LogLevel_Info, __VA_ARGS__)
#define logWarn(...) Logger_LogPrintf(LogLevel_Warn, __VA_ARGS__)
#define logError(...) Logger_LogPrintf(LogLevel_Error, __VA_ARGS__)

typedef enum LogLevel
{
    LogLevel_Info,
    LogLevel_Warn,
    LogLevel_Error
} LogLevel_t;

/**
 * @brief Initialize the logger.
 * @param block_time The maximum number of ticks that the logger waits for the logger mutex.
 */
void Logger_Init(TickType_t block_time);

/**
 * @brief The printf function guarded by the logger mutex.
 * @param format Format for the printf function.
 */
void Logger_Printf(const char *format, ...);

/**
 * @brief The printf funtion complemented with other information.
 * @details
 * Prints formatted text complemented with information about the log level,
 * current system time (tick count) and the caller tasks name. The output looks
 * like: "[LOG LEVEL] [TICK COUNT] [TASK NAME]: Some message\n". New line is
 * always printed at the end.
 * @param log_level Log level.
 * @param format Format for the printf function.
 */
void Logger_LogPrintf(LogLevel_t log_level, const char *format, ...);

#endif /* LOGGER_H_ */
