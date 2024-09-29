#include "logger.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"

static StaticSemaphore_t loggerMutexBuffer;
SemaphoreHandle_t loggerMutexHandle = NULL;

static TickType_t blockTime = portMAX_DELAY;

void Logger_Init(TickType_t block_time)
{
    loggerMutexHandle = xSemaphoreCreateMutexStatic(&loggerMutexBuffer);
    configASSERT(loggerMutexHandle != NULL);

    blockTime = block_time;

    xSemaphoreGive(loggerMutexHandle);
}

void Logger_Printf(const char *format, ...)
{
    if (xSemaphoreTake(loggerMutexHandle, blockTime) == pdTRUE)
    {
        va_list args;
        va_start(args, format);

        vprintf(format, args);

        va_end(args);

        xSemaphoreGive(loggerMutexHandle);
    }
}

void Logger_LogPrintf(LogLevel_t log_level, const char *format, ...)
{
    if (xSemaphoreTake(loggerMutexHandle, blockTime) == pdTRUE)
    {
        va_list args;
        va_start(args, format);

        TickType_t tick_count = xTaskGetTickCount();
        const char *task_name = pcTaskGetName(xTaskGetCurrentTaskHandle());

        switch (log_level)
        {
        case LogLevel_Info:
            printf("[INFO] ");
            break;
        case LogLevel_Warn:
            printf("[WARN] ");
            break;
        case LogLevel_Error:
            printf("[ERROR] ");
            break;
        default:
            break;
        }
        printf("[%lu] [%s]: ", tick_count, task_name);
        vprintf(format, args);
        printf("\n");

        va_end(args);

        xSemaphoreGive(loggerMutexHandle);
    }
}
