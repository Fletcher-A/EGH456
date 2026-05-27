/*
 * uart_log.c — Thread-safe logging via xUARTMutex.
 */

#include <stdarg.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "utils/uartstdio.h"
#include "shared.h"

void uart_log_vprintf(const char *pcFormat, va_list vaArgP)
{
    if (xUARTMutex != NULL &&
        xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        UARTvprintf(pcFormat, vaArgP);
        xSemaphoreGive(xUARTMutex);
    }
}

void uart_log_printf(const char *pcFormat, ...)
{
    va_list vaArgP;

    va_start(vaArgP, pcFormat);
    uart_log_vprintf(pcFormat, vaArgP);
    va_end(vaArgP);
}

void uart_plot_printf(const char *pcFormat, ...)
{
    va_list vaArgP;

    va_start(vaArgP, pcFormat);
    /* Plot task is the only CSV writer; avoid dropping lines on mutex timeout. */
    if (xUARTMutex != NULL)
    {
        xSemaphoreTake(xUARTMutex, portMAX_DELAY);
    }
    UARTvprintf(pcFormat, vaArgP);
    if (xUARTMutex != NULL)
    {
        xSemaphoreGive(xUARTMutex);
    }
    va_end(vaArgP);
}
