/*
 * uart_log.h — Mutex-serialised UART output for multi-task use.
 *
 * UARTprintf() is not thread-safe. Any task that logs after the scheduler
 * starts must use uart_log_printf() so lines are not interleaved.
 */
#ifndef UART_LOG_H
#define UART_LOG_H

#include <stdarg.h>

void uart_log_printf(const char *pcFormat, ...);
void uart_log_vprintf(const char *pcFormat, va_list vaArgP);

#endif /* UART_LOG_H */
