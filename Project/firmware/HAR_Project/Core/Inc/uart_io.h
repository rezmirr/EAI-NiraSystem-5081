#ifndef UART_IO_H
#define UART_IO_H

#include "main.h"

#define UART_TX_BUF_LEN			768U
#define UART_TX_TIMEOUT_MS	200U

void uart_flush(void);
int uart_write(const uint8_t *data, uint16_t len);
int uart_printf(const char *fmt, ...);
#endif
