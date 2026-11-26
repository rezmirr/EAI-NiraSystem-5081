#include "uart_io.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

static uint8_t tx_buf[UART_TX_BUF_LEN];
static char fmt_buf[UART_TX_BUF_LEN];
static volatile uint8_t tx_busy = 0;

static void uart_wait_idle(uint32_t timeout_ms){
	uint32_t t0 = HAL_GetTick();
	while(tx_busy){
		if((HAL_GetTick() - t0) > timeout_ms){
			HAL_UART_AbortTransmit(&huart1);
			tx_busy = 0;
			break;
		}
	}
}

void uart_flush(void){
	uart_wait_idle(UART_TX_TIMEOUT_MS);
}

int uart_write(const uint8_t *data, uint16_t len){
	if(len == 0U) return 0;
	if(len > UART_TX_BUF_LEN) len = UART_TX_BUF_LEN;
	
	uart_wait_idle(UART_TX_TIMEOUT_MS);
	memcpy(tx_buf, data, len);
	
	tx_busy = 1;
	
	if(HAL_UART_Transmit_DMA(&huart1, tx_buf, len) != HAL_OK){
		tx_busy = 0;
		return -1;
	}
	
	return (int)len;
}

int uart_printf(const char *fmt, ...){
	va_list ap;
	
	va_start(ap, fmt);
	int n = vsnprintf(fmt_buf, sizeof(fmt_buf), fmt, ap);
	va_end(ap);
	
	if(n < 0) return n;
	if(n >= (int)sizeof(fmt_buf)) n = (int)sizeof(fmt_buf) - 1;
	
	return uart_write((const uint8_t *)fmt_buf, (uint16_t)n);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1) tx_busy = 0;
}

