#ifndef SERIAL_H_
#define SERIAL_H_

#include <Arduino.h>
#include "driver/uart.h"

#define BUF_SIZE		(1024 * 2)
#define RX_BUFFER_SIZE	32
#define QUEUE_SIZE		20

typedef struct {
	int tx_pin;
	int rx_pin;
	int rts_pin;
	int cts_pin;
	int en_pin;
} uart_pin_t;

typedef struct {
    unsigned long fifo_ovl;
    unsigned long buffer_full;
    unsigned long rx_break;
    unsigned long parity_err;
    unsigned long frame_err;
} uart_error_t;

class Uart {
	private:
		uart_port_t _uart_num;
		TaskHandle_t xHandle;
		void (*_handler)(uint8_t *data, int data_size);
		uart_error_t uart_error;
		int en_pin = UART_PIN_NO_CHANGE;
	public:
		Uart();
		Uart(uart_port_t uart_num);
		void setNum(uart_port_t uart_num);
		esp_err_t begin(uart_pin_t uart_pin, uart_config_t uart_config);
		void end(void);
		esp_err_t setHandler(void(*callback)(uint8_t *data, int data_size));
		static void UartIrqHandler(void *pvParameters);
		void writeData(uint8_t *data, int data_size);
};


#endif  /* SERIAL_H_ */