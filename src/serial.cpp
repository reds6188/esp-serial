#include "serial.h"

Uart::Uart() {
}

Uart::Uart(uart_port_t uart_num, uint8_t buffer_size) {
    _uart_num = uart_num;
    _buffer_size = buffer_size;
}

void Uart::setNum(uart_port_t uart_num) {
    _uart_num = uart_num;
}

void Uart::setBufferSize(uint8_t buffer_size) {
    _buffer_size = buffer_size;
}

esp_err_t Uart::begin(uart_pin_t uart_pin, uart_config_t uart_config) {

    if(uart_param_config(_uart_num, &uart_config) != ESP_OK) {
        return ESP_FAIL;
    }

    //uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
    
    if(uart_set_pin(_uart_num, uart_pin.tx_pin, uart_pin.rx_pin, uart_pin.rts_pin, uart_pin.cts_pin) != ESP_OK) {
        return ESP_FAIL;
    }

    if(uart_driver_install(_uart_num, BUF_SIZE, BUF_SIZE, QUEUE_SIZE, &uart_queue[_uart_num], 0) != ESP_OK) {
        return ESP_FAIL;
    }

    if(uart_pin.en_pin != UART_PIN_NO_CHANGE) {
        en_pin = uart_pin.en_pin;
        pinMode(uart_pin.en_pin, OUTPUT);       // Pin per il driver esterno
        digitalWrite(uart_pin.en_pin, HIGH);    // Driver in ricezione
    }

    return ESP_OK;
}

void Uart::end(void) {
    if(uart_wait_tx_done(_uart_num, 100) == ESP_ERR_TIMEOUT)
        Serial.println("UART TX FIFO Empty timeout error");

    if(uart_driver_delete(_uart_num) != ESP_OK) {
        Serial.println("Failed to delete drivers for UART" + String(_uart_num));
    }
    else {
        Serial.println("Drivers for UART" + String(_uart_num) + " were deleted");
        vTaskDelete(xHandle);
    }
}

esp_err_t Uart::setHandler(void(*callback)(uint8_t *data, int data_size)) {

    BaseType_t result = pdPASS;
    _handler = callback;
    xHandle = NULL;
    if(xTaskCreate(UartIrqHandler, "UART_ISR_ROUTINE", 2048, this, 12, &xHandle) == pdPASS) {
        return ESP_OK;
    }

    return ESP_FAIL;
}

void Uart::UartIrqHandler(void *pvParameters) {
    Uart *pThis = (Uart *) pvParameters;
    uart_event_t event;
    uint8_t *uart_data = (uint8_t*) malloc(pThis->_buffer_size);
    uart_port_t uart_n = pThis->_uart_num;
    //console.log(uart_tag[uart_n], "Rx handler was set");

	bool exit_condition = false;

    while(1)
    {
        if(xQueueReceive(uart_queue[uart_n], (void * )&event, (portTickType)portMAX_DELAY))
        {
            int uart_data_length = 0;
            bzero(uart_data, pThis->_buffer_size);
            switch(event.type)
            {
                // Event of UART receving data
                case UART_DATA:
                    //int uart_data_length = 0;
                    uart_get_buffered_data_len(uart_n, (size_t*)&uart_data_length);
                    //Serial.printf("uart length: %d\n", uart_data_length);
                    if(uart_data_length > pThis->_buffer_size) {
                        Serial.println("Length exceeds the maximum allowed");
                        uart_flush_input(uart_n);
                    }
                    else {
                        uart_data_length = uart_read_bytes(uart_n, uart_data, uart_data_length, 100);
                        if(pThis->_handler != nullptr) {
                            (*(pThis->_handler))(uart_data, uart_data_length);
                        }
                        else {
                            Serial.println("Pointer is null");
                        }
                    }
                    break;
                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_n);
                    xQueueReset(uart_queue[uart_n]);
                    break;
                // Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uart_n);
                    xQueueReset(uart_queue[uart_n]);
                    break;
                // Event of UART RX break detected
                case UART_BREAK:
                    break;
                // Event of UART parity check error
                case UART_PARITY_ERR:
                    //pThis->uart_error.parity_err++;
                    break;
                // Event of UART frame error
                case UART_FRAME_ERR:
                    //pThis->uart_error.frame_err++;
                    break;
                case UART_PATTERN_DET:
                    break;
                default:
                    break;
            }



        }
        //If you want to break out of the loop due to certain conditions, set exit condition to true
        if(exit_condition) {
            break;
        }
    }

    //Out side of loop now. Task needs to clean up and self terminate before returning
    free(uart_data);
    uart_data = NULL;
    vTaskDelete(NULL);
}

void Uart::writeData(uint8_t *data, int data_size) {
	if(en_pin != UART_PIN_NO_CHANGE)
		digitalWrite(en_pin, HIGH);
	uart_write_bytes(_uart_num, (const char*)data, data_size);
	uart_wait_tx_done(_uart_num, 100);
	if(en_pin != UART_PIN_NO_CHANGE)
		digitalWrite(en_pin, LOW);
}