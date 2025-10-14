#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#define UART_NUM UART_NUM_2
#define TXD_PIN 17
#define RXD_PIN 16
#define BUF_SIZE 1024
#define RD_BUF_SIZE (BUF_SIZE)

QueueHandle_t uart_queue;

// UART event task: receives data
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);

    for(;;) {

        if(xQueueReceive(uart_queue, (void * )&event, portMAX_DELAY)) {

            bzero(dtmp, RD_BUF_SIZE);

            switch(event.type) {

                case UART_DATA:
                
                    uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                    dtmp[event.size] = '\0';  
                    // Null-terminate string for convenience

                    printf("Received data: %s\n", dtmp);
                    // At this point, dtmp contains your received message

                    break;

                case UART_FIFO_OVF:
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_PARITY_ERR:

                case UART_FRAME_ERR:
                    break;
                default:
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

