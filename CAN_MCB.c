#include "include/CAN_MCB.h"
#include "string.h"
#include "../../../include/main.h"              // TODO: mirar este def
#include "esp_log.h"

#define PATTERN_QUEUE_SIZE  20

extern QueueHandle_t newMcbQueueHandler; 
extern QueueHandle_t motorControlQueueHandler; 

config_init_mcb_t mcbConfigInit;
static QueueHandle_t spp_uart_queue;
static const char *TAG = "CAN_MCB";

static void sendMotorData(int16_t motR,int16_t motL,uint8_t enable) {
    uint16_t calcChecksum = START_CODE_HEADER^motL^motR^enable;
    tx_motor_control_board_t command = {
        .start = START_CODE_HEADER,
        .speedL = motL,
        .speedR = motR,
        .enable = enable,
        .checksum = calcChecksum
    };
    uart_write_bytes(mcbConfigInit.numUart,&command, sizeof(command));
}

static void processMCBData(uint8_t *data) {
    rx_motor_control_board_t newMcbData;
    uint16_t newChecksum = 0;

    memcpy(&newMcbData,data,sizeof(rx_motor_control_board_t));

    if (newMcbData.start == START_CODE_HEADER) {

        newChecksum = (uint16_t)(newMcbData.start ^ newMcbData.cmd1 ^ newMcbData.cmd2 ^ newMcbData.speedR_meas ^ newMcbData.speedL_meas 
                                       ^ newMcbData.batVoltage ^ newMcbData.boardTemp ^ newMcbData.cmdLed ^ newMcbData.isCharging ^ newMcbData.currentR ^ newMcbData.currentL);
        if( newChecksum == newMcbData.checksum){
            // printf("Nuevo paquete OK -> voltage: %d\tposL: %ld\tposR:%ld\n",newMcbData.batVoltage,newMcbData.posL,newMcbData.posR);
            xQueueSend(newMcbQueueHandler,&newMcbData,0);
        }
        // else {
        //     ESP_LOGI(TAG,"Error checksum");
        // }
    }
}

static void controlHandler(void *pvParameters) {
    output_motors_t newVel;
    uart_event_t event;
    uint8_t data[100];

    while(true) {
        if (xQueueReceive(motorControlQueueHandler,&newVel,0)) {
            sendMotorData(newVel.motorR,newVel.motorL,newVel.enable);
        }

        if (xQueueReceive(spp_uart_queue, (void * )&event, 0)) {
            
            switch(event.type) {
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(mcbConfigInit.numUart);
                    xQueueReset(spp_uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(mcbConfigInit.numUart);
                    xQueueReset(spp_uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:

                    size_t sizeBuffer;
                    uart_get_buffered_data_len(mcbConfigInit.numUart, &sizeBuffer);
                    if (sizeBuffer < sizeof(rx_motor_control_board_t)) { 
                        break;                                                  // wait for the correct size
                    }
                    int pos = uart_pattern_pop_pos(mcbConfigInit.numUart);                 // get the position of the pattern
                    // ESP_LOGI(TAG, "[UART PATTERN DETECTED] buffered size: %d", sizeBuffer);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(mcbConfigInit.numUart);
                        // Reiniciar la cola de patrones si está llena
                        uart_pattern_queue_reset(mcbConfigInit.numUart, PATTERN_QUEUE_SIZE);
                    } else {
                        uint8_t discard[50];         // TODO: mejorar metodo de descarte, estoy usando memoria inutilmente
                        uart_read_bytes(mcbConfigInit.numUart, discard, pos,0);   // just to discard previous bytes
                        uart_read_bytes(mcbConfigInit.numUart, data, sizeof(rx_motor_control_board_t),0);
                        processMCBData(data);
                        uart_flush_input(mcbConfigInit.numUart);
                    }
                break;
                default:
                    if (event.type != UART_DATA) {
                        ESP_LOGI(TAG, "unhandled event type: %d", event.type);
                    }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mcbInit(config_init_mcb_t *config) {
    mcbConfigInit = *config;
    /*configuro periferico*/
    uart_config_t uartConfig={
        .baud_rate = MAINBOARD_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .stop_bits = UART_STOP_BITS_1, 
        .source_clk = UART_SCLK_APB,
    };

    // buffer size of tx y rx
    const int uart_buffer_size = (1024 * 4);
    ESP_ERROR_CHECK(uart_driver_install(config->numUart, uart_buffer_size, uart_buffer_size, 10, &spp_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(config->numUart,&uartConfig));
    /* configure pinout uart*/
    ESP_ERROR_CHECK(uart_set_pin(config->numUart,config->txPin,config->rxPin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(config->numUart, START_CODE_HEADER_PATTERN, 1, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(config->numUart, PATTERN_QUEUE_SIZE);

    xTaskCreatePinnedToCore(controlHandler,"MCB handler task",4096,NULL,10,NULL,config->core);      // TODO: pasar queue
    ESP_LOGI(TAG,"initialized");
}
