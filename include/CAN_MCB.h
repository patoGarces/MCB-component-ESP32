#ifndef CAN_MCB_H
#define CAN_MCB_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"

#define MAINBOARD_BAUDRATE          921600      // 230400
#define START_CODE_HEADER           0xABCD
#define START_CODE_HEADER_PATTERN   0xCD        // just one byte pattern, litle endian
#define ID_MOTOR_MODULE     0xF0

// MotorControlBoard = MCB

typedef struct {
    gpio_num_t txPin;
    gpio_num_t rxPin;
    uart_port_t numUart;
    QueueHandle_t queue;
    uint8_t core;
}config_init_mcb_t;

typedef struct {
    uint16_t start;
    int16_t  speedR;
    int16_t  speedL;
    uint8_t enable;
    uint16_t checksum;
} tx_motor_control_board_t;

typedef struct {
    uint16_t  start;
    int16_t   cmd1;
    int16_t   cmd2;
    int16_t   speedR_meas;
    int16_t   speedL_meas;
    int32_t   posR;
    int32_t   posL;
    int16_t   currentR;
    int16_t   currentL;
    int16_t   batVoltage;
    int16_t   boardTemp;
    uint16_t  cmdLed;
    uint8_t   isCharging;
    uint16_t  checksum;

    // uint8_t     ordenCode;
    // uint8_t     errorCode;
} rx_motor_control_board_t;

void mcbInit(config_init_mcb_t *config);

#endif