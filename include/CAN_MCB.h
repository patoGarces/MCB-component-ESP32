#ifndef CAN_MCB_H
#define CAN_MCB_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "../../../include/main.h"

#ifdef HARDWARE_SPLITBOARD
    #define MAINBOARD_BAUDRATE          19200
#else
    #define MAINBOARD_BAUDRATE          230400 // 921600
#endif 

#define START_CODE_HEADER           0xABCD
#define START_CODE_HEADER_PATTERN   0xCD        // just one byte pattern, litle endian

// MotorControlBoard = MCB

typedef struct {
    gpio_num_t txPin;
    gpio_num_t rxPin;
    uart_port_t numUart;
    QueueHandle_t queue;
    uint8_t core;
}config_init_mcb_t;

typedef enum {
    NO_ERROR_MCB,
    ERROR_MCB_BATTERY,
    ERROR_MCB_TEMP,
    ERROR_MCB_HALL_L,
    ERROR_MCB_HALL_R,
    ERROR_MCB_INACTIVITY,
} status_code_mcb_t;

#ifdef HARDWARE_SPLITBOARD

    typedef struct __attribute__((packed)) {
        uint8_t start;
        int16_t  speedTargetR;
        int16_t  speedTargetL;
        uint8_t  unusedStateMaster;
        uint8_t  unusedStateSlave;
        uint16_t checksum;
    } tx_motor_control_board_t;

    typedef struct __attribute__((packed)) {
        uint16_t  start;
        int16_t   speedL_meas;  // 100* km/h
        int16_t   speedR_meas;
        uint16_t  batVoltage;   // 100* V
        int16_t   currentL;     // 100* A
        int16_t   currentR;
        int32_t   posL;
        int32_t   posR;
        // int16_t   boardTemp;
        // uint16_t  cmdLed;
        // uint8_t   isCharging;
        uint16_t  checksum;

        // uint8_t     ordenCode;
        // uint8_t     errorCode;
    } rx_motor_control_board_t;
#elif defined(HARDWARE_MAINBOARD)
    typedef struct __attribute__((packed)){
        uint16_t start;
        int16_t  speedTargetR;
        int16_t  speedTargetL;
        uint8_t  enable;
        uint16_t checksum;
    } tx_motor_control_board_t;

    typedef struct __attribute__((packed)){
        uint16_t    start;
        int16_t     cmd1;
        int16_t     cmd2;
        int16_t     speedR_meas;
        int16_t     speedL_meas;
        int32_t     posR;
        int32_t     posL;
        int16_t     currentR;
        int16_t     currentL;
        int16_t     batVoltage;
        int16_t     boardTemp;
        uint8_t     statusCode;     // es status_code_mcb_t, pero por defecto ocupa mas de 1 byte un enum en GCC
        uint8_t     isCharging;
        uint16_t    checksum;
        // uint8_t     ordenCode;
    } rx_motor_control_board_t;
#else 
    #error Error hardware mainboard selected
#endif 



void mcbInit(config_init_mcb_t *config);

#endif