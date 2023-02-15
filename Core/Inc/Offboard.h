#ifndef OFFBOARD_H_
#define OFFBOARD_H_

// HAL Library
#include "stm32f4xx_hal.h"

// TODO
// - structs (handle types, protocols, buffers, etc...)
// - initialisation
// - read
// - write

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t tx_buf[256],
            rx_buf[256];
} Offboard_TypeDef;

typedef enum {
    GOOD_TX = 0,
    BAD_TX,
} Offboard_RX_Enum;

Offboard_TypeDef *offb_dtype;

// What process do I want for initializing
//  this process ?
// - handshake?
void initiate_connection(Offboard_TypeDef *);

void check_buffer();

#endif