#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#define assert_packed _Static_assert

// HAL Library
#include "stm32f4xx_hal.h"

// TODO
// - structs (handle types, protocols, buffers, etc...)
// - initialisation
// - read
// - write

#define RX_BUF_SIZE 4
#define TX_BUF_SIZE 1

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t tx_buf[TX_BUF_SIZE],
            rx_buf[RX_BUF_SIZE];
} Offboard_TypeDef;

Offboard_TypeDef *offb_dtype;

typedef struct __attribute__((__packed__))
{
    uint8_t valid;
    float thrust,
            yaw,
            roll,
            pitch;
} Motor_Scalars;

assert_packed(sizeof(Motor_Scalars) == 17, "Motor Input scalars");

typedef enum {
    GOOD_TX = 0,
    BAD_TX,
} Offboard_RX_Enum;

// What process do I want for initializing
//  this process ?
// - handshake?
void initiate_connection(Offboard_TypeDef *);
void check_buffer();
void motor_cmd(Motor_Scalars *);

#endif