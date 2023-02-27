#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#define assert_packed _Static_assert

// HAL Library
#include "stm32f4xx_hal.h"
#include "Dshot.h"

// TODO
// - structs (handle types, protocols, buffers, etc...)
// - initialisation
// - read
// - write

#define RX_BUF_SIZE 16
#define TX_BUF_SIZE (RX_BUF_SIZE)

typedef struct {
    UART_HandleTypeDef *huart;
    Motor_HandleTypeDef *quad_motors;

    uint8_t tx_buf[TX_BUF_SIZE],
            rx_buf[RX_BUF_SIZE];
} Offboard_TypeDef;

Offboard_TypeDef *offb_dtype;
Motor_HandleTypeDef *motors;

typedef struct __attribute__((__packed__))
{
    float thrust,
            yaw,
            roll,
            pitch;
} Motor_Scalars;

assert_packed(sizeof(Motor_Scalars) == 16, "Motor Input scalars");

typedef enum {
    GOOD_TX = 0,
    BAD_TX,
} Offboard_RX_Enum;

// What process do I want for initializing
//  this process ?
// - handshake?
void Offboard_Init(Offboard_TypeDef *, Motor_HandleTypeDef *);
void Offboard_Check_Buffer();
void set_motor_speed(Motor_Scalars *);


#endif