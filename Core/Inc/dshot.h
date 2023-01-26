#ifndef DSHOT_H
#define DSHOT_H

// Using HAL libraries to write PWM through DMA
#include "stm32f4xx_hal.h"

// MACROS
#define DSHOT_0 89
#define DSHOT_1 180

#define DSHOT300_ARR 240 // 3.33 uS bit length (DSHOT)
#define ARM_TIME 2500

#define MOTOR_1_CHANNEL TIM_CHANNEL_3
#define MOTOR_2_CHANNEL TIM_CHANNEL_4
#define MOTOR_3_CHANNEL TIM_CHANNEL_1
#define MOTOR_4_CHANNEL TIM_CHANNEL_2

// typedef enum Motor_EnumTypeDef {
//     MOTOR_1 = 0,
//     MOTOR_2,
//     MOTOR_3,
//     MOTOR_4
// } Motor_Type;

typdef struct Motor_HandleTypeDef {
    TIM_HandleTypeDef *tim;

} Motor_Handle;

typedef struct QuadMotor_TypeDef {
    Motor
} QuadMotorStruct;

// Constants
const static uint16_t MAX_THROTTLE = 2047;
const static uint16_t MIN_THROTTLE = 48;

static uint16_t MOTOR_1_BUF[18];
static uint16_t MOTOR_2_BUF[18];
static uint16_t MOTOR_3_BUF[18];
static uint16_t MOTOR_4_BUF[18];

/**
 * @brief Initializing the timing and packet buffer for the dshot protocol
 */
void DSHOT_init();

/**
 * @brief Arming sequence for the BHEli_S ESC
 *
 */
void DSHOT_arm();

/**
 * @brief Create a packet of buffer data that prescribes todo
 *          the dshot 300 protocol
 * @param the throttle value to be sent to the ESC
 * @note val is clamped at THROTTLE_MIN and THROTTLE_MAX
 */
void DSHOT_create_packet(uint16_t val, uint16_t *buf);

/**
 * @brief Read from DMA and create the PWM signal
 * @param motor Motor type def to determine which PWM channel to send data to
 * @param val Throttle value to be sent to the ESC/motor
 */
void DSHOT_command_motor(Motor_Type motor, uint16_t val);

#endif