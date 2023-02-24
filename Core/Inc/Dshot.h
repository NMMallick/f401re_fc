#ifndef DSHOT_H_
#define DSHOT_H_

// Using HAL libraries to write PWM through DMA
#include "stm32f4xx_hal.h"

// MACROS
#define DSHOT_0 89
#define DSHOT_1 180

#define DSHOT300_ARR 240 // 3.33 uS bit length (DSHOT 300)
#define ARM_TIME 3000

#define MAX_BUFF_SIZE 16

#define NUM_MOTORS 4

// Constants
const static uint16_t MAX_THROTTLE = 2047;
const static uint16_t MIN_THROTTLE = 48;


/**
 * @brief Generic motor structure for a PWM interface
 */
typedef struct {
    TIM_HandleTypeDef *tim;
    uint32_t channel;
    uint16_t buffer[MAX_BUFF_SIZE+2];
    uint16_t speed;
} Motor_HandleTypeDef;

/**
 * @brief Quadcopter motor structure
 * @todo this may be excessive if more info isn't added
 */
typedef struct {
    Motor_HandleTypeDef motors[4];
} QuadMotor_HandleTypeDef;

QuadMotor_HandleTypeDef *quad_motors;

/**
 * @brief Initialisation function for the timing and packet
 *          buffer needed for the dshot protocol
 */
void DSHOT_Init(QuadMotor_HandleTypeDef *motors);

/**
 * @brief Arming sequence for the BHEli_S ESC
 * @note You must continuously send datat to the ESCs after
 *          this function has finished else the motors will
 *          not be armed.
 */
void DSHOT_Arm();

/**
 * @brief Create a packet of buffer data that prescribes todo
 *          the dshot 300 protocol
 * @param the throttle value to be sent to the ESC
 */
void DSHOT_Create_Packet(uint16_t val, uint16_t *buf);

/**
 * @brief Read from DMA and create the PWM signal
 * @param motor Motor type def to determine which PWM channel to send data to
 * @param val Throttle value to be sent to the ESC/motor
 * @note val is clamped at THROTTLE_MIN and THROTTLE_MAX
 */
void DSHOT_Command_Motor(Motor_HandleTypeDef *motor);

/**
 * @brief Command all motors with the most recent throttle value given
 * @note calls DSHOT_command_motor() on all motors available
 */
void DSHOT_Command_All_Motors();

#endif