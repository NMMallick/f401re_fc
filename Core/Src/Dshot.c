#include "Dshot.h"

//
void DSHOT_Init(QuadMotor_HandleTypeDef *motors)
{
    quad_motors = motors;

    // Set auto reload value for 3.33uS
    __HAL_TIM_SET_AUTORELOAD(motors->motors[0].tim, DSHOT300_ARR);
    __HAL_TIM_SET_AUTORELOAD(motors->motors[1].tim, DSHOT300_ARR);
    __HAL_TIM_SET_AUTORELOAD(motors->motors[2].tim, DSHOT300_ARR);
    __HAL_TIM_SET_AUTORELOAD(motors->motors[3].tim, DSHOT300_ARR);

    // Set the last two bytes of the packet to zero
    motors->motors[0].buffer[16] = 0x00;
    motors->motors[0].buffer[17] = 0x00;

    motors->motors[1].buffer[16] = 0x00;
    motors->motors[1].buffer[17] = 0x00;

    motors->motors[2].buffer[16] = 0x00;
    motors->motors[2].buffer[17] = 0x00;

    motors->motors[3].buffer[16] = 0x00;
    motors->motors[3].buffer[17] = 0x00;
}

void DSHOT_Arm()
{
    // Creating packet info with throttle field value of 0x00
    DSHOT_Create_Packet(0, (uint16_t *)quad_motors->motors[0].buffer);
    DSHOT_Create_Packet(0, (uint16_t *)quad_motors->motors[1].buffer);
    DSHOT_Create_Packet(0, (uint16_t *)quad_motors->motors[2].buffer);
    DSHOT_Create_Packet(0, (uint16_t *)quad_motors->motors[3].buffer);

    // We will send 0x00 to each ESC for about 3 seconds
    uint32_t millis = HAL_GetTick();

    while ((HAL_GetTick() - millis) < ARM_TIME)
    {
        // Start the DMA transmission to the PWM periphal
        HAL_TIM_PWM_Start_DMA(quad_motors->motors[0].tim, quad_motors->motors[0].channel, (uint32_t *)quad_motors->motors[0].buffer, 18);
        HAL_TIM_PWM_Start_DMA(quad_motors->motors[1].tim, quad_motors->motors[1].channel, (uint32_t *)quad_motors->motors[1].buffer, 18);
        HAL_TIM_PWM_Start_DMA(quad_motors->motors[2].tim, quad_motors->motors[2].channel, (uint32_t *)quad_motors->motors[2].buffer, 18);
        HAL_TIM_PWM_Start_DMA(quad_motors->motors[3].tim, quad_motors->motors[3].channel, (uint32_t *)quad_motors->motors[3].buffer, 18);

        HAL_Delay(1);
    }
}

void DSHOT_Create_Packet(uint16_t val, uint16_t *buf)
{
    // Disable telemetry bit
    val = (val << 1) & 0xfffe;

    // Compute the 4-bit CRC
    uint8_t crc = (val ^ (val >> 4) ^ (val >> 8)) & 0x0f;

    // Create 16 bit DSHOT 300 message
    // [0:4] - CRC
    // [5] - Telemetry bit
    // [6:15] - Throttle value
    uint16_t data = (val << 4) | crc;

    // Fill the buffer with the corresponding
    //  value of each bit in data
    for (int i = 15; i >= 0; i--)
    {
        buf[i] = (data & 0x01) ? DSHOT_1 : DSHOT_0;
        data = data >> 1;
    }
}

void DSHOT_Command_Motor(Motor_HandleTypeDef *motor)
{
    // Clamp the motor speed
    if (motor->speed < MIN_THROTTLE)
        motor->speed = MIN_THROTTLE;

    if (motor->speed > MAX_THROTTLE)
        motor->speed = MAX_THROTTLE;

    // Create the packet and begin the transmission
    DSHOT_Create_Packet(motor->speed, (uint16_t *)motor->buffer);
    HAL_TIM_PWM_Start_DMA(motor->tim, motor->channel, (uint32_t *)motor->buffer, 17);
}

void DSHOT_Command_All_Motors()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        DSHOT_Command_Motor(&quad_motors->motors[i]);
    }
}


