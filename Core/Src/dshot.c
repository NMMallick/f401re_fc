#include "dshot.h"

//
void DSHOT_init()
{
    // Set auto reload value for 3.33uS
    __HAL_TIM_SET_AUTORELOAD(tim, 240);

    // Set the end of the packet/buffer to always have 4 bytes of 0
    MOTOR_1_BUF[16] = 0x00;
    MOTOR_1_BUF[17] = 0x00;

    MOTOR_2_BUF[16] = 0x00;
    MOTOR_2_BUF[17] = 0x00;

    MOTOR_3_BUF[16] = 0x00;
    MOTOR_3_BUF[17] = 0x00;

    MOTOR_4_BUF[16] = 0x00;
    MOTOR_4_BUF[17] = 0x00;
}

void DSHOT_arm()
{
    uint32_t millis = HAL_GetTick();

    DSHOT_create_packet(0, (uint16_t *)MOTOR_1_BUF);
    // DSHOT_create_packet(0, (uint16_t *)MOTOR_BUF_2);
    // DSHOT_create_packet(0, (uint16_t *)MOTOR_BUF_3);
    // DSHOT_create_packet(0, (uint16_t *)MOTOR_BUF_4);


    // HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_2, (uint32_t *)MOTOR_BUF_2, 17);
    // HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_3, (uint32_t *)MOTOR_BUF_3, 17);
    // HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_4, (uint32_t *)MOTOR_BUF_4, 17);

    HAL_TIM_PWM_Start_DMA(MOTOR_1, MOTOR_1_CHANNEL, (uint32_t *)MOTOR_1_BUF, 17);
    while ((HAL_GetTick() - millis) < ARM_TIME)
    {
        // HAL_TIM_PWM_Stop_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_1);
        HAL_Delay(10);
    }

    // HAL_TIM_PWM_Stop_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_1);
}

void DSHOT_create_packet(uint16_t val, uint16_t *buf)
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

void DSHOT_command_motor(Motor_Type motor, uint16_t val)
{
    // Clamp the motor speed
    if (val < MIN_THROTTLE)
        val = MIN_THROTTLE;

    if (val > MAX_THROTTLE)
        val = MAX_THROTTLE;

    // Switch case for each motors (1-4)
    DSHOT_create_packet(val, (uint16_t *)MOTOR_BUF_1);
    HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_1, (uint32_t *)MOTOR_BUF_1, 17);
    // switch (motor)
    // {
    //     case MOTOR_1:
    //         DSHOT_create_packet(val, (uint16_t *)MOTOR_BUF_1);
    //         HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_1, (uint32_t *)MOTOR_BUF_1, 17);
    //         break;
    //     case MOTOR_2:
    //         DSHOT_create_packet(val, (uint16_t *)MOTOR_BUF_2);
    //         HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_2, (uint32_t *)MOTOR_BUF_2, 17);
    //         break;
    //     case MOTOR_3:
    //         DSHOT_create_packet(val, (uint16_t *)MOTOR_BUF_3);
    //         HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_3, (uint32_t *)MOTOR_BUF_3, 17);
    //         break;
    //     case MOTOR_4:
    //         DSHOT_create_packet(val, (uint16_t *)MOTOR_BUF_4);
    //         HAL_TIM_PWM_Start_DMA(PWM_TIM, MOTOR_PWM_CHANNEL_4, (uint32_t *)MOTOR_BUF_4, 17);
    //         break;
    //     default:
    //         break;
    // }
}


