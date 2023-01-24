#include "dshot.h"

//
void DSHOT_init(TIM_HandleTypeDef tim)
{
    // Set auto reload value for 3.33uS
    __HAL_TIM_SET_AUTORELOAD(&tim, ARR);

    // Set the end of the packet/buffer to always have 4 bytes of 0
    MOTOR_BUF_1[16] = 0x0000;
    MOTOR_BUF_1[17] = 0x0000;

    MOTOR_BUF_2[16] = 0x0000;
    MOTOR_BUF_2[17] = 0x0000;

    MOTOR_BUF_3[16] = 0x0000;
    MOTOR_BUF_3[17] = 0x0000;

    MOTOR_BUF_4[16] = 0x0000;
    MOTOR_BUF_4[17] = 0x0000;

    // Save the timer
    PWM = tim;
}

void DSHOT_arm()
{
    uint32_t millis = HAL_GetTick();

    while ((HAL_GetTick() - millis) < ARM_TIME)
    {
        DSHOT_create_packet(0);
        DSHOT_write();

        HAL_Delay(10);
    }
}

// (TODO) I might want to add a parameter that specifies which motor
void DSHOT_create_packet(uint16_t val)
{
    // Disable telemetry bit
    val = (val << 1) & 0xfffe;

    // Compute the CRC
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
        MOTOR_BUF_1[i] = (data & 0x01) ? DSHOT_1 : DSHOT_O;
        data = data >> 1;
    }
}

void DSHOT_write()
{
    HAL_TIM_PWM_Start_DMA(PWM_TIM, TIM_CHANNEL_1, (uint32_t *)MOTOR_BUF_1, 17);
}


