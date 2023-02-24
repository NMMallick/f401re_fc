#include "Offboard.h"
static volatile int k = 0;

void Offboard_Init(Offboard_TypeDef *dtype)
{
    offb_dtype = dtype;

    for (uint32_t i = 0; i < RX_BUF_SIZE; i ++)
        offb_dtype->rx_buf[i] = 0x00;

    for (uint32_t i = 0; i < TX_BUF_SIZE; i++)
        offb_dtype->tx_buf[i] = 0x00;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        offb_dtype->quad_motors->motors[i].speed = 0x00;
    }

    HAL_UART_Receive_DMA(offb_dtype->huart, offb_dtype->rx_buf, RX_BUF_SIZE);
}

void Offboard_Check_Buffer()
{
    // Copy over the RX buffer to send back to host for
    //  validation
    for (int i = 0; i < TX_BUF_SIZE; i++)
        offb_dtype->tx_buf[i] = offb_dtype->rx_buf[i];

    // Command motors
    set_motor_speed((Motor_Scalars *)offb_dtype->rx_buf);

    // Transmit data back after commanding motors to avoid
    //  over transmitting
    HAL_UART_Transmit_DMA(offb_dtype->huart, offb_dtype->tx_buf, TX_BUF_SIZE);
    HAL_UART_Receive_DMA(offb_dtype->huart, offb_dtype->rx_buf, RX_BUF_SIZE);
}

void set_motor_speed(Motor_Scalars *input)
{
    // Validation
    float validation = input->pitch * input->roll * input->thrust * input->yaw;
    if (validation > 1.0 || validation < -1.0)
        return;

    float mixers[4];

    mixers[0] = input->pitch + input->roll + input->pitch + input->yaw;
    mixers[1] = input->pitch - input->roll + input->pitch - input->yaw;
    mixers[2] = input->pitch + input->roll - input->pitch - input->yaw;
    mixers[3] = input->pitch - input->roll - input->pitch + input->yaw;

    uint16_t val;
    for (int i = 0; i < 4; i++)
    {
        val = mixers[0] * (MAX_THROTTLE - MIN_THROTTLE) + mixers[i];
        offb_dtype->quad_motors->motors[i].speed = val;
    }

}

// TODO