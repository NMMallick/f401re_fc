#include "Offboard.h"

void initiate_connection(Offboard_TypeDef *dtype)
{
    offb_dtype = dtype;

    HAL_UART_Receive_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, 1);
}

void check_buffer()
{
    offb_dtype->tx_buf[0] = GOOD_TX;

    // motor_cmd((Motor_Scalars *)&offb_dtype->rx_buf);
    // uint8_t valid = offb_dtype->rx_buf[RX_BUF_SIZE-1];
    // uint8_t *pitchb = &offb_dtype->rx_buf[0];

    uint8_t *buf = &offb_dtype->rx_buf;

    HAL_UART_Transmit_DMA(offb_dtype->huart, (uint8_t *)buf, 4);
    HAL_UART_Receive_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, RX_BUF_SIZE);
}

void motor_cmd(Motor_Scalars *motor_input)
{
    if (!motor_input->valid)
    {
        offb_dtype->tx_buf[0] = BAD_TX;
        return;
    }
}

// TODO