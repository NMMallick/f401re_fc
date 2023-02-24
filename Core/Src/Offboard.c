#include "Offboard.h"
static volatile int k = 0;

void initiate_connection(Offboard_TypeDef *dtype)
{
    offb_dtype = dtype;

    for (uint32_t i = 0; i < RX_BUF_SIZE; i ++)
        offb_dtype->rx_buf[i] = 0x00;

    for (uint32_t i = 0; i < TX_BUF_SIZE; i++)
        offb_dtype->tx_buf[i] = 0x00;

    HAL_UART_Receive_DMA(offb_dtype->huart, offb_dtype->rx_buf, 16);
}

void check_buffer()
{
    // offb_dtype->tx_buf[0] = GOOD_TX;

    // motor_cmd((Motor_Scalars *)&offb_dtype->rx_buf);
    // uint8_t valid = offb_dtype->rx_buf[RX_BUF_SIZE-1];
    // uint8_t *pitchb = &offb_dtype->rx_buf[0];

    // uint8_t *buf = &offb_dtype->rx_buf;
    // memcpy(&offb_dtype->tx_buf, offb_dtype->rx_buf, TX_BUF_SIZE);
    for (int i = 0; i < TX_BUF_SIZE; i++)
        offb_dtype->tx_buf[i] = offb_dtype->rx_buf[i];

    float axes[4];
    axes[0] = *(float *)(&offb_dtype->tx_buf[0]);
    axes[1] = *(float *)(&offb_dtype->tx_buf[4]);
    axes[2] = *(float *)(&offb_dtype->tx_buf[8]);
    axes[3] = *(float *)(&offb_dtype->tx_buf[12]);

    axes[0] /= axes[0];
    axes[1] = 0.5;
    axes[2] /= axes[2];
    axes[3] = 0.6;

    uint8_t *ptr = (uint8_t*)axes;

    for (int i = 0; i < 16; i++)
    {
        offb_dtype->tx_buf[i] = *(ptr + i);
    }

    HAL_UART_Transmit_DMA(offb_dtype->huart, offb_dtype->tx_buf, TX_BUF_SIZE);
    // for (uint32_t i = 0; i < RX_BUF_SIZE; i ++)
    //     offb_dtype->rx_buf[i] = 0x00;

    HAL_UART_Receive_DMA(offb_dtype->huart, offb_dtype->rx_buf, RX_BUF_SIZE);
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