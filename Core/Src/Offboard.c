#include "Offboard.h"

void initiate_connection(Offboard_TypeDef *dtype)
{
    offb_dtype = dtype;

    HAL_UART_Receive_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, 1);
}

void check_buffer()
{
    offb_dtype->tx_buf[0] = GOOD_TX;

    for (size_t i = 0; i < 9; i++)
    {
        if (offb_dtype->rx_buf[i] != 0xff)
        {
            offb_dtype->tx_buf[0] = BAD_TX;
            break;
        }
    }

    // HAL_Delay(50);
    HAL_UART_Transmit_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->tx_buf, 1);
    HAL_UART_Receive_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, 9);
}
// TODO