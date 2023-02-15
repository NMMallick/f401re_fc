#include "Offboard.h"

void initiate_connection(Offboard_TypeDef *dtype)
{
    offb_dtype = dtype;

    HAL_UART_Receive_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, 1);

    // if ((char)offb_dtype->rx_buf[0] == 'N')
    //     offb_dtype->tx_buf[0] = 'A';

    // HAL_UART_Transmit_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->tx_buf, 1);

    // // This is bad - fix to use an interupt or DMA + interupt
    // HAL_UART_Receive(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, 1, 0);

    // if ((char)offb_dtype->rx_buf[0] == 'T')
    //     offb_dtype->tx_buf[0] = 'E';
    // else
    //     offb_dtype->tx_buf[0] = 'e';
    // // offb_dtype->tx_buf[0] = 'E';
    // HAL_UART_Transmit_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->tx_buf, 1);
}

void check_buffer()
{
    offb_dtype->tx_buf[0] = 0xff;

    for (size_t i = 0; i < 9; i++)
    {
        if (offb_dtype->rx_buf[i] != 0xff)
        {
            offb_dtype->tx_buf[0] = 0x00;
            break;
        }
    }

    // HAL_Delay(50);
    HAL_UART_Transmit_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->tx_buf, 1);
    HAL_UART_Receive_DMA(offb_dtype->huart, (uint8_t *)offb_dtype->rx_buf, 9);
}
// TODO