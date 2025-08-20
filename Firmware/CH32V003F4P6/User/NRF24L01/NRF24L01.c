/*****************************************************************
 * File Name          : NRF24L01.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/07/06 
 * Description        : Driver for NRF24L01 radio (functions)
*****************************************************************/

/* Includes */
#include "NRF24L01.h"

/* Functions */
//uint8_t SPI_TransceiveData(nRF24L01 *dev, uint8_t tx_data);




/*********************************************************************
 * @fcn     SPI_TransceiveData
 *
 * @brief   Send and receive data over SPI, before and after call 
 *          CS pin must be manipulated
 *
 * @return  none
 */
// uint8_t SPI_TransceiveData(nRF24L01 *dev, uint8_t tx_data)
// {
//     // Wait until TXE flag
//     while (SPI_I2S_GetFlagStatus(dev->SPI_Handle.Instance, SPI_I2S_FLAG_TXE) == RESET);
//     SPI_I2S_SendData(dev->Instance, tx_data);

//     // Wait until RXNE flag
//     while (SPI_I2S_GetFlagStatus(dev->Instance, SPI_I2S_FLAG_RXNE) == RESET);
//     return SPI_I2S_ReceiveData(dev->Instance);
// }