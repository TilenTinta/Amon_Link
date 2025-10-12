/*****************************************************************
 * File Name          : NRF24L01.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/08/24 
 * Description        : Driver for NRF24L01 radio (functions)
*****************************************************************/

/* Includes */
#include "NRF24L01.h"

/* Private functions - used to manipulate pin states */
static inline void nrf_csn_low (nRF24L01 *dev)      // Reset CS pin
{ 
    GPIO_ResetBits(dev->CS_Port, dev->CS_Pin); 
}   

static inline void nrf_csn_high(nRF24L01 *dev)      // Set CS pin
{ 
    GPIO_SetBits(dev->CS_Port, dev->CS_Pin); 
}   

static inline void nrf_ce_low  (nRF24L01 *dev)      // Reset CE pin
{ 
    GPIO_ResetBits(dev->CE_Port, dev->CE_Pin); 
}   

static inline void nrf_ce_high (nRF24L01 *dev)      // Set CE pin
{ 
    GPIO_SetBits(dev->CE_Port, dev->CE_Pin); 
}   

static inline uint8_t nrf_spi_txrx(nRF24L01 *dev, uint8_t byte)     // Send and receive data over SPI
{
    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(dev->SPIx, byte);

    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_RXNE) == RESET) { }
    return (uint8_t)SPI_I2S_ReceiveData(dev->SPIx);
}



/*********************************************************************
 * @fcn     NRF24_Attach
 *
 * @brief   Bind SPI and CS/CE pins to a device struct
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_Attach(nRF24L01 *dev, SPI_TypeDef *SPIx, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin)
{
    dev->SPIx    = SPIx;
    dev->CS_Port = cs_port;
    dev->CS_Pin  = cs_pin;
    dev->CE_Port = ce_port;
    dev->CE_Pin  = ce_pin;

    // Set default pin state
    nrf_csn_high(dev);
    nrf_ce_low(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_SPI_Write
 *
 * @brief   Write/send raw bytes
 *          Discarding RX bytes
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_SPI_Write(nRF24L01 *dev, const uint8_t *tx, uint8_t len)
{
    nrf_csn_low(dev);

    for (uint8_t i = 0; i < len; i++) {
        (void)nrf_spi_txrx(dev, tx[i]);
    }

    nrf_csn_high(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_SPI_Read
 *
 * @brief   Read raw bytes
 *          Send NOP byte
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_SPI_Read(nRF24L01 *dev, uint8_t *rx, uint8_t len, uint8_t fill_byte)
{
    nrf_csn_low(dev);

    for (uint8_t i = 0; i < len; i++) {
        rx[i] = nrf_spi_txrx(dev, fill_byte);
    }

    nrf_csn_high(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_ReadRegister
 *
 * @brief   Read one register (R_REGISTER | reg),
 *          STATUS is shifted out as the first returned byte,
 *              - returned via status_out if not NULL.
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_ReadRegister(nRF24L01 *dev, uint8_t reg, uint8_t *value, uint8_t *status_out)
{
    nrf_csn_low(dev);

    uint8_t status = nrf_spi_txrx(dev, (uint8_t)(R_REGISTER | (reg & 0x1F)));  
    *value = nrf_spi_txrx(dev, NOP);  

    nrf_csn_high(dev);

    if (status_out) *status_out = status;

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_WriteRegister
 *
 * @brief   Write one register (W_REGISTER | reg),
 *          STATUS returned too.
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_WriteRegister(nRF24L01 *dev, uint8_t reg, uint8_t value, uint8_t *status_out)
{
    nrf_csn_low(dev);

    // Correct method with command and register 
    uint8_t status = nrf_spi_txrx(dev, (uint8_t)(W_REGISTER | (reg & 0x1F)));
    (void)nrf_spi_txrx(dev, value);

    nrf_csn_high(dev);

    if (status_out) *status_out = status;

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_SPI_Transceive
 *
 * @brief   Send and receive data from the device
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_SPI_Transceive(nRF24L01 *dev, const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    nrf_csn_low(dev);

    for (uint8_t i = 0; i < len; i++) {
        rx[i] = nrf_spi_txrx(dev, tx[i]);
    }

    nrf_csn_high(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_ReadStatus
 *
 * @brief   Quick alive test (NOP) ¡ª returns STATUS
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_ReadStatus(nRF24L01 *dev, uint8_t *status_out)
{
    nrf_csn_low(dev);

    *status_out = nrf_spi_txrx(dev, NOP);   // No Operation. Might be used to read the STATUS register

    nrf_csn_high(dev);

    // A dead bus often reads 0xFF or 0x00
    return 0;
}