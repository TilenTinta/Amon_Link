/*****************************************************************
 * File Name          : s_nRF24L01.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/12/29 
 * Description        : Driver for s_nRF24L01 radio (functions)
*****************************************************************/

/* Includes */
#include "nRF24L01.h"


/*###########################################################################################################################################################*/
/* Private functions - used to manipulate pin states and SPI bus*/

// Reset CS pin
static inline void nrf_cs_low(s_nRF24L01 *dev)      
{ 
    GPIO_ResetBits(dev->CS_Port, dev->CS_Pin); 
}   

// Set CS pin
static inline void nrf_cs_high(s_nRF24L01 *dev)      
{ 
    GPIO_SetBits(dev->CS_Port, dev->CS_Pin); 
}   

// Reset CE pin
static inline void nrf_ce_low(s_nRF24L01 *dev)      
{ 
    GPIO_ResetBits(dev->CE_Port, dev->CE_Pin); 
}   

// Set CE pin
static inline void nrf_ce_high(s_nRF24L01 *dev)      
{ 
    GPIO_SetBits(dev->CE_Port, dev->CE_Pin); 
}   

// Send and receive data over SPI
static inline uint8_t nrf_spi_txrx(s_nRF24L01 *dev, uint8_t byte)     
{
    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(dev->SPIx, byte);

    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_I2S_FLAG_RXNE) == RESET) { }
    return (uint8_t)SPI_I2S_ReceiveData(dev->SPIx);
}


/*###########################################################################################################################################################*/
/* Functions */


/*********************************************************************
 * @fcn     NRF24_pin_config
 *
 * @param   *dev: pointer to device struct
 * @param   *SPIx: pointer to SPI peripheral
 * @param   *cs_port: pointer to chip select port
 * @param   cs_pin: chip select pin number
 * @param   *ce_port: pointer to chip enable port
 * @param   ce_pin: chip enable pin number
 *
 * @brief   Set SPI and CS/CE pins to a device struct
 *
 * @return  None
 */
void NRF24_pin_config(s_nRF24L01 *dev, SPI_TypeDef *SPIx, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin)
{
    // Save to struct for future use
    dev->SPIx    = SPIx;
    dev->CS_Port = cs_port;
    dev->CS_Pin  = cs_pin;
    dev->CE_Port = ce_port;
    dev->CE_Pin  = ce_pin;

    // Set default pin state
    nrf_cs_high(dev);
    nrf_ce_low(dev);
}


/*********************************************************************
 * @fcn     NRF24_SPI_Write
 *
 * @param   *dev: device struct 
 * @param   *tx: pointer to data for transmition
 * @param   len: lenght of data to tansmit
 *
 * @brief   Write/send raw bytes
 *          Discarding RX bytes
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_SPI_Write(s_nRF24L01 *dev, const uint8_t *tx, uint8_t len)
{
    nrf_cs_low(dev);

    for (uint8_t i = 0; i < len; i++) {
        nrf_spi_txrx(dev, tx[i]);
    }

    nrf_cs_high(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_SPI_Read
 *
 * @param   *dev: device struct
 * @param   *rx: pointer to received data (where to save data)
 * @param   len: lenght of data that will be reveived
 * @param   fill_byte: NOP (send NOP data just to get response)
 *
 * @brief   Read raw bytes
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_SPI_Read(s_nRF24L01 *dev, uint8_t *rx, uint8_t len, uint8_t fill_byte)
{
    nrf_cs_low(dev);

    for (uint8_t i = 0; i < len; i++) {
        rx[i] = nrf_spi_txrx(dev, fill_byte);
    }

    nrf_cs_high(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_ReadRegister
 *
 * @param   *dev: device struct
 * @param   reg: register value you want to read
 * @param   *value: pointer to returned value
 * @param   *status_out: pointer to returned value of device status
 *
 * @brief   Read one register (R_REGISTER | reg),
 *          STATUS is first returned byte - status_out if not NULL.
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_ReadRegister(s_nRF24L01 *dev, uint8_t reg, uint8_t *value, uint8_t *status_out)
{
    nrf_cs_low(dev);

    uint8_t status = nrf_spi_txrx(dev, (uint8_t)(R_REGISTER | (reg & 0x1F)));  
    *value = nrf_spi_txrx(dev, NOP);  

    nrf_cs_high(dev);

    if (status_out) *status_out = status;

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_WriteRegister
 *
 * @param   *dev: device struct
 * @param   reg: register value you want to read
 * @param   value: value you want to send
 * @param   *status_out: pointer to returned value of device status
 *
 * @brief   Write one register (W_REGISTER | reg),
 *          STATUS returned too.
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_WriteRegister(s_nRF24L01 *dev, uint8_t reg, uint8_t value, uint8_t *status_out)
{
    nrf_cs_low(dev);

    uint8_t status = nrf_spi_txrx(dev, (uint8_t)(W_REGISTER | (reg & 0x1F)));
    (void)nrf_spi_txrx(dev, value); // (void) "function cast" - i dont need returned value

    nrf_cs_high(dev);

    if (status_out) *status_out = status;

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_SPI_Transceive
 *
 * @param   *dev: device struct
 * @param   *tx: pointer to data you want to transmit
 * @param   *rx: pointer to location you want to save received data
 * @param   len: lenght of transmited data
 *
 * @brief   Send and receive data from the device
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_SPI_Transceive(s_nRF24L01 *dev, const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    nrf_cs_low(dev);

    for (uint8_t i = 0; i < len; i++) {
        rx[i] = nrf_spi_txrx(dev, tx[i]);
    }

    nrf_cs_high(dev);

    return 0;
}


/*********************************************************************
 * @fcn     NRF24_ReadStatus
 *
 * @param   *dev: device struct
 * @param   *status_out: pointer to variable to save returned device status
 *
 * @brief   Quick alive test (NOP) ¡ª returns STATUS
 *
 * @return  0 OK, 1 NOK
 */
uint8_t NRF24_ReadStatus(s_nRF24L01 *dev, uint8_t *status_out)
{
    nrf_cs_low(dev);

    *status_out = nrf_spi_txrx(dev, NOP);   // No Operation. Might be used to read the STATUS register

    nrf_cs_high(dev);

    // A dead bus often reads 0xFF or 0x00
    return 0;
}