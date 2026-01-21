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
/* Private functions - used to manipulate pin states and SPI bus */

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



/*********************************************************************
 * @fn      NRF24_pin_config
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
 * @fn      NRF24_SPI_Write
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
 * @fn      NRF24_SPI_Read
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
 * @fn      NRF24_ReadRegister
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
 * @fn      NRF24_WriteRegister
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
 * @fn      NRF24_SPI_Transceive
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
 * @fn      NRF24_ReadStatus
 *
 * @param   *dev: device struct
 * @param   *status_out: pointer to variable to save returned device status
 *
 * @brief   Quick alive test (NOP) 〞 returns STATUS
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



/*###########################################################################################################################################################*/
/* API - User accesable functions for driver */

/*********************************************************************
 * @fn      NRF24_HandleIRQ
 *
 * @param   *dev: device struct
 *
 * @brief   Manage IRQ event
 *
 * @return  none
 */
void NRF24_HandleIRQ(s_nRF24L01 *dev)
{
    uint8_t status;
    uint8_t pipe;
    NRF24_ReadStatus(dev, &status);
    dev->irq_status = status;

    // RX packet ready (new data available)
    if (status & RX_DR) 
    {
        dev->buffers.flag_new_rx = 1;

        pipe = (status & RX_P_NO) >> 1;

        // valid pipe
        if (pipe != RX_P_NO_EMPTY) 
        {
            dev->irq_on_pipe = pipe;
        }
    }

    // TX done (if ACK used this happens after ACK received)
    if (status & TX_DS) 
    {
        dev->buffers.flag_tx_done = 1;
    }

    // max number of retransmits reached
    if (status & MAX_RT) 
    {
        dev->buffers.flag_max_rxs_reached = 1;
    }

    // clear only the flags that were set
    NRF24_WriteRegister(dev, STATUS, status & (RX_DR | TX_DS | MAX_RT), NULL);
}



/*********************************************************************
 * @fn      NRF24_init
 *
 * @param   *dev: device struct
 *
 * @brief   Configure device 
 *
 * @return  none
 */
void NRF24_init(s_nRF24L01 *dev)
{
    uint8_t reg;

    // Define pin state and default values
    nrf_ce_low(dev);
    nrf_cs_high(dev);

    dev->irq_flag = 0;
    dev->irq_status = 0;

    memset(&dev->buffers, 0, sizeof(dev->buffers)); // clean up / set everything to 0

    // Check and set device power bit 
    NRF24_ReadRegister(dev, CONFIG, &reg, NULL);
    reg &= ~PWR_UP;
    NRF24_WriteRegister(dev, CONFIG, reg, NULL);
    dev->op_modes = NRF_MODE_PWR_DOWN;

    Delay_Us(100);

    // Set address width
    NRF24_WriteRegister(dev, SETUP_AW, dev->config->addr_width & AW, NULL);

    // Set RF channel
    NRF24_WriteRegister(dev, RF_CH, dev->config->channel & RF_CH, NULL);

    // Set datarate and power
    reg = 0;

    if (dev->config->datarate == NRF_DATARATE_2MBPS) {
        reg |= RF_DR;
    }

    switch (dev->config->power) 
    {
        case NRF_POWER_0DBM:        
            reg |= RF_PWR_0DBM << 1; 
            break;
        case NRF_POWER_MINUS_6DBM:  
            reg |= RF_PWR_MIN6DBM << 1; 
            break;
        case NRF_POWER_MINUS_12DBM: 
            reg |= RF_PWR_MIN12DBM << 1; 
            break;
        case NRF_POWER_MINUS_18DBM: 
            reg |= RF_PWR_MIN18DBM << 1; 
            break;
        default:                    
            reg |= RF_PWR_MIN18DBM << 1; 
            break;
    }

    NRF24_WriteRegister(dev, RF_SETUP, reg, NULL);

    // Set auto retransmition
    reg = ((dev->config->retry_delay << 4) & ARD) | (dev->config->retries & ARC);

    NRF24_WriteRegister(dev, SETUP_RETR, reg, NULL);

    // Set auto ACK
    if (dev->config->auto_ack) 
    {
        NRF24_WriteRegister(dev, EN_AA, ENAA_P0, NULL);
    } 
    else 
    {
        NRF24_WriteRegister(dev, EN_AA, 0x00, NULL);
    }

    // Enable RX pipe 0
    NRF24_WriteRegister(dev, EN_RXADDR, ERX_P0, NULL);

    // Enable dynamic payload
    if (dev->config->dynamic_payload) 
    {
        uint8_t activate_cmd[2] = {ACTIVATE, 0x73};
        NRF24_SPI_Write(dev, activate_cmd, 2);

        NRF24_WriteRegister(dev, FEATURE, EN_DPL, NULL);
        NRF24_WriteRegister(dev, DYNPD, DPL_P0, NULL);
    } 
    else 
    {
        NRF24_WriteRegister(dev, FEATURE, 0x00, NULL);
        NRF24_WriteRegister(dev, DYNPD, 0x00, NULL);
    }

    // Flush FIFO registers
    uint8_t cmd;
    cmd = FLUSH_RX;
    NRF24_SPI_Write(dev, &cmd, 1);

    cmd = FLUSH_TX;
    NRF24_SPI_Write(dev, &cmd, 1);

    // Clear all IRQs if present
    NRF24_WriteRegister(dev, STATUS, RX_DR | TX_DS | MAX_RT, NULL);

    // Config CRC settings
    reg = EN_CRC | CRCO;   // CRC enabled, 2-byte CRC

    if (dev->role == NRF_ROLE_PRX) 
    {
        reg |= PRIM_RX;
    }

    NRF24_WriteRegister(dev, CONFIG, reg, NULL);

    // Power up device
    NRF24_ReadRegister(dev, CONFIG, &reg, NULL);
    reg |= PWR_UP;
    NRF24_WriteRegister(dev, CONFIG, reg, NULL);

    Delay_Ms(2);   // 1.5ms required by datasheet

    // Put device in its operationg mode
    if (dev->role == NRF_ROLE_PRX) 
    {
        nrf_ce_high(dev);
        dev->op_modes = NRF_MODE_RX_MODE;
    } 
    else 
    {
        nrf_ce_low(dev);
        dev->op_modes = NRF_MODE_STANDBY_I;
    }  
}



/*********************************************************************
 * @fn      NRF24_Send
 *
 * @param   *dev: device struct
 *
 * @brief   Send data
 *
 * @return  1: error, 0: OK
 */
uint8_t NRF24_Send(s_nRF24L01 *dev)
{
    uint8_t cmd;

    // Device config check
    if (dev->role != NRF_ROLE_PTX) 
    {
        dev->radioErr = NRF_ERR_INVALID_STATE;
        return 1;
    }

    // Check payload lenght
    if (dev->buffers.tx_lenght == 0 || dev->buffers.tx_lenght > 32) 
    {
        return 1;
    }

    nrf_ce_low(dev);    // just in case

    // Clear old flags if set and IRQs
    dev->buffers.flag_tx_done = 0;
    dev->buffers.flag_max_rxs_reached = 0;

    NRF24_WriteRegister(dev, STATUS, TX_DS | MAX_RT, NULL);

    // Flush FIFO buffer
    cmd = FLUSH_TX;
    NRF24_SPI_Write(dev, &cmd, 1);

    // Write payload in device - direct
    nrf_cs_low(dev);
    nrf_spi_txrx(dev, W_TX_PAYLOAD);

    for (uint8_t i = 0; i < dev->buffers.tx_lenght; i++) 
    {
        nrf_spi_txrx(dev, dev->buffers.TX_FIFO[i]);
    }
    nrf_cs_high(dev);

    // Start transmition
    nrf_ce_high(dev);
    Delay_Us(100);                 // >10?s required
    nrf_ce_low(dev);

    return 0;
}



/*********************************************************************
 * @fn      NRF24_ReadRXPayload
 *
 * @param   *dev: device struct
 * @param   *lenght: pointer to number of bytes received
 *
 * @brief   read received data
 *
 * @return  1: error, 0: OK
 */
uint8_t NRF24_ReadRXPayload(s_nRF24L01 *dev)
{
    uint8_t payload_len;
    uint8_t cmd;

    // Device config check
    if (dev->role != NRF_ROLE_PRX) 
    {
        dev->radioErr = NRF_ERR_INVALID_STATE;
        return 1;
    }

    memset(&dev->buffers.RX_FIFO, 0, sizeof(dev->buffers.RX_FIFO)); // clean up / set everything to 0
    dev->buffers.rx_lenght = 0;
    dev->buffers.pipe_data = 0;

    // Read payload width
    nrf_cs_low(dev);
    nrf_spi_txrx(dev, R_RX_PL_WID);
    payload_len = nrf_spi_txrx(dev, NOP);
    nrf_cs_high(dev);

    // Check payload width
    if (payload_len == 0 || payload_len > 32) 
    {
        //RX FIFO corrupted -> flush, return error
        cmd = FLUSH_RX;
        NRF24_SPI_Write(dev, &cmd, 1);

        NRF24_WriteRegister(dev, STATUS, RX_DR, NULL);
        dev->buffers.flag_new_rx = 0;

        dev->radioErr = NRF_ERR_RX_OVERFLOW;
        return 1;
    }

    // Read payload
    nrf_cs_low(dev);
    nrf_spi_txrx(dev, R_RX_PAYLOAD);

    for (uint8_t i = 0; i < payload_len; i++) 
    {
        dev->buffers.RX_FIFO[i] = nrf_spi_txrx(dev, NOP);
    }
    nrf_cs_high(dev);

    // Clear IRQ
    // NRF24_WriteRegister(dev, STATUS, RX_DR, NULL);

    dev->buffers.flag_new_rx = 0;   // Data received no new data
    dev->buffers.rx_lenght = payload_len;   // save lenght


    return 0;
}



/*********************************************************************
 * @fn      NRF24_SetTXAddress
 *
 * @param   *dev: device struct
 * @param   *addr: pointer to address that will be set
 *
 * @brief   Set address for RX
 *
 * @return  1: error, 0: OK
 */
uint8_t NRF24_SetTXAddress(s_nRF24L01 *dev, const uint8_t *addr)
{
    // uint8_t addr_len = dev->config->addr_width + 2; // AW encoding: 3每5 bytes (register value + 2)

    // // TX address
    // nrf_cs_low(dev);

    // nrf_spi_txrx(dev, W_REGISTER | TX_ADDR);

    // for (uint8_t i = 0; i < addr_len; i++) 
    // {
    //     nrf_spi_txrx(dev, addr[i]);
    // }

    // nrf_cs_high(dev);

    // // RX_ADDR_P0 must match TX_ADDR for auto-ack
    // nrf_cs_low(dev);

    // nrf_spi_txrx(dev, W_REGISTER | RX_ADDR_P0);

    // for (uint8_t i = 0; i < addr_len; i++) 
    // {
    //     nrf_spi_txrx(dev, addr[i]);
    // }

    // nrf_cs_high(dev);

    // return 0;



    uint8_t addr_len = dev->config->addr_width + 2;

    // Set TX address only
    nrf_cs_low(dev);
    nrf_spi_txrx(dev, W_REGISTER | TX_ADDR);
    for (uint8_t i = 0; i < addr_len; i++) {
        nrf_spi_txrx(dev, addr[i]);
    }
    nrf_cs_high(dev);

    // ONLY match RX_ADDR_P0 if this device expects ACKs
    if (dev->config->auto_ack) {
        nrf_cs_low(dev);
        nrf_spi_txrx(dev, W_REGISTER | RX_ADDR_P0);
        for (uint8_t i = 0; i < addr_len; i++) {
            nrf_spi_txrx(dev, addr[i]);
        }
        nrf_cs_high(dev);
    }

    return 0;
}



/*********************************************************************
 * @fn      NRF24_SetRXAddress
 *
 * @param   *dev: device struct
 * @param   pipe: pipe number 0-5
 * @param   *addr: pointer to address that will be set
 *              - Pipe 0 & 1: full address
 *              - Pipe 2每5: only LSB is written (one byte)
 *
 * @brief   Set address for each pipe
 *
 * @return  1: error, 0: OK
 */
 uint8_t NRF24_SetRXAddress(s_nRF24L01 *dev, uint8_t pipe, const uint8_t *addr)
{
    uint8_t addr_len = dev->config->addr_width + 2; // AW encoding: 3每5 bytes (register value + 2)

    if (pipe > 5) 
    {
        return 1;
    }

    nrf_cs_low(dev);

    nrf_spi_txrx(dev, W_REGISTER | (RX_ADDR_P0 + pipe));

    if (pipe <= 1) 
    {   
        // Pipe 0 and 1
        for (uint8_t i = 0; i < addr_len; i++) 
        {
            nrf_spi_txrx(dev, addr[i]);
        }
    } 
    else 
    {
        // Pipes 2每5
        nrf_spi_txrx(dev, addr[0]);
    }

    nrf_cs_high(dev);

    return 0;
}



/*********************************************************************
 * @fn      NRF24_GetLinkQuality - Experimental
 *
 * @param   *dev: device struct
 *
 * @brief   Get estimated quality of connection
 *
 * @return  | ARC_CNT | Link quality |
 *          | ------- | ------------ |
 *          | 0每1     | Excellent    |
 *          | 2每4     | Good         |
 *          | 5每8     | Marginal     |
 *          | 9每15    | Bad          |
 */
uint8_t NRF24_GetLinkQuality(s_nRF24L01 *dev)
{
    uint8_t observe;
    NRF24_ReadRegister(dev, OBSERVE_TX, &observe, NULL);
    return observe & ARC_CNT;
}



uint8_t NRF24_GetRxPipe(s_nRF24L01 *dev)
{
    uint8_t status;
    uint8_t pipe;

    NRF24_ReadStatus(dev, &status);

    pipe = (status & RX_P_NO) >> 1;

    if (pipe == RX_P_NO_EMPTY) {
        return 0xFF; // no valid pipe
    }

    return pipe;
}
