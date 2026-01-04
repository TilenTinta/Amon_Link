/*****************************************************************
 * File Name          : NRF24L01.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/12/29
 * Description        : Driver for NRF24L01 radio (definitions and constants)
 * Documents          : nRF24L01_prodict_specification_v2_0-9199.pdf
*****************************************************************/

#ifndef USER_NRF24L01_NRF24L01_H_
#define USER_NRF24L01_NRF24L01_H_

/* Include */
#include "debug.h"
#include <stdio.h>
#include <string.h>


/*###########################################################################################################################################################*/
/* Defines */

// Operating modes [Chapter 6.1, Figure 3, Table 12]
#define NRF_MODE_PWR_ON_RST     0       // Undefined -> VDD>=1.9V
#define NRF_MODE_PWR_DOWN       1       // Device disabled (enter: PWR_UP bit in CONFIG low)
#define NRF_MODE_STANDBY_I      2       // Short start up times, CE low -> device from/to RX/TX returns here (enter: PWR_UP bit in CONFIG high, 1,5ms delay)
#define NRF_MODE_STANDBY_II     3       // When CE high on PTX when TX FIFO empty, new packet to FIFO starts transmit
#define NRF_MODE_RX_MODE        4       // When receiving (enter: PWR_UP bit in CONFIG high, PRIM_RX bit high, CE pin high), data in RX FIFO, if FIFO full data discarted, CD high when signal detected
#define NRF_MODE_TX_MODE        5       // When transmiting (enter: PWR_UP bit in CONFIG high, PRIM_RX bit low, CE pin HIGH (>10us), data in TX FIFO, if CD pin low -> Standby-I, CE high and data in FIFO -> TX mode (max 4ms), CE high and no data in FIFO -> Standby-II  

// Commands [Chapter 8.3, Table 16] (A = register map address)
#define R_REGISTER              0x00    // 000A AAAA - Read register
#define W_REGISTER              0x20    // 001A AAAA - Write register
#define R_RX_PAYLOAD            0x61    // 0110 0001 - Read RX payload
#define W_TX_PAYLOAD            0xA0    // 1010 0000 - Write TX payload 
#define FLUSH_RX                0xE1    // 1110 0001 - Emptys RX FIFO buffer
#define FLUSH_TX                0xE2    // 1110 0010 - Emptys TX FIFO buffer
#define REUSE_TX_PL             0xE3    // 1110 0011 - Reuse last transmited payload
#define ACTIVATE                0x50    // 0101 0000 - toggle: R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK
#define R_RX_PL_WID             0x60    // 0110 0000 - Read RX payload width
#define W_ACK_PAYLOAD           0xA8    // 1010 1PPP - Write ACK payload for transmit - last tree bits can be 000 to 101
#define W_TX_PAYLOAD_NOACK      0xB0    // 1011 000  - Disable AUTOACK - datasheet only three bits???
#define NOP                     0xFF    // 1111 1111 - No operation (*to read status register)

// Register map [Chapter 9, Table 24]
#define CONFIG                  0x00    // RX/TX control: PRIM_RX, No. of CRC bit: CRCO
#define EN_AA                   0x01    // Auto Acknowledgement feature (NO_ACK must be off)
#define EN_RXADDR               0x02    // Enable multiple data pipes
#define SETUP_AW                0x03    // Set device address
#define SETUP_RETR              0x04    // Set a number of retransmits if the function is enabled, set timer of retransmit (250us increment)
#define RF_CH                   0x05    // RF channel frequency setup: 2Mbps must use more than 2MHz between channels
#define RF_SETUP                0x06    // Air data rate setup: RF_DR (1 or 2 Mbps -> 125 / 265kB/s), F0 = 2400 + RF_CH [MHz]; PA control - output power: RF_PWR (2 bit); LNA gain: LNA_HCURR
#define STATUS                  0x07    // The "main" register shifted out every time (IRQ trigger source)
#define OBSERVE_TX              0x08    // two counters for lost packets: ARC_CNT and PLOS_CNT
#define CD                      0x09    // Carrier detect
#define RX_ADDR_P0              0x0A    // Data pipe 0 config
#define RX_ADDR_P1              0x0B    // Data pipe 1 config
#define RX_ADDR_P2              0x0C    // Data pipe 2 config
#define RX_ADDR_P3              0x0D    // Data pipe 3 config
#define RX_ADDR_P4              0x0E    // Data pipe 4 config
#define RX_ADDR_P5              0x0F    // Data pipe 5 config
#define TX_ADDR                 0x10    // Address of device (with AW reg)
#define RX_PW_P0                0x11    // Packet lenght (no DPL)
#define RX_PW_P1                0x12    // Packet lenght (no DPL)
#define RX_PW_P2                0x13    // Packet lenght (no DPL)
#define RX_PW_P3                0x14    // Packet lenght (no DPL)
#define RX_PW_P4                0x15    // Packet lenght (no DPL)
#define RX_PW_P5                0x16    // Packet lenght (no DPL)
#define FIFO_STATUS             0x17    // FIFO registers info
#define DYNPD                   0x1C    // To use DLF set: DPL_P0
#define FEATURE                 0x1D    // DPL - Dynamic Payload Length: EN_DPL, Enable ACK signal: EN_DYN_ACK, Auto Acknowledgement payload: EN_ACK_PAY


/*###########################################################################################################################################################*/
/* Structs */

typedef struct {

    SPI_TypeDef         *SPIx;          // SPI instance
    GPIO_TypeDef        *CS_Port;       // CSN port
    uint16_t            CS_Pin;         // CSN pin
    GPIO_TypeDef        *CE_Port;       // CE port
    uint16_t            CE_Pin;         // CE pin

    uint8_t             radioErr;       // Error flag - block device 

    uint8_t             radio_mode;     // Set radio as receiver/transmiter
    uint8_t             radioID;        // Radio ID
    uint8_t             flag_IRQ;       // Flag that indicates interrupt
    uint8_t             op_modes;       // value of current internal device state
    uint8_t             TX_FIFO[32];    // TX buffer
    uint8_t             RX_FIFO[32];    // RX buffer

} s_nRF24L01;


/*###########################################################################################################################################################*/
/* Functions */

// Attach pins to each radio
void NRF24_pin_config(s_nRF24L01 *dev,
                      SPI_TypeDef *SPIx,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      GPIO_TypeDef *ce_port, uint16_t ce_pin);

uint8_t NRF24_SPI_Write(s_nRF24L01 *dev, const uint8_t *tx, uint8_t len);                     // write data
uint8_t NRF24_SPI_Read(s_nRF24L01 *dev, uint8_t *rx, uint8_t len, uint8_t fill_byte);         // read data
uint8_t NRF24_SPI_Transceive(s_nRF24L01 *dev, const uint8_t *tx, uint8_t *rx, uint8_t len);   // write-read data

// Register helpers
uint8_t NRF24_ReadRegister(s_nRF24L01 *dev, uint8_t reg, uint8_t *value, uint8_t *status_out); // Read register
uint8_t NRF24_WriteRegister(s_nRF24L01 *dev, uint8_t reg, uint8_t value, uint8_t *status_out); // Write register

uint8_t NRF24_ReadStatus(s_nRF24L01 *dev, uint8_t *status_out);   // Alive check


#endif /* USER_NRF24L01_NRF24L01_H_*/
