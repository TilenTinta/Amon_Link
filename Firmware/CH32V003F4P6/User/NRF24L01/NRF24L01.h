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

/*  User must define pinouts based on its wiring.
 *   Pins that are required for one radio are:
 *       - MISO - SPI
 *       - MOSI - SPI
 *       - SCK - SPI
 *       - CS - chip select - SPI
 *       - CE - chip enable
 *       - IRQ - interrupt pin (pulled low when interrupt happens)
 *   SPI peripheral configuration:
 *       - 0 - 8Mbps 4-wire SPI serial interface
 *       - 8 bit command set
 */

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


// Register map [Chapter 9.1, Table 24]
#define CONFIG                  0x00    // RX/TX control: PRIM_RX, No. of CRC bit: CRCO
#define EN_AA                   0x01    // Auto Acknowledgement feature (NO_ACK must be off)
#define EN_RXADDR               0x02    // Enable multiple data pipes
#define SETUP_AW                0x03    // Set device address
#define SETUP_RETR              0x04    // Set a number of retransmits if the function is enabled, set timer of retransmit (250us increment)
#define RF_CH                   0x05    // RF channel frequency setup: 2Mbps must use more than 2MHz between channels
#define RF_SETUP                0x06    // Air data rate setup: RF_DR (1 or 2 Mbps -> 125 / 265kB/s), F0 = 2400 + RF_CH [MHz]; PA control - output power: RF_PWR (2 bit); LNA gain: LNA_HCURR
#define STATUS                  0x07    // The "main" register shifted out every time (IRQ trigger sources)
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



// CONFIG - Configuration Register, register bits [Chapter 9.1, Table 24]
#define MASK_RX_DR              (1 << 6)    // bit 6 in CONFIG register - Mask interrupt caused by RX_DR
#define MASK_TX_DS              (1 << 5)    // bit 5 in CONFIG register - Mask interrupt caused by TX_DS
#define MASK_MAX_RT             (1 << 4)    // bit 4 in CONFIG register - Mask interrupt caused by MAX_RT
#define EN_CRC                  (1 << 3)    // bit 3 in CONFIG register - Enable CRC
#define CRCO                    (1 << 2)    // bit 2 in CONFIG register - CRC encoding scheme
#define PWR_UP                  (1 << 1)    // bit 1 in CONFIG register - 1: POWER UP, 0:POWER DOWN
#define PRIM_RX                 (1 << 0)    // bit 0 in CONFIG register - RX/TX control

// EN_AA - Enable ¡®Auto Acknowledgment¡¯ Function, register bits [Chapter 9.1, Table 24]
#define ENAA_P5                 (1 << 5)    // bit 5 in EN_AA register - Enable auto acknowledgement data pipe 5
#define ENAA_P4                 (1 << 4)    // bit 4 in EN_AA register - Enable auto acknowledgement data pipe 4
#define ENAA_P3                 (1 << 3)    // bit 3 in EN_AA register - Enable auto acknowledgement data pipe 3
#define ENAA_P2                 (1 << 2)    // bit 2 in EN_AA register - Enable auto acknowledgement data pipe 2
#define ENAA_P1                 (1 << 1)    // bit 1 in EN_AA register - Enable auto acknowledgement data pipe 1
#define ENAA_P0                 (1 << 0)    // bit 0 in EN_AA register - Enable auto acknowledgement data pipe 0

// EN_RXADDR - Enabled RX Addresses, register bits [Chapter 9.1, Table 24]
#define ERX_P5                  (1 << 5)    // bit 5 in EN_RXADDR register - Enable data pipe 5
#define ERX_P4                  (1 << 4)    // bit 4 in EN_RXADDR register - Enable data pipe 4
#define ERX_P3                  (1 << 3)    // bit 3 in EN_RXADDR register - Enable data pipe 3
#define ERX_P2                  (1 << 2)    // bit 2 in EN_RXADDR register - Enable data pipe 2
#define ERX_P1                  (1 << 1)    // bit 1 in EN_RXADDR register - Enable data pipe 1
#define ERX_P0                  (1 << 0)    // bit 0 in EN_RXADDR register - Enable data pipe 0

// SETUP_AW - Setup of Address Widths, register bits [Chapter 9.1, Table 24]
#define AW                       0x03       // bit 1:0 in SETUP_AW register - RX/TX Address field width

#define AW_3BYTE                 0x01       // bit 1:0 in SETUP_AW register - - reg. value 01 - 3 bytes 
#define AW_4BYTE                 0x02       // bit 1:0 in SETUP_AW register - - reg. value 10 - 4 bytes 
#define AW_5BYTE                 0x03       // bit 1:0 in SETUP_AW register - - reg. value 11 - 5 bytes   

// SETUP_RETR - Setup of Automatic Retransmission, register bits [Chapter 9.1, Table 24]
#define ARD                      0xF0       // bit 7:4 in SETUP_RETR register - Auto Retransmit Delay
#define ARC                      0x0F       // bit 3:0 in SETUP_RETR register - Auto Retransmit Count

#define ARD_250us                0x00       // bit 7:4 in SETUP_RETR register - Wait 250¦Ìs
#define ARD_500us                0x10       // bit 7:4 in SETUP_RETR register - Wait 500us
#define ARD_750us                0x20       // bit 7:4 in SETUP_RETR register - Wait 750us
#define ARD_1000us               0x30       // bit 7:4 in SETUP_RETR register - Wait 1000¦Ìs
#define ARD_1250us               0x40       // bit 7:4 in SETUP_RETR register - Wait 1250¦Ìs
#define ARD_1500us               0x50       // bit 7:4 in SETUP_RETR register - Wait 1500¦Ìs
#define ARD_1750us               0x60       // bit 7:4 in SETUP_RETR register - Wait 1750¦Ìs
#define ARD_2000us               0x70       // bit 7:4 in SETUP_RETR register - Wait 2000¦Ìs
#define ARD_2250us               0x80       // bit 7:4 in SETUP_RETR register - Wait 2250¦Ìs
#define ARD_2500us               0x90       // bit 7:4 in SETUP_RETR register - Wait 2500¦Ìs
#define ARD_2750us               0xA0       // bit 7:4 in SETUP_RETR register - Wait 2750¦Ìs
#define ARD_3000us               0xB0       // bit 7:4 in SETUP_RETR register - Wait 3000¦Ìs
#define ARD_3250us               0xC0       // bit 7:4 in SETUP_RETR register - Wait 3250¦Ìs
#define ARD_3500us               0xD0       // bit 7:4 in SETUP_RETR register - Wait 3500¦Ìs
#define ARD_3750us               0xE0       // bit 7:4 in SETUP_RETR register - Wait 3750¦Ìs
#define ARD_4000us               0xF0       // bit 7:4 in SETUP_RETR register - Wait 4000¦Ìs

#define ARC_OFF                  0x00       // bit 7:4 in SETUP_RETR register - retransmit: off
#define ARC1                     0x10       // bit 7:4 in SETUP_RETR register - retransmit: 1x
#define ARC2                     0x20       // bit 7:4 in SETUP_RETR register - retransmit: 2x
#define ARC3                     0x30       // bit 7:4 in SETUP_RETR register - retransmit: 3x
#define ARC4                     0x40       // bit 7:4 in SETUP_RETR register - retransmit: 4x
#define ARC5                     0x50       // bit 7:4 in SETUP_RETR register - retransmit: 5x
#define ARC6                     0x60       // bit 7:4 in SETUP_RETR register - retransmit: 6x
#define ARC7                     0x70       // bit 7:4 in SETUP_RETR register - retransmit: 7x
#define ARC8                     0x80       // bit 7:4 in SETUP_RETR register - retransmit: 8x
#define ARC9                     0x90       // bit 7:4 in SETUP_RETR register - retransmit: 9x
#define ARC10                    0xA0       // bit 7:4 in SETUP_RETR register - retransmit: 10x
#define ARC11                    0xB0       // bit 7:4 in SETUP_RETR register - retransmit: 11x
#define ARC12                    0xC0       // bit 7:4 in SETUP_RETR register - retransmit: 12x
#define ARC13                    0xD0       // bit 7:4 in SETUP_RETR register - retransmit: 13x
#define ARC14                    0xE0       // bit 7:4 in SETUP_RETR register - retransmit: 14x
#define ARC15                    0xF0       // bit 7:4 in SETUP_RETR register - retransmit: 15x

// RF_CH - RF Channel, register bits [Chapter 9.1, Table 24]
#define RF_CH_REG                0x7f       // bit 6:0 in RF_CH register - Sets the frequency channel

// RF_SETUP - RF Setup Register, register bits [Chapter 9.1, Table 24]
#define PLL_LOCK                (1 << 4)    // bit 4 in RF_SETUP register - Force PLL lock signal. Only used in test
#define RF_DR                   (1 << 3)    // bit 3 in RF_SETUP register - Air Data Rate
#define RF_PWR                  (0x03 << 1) // bit 2:1 in RF_SETUP register - Set RF output power in TX mode

#define RF_DR_1MBPS             0x00        // bit 3 in RF_SETUP register - 1Mbps
#define RF_DR_2MBPS             0x01        // bit 3 in RF_SETUP register - 2Mbps
#define RF_PWR_MIN18DBM         0x00        // bit 2:1 in RF_SETUP register - -18dBm
#define RF_PWR_MIN12DBM         0x01        // bit 2:1 in RF_SETUP register - -12dBm
#define RF_PWR_MIN6DBM          0x02        // bit 2:1 in RF_SETUP register - -6dBm
#define RF_PWR_0DBM             0x03        // bit 2:1 in RF_SETUP register - -0dBm

// STATUS - Status Register, register bits [Chapter 9.1, Table 24]
#define RX_DR                   (1 << 6)    // bit 6 in STATUS register - Data Ready RX FIFO interrupt
#define TX_DS                   (1 << 5)    // bit 5 in STATUS register - Data Sent TX FIFO interrupt
#define MAX_RT                  (1 << 4)    // bit 4 in STATUS register - Maximum number of TX retransmits interrupt
#define RX_P_NO                 (0x07 << 1) // bit 3:1 in STATUS register - Data pipe number for the payload available for reading from RX_FIFO
#define TX_FULL                 (1 << 0)    // bit 0 in STATUS register - TX FIFO full flag

#define RX_P_NO_EMPTY           0x07        // bit 3:1 in STATUS register - reg. value 111
#define RX_P_NO_UNUSED          0x06        // bit 3:1 in STATUS register - reg. value 110
#define RX_P_NO_PIPE0           0x00        // bit 3:1 in STATUS register - reg. value 000
#define RX_P_NO_PIPE1           0x01        // bit 3:1 in STATUS register - reg. value 001
#define RX_P_NO_PIPE2           0x02        // bit 3:1 in STATUS register - reg. value 010
#define RX_P_NO_PIPE3           0x03        // bit 3:1 in STATUS register - reg. value 011
#define RX_P_NO_PIPE4           0x04        // bit 3:1 in STATUS register - reg. value 100
#define RX_P_NO_PIPE5           0x05        // bit 3:1 in STATUS register - reg. value 101

// OBSERVE_TX - Transmit observe register, register bits [Chapter 9.1, Table 24]
#define PLOS_CNT                0xF0        // bit 7:4 in OBSERVE_TX register - Counter for lost packets (max. 15), reset with RF_CH
#define ARC_CNT                 0x0F        // bit 3:0 in OBSERVE_TX register - Counter for retransmited packets, reset with new packet

// CD - Carrier Detect, register bits [Chapter 9.1, Table 24]
#define CD_REG                  0x00        // bit 0 in CD register - Carrier Detect

// RX_PW_P0 - Number of bytes in RX payload in data pipe 0, register bits [Chapter 9.1, Table 24]
#define RX_PW_P0_REG            0x3F        // bit 5:0 in RX_PW_P0 register - Number of bytes in RX payload (1-32 bytes)

// RX_PW_P1 - Number of bytes in RX payload in data pipe 1, register bits [Chapter 9.1, Table 24]
#define RX_PW_P1_REG            0x3F        // bit 5:0 in RX_PW_P1 register - Number of bytes in RX payload (1-32 bytes)

// RX_PW_P2 - Number of bytes in RX payload in data pipe 2, register bits [Chapter 9.1, Table 24]
#define RX_PW_P2_REG            0x3F        // bit 5:0 in RX_PW_P2 register - Number of bytes in RX payload (1-32 bytes)

// RX_PW_P3 - Number of bytes in RX payload in data pipe 3, register bits [Chapter 9.1, Table 24]
#define RX_PW_P3_REG            0x3F        // bit 5:0 in RX_PW_P3 register - Number of bytes in RX payload (1-32 bytes)

// RX_PW_P4 - Number of bytes in RX payload in data pipe 4, register bits [Chapter 9.1, Table 24]
#define RX_PW_P4_REG            0x3F        // bit 5:0 in RX_PW_P4 register - Number of bytes in RX payload (1-32 bytes)

// RX_PW_P5 - Number of bytes in RX payload in data pipe 5, register bits [Chapter 9.1, Table 24]
#define RX_PW_P5_REG            0x3F        // bit 5:0 in RX_PW_P5 register - Number of bytes in RX payload (1-32 bytes)

// FIFO_STATUS - FIFO Status Register, register bits [Chapter 9.1, Table 24]
#define TX_REUSE                (1 << 6)    // bit 6 in FIFO_STATUS register - Reuse last transmitted data packet
#define TX_FULL_REG             (1 << 5)    // bit 5 in FIFO_STATUS register - TX FIFO full flag
#define TX_EMPTY                (1 << 4)    // bit 4 in FIFO_STATUS register - TX FIFO empty flag
#define RX_FULL                 (1 << 1)    // bit 1 in FIFO_STATUS register - RX FIFO full flag
#define RX_EMPTY                (1 << 0)    // bit 0 in FIFO_STATUS register - RX FIFO empty flag

// DYNPD - Enable dynamic payload length, register bits [Chapter 9.1, Table 24]
#define DPL_P5                  (1 << 5)    // bit 5 in DYNPD register - Enable dyn. payload length data pipe 5
#define DPL_P4                  (1 << 4)    // bit 4 in DYNPD register - Enable dyn. payload length data pipe 4
#define DPL_P3                  (1 << 3)    // bit 3 in DYNPD register - Enable dyn. payload length data pipe 3
#define DPL_P2                  (1 << 2)    // bit 2 in DYNPD register - Enable dyn. payload length data pipe 2
#define DPL_P1                  (1 << 1)    // bit 1 in DYNPD register - Enable dyn. payload length data pipe 1
#define DPL_P0                  (1 << 0)    // bit 0 in DYNPD register - Enable dyn. payload length data pipe 0

// FEATURE - Feature Register, register bits [Chapter 9.1, Table 24]
#define EN_DPL                  (1 << 2)    // bit 2 in FEATURE register - Enables Dynamic Payload Length
#define EN_ACK_PAY              (1 << 1)    // bit 1 in FEATURE register - Enables Payload with ACK
#define EN_DYN_ACK              (1 << 0)    // bit 0 in FEATURE register - Enables the W_TX_PAYLOAD_NOACK command



/*###########################################################################################################################################################*/
/* Structs & Enums */

// Internal available state machine staes
typedef enum {
    // Operating modes [Chapter 6.1, Figure 3, Table 12]
    NRF_MODE_PWR_ON_RST,                // Undefined -> VDD>=1.9V
    NRF_MODE_PWR_DOWN,                  // Device disabled (enter: PWR_UP bit in CONFIG low)
    NRF_MODE_STANDBY_I,                 // Short start up times, CE low -> device from/to RX/TX returns here (enter: PWR_UP bit in CONFIG high, 1,5ms delay)
    NRF_MODE_STANDBY_II,                // When CE high on PTX when TX FIFO empty, new packet to FIFO starts transmit
    NRF_MODE_RX_MODE,                   // When receiving (enter: PWR_UP bit in CONFIG high, PRIM_RX bit high, CE pin high), data in RX FIFO, if FIFO full data discarted, CD high when signal detected
    NRF_MODE_TX_MODE                    // When transmiting (enter: PWR_UP bit in CONFIG high, PRIM_RX bit low, CE pin HIGH (>10us), data in TX FIFO, if CD pin low -> Standby-I, CE high and data in FIFO -> TX mode (max 4ms), CE high and no data in FIFO -> Standby-II 
} e_op_modes;


// Device available datarates
typedef enum {
    NRF_DATARATE_1MBPS,
    NRF_DATARATE_2MBPS
} e_nrf_datarate_t;


// Device available powers
typedef enum {
    NRF_POWER_0DBM,
    NRF_POWER_MINUS_6DBM,
    NRF_POWER_MINUS_12DBM,
    NRF_POWER_MINUS_18DBM
} e_nrf_power_t;


// Possible error 
typedef enum {
    NRF_ERR_NONE,
    NRF_ERR_BOOT,
    NRF_ERR_MAX_RT,
    NRF_ERR_RX_OVERFLOW,
    NRF_ERR_INVALID_STATE
} e_nrf_error;


// Role of each radio
typedef enum {
    NRF_ROLE_PTX,   // primary transmitter
    NRF_ROLE_PRX    // primary receiver
} e_nrf_role;


// Radio ID (if used more than one)
typedef enum {
    NRF_ID_1,       // Radio 1
    NRF_ID_2,       // Radio 2
    NRF_ID_3,       // Radio 3
    NRF_ID_4,       // Radio 4   
} e_nrf_radion_id;

// Connection quality marks
typedef enum {
    LINK_EXCELLENT,
    LINK_GOOD,
    LINK_WEAK,
    LINK_BAD
} e_nrf_conn_quality;

// Device substruct for configuration - constant values that usualy not change when operating
typedef struct {
    uint8_t             channel;        // Used channel
    uint8_t             addr_width;     // Lenght of address in bytes
    uint8_t             auto_ack;       // Enable auto acknowledgement
    uint8_t             dynamic_payload;// Enable dynamic payload functionality
    uint8_t             retries;        // Number of retries
    uint16_t            retry_delay;    // Delay between retrie
    e_nrf_datarate_t    datarate;       // Device datarate
    e_nrf_power_t       power;          // Transmit power

} s_nrf_config;

// Device substruct for pipe addresses
typedef struct {
    uint8_t             tx_addr[5];     // Address of TX - destination address
    uint8_t             pipe0_rx_addr[5]; // Address of RX - address used for ACK
    uint8_t             pipe1_rx_addr[5]; // Address of RX
    uint8_t             pipe2_rx_addr; // Address of RX - address used for ACK - only last byte
    uint8_t             pipe3_rx_addr; // Address of RX - address used for ACK - only last byte
    uint8_t             pipe4_rx_addr; // Address of RX - address used for ACK - only last byte
    uint8_t             pipe5_rx_addr; // Address of RX - address used for ACK - only last byte
} s_pipe_addr;

// Databuffers for transmiting and receiving data
typedef struct {
    uint8_t             TX_FIFO[32];    // TX buffer
    uint8_t             tx_lenght;      // lenght of tx data in buffer 
    uint8_t             RX_FIFO[32];    // RX buffer
    uint8_t             rx_lenght;      // lenght of rx data in buffer 
    uint8_t             pipe_data;      // Number of pipe from where data was received

    uint8_t             flag_new_rx;    // New RX data available
    uint8_t             flag_tx_done;   // Data transmited sucessfuly
    uint8_t             flag_max_rxs_reached; // Maximum nuber of retransmitions reached

} s_nrf_payloads;


// Main devide struct (pinout, configurations, etc.)
typedef struct {
    SPI_TypeDef         *SPIx;          // SPI instance
    GPIO_TypeDef        *CS_Port;       // CSN port
    uint16_t            CS_Pin;         // CSN pin
    GPIO_TypeDef        *CE_Port;       // CE port
    uint16_t            CE_Pin;         // CE pin

    e_nrf_error         radioErr;       // Error flag - block device 
    uint8_t             irq_flag;       // IRQ flag - interrupt detected
    uint8_t             irq_on_pipe;    // number of pipe where irq is detected
    uint8_t             irq_status;     // Flag that indicates interrupt

    e_nrf_radion_id     id;             // Radio ID
    e_nrf_role          role;           // Set radio as receiver/transmiter
    const s_nrf_config  *config;        // Device config
    const s_pipe_addr   *address;       // Address for all pipes and end device
    e_op_modes          op_modes;       // Current operation mode / internal SS state
    e_nrf_conn_quality  strength;       // Quality of connection
    s_nrf_payloads      buffers;        // Buffers for RX and TX data

} s_nRF24L01;




/*###########################################################################################################################################################*/
/* Functions */

// Attach pins to each radio
void NRF24_pin_config(s_nRF24L01 *dev, SPI_TypeDef *SPIx, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin);
uint8_t NRF24_SPI_Write(s_nRF24L01 *dev, const uint8_t *tx, uint8_t len);                       // write data
uint8_t NRF24_SPI_Read(s_nRF24L01 *dev, uint8_t *rx, uint8_t len, uint8_t fill_byte);           // read data
uint8_t NRF24_SPI_Transceive(s_nRF24L01 *dev, const uint8_t *tx, uint8_t *rx, uint8_t len);     // write-read data

// Register helpers
uint8_t NRF24_ReadRegister(s_nRF24L01 *dev, uint8_t reg, uint8_t *value, uint8_t *status_out);  // Read register
uint8_t NRF24_WriteRegister(s_nRF24L01 *dev, uint8_t reg, uint8_t value, uint8_t *status_out);  // Write register
uint8_t NRF24_ReadStatus(s_nRF24L01 *dev, uint8_t *status_out);                                 // Alive check

// API functions
void NRF24_HandleIRQ(s_nRF24L01 *dev);                                                          // Interrupt handler
void NRF24_init(s_nRF24L01 *dev);                                                               // Device initialization functio
uint8_t NRF24_Send(s_nRF24L01 *dev);                                                            // Send data over RF
uint8_t NRF24_ReadRXPayload(s_nRF24L01 *dev);                                                   // Read received data from device
uint8_t NRF24_SetTXAddress(s_nRF24L01 *dev, const uint8_t *addr);                               // Set address of TX pipe
uint8_t NRF24_SetRXAddress(s_nRF24L01 *dev, uint8_t pipe, const uint8_t *addr);                 // Set address of RX pipe
uint8_t NRF24_GetLinkQuality(s_nRF24L01 *dev);                                                  // Connection quality
// TODO: Broadcast

#endif /* USER_NRF24L01_NRF24L01_H_*/
