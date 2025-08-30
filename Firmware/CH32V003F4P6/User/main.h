/*****************************************************************
 * File Name          : main.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/02/11
 * Description        : Main header file of Amon Link
*****************************************************************/

#ifndef USER_MAIN_H_
#define USER_MAIN_H_

/* Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "NRF24L01/NRF24L01.h"          // Library for radios
#include "DataDecoder/dataDecoder.h"    // Encoders and decoder for data frafic


/*###########################################################################################################################################################*/
/* Defines */

// Device constants
#define DEVICE_NAME    "AMON Link"      // Name of the device
#define SW_VER         "0.0.1"          // Software version
#define SW_DATE        "February 2025"  // Date of software

#define RADIO_NO            2           // Number of radios (1 or 2)

#define STATE_INIT          0           // State machine value: boot-up (blue light)
#define STATE_CONN_START    1           // State machine value: try to connect (reconnect triggers the same state)
#define STATE_CONN_OK       2           // State machine value: successfully connected (green light on)
#define STATE_CONN_FAIL     3           // State machine value: connect failed (red light blinking, do nothing)
#define STATE_IDLE          4           // State machine value: state where device doesnt do anything
#define STATE_FAIL          5           // State machine value: device initialization fail

#define CONN_STATUS_OK      1           // Flag that indicates the connection status - connected
#define CONN_STATUS_NOK     0           // Flag that indicates the connection status - disconnected

#define STATE_DATA_TRANSMIT 0           // State of communication where only commands and response will be transmited - changed with take off / landing
#define STATE_DATA_STREAM   1           // State of communication where drone will stream telemetry and stuf - changed with take off / landing


// Pinout
#define LED_RED             GPIO_Pin_0  // PC0: Pin of red LED
#define LED_BLUE            GPIO_Pin_0  // PD0: Pin of blue LED
#define BTN_PIN             GPIO_Pin_4  // PD4: Pin of reconnect button

#define USART_RX            GPIO_Pin_6  // PD6: USART RX pin (mistake - pin remaping)
#define USART_TX            GPIO_Pin_5  // PD5: USART TX pin (mistake - pin remaping)

#define NRF_MOSI            GPIO_Pin_6  // PC6: SPI MOSI pin for radios
#define NRF_MISO            GPIO_Pin_7  // PC7: SPI MISO pin for radios
#define NRF_SCK             GPIO_Pin_5  // PC5: SPI SCK pin for radios
#define NRF_CS1             GPIO_Pin_4  // PC4: SPI chip select pin for radio 1
#define NRF_CS2             GPIO_Pin_2  // PD2: SPI chip select pin for radio 2
#define NRF_CE1             GPIO_Pin_3  // PC3: chip enable pin for radio 1
#define NRF_CE2             GPIO_Pin_2  // PC2: chip enable pin for radio 2
#define NRF_IRQ1            GPIO_Pin_3  // PD3: interrupt pin for radio 1
#define NRF_IRQ2            GPIO_Pin_1  // PC1: interrupt pin for radio 2


/*###########################################################################################################################################################*/
/* Structs */

typedef struct{
    uint32_t    device_id;              // MCU ID number (identifying the device)
    uint8_t     init_done;              // Block the initialisation process
    uint8_t     state;                  // Main state machine variable
    uint8_t     conn_status;            // Flag that indicates state of connection with drone
    uint8_t     conn_type;              // Type of communication - transmit/stream
    uint8_t     err_code;               // Error code used for error report
    uint16_t    pct_tx_cnt;             // Send packets counter
    uint16_t    pct_rx_cnt;             // Received packets counter
    uint16_t    pct_fail_cnt;           // Counter for failed packets (calculated based on packet number - if next packet num. is not +1 -> pct_fail_cnt++)
} DEVICE;


typedef struct {
    // PC -> Drone and Drone -> PC with IRQs //
    // Used for parameters and mid flyght to send commands to drone
    uint8_t     buffer_UART[64];        // Buffer for saving USB data
    uint8_t     cntBuffer_UART;         // Counter for USB buffer
    uint8_t     flag_new_rx_data;       // Flag indicating a new data has arrived (packet is not complete)
    uint8_t     flag_USB_RX_end;        // Flag for new complete USB command (PC -> link)
    
    uint8_t     buffer_RF[64];          // Buffer for RF data
    uint8_t     cntBuffer_RF;           // Counter for RF buffer

    // Drone -> PC [DMA] //
    // High efficient data transfer
    uint8_t     bufferDMA_RF_UART[3];   // Buffer for saving RF data
} BUFFERS;


typedef struct {
    SPI_TypeDef *Instance;              // SPI1
    SPI_InitTypeDef Init;               // built-in SPI struct
} SPI_HandleTypeDef;



/*###########################################################################################################################################################*/
/* Functions */

void USART1_Init(void);
void SPI1_Init(void);
void LinkPinout_Init(void);
void TIM_INT_Init(void);
void UartSendBuffer(uint8_t* buffer, uint16_t length);
void USART_DMA_TX_Config();
void Start_DMA_USART_TX(uint8_t len);
void SPI_DMA_RX_Config();
void Start_DMA_SPI_RX(void);


#endif /* USER_MAIN_H_ */
