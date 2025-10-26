/*****************************************************************
 * File Name          : bootloader.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/25
 * Description        : Custom bootloader for updating firmware 
 *                      over UART (USB)
*****************************************************************/

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "ch32v00x.h" 


/*###########################################################################################################################################################*/
/* Defines */

// Device constants
#define BOOT_DATE        "October 2025" // Date of software
#define APP_START_ADDRESS  ((uint32_t)0x08001000u)

// Pinout
#define LED_RED             GPIO_Pin_0  // PC0: Pin of red LED
#define LED_BLUE            GPIO_Pin_0  // PD0: Pin of blue LED
#define BTN_PIN             GPIO_Pin_4  // PD4: Pin of reconnect button

#define USART_RX            GPIO_Pin_6  // PD6: USART RX pin (mistake - pin remaping)
#define USART_TX            GPIO_Pin_5  // PD5: USART TX pin (mistake - pin remaping)


// Driver defines
#define DRIVER_VER              0x01    // Version of current driver

// Entities / IDs (1 byte)
#define ID_PC                   0x01    // Address: PC
#define ID_LINK                 0x10    // Address: Link board

// Signaling (1 byte)
#define HEADER_SHIFT            0x08    // Number of bytes before payload
#define SIG_SOF                 0xAA    // Start-Of-Frame

// Flags (1 byte) (bit on this place is set or not)
#define FLAG_ACK                0x01    // frame is a reply
#define FLAG_ERR                0x02    // processing error; payload carries error code
#define FLAG_STREAM             0x03    // telemetry stream frame
#define FLAG_FRAG               0x04    // fragmented payload; optional future use
#define FLAG_NaN                0xF0    // bit4..7: reserved



/*###########################################################################################################################################################*/
/* Structs */
typedef struct {

    // PC -> Drone and Drone -> PC //
    uint8_t     buffer_UART[64];        // Buffer for saving USB data
    uint8_t     flag_new_uart_rx_data;  // Flag indicating a new data has arrived (packet is not complete)
    uint8_t     flag_new_uart_tx_data;  // Flag indicating a new data is ready to send
    uint8_t     flag_USB_RX_end;        // Flag for new complete USB command (PC -> link) - start decode

} buffers_t;


typedef struct {

    uint8_t SOF;        
    uint8_t CMD;
    uint16_t LEN;
    uint8_t PAYLOAD[256];
    uint16_t CRC16;

} packet_t;

#endif
