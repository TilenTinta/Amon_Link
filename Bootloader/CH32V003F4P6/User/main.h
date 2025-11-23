/*****************************************************************
 * File Name          : bootloader.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/25
 * Description        : Custom bootloader for updating firmware 
 *                      over UART (USB)
*****************************************************************/

// TODO: Build - Optimized for debug!!!!!

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "ch32v00x.h" 
#include "ch32v00x_flash.h"


/*###########################################################################################################################################################*/
/* Defines */

// Device constants
#define BOOT_DATE        "October 2025"             // Date of software
#define SW_VER                  0x01                // Version of software
#define APP_START_ADDRESS  ((uint32_t)0x00001000u)  // Address of start of main program
#define APP_END_ADDRESS     0x00003FFF              // End address of flash
#define PAGE_SIZE           64                      // Page size in bytes

// Pinout
#define LED_RED             GPIO_Pin_0  // PC0: Pin of red LED
#define LED_BLUE            GPIO_Pin_0  // PD0: Pin of blue LED
#define BTN_PIN             GPIO_Pin_4  // PD4: Pin of reconnect button

#define USART_RX            GPIO_Pin_6  // PD6: USART RX pin (mistake - pin remaping)
#define USART_TX            GPIO_Pin_5  // PD5: USART TX pin (mistake - pin remaping)

// IDs (1 byte)
#define ID_PC                   0x01    // Address: PC
#define ID_LINK                 0x10    // Address: Link board

// Signaling (1 byte)
#define HEADER_SHIFT            0x04    // Number of bytes before payload
#define SIG_SOF                 0xAA    // Start-Of-Frame

// Commands (1 byte)
#define CMD_GET_INFO            0x01    // Bootloader version
#define CMD_ERASE_APP           0x02    // Erase application area (0x08001000 .. end)
#define CMD_WRITE               0x03    // Write a chunk: [addr(4B) + data...]
#define CMD_VERIFY              0x04    // Optional: verify CRC of whole app
#define CMD_JUMP_APP            0x05    // Jump to application
#define CMD_ACK                 0x80    // Replies
#define CMD_ERR                 0x81    // Error reply with error code

// Codes (1 byte)
#define CODE_BAD_CRC            0x01    // calculated CRC and received CRC doesnt match
#define CODE_SW_VER             0x02    // Return software version
#define CODE_EXIT_BOOT          0x03    // Exiting bootloader, jumping to application
#define CODE_DATA_WRITEN        0x10    // Succesfuly writen data packet


/*###########################################################################################################################################################*/
/* Structs */
typedef struct {

    // PC -> Drone and Drone -> PC //
    uint8_t     buffer_UART[128];       // Buffer for saving USB data
    uint8_t     flag_new_uart_rx_data;  // Flag indicating a new data has arrived (packet is not complete)
    uint8_t     flag_new_uart_tx_data;  // Flag indicating a new data is ready to send
    uint8_t     flag_USB_RX_end;        // Flag for new complete USB command (PC -> link) - start decode

    uint8_t     flag_update_NOK;        // Flag for indicating end of unsucessful update

} buffers_t;


typedef struct {

    uint8_t sof;                        // Start of frame
    uint8_t plen;                       // Packet lenght
    uint8_t addr;                       // Address of targeted device
    uint8_t cmd;                        // Command
    uint8_t payload[128];               // Payload data
    uint16_t crc16;                     // CRC calculated over: plen + addr + cmd + payload

} packet_t;



#endif
