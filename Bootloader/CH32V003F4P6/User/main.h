/*****************************************************************
 * File Name          : main.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/25
 * Description        : Custom bootloader for updating firmware 
 *                      over UART (USB)
*****************************************************************/

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "ch32v00x.h" 
#include "ch32v00x_flash.h"


/*###########################################################################################################################################################*/
/* Defines */

// Device constants
#define BOOT_DATE           "October 2025"          // Date of software
#define BOOT_VER            0x01                    // Version of software
#define APP_START_ADDRESS  ((uint32_t)0x00001000u)  // Start address of the main program (jump location)
#define APP_FW_START_ADDR   0x08001000              // Start address for firmware update
#define APP_END_ADDRESS     0x08004000              // End address of flash
#define PAGE_SIZE           64                      // Page size in bytes
#define APP_METADATA_ADDR   APP_END_ADDRESS - PAGE_SIZE // Address in flash where FW data is stored (flags, CRC32) - 64 Bytes


// Pinout
#define LED_RED             GPIO_Pin_0              // PC0: Pin of red LED
#define LED_BLUE            GPIO_Pin_0              // PD0: Pin of blue LED
#define BTN_PIN             GPIO_Pin_4              // PD4: Pin of reconnect button

#define USART_RX            GPIO_Pin_6              // PD6: USART RX pin (mistake - pin remaping)
#define USART_TX            GPIO_Pin_5              // PD5: USART TX pin (mistake - pin remaping)


/* --------------------- UART ---------------------
 * + Relation: PC - Link
 * + Framing: SOF | LEN | ADDRESS | COMMAND | PAYLOAD | CRC16.
 * + Layout (little-endian for multibyte fields):
 *	  - SOF    (1B): 0xAA
 *	  - LEN    (1B): number of bytes in ADDRESS+COMMAND+PAYLOAD+CRC (0..255)
 *	  - COMMAND (1B):
 *	  - PAYLOAD (N bytes)
 *	  - CRC16  (2B): CRC-16/CCITT (poly 0x1021, init 0xFFFF) over LEN..PAYLOAD
 */

// Calculation/program variable
#define HEADER_SHIFT            0x04                // Number of bytes before payload

// Signaling (1 byte)
#define SIG_SOF                 0xAA                // Start-Of-Frame

// IDs/Address (1 byte)
#define ID_PC                   0x01                // Address: PC
#define ID_LINK_BOOT            0x10                // Address: Link board - bootloader
#define ID_LINK_SW              0x11                // Address: Link board - software
#define ID_DRONE                0x20                // Address: Drone (can be multiple)
#define ID_BROADCAST            0xFF                // Address: Broadcast

// Commands/Responses (1 byte)
#define CMD_INFO                0x01                // e.g. bootloader version [CMD_READ]
#define CMD_WRITE               0x03                // Write a chunk: [addr(4B) + data...]
#define CMD_ERASE               0x02                // Erase application area (0x08001000 .. end)
#define CMD_VERIFY              0x04                // Optional: verify CRC of whole app
#define CMD_JUMP_APP            0x05                // Jump to application
#define CMD_END_OF_FW           0x06                // End of firmware update, jump to application
#define CMD_ACK                 0x80                // Response - Replies with info
#define CMD_ERR                 0x81                // Response - Error reply with error code           

// Codes/payload (1 byte)
#define CODE_BAD_CRC            0x01                // calculated CRC and received CRC doesnt match
#define CODE_BOOT_VER           0x02                // Return bootloader version
#define CODE_SW_CRC             0x02                // Return current CRC of main firmware
#define CODE_EXIT_BOOT          0x03                // Exiting bootloader, jumping to application
#define CODE_DATA_WRITEN        0x10                // Succesfuly writen data packet


/*###########################################################################################################################################################*/
/* Structs */
typedef struct {

    // PC -> Link -> PC //
    uint8_t             buffer_UART[128];           // Buffer for saving USB data
    uint8_t             flag_new_uart_rx_data;      // Flag indicating a new data has arrived (packet is not complete)
    volatile uint8_t    flag_USB_RX_end;            // Flag for new complete USB command (PC -> link) - start decode
    uint32_t            flash_address_cnt;          // Address counter of a current flash location

} s_buffers;


typedef struct {

    // uint8_t sof;                                 // Start of frame
    uint8_t plen;                                   // Packet lenght
    uint8_t addr;                                   // Address of targeted device
    uint8_t cmd;                                    // Command
    uint8_t payload[64];                            // Payload data (64bytes is one page)                            
    uint16_t crc16;                                 // CRC calculated over: plen + addr + cmd + payload

} s_packet;


typedef struct {
    
    uint8_t flags;                                  // Flags - go to main app, stay in bootloader
    //uint8_t reserved1;                            // Reserved byte 1 - paddinng/aligment
    //uint8_t reserved2;                            // Reserved byte 2 - paddinng/aligment
    //uint8_t reserved3;                            // Reserved byte 3 - paddinng/aligment
    uint32_t fw_crc32;                              // Current fw crc32 value

} s_meta_data;

#endif
