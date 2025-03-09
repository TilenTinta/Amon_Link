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
#include <string.h>
#include "NRF24L01/NRF24L01.h"          // Library for radios



/* Defines */
// Device constants, pins //
#define DEVICE_NAME    "AMON Link"      // Name of the device
#define SW_VER         "0.0.1"          // Software version
#define SW_DATE        "February 2025"  // Date of software

#define RADIO_NO            1           // Number of radios (1 or 2)

#define CONN_STATUS_OK      1           // Flag that indicates the connection status - connected
#define CONN_STATUS_NOK     0           // Flag that indicates the connection status - disconnected

#define STATE_INIT          0           // State machine value: boot-up (blue light)
#define STATE_CONN_START    1           // State machine value: try to connect (reconnect triggers the same state)
#define STATE_CONN_OK       2           // State machine value: successfully connected (green light on)
#define STATE_CONN_FAIL     3           // State machine value: connect failed (red light blinking, do nothing)

#define LED_RED             GPIO_Pin_0  // Pin of red LED
#define LED_BLUE            GPIO_Pin_0  // Pin of blue LED
#define BTN_PIN             GPIO_Pin_4  // Pin of reconnect button

#define USART_RX            GPIO_Pin_6  // USART RX pin
#define USART_TX            GPIO_Pin_5  // USART TX pin

#define NRF_MOSI            GPIO_Pin_6  // SPI MOSI pin for radios
#define NRF_MISO            GPIO_Pin_7  // SPI MISO pin for radios
#define NRF_SCK             GPIO_Pin_5  // SPI SCK pin for radios
#define NRF_CS1             GPIO_Pin_4  // SPI chip select pin for radio 1
#define NRF_CS2             GPIO_Pin_2  // SPI chip select pin for radio 2
#define NRF_CE1             GPIO_Pin_3  // chip enable pin for radio 1
#define NRF_CE2             GPIO_Pin_2  // chip enable pin for radio 2
#define NRF_IRQ1            GPIO_Pin_3  // interrupt pin for radio 1
#define NRF_IRQ2            GPIO_Pin_1  // interrupt pin for radio 2


// Communication - USB and RF //
// - Byte 0: Address and command
//   [8-4: address]
#define B0_1_NULL           0b0000      // Reserved / null
#define B0_1_PC             0b0001      // PC
#define B0_1_LINK           0b0010      // Link (0011 is still link no. 2)
#define B0_1_DRONE          0b0100      // Drone (all higher numbers can be other drones)

// [4-0: code of the command]
#define B0_0_NULL           0b0000      // Reserved / null
#define B0_0_PING           0b0001      // (Link/Drone) Check connection / status report (ping)
#define B0_0_ERR            0b0010      // (Link/Drone) Error report - it depands on address
#define B0_0_LINK_CONN      0b0100      // (Link) Pair status (link to drone)
#define B0_0_LINK_PAIR      0b0101      // (Link) Start pairing (if not allready paired)
#define B0_0_LINK_PARAM     0b0110      // (Link) Current parameters
#define B0_0_LINK_MOD       0b0111      // (Link) Change parameters on link
#define B0_0_DRONE_PARAM    0b1000      // (Drone) Current parameters
#define B0_0_DRONE_MOD      0b1001      // (Drone) Change parameters on drone
#define B0_0_DRONE_STATUS   0b1010      // (Drone) Change status on drone
#define B0_0_DRONE_TASK     0b1011      // (Drone) Drone commands (calibrate, save to SD...)
#define B0_0_DRONE_TELEM    0b1100      // (Drone) Real time data - telemetry



/* Structs */
typedef struct{
    uint32_t    device_id;              // MCU ID number (identifying the device)
    uint8_t     state;                  // state machine variable
    uint8_t     init_done;              // block the initialisation process
    uint8_t     err_code;               // error code used for error report
    uint16_t    pct_tx_cnt;             // send packets counter
    uint16_t    pct_rx_cnt;             // received packets counter
    uint16_t    pct_fail_cnt;           // counter for failed packets (calculated based on packet number - if next packet num. is not +1 -> pct_fail_cnt++)
}DEVICE;

typedef struct {
    uint8_t dataBuffer_USB[100];        // Buffer for saving USB data
    uint8_t cntBugger_USB;              // Counter for USB buffer
    uint8_t dataBuffer_RF[100];         // Buffer for saving RF data
    uint8_t cntBuffer_RF;               // Counter for RF buffer
}Buffers;



/* Functions */
void USART1_Init(void);
void LinkPinout_Init(void);
void UartSendBuffer(uint8_t* buffer, uint16_t length);
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


#endif /* USER_MAIN_H_ */
