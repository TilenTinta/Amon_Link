/*
 * main.h
 *
 *  Created on: 11 Feb 2025
 *  Author: Tinta T.
 */

#ifndef USER_MAIN_H_
#define USER_MAIN_H_

/* Defines */
#define DEVICE_NAME    "AMON Link"      // Name of the device
#define SW_VER         "0.0.1"          // Software version
#define SW_DATE        "February 2025"  // Date of software

#define NO_RADIO            1           // Number of radios (1 or 2)

#define CONN_STATUS_OK      1           // Flag that indicates the connection status - connected
#define CONN_STATUS_NOK     0           // Flag that indicates the connection status - disconnected

#define STATE_INIT          0           // State machine value: boot-up (blue light)
#define STATE_CONN_START    1           // State machine value: try to connect (reconnect triggers the same state)
#define STATE_CONN_OK       2           // State machine value: successfully connected (green light on)
#define STATE_CONN_FAIL     3           // State machine value: connect failed (red light blinking, do nothing)



#define LED_RED             GPIO_Pin_4  // Pin of red LED
#define LED_BLUE            GPIO_Pin_6  // Pin of blue LED
#define BTN_PIN             GPIO_Pin_7  // Pin of reconnect button

#define NRF_MOSI                        // SPI MOSI pin for radios
#define NRF_MISO                        // SPI MISO pin for radios
#define NRF_SCK                         // SPI SCK pin for radios
#define NRF_CS1                         // SPI chip select pin for radio 1
#define NRF_CS2                         // SPI chip select pin for radio 2
#define NRF_CE1                         // chip enable pin for radio 1
#define NRF_CE2                         // chip enable pin for radio 2
#define NRF_IRQ1                        // interrupt pin for radio 1
#define NRF_IRQ2                        // interrupt pin for radio 2


/* Structs */

typedef struct{
    uint32_t device_id;         // MCU ID number (identifying the device)
    uint8_t state;              // state machine variable
    uint8_t init_done;          // block the initialization process
    uint8_t err_code;           // error code used for error report
    uint16_t pct_tx_cnt;        // send packets counter
    uint16_t pct_rx_cnt;        // received packets counter
    uint16_t pct_fail_cnt;      // counter for failed packets (calculated based on packet number - if next packet num. is not +1 -> pct_fail_cnt++)

}DEVICE;


/* Functions */

void USART1_Init(void);
void Pinout_Init(void);



#endif /* USER_MAIN_H_ */
