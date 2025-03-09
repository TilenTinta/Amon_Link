/*****************************************************************
 * File Name          : NRF24L01.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/02
 * Description        : Driver for NRF24L01 radio (definitions and constants)
*****************************************************************/

#ifndef USER_NRF24L01_NRF24L01_H_
#define USER_NRF24L01_NRF24L01_H_


// Commands
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define R_TX_PAYLOAD        0xA0
#define FLUSH_RX            0xE1
#define FLUSH_TX            0xE2
#define REUSE_TX_PL         0xE3
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8 // last tree bits can be 000 to 101
#define W_TX_PAYLOAD_NOACK  0xB0 // ?? datasheet only three bits
#define NOP                 0xFF

// Registers
#define CONFIG              0x00
#define EN_AA               0x01
#define EN_RXADDR           0x02
#define SETUP_AW            0x03
#define SETUP_RETR          0x04
#define RF_CH               0x05
#define RF_SETUP            0x06
#define STATUS              0x07
#define OBSERVE_TX          0x08
#define CD                  0x09
#define RX_ADDR_P0          0x0A
#define RX_ADDR_P1          0x0B
#define RX_ADDR_P2          0x0C
#define RX_ADDR_P3          0x0D
#define RX_ADDR_P4          0x0E
#define RX_ADDR_P5          0x0F
#define TX_ADDR             0x10
#define RX_PW_P0            0x11
#define RX_PW_P1            0x12
#define RX_PW_P2            0x13
#define RX_PW_P3            0x14
#define RX_PW_P4            0x15
#define RX_PW_P5            0x16
#define FIFO_STATUS         0x17
#define DYNPD               0x1C
#define FEATURE             0x1D


#endif /* USER_NRF24L01_NRF24L01_H_*/
