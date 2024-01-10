/*
 * main.h
 *
 * Created: 26/02/2023 22:25:05
 *  Author: Tinta T.
 */ 

#ifndef MAIN_H_
#define MAIN_H_


#ifndef F_CPU
	#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "../inc/LL_Fcn.h"
#include "../inc/nRF24L01.h"
#include "../inc/UART.h"

// Values in () are connections on Arduino Nano
#define LED_Red		PINB0			// Led (D8)
#define LED_Blue	PINB1			// Led (D9)
#define LED_Green	PINB2			// Led (D10)

#define SW_RF_NUM_1	PINC0			// Switch for number of radios (A0)
#define SW_RF_NUM_2	PINC1			// Switch for number of radios (A1)
#define BTN_Pair	PINC2			// Button for pairing (A2)

#define RF_MOSI		PINB3			// SPI - MOSI (D11)
#define RF_MISO		PINB4			// SPI - MISO (D12)
#define RF_SCK		PINB5			// SPI - SCK (D13)
#define RF_CSN1		PIND3			// SPI - CS radio 1 (D3)
#define RF_CSN2		PIND7			// SPI - CS radio 2 (D7)
#define RF_IRQ1		PIND2			// SPI - interrupt radio 1 (D2)
#define RF_IRQ2		PIND5			// SPI - interrupt radio 2 (D5)
#define RF_CE1		PIND4			// SPI - enable 1 (D4)
#define RF_CE2		PIND6			// SPI - enable 2 (D6)


// STATE Machine states value
#define STATE_INIT			0		// Initialization routine
#define STATE_NOT_CONNECTED	1		// Not connected to drone
#define STATE_CONNECTED		2		// Connected to drone
#define STATE_CONNECTING	3		// Connecting
#define STATE_ERROR			4		// Error - device fault (not implemented)

#define CONNECTED			1		// Status of communication 
#define NOT_CONNECTED		0


// Variables for UART
#define USART_BAUD			9600
#define uC_FREQ				16000000UL
#define USART_PRESC			((F_CPU/(USART_BAUD * 16UL)) - 1)

#define USART_MODE			(0<<UMSEL00)
#define USART_PARITY_MODE	(0<<UPM00)
#define USART_STOP_BIT		(0<<USBS0)
#define USART_DATA_BIT		(3<<UCSZ00)

#define RX_IRQ_DONE			(1<<RXCIE0)
#define DATA_EMPTY_REG		(1<<UDRIE0)


// Variables for NRF24L01



// Variables
uint8_t DeviceState = 0;			// State machine variable
uint8_t InitEND = 0;				// Init Time elapsed
uint8_t InitError = 0;				// Error of init routine to trigger error
uint8_t InitTime = 0;				// Time of init has elapsed - Include time value 
uint8_t PairTime = 0;				// Time of pairing has elapsed when 1
uint8_t PairTrig = 0;				// Start pairing process
uint8_t PairStatus = 0;				// Status of connection link <-> drone

uint8_t Data_Count = 0;
volatile uint8_t RecievedDataBuffer;
uint8_t UART_DATA_Recieve[100] = {};


/* Main functions */
uint8_t DeviceInit();
void IRQ_Timer1_Init();
void INT0_interrupt_Init();
void USART_Init(unsigned int presc);
uint8_t USART_Receive();

#endif /* MAIN_H_ */