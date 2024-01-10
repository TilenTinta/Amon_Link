/*
 * USART.c
 *
 * Created: 3/25/2023 10:33:00 PM
 *  Author: Tinta T.
 */ 
#include "../inc/UART.h"

// Variables
volatile uint8_t TransmitDataBuffer;
uint8_t UART_DATA_Buffer_Transmit[100] = {};


/* UART decode and communication */
void USART_RX_DATA_Decode(uint8_t *data)
{
	uint8_t ResponsePacket[100] = {};
	

}

/* UART transmit data */
void USART_DATA_Transmit(uint8_t *data)
{
	for(int i = 0; i<100;i++){
		if ((*(data+i)) == '\0'){
			break;
		}else{
			while ( !( UCSR0A & (1<<UDRE0)) );		
			UDR0 = (*(data+i)) ; // TransmitDataBuffer
		}
	}
}

/* USART Transmit Interrupt *** */
ISR(USART_TX_vect)
{
	// TRANSMIT...
}