/*
 * USART.h
 *
 * Created: 25/03/2023 22:29:21
 *  Author: titit
 */ 


#ifndef USART_H_
#define USART_H_

#include <avr/interrupt.h>
#include <avr/io.h>


// Defines for communication codes
#define CODE_ID_NULL			00		// reserved
#define CODE_ID_PING			10		// Ping device to test connection
#define CODE_ID_ERROR			20		// Error report
#define CODE_ID_PAIR_STAT		30		// Pair status link <-> drone
#define CODE_ID_PAIR			31		// Start pairing
#define CODE_ID_PARASTAT		40		// Amon configure parameters
#define CODE_ID_AMON_ERROR		12		// errors on craft
#define CODE_ID_AMON_PARAMETERS 13		// list current parameters
#define CODE_ID_AMON_RESPONSE	14		// Code of response packet
#define CODE_ID_AMON_ARM		20		// Arm craft before flight
#define CODE_ID_AMON_FLY_DATA	21		// Real time fly data
#define CODE_ID_AMON_LAND		30		// Status of craft after flight


// Functions
void USART_RX_DATA_Decode(uint8_t *data);
void USART_DATA_Transmit(uint8_t *data);

#endif /* USART_H_ */