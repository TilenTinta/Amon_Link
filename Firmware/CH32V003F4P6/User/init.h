/*****************************************************************
 * File Name          : init.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/07/16
 * Description        : Initialization of all peripheral devices
 * Documents          : CH32V003RM.PDF
*****************************************************************/
#ifndef USER_INIT_H_
#define USER_INIT_H_

#include "ch32v00x.h"
#include "main.h"
#include "NRF24L01/NRF24L01.h"



/* Structs */
typedef struct {
    SPI_TypeDef *Instance;              // SPI1
    SPI_InitTypeDef Init;               // built-in SPI struct
} SPI_HandleTypeDef;

/* Functions */

void LinkPinout_Init(void);                                      // Init the pinout
void TIM_INT_Init(void);
void USART1_Init(void);
void USART_DMA_TX_Config(void);
void Start_DMA_USART_TX(uint8_t len);
void SPI1_Init(void);
void SPI_DMA_RX_Config(void);
void Start_DMA_SPI_RX(void);
void UartSendBuffer(uint8_t* buffer, uint16_t length);
void USART1_IRQHandler(void);
void EXTI7_0_IRQHandler(void);








#endif /* USER_INIT_H_ */