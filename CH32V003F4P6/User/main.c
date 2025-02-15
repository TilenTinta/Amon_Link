/*****************************************************************
 * File Name          : main.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/02/11
 * Description        : Main file of Amon Link
*****************************************************************/

#include "debug.h"
#include "main.h"


/* Global define */
DEVICE device;


/* Global Variable */



int main(void)
{
    // Device init
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    USART_Printf_Init(115200);

    device.device_id = DBGMCU_GetCHIPID();
    device.state = STATE_INIT; // Start state
    printf("Test dev id: %d\n\r", device.device_id);

    USART1_Init();
    Pinout_Init();

    GPIO_WriteBit(GPIOD, LED_RED, Bit_RESET);

    while(1)
    {
        switch(device.state)
        {
        case STATE_INIT:

            break;

        case STATE_CONN_START:

            break;

        case STATE_CONN_OK:

            break;

        case STATE_CONN_FAIL:

            break;
        }


    }
}



/*********************************************************************
 * @fcn      USART1_Init
 *
 * @brief   Initializes the USART1 peripheral
 *
 * @return  none
 */
void USART1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    // Set pin TX-->D.5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Set pin: RX-->D.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // USART1 setup
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    // Set LED
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);


}


/*********************************************************************
 * @fcn      GPIO_Init
 *
 * @brief   Initializes the GPIOs
 *
 * @return  none
 */
void Pinout_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Set LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
