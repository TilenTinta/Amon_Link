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
Buffers buffers;


/* Global Variable */
uint8_t flag_USB_RX_end = 0;        // Flag that indicates the end of string received from USB


int main(void)
{
    // Device init
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    USART_Printf_Init(115200);                  // Enable USART

    device.device_id = DBGMCU_GetCHIPID();      // Get unic device ID
    device.state = STATE_INIT;                  // Start state

    char startDevInfo[13] = {"Test dev id: "};
    UartSendBuffer(startDevInfo, sizeof(startDevInfo));
    UartSendBuffer((uint8_t*)&device.device_id, sizeof(device.device_id));

    USART1_Init();
    LinkPinout_Init();


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

    GPIO_WriteBit(GPIOD, LED_RED, Bit_RESET);
    Delay_Ms(100);
    GPIO_WriteBit(GPIOD, LED_RED, Bit_SET);
    Delay_Ms(100);

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
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    // Set pin USART TX-->D.5
    GPIO_InitStructure.GPIO_Pin = USART_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Set pin: USART RX-->D.6
    GPIO_InitStructure.GPIO_Pin = USART_RX;
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

    // USART1 IRQ
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}


/*********************************************************************
 * @fcn     GPIO_Init
 *
 * @brief   Initialises the GPIOs
 *
 * @return  none
 */
void LinkPinout_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Set RED LED
    GPIO_InitStructure.GPIO_Pin = LED_RED;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Set BLUE LED
    GPIO_InitStructure.GPIO_Pin = LED_BLUE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);



}



/*********************************************************************
 * @fcn     UartSendBuffer
 *
 * @brief   Send data over USART - USB comunication
 *
 * @return  none
 */
void UartSendBuffer(uint8_t* buffer, uint16_t length)
{

    for(uint16_t cnt =0; cnt < length; cnt++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) // waiting for sending finish
        {
            // Wait , do nothing
        }
        USART_SendData(USART1, buffer[cnt]);
    }
}


/*********************************************************************
 * @fcn     USART1_IRQHandler
 *
 * @brief   Receive data over USART - USB comunication and saves
 *          them to the array in struct. Once the entire string is
 *          received (end marked with \n) the flag is set. This
 *          indicates that data can be decoded.
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // Save received data
        buffers.dataBuffer_USB[buffers.cntBugger_USB] = USART_ReceiveData(USART1);

        // Detect end of data string
        if (USART_ReceiveData(USART1) == '\n' || USART_ReceiveData(USART1) == '\r')
        {
            flag_USB_RX_end = 1;
            buffers.cntBugger_USB = 0;
        }
        else
        {
            flag_USB_RX_end = 0;
            buffers.cntBugger_USB++;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE); // Clear RX flag (otherwise constantly triggered IRQ)
    }
}



