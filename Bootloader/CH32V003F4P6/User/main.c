/*****************************************************************
 * File Name          : bootloader.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/25
 * Description        : Custom bootloader for updating firmware 
 *                      over UART (USB)
*****************************************************************/

#include "debug.h"
#include "main.h"

/* IRQs*/
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Functions */
static void USART1_Init(void);
static void LinkPinout_Init(void);
static void UartSendBuffer(uint8_t* buffer, uint16_t length);
static void UART_buffer_clear(void);

void jump_to_app(void);


/* Structs */
buffers_t buffers;
typedef void (*app_entry_t)(void);

int main(void)
{
    // Init //
    SystemCoreClockUpdate(); 
    LinkPinout_Init();
    USART1_Init();

    // Check if button is pressed
    if (GPIO_ReadInputDataBit(GPIOD, BTN_PIN) == Bit_SET) 
    {
        // Turn on RED LED
        GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);
        while(1){};
    }
    else
    {
        GPIO_WriteBit(GPIOC, LED_RED, Bit_RESET);
        jump_to_app();
    }
    return 0;
}



/*###########################################################################################################################################################*/
/* Functions */


/*********************************************************************
 * @fcn     GPIO_Init
 *
 * @brief   Initialises the GPIOs
 *
 * @return  none
 */
static void LinkPinout_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

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

    // Set Button pin - pin & interrupt
    GPIO_InitStructure.GPIO_Pin = BTN_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}


/*********************************************************************
 * @fcn      USART1_Init
 *
 * @brief   Initializes the USART1 peripheral
 *
 * @return  none
 */
static void USART1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    // Pin remaping (TX=PD6, RX=PD5)
    AFIO->PCFR1 &= ~((1U << 21) | (1U << 2));  // clear USART1_RM1 (bit21) and USART1_RM (bit2)
    AFIO->PCFR1 |=  (1U << 21);                // set USART1_RM1=1, USART1_RM=0  -> 10b

    // Set pin USART TX
    GPIO_InitStructure.GPIO_Pin = USART_RX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Set pin: USART RX
    GPIO_InitStructure.GPIO_Pin = USART_TX;
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

    // USART1 IRQ - RX
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

}


/*********************************************************************
 * @fcn     UartSendBuffer
 *
 * @brief   Send data over USART - USB comunication (parameters, setup...)
 *
 * @return  none
 */
static void UartSendBuffer(uint8_t* buffer, uint16_t length)
{
    DMA_Cmd(DMA1_Channel4, DISABLE);                                            // Disable DMA

    for(uint16_t cnt = 0; cnt < length; cnt++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)             // Waiting for sending finish
        {
            // Wait, do nothing
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
 *          Used only for parameters, setup...
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        static uint8_t len_new_rx_data = 0;
        static uint8_t cntBuffer_UART = 0;

        // Save received data
        uint8_t data = USART_ReceiveData(USART1);                               // Read only once
        buffers.buffer_UART[cntBuffer_UART] = data;
        cntBuffer_UART++;

        // Detect start of frame and set flags
        if (buffers.flag_new_uart_rx_data == 1 && len_new_rx_data == 0)         // Flag for new packet, but no lenght of packet
        {
            len_new_rx_data = data;                                             // Save packet lenght
            buffers.flag_new_uart_rx_data = 0;                                  // Clear flag for new data
        }
        else if (data == SIG_SOF && len_new_rx_data == 0 && len_new_rx_data == 0)  // No flag for new packet and SOA packet
        {
            buffers.flag_new_uart_rx_data = 1;                                  // Indicate new data received
            buffers.flag_USB_RX_end = 0;                                        // Clear end of packet flag
            len_new_rx_data = 0;                                                // Clear packet counter
        }
        else if (cntBuffer_UART >= (len_new_rx_data + 2))                       // Detect end of complete packet
        {
            buffers.flag_USB_RX_end = 1;                                        // Indicate end of packet
            cntBuffer_UART = 0;                                                 // Clear UART counter
        }

        // Clear RX flag (otherwise constantly triggered IRQ)
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
    }
}


/*********************************************************************
 * @fn      UART_buffer_clear
 *
 * @brief   Clear data in UART buffer
 *
 * @return  none
 */
static void UART_buffer_clear(void)
{
    // Clear buffer
    for (int i = 0; i < sizeof(buffers.buffer_UART); i++)
    {
        buffers.buffer_UART[i] = 0;
    }
}



/*********************************************************************
 * @fn      jump_to_app
 *
 * @brief   Jump from bootloader to main program
 *
 * @return  none
 */
void jump_to_app(void)
{
    // Get app vector table
    uint32_t *app_vector_table = (uint32_t *)APP_START_ADDRESS;
    uint32_t app_sp = app_vector_table[0];                      // app init stack pointer
    uint32_t app_reset_handler = app_vector_table[1];           // app reset handler address

    // Disable all interrupts
    __disable_irq();

    // Clear any pending interrupts
    NVIC_ClearPendingIRQ(USART1_IRQn);
    
    // Deinit peripherals
    USART_DeInit(USART1);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOC);
    
    // Disable peripheral clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, DISABLE);

    // Set app stack pointer
    __asm volatile ("mv sp, %0" : : "r" (app_sp));

    // Jump to app reset handler
    app_entry_t app_start = (app_entry_t)app_reset_handler;
    app_start();

    // If something goes wrong stop here
    while (1) { }
}

// void jump_to_app(void)
// {
//     __disable_irq(); // disable interrupts
//     NVIC_ClearPendingIRQ(USART_IT_RXNE);

//     USART_DeInit(USART1);
//     GPIO_DeInit(GPIOD);
//     GPIO_DeInit(GPIOC);

//     // Jump
//     app_entry_t app_reset = (app_entry_t)(APP_START_ADDRESS);
//     app_reset();

//     // optional infinite loop
//     while (1) { }
// }
