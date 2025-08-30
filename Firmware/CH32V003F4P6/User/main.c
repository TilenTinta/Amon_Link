/*****************************************************************
 * File Name          : main.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/02/11
 * Description        : Main file of Amon Link
*****************************************************************/

/* Includes */
#include "debug.h"
#include "main.h"


/* Interrupts defines */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


/* Global define */
DEVICE device;                                              // Device variables struct
BUFFERS buffers;                                            // Device buffers
nRF24L01 radio1;                                            // Radio variables 1
nRF24L01 radio2;                                            // Radio variables 2
SPI_HandleTypeDef hspi1;                                    // SPI handle


int main(void)
{
    // Device init //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);         // Init NVIC
    SystemCoreClockUpdate();                                // Init all the clocks
    Delay_Init();                                           // Enable delay
    LinkPinout_Init();                                      // Init the pinout
    TIM_INT_Init();                                         // Init the timer
    USART1_Init();                                          // Init USART
    USART_Printf_Init(115200);                              // Enable USART
    USART_DMA_TX_Config();                                  // Configure DMA channel 4 - USART TX (UDP style telemetry data)
    SPI1_Init();                                            // Init SPI
    //SPI_DMA_RX_Config();                                    // Configure DMA channel 2 - SPI RX (UDP style telemetry data)


    // Device start //
    if (device.init_done == 0)
    {
        // Device is alive 
        GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);             
        GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET); 

        GPIO_WriteBit(GPIOC, NRF_CE1, Bit_RESET);
        GPIO_WriteBit(GPIOC, NRF_CE2, Bit_RESET); 
        GPIO_WriteBit(GPIOC, NRF_CS1, Bit_SET);
        GPIO_WriteBit(GPIOD, NRF_CS2, Bit_SET);         

        device.device_id = DBGMCU_GetCHIPID();              // Get unic device ID
        device.state = STATE_INIT;                          // Start device state
        device.conn_status = CONN_STATUS_NOK;               // Connection status
        device.conn_type = STATE_DATA_TRANSMIT;             // Communication type - clasic
        device.init_done = 1;                               // Block init 
    }


    while(1)
    {
        /* MAIN STATE MACHINE */
        switch(device.state)
        {

        // State: Initialize all devices on the PCB
        case STATE_INIT:

            // Short pause for devices to fully power on
            Delay_Ms(1000); 
          
            // Base UART data
            char startDevInfo[] = "----- AMON Link -----\r\n";
            UartSendBuffer((uint8_t*)startDevInfo, strlen(startDevInfo)); 

            char idLabel[] = "Device ID: ";
            UartSendBuffer((uint8_t*)idLabel, strlen(idLabel));
            UartSendBuffer((uint8_t*)&device.device_id, sizeof(device.device_id));

            char newline[] = "\r\n";
            UartSendBuffer((uint8_t*)newline, strlen(newline));

            // Radios initialization and setup
            NRF24_Attach(&radio1, SPI1, GPIOC, NRF_CS1, GPIOC, NRF_CE1);        // Map pins for radio 1
            NRF24_Attach(&radio2, SPI1, GPIOD, NRF_CS2, GPIOC, NRF_CE2);        // Map pins for radio 2

            uint8_t status = 0;
            if (NRF24_ReadStatus(&radio1, &status) == 0) 
            {
                if (status != 0x0E) radio1.radioErr = 1;
            }

            if (NRF24_ReadStatus(&radio2, &status) == 0) 
            {
                if (status != 0x0E) radio2.radioErr = 1;
            }




            // End of initialization
            GPIO_WriteBit(GPIOC, LED_RED, Bit_RESET);
            GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);

            if (radio1.radioErr == 1 || radio2.radioErr == 1)
            {
                device.state = STATE_FAIL;          // Trigger init fail
            } 
            else
            {
                device.state = STATE_CONN_START;    // OK... continue
            }
                                             
            break;



        // State: Start pairing routine to connect device with a drone
        case STATE_CONN_START:

            // TODO: pairing routine


            // If pairing is successfull UNCOMMENT
            if (device.conn_status == CONN_STATUS_OK) 
            {
                GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);
                device.state = STATE_CONN_OK;
            }
            
            // If pairing is NOT successfull -> IRQ timeout trigger 

            break;



        // State: Device and drone are connected, data can be send/received
        case STATE_CONN_OK:

            // Select what type of communication will be used
            switch (device.conn_type) 
            {
                // Clasic data transmition - IRQ, parameters to/from drone
                case STATE_DATA_TRANSMIT:
                // TODO: data manipulation logic

                break;

                // High speed data transmition - UDP style
                case STATE_DATA_STREAM:
                // TODO: Optimize for dma (disable all timer interrupts, SPI optimization)
                //Start_DMA_USART_TX(sizeof(buffers.bufferDMA_RF_UART));
                //UartSendBuffer((uint8_t*)&device.device_id, sizeof(device.device_id));

                break;

                // Undefined - go in error
                default:
                    device.state = STATE_CONN_FAIL;
                break;
            }

            break;



        // State: Pairing routine failed, reconnection must be triggered
        case STATE_CONN_FAIL:

            // Report failed connection
            char statusLabel[] = "Connection FAILED!\r\n";
            UartSendBuffer((uint8_t*)statusLabel, strlen(statusLabel));
            GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);                     // Indicate error
            device.state = STATE_IDLE;                                  // Go in idle mode and wait on user response

            break;



        // State: Idle mode where device just waits on user response
        case STATE_IDLE:
            
            // DO NOTHING... Wait on user

            break;



        // State: Fail state if device / radion fail to initialize
        case STATE_FAIL:
            
            // DO NOTHING... Fast red LED blinks

            break;



        // State: Critical error in software
        default:
            // Problem... - sw error
        break;
        }
    }
}




/*###########################################################################################################################################################*/

/* Initialization */

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
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

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

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Set IRQ1 (Radio interrupt pin 1)
    GPIO_InitStructure.GPIO_Pin = NRF_IRQ1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Set IRQ2 (Radio interrupt pin 2)
    GPIO_InitStructure.GPIO_Pin = NRF_IRQ2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Set CE1 (Chip Enable 1)
    GPIO_InitStructure.GPIO_Pin = NRF_CE1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Set CE2 (Chip Enable 2)
    GPIO_InitStructure.GPIO_Pin = NRF_CE2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Set CS1 (Chip Select 1)
    GPIO_InitStructure.GPIO_Pin = NRF_CS1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Set CS2 (Chip Select 2)
    GPIO_InitStructure.GPIO_Pin = NRF_CS2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}



/*********************************************************************
 * @fcn     TIM_INT_Init
 *
 * @brief   Initialises Timer2: 50Hz / 20ms
 *
 * @return  none
 */
void TIM_INT_Init(void)
{
    // TIM1 - Advanced-control Timer (ADTM)
    // TIM2 - General-purpose Timer (GPTM)

    TIM_TimeBaseInitTypeDef TIMBase_InitStruct = {0};
    NVIC_InitTypeDef NVIC_InitStruct = {0};

    // Set values of registers
    uint16_t arr = 59999;
    uint16_t psc = 15; 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIMBase_InitStruct.TIM_Period = arr;
    TIMBase_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIMBase_InitStruct.TIM_Prescaler = psc;
    TIMBase_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIMBase_InitStruct);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStruct);
    #ifdef DEBUG
        Delay_Ms(50);
    #endif
    TIM_Cmd(TIM2, ENABLE);
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
 * @fcn     USART_DMA_TX_Config
 *
 * @brief   Initializes DMA for USART send data
 *
 * @return  none
 */
void USART_DMA_TX_Config(void) 
{
    DMA_InitTypeDef DMA_InitStructure = {0};                                    // DMA Init struct

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);                          // Enable DMA Clock
    DMA_DeInit(DMA1_Channel4);                                                  // Deinitialize DMA1 channel4 - used by usart

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;        // Set peripheral that will use DMA
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffers.bufferDMA_RF_UART; // Set buffer used to read data
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          // DMA data direction: mem to periph
    DMA_InitStructure.DMA_BufferSize = sizeof(buffers.bufferDMA_RF_UART);       // Size of buffer to send
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // Keep peripheral address
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Enable mem pointer increment
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     // Send byte by byte - periph. side
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             // Send byte by byte - mem. side
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // Single DMA run
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       // Medium priority
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // Not mem to mem transfer

    DMA_Init(DMA1_Channel4, &DMA_InitStructure);                                // Apply all the settings to DMA
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);                              // Enable DMA

}



/*********************************************************************
 * @fcn     Start_DMA_USART_TX
 *
 * @brief   Initializes DMA and transmit data over UART
 *
 * @return  none
 */
void Start_DMA_USART_TX(uint8_t len) 
{
    DMA_Cmd(DMA1_Channel4, DISABLE);                                            // Disable DMA
    DMA_ClearFlag(DMA1_FLAG_TC4);                                               // Clear DMA flag if exist
    DMA1_Channel4->CNTR = len;                                                  // Set transmited data lenght
    DMA1_Channel4->MADDR = (uint32_t)buffers.bufferDMA_RF_UART;                 // Set data buffer address
    DMA_Cmd(DMA1_Channel4, ENABLE);                                             // Re-enable DMA

}



/*********************************************************************
 * @fcn     SPI1_Init
 *
 * @brief   Initializes spi communication
 *
 * @return  none
 */
void SPI1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    //SPI_InitTypeDef   SPI_InitStructure;

    // Enable clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    // Configure SCK and MOSI as AF push-pull
    GPIO_InitStructure.GPIO_Pin   = NRF_SCK | NRF_MOSI;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure MISO as floating input
    GPIO_InitStructure.GPIO_Pin  = NRF_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Fill handle and init struct
    hspi1.Init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;     // Two line SPI
    hspi1.Init.SPI_Mode = SPI_Mode_Master;                          // SPI Mode - Master
    hspi1.Init.SPI_DataSize = SPI_DataSize_8b;                      // Data lenght
    hspi1.Init.SPI_CPOL = SPI_CPOL_Low;                             // Idle low
    hspi1.Init.SPI_CPHA = SPI_CPHA_1Edge;                           // 1st edge
    hspi1.Init.SPI_NSS = SPI_NSS_Soft;                              // Software NSS
    hspi1.Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;    // fPCLK/16
    hspi1.Init.SPI_FirstBit = SPI_FirstBit_MSB;                     // MSB bit first
    //hspi1.SPI_CRCPolynomial = 7;                                  // CRC lenght

    // Apply config and enable
    SPI_Init(SPI1, &hspi1.Init);                                    // Apply SPI configurations
    //SPI_CalculateCRC(SPI_InitStructure.Instance, DISABLE);         
    SPI_Cmd(SPI1, ENABLE);                                          // Enable SPI

}



/*********************************************************************
 * @fcn     SPI_DMA_RX_Config
 *
 * @brief   Initializes DMA for SPI RX
 *
 * @return  none
 */
void SPI_DMA_RX_Config(void) 
{
    DMA_InitTypeDef DMA_InitStructure = {0};                                    // DMA Init struct

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);                          // Enable DMA Clock
    DMA_DeInit(DMA1_Channel2);                                                  // Deinitialize DMA1 channel2 - used by SPI

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DATAR;          // Set periptheral that will use DMA
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffers.bufferDMA_RF_UART; // Set buffer used to read data
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          // DMA data direction: periph to mem 
    DMA_InitStructure.DMA_BufferSize = sizeof(buffers.bufferDMA_RF_UART);       // Size of buffer to send
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // Keep peripheral address
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Enable mem pointer increment
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     // Send byte by byte - periph. side
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             // Send byte by byte - mem. side
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // Single DMA run
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         // High priority
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // Not mem to mem transfer

    DMA_Init(DMA1_Channel2, &DMA_InitStructure);                                // Apply all the settings to DMA
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);                            // Enable DMA

}



/*********************************************************************
 * @fcn     Start_DMA_SPI_RX
 *
 * @brief   Initializes DMA and receive data over SPI
 *
 * @return  none
 */
void Start_DMA_SPI_RX(void) 
{
    DMA_Cmd(DMA1_Channel2, DISABLE);                                            // Disable DMA
    DMA_ClearFlag(DMA1_FLAG_TC2);                                               // Clear DMA flag if exist
    DMA1_Channel2->CNTR = sizeof(buffers.bufferDMA_RF_UART);                    // Set transmited data lenght
    DMA1_Channel2->MADDR = (uint32_t)buffers.bufferDMA_RF_UART;                 // sed data buffer address
    DMA_Cmd(DMA1_Channel2, ENABLE);                                             // Re-enable DMA
}



/*********************************************************************
 * @fcn     UartSendBuffer
 *
 * @brief   Send data over USART - USB comunication (parameters, setup...)
 *
 * @return  none
 */
void UartSendBuffer(uint8_t* buffer, uint16_t length)
{
    DMA_Cmd(DMA1_Channel4, DISABLE);                                 // Disable DMA

    for(uint16_t cnt = 0; cnt < length; cnt++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) // Waiting for sending finish
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
        // Save received data
        uint8_t data = USART_ReceiveData(USART1);           // Read only once
        buffers.buffer_UART[buffers.cntBuffer_UART] = data;
        buffers.cntBuffer_UART++;

        // Detect overflow
        if (buffers.cntBuffer_UART >= sizeof(buffers.cntBuffer_UART))
        {
            buffers.cntBuffer_UART = 0;
        }

        // Indicate new data received
        if (buffers.cntBuffer_UART > 0) buffers.flag_new_rx_data = 1;

        // Clear RX flag (otherwise constantly triggered IRQ)
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
    }
}



/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   External IRQ:
 *          - EXTI_Line4 - button to triger reconnecting
 *          - EXTI_Line3 - Radio 1 IRQ pin
 *          - EXTI_Line1 - Radio 2 IRQ pin
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void)
{
    // External button - reconnect (no debauncing logic)
    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        if (device.state != STATE_CONN_START)
        {
            device.state = STATE_CONN_START;
            GPIO_WriteBit(GPIOC, LED_RED, Bit_RESET);
        }

        EXTI_ClearITPendingBit(EXTI_Line4);     // Clear Flag
    }

    // Radio 1 IRQ pin
    if(EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        if (device.conn_status == CONN_STATUS_OK) radio1.flag_IRQ = 1;
        EXTI_ClearITPendingBit(EXTI_Line3);     // Clear Flag
    }

    // Radio 2 IRQ pin
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        if (device.conn_status == CONN_STATUS_OK) radio2.flag_IRQ = 1;
        EXTI_ClearITPendingBit(EXTI_Line1);     // Clear Flag
    }
}


/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   Timer interrupt - 50Hz/20ms
 *
 * @return  none
 */
void TIM2_IRQHandler(void)
{

    static uint8_t cntBlink = 0;
    static uint16_t cntConnectTimeout = 0;
    static uint8_t cntFailBlink = 0;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {

        if (device.state == STATE_CONN_START)
        {
            // Timer counters
            cntBlink++;
            cntConnectTimeout++;

            // Blink LED (twice per second)
            if (cntBlink >= 12)
            {
                GPIO_WriteBit(GPIOD, LED_BLUE, (BitAction)!GPIO_ReadOutputDataBit(GPIOD, LED_BLUE));
                cntBlink = 0;
            }

            // Signal failed connection (after 10 seconds)
            if (cntConnectTimeout >= 500)
            {
                cntConnectTimeout = 0;
                device.state = STATE_CONN_FAIL;             // failed connection
                device.conn_status = CONN_STATUS_NOK;       // discnnected
                GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);  // Turn off blue LED
            }
        }

        // Reset conter for connection timeout if connection is established
        if (device.conn_status == CONN_STATUS_OK && cntConnectTimeout > 0) cntConnectTimeout = 0;

        // Fail indicator
        if (device.state == STATE_FAIL)
        {
            cntFailBlink++;

            // Fast red LED blink
            if (cntFailBlink >= 4)
            {
                GPIO_WriteBit(GPIOC, LED_RED, (BitAction)!GPIO_ReadOutputDataBit(GPIOC, LED_RED));
                cntFailBlink = 0;
            }
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   // Clear flag            
    }
}



