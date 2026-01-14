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


/*###########################################################################################################################################################*/
/* Interrupts defines */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));



/*###########################################################################################################################################################*/
/* Functions */
void USART1_Init(void);
void SPI1_Init(void);
void LinkPinout_Init(void);
void TIM_INT_Init(void);
void UartSendBuffer(uint8_t* buffer, uint16_t length);
void UART_buffer_clear(void);
void USART_DMA_TX_Config();
void Start_DMA_USART_TX(uint8_t len);
void SPI_DMA_RX_Config();
void Start_DMA_SPI_RX(void);

// Bootloader related functions
void flash_read_boot_flag(s_meta_data *data);
void flash_read_crc32(s_meta_data *data);
void system_reset_trigger(void);
void flash_save_metadata(s_meta_data *data);


/*###########################################################################################################################################################*/
/* Global define */
s_device device;                                            // Device variables struct
s_buffers buffers;                                          // Device buffers
s_packets data_packets;                                     // Packets struct
s_meta_data metadata;                                       // Metadata used for bootloader
s_nRF24L01 radio1;                                          // Radio variables 1
s_nRF24L01 radio2;                                          // Radio variables 2
SPI_HandleTypeDef hspi1;                                    // SPI handle


// Radio 1 configurations (application specific)
static const s_nrf_config radio_tx_cfg = {
    .channel = 40,
    .addr_width = AW_5BYTE,
    .auto_ack = 1,
    .dynamic_payload = 1,
    .retries = 3,
    .retry_delay = ARD_500us,
    .datarate = NRF_DATARATE_1MBPS,
    .power = NRF_POWER_0DBM,  
};

static const s_pipe_addr radio_tx_addr = {
    .tx_addr = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }
};

// Radio 1 configurations (application specific)
static const s_nrf_config radio_rx_cfg = {
    .channel = 40,
    .addr_width = AW_5BYTE,
    .auto_ack = 1,
    .dynamic_payload = 1,
    .retries = 0,
    .retry_delay = 0,
    .datarate = NRF_DATARATE_1MBPS,
    .power = NRF_POWER_MINUS_6DBM,
};

static const s_pipe_addr radio_rx_addr = {
    .pipe0_rx_addr = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }
};



/*############################# ---MAIN--- ######################################################################################################################*/

int main(void)
{
    // Device init //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);         // Init NVIC
    SystemCoreClockUpdate();                                // Init all the clocks
    Delay_Init();                                           // Enable delay
    LinkPinout_Init();                                      // Init the pinout
    TIM_INT_Init();                                         // Init the timer
    USART1_Init();                                          // Init USART
    //USART_Printf_Init(115200);                              // Enable USART
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
        flash_read_boot_flag(&metadata);                    // Read boot flag value
        flash_read_crc32(&metadata);                        // Read current firmware CRC32
        if (metadata.flags == 1)                            // Bootloop protection
        {
            metadata.flags = 0;
            flash_save_metadata(&metadata);                 
        } 
        device.version = SW_VER;                            // Version of software
        device.state = STATE_INIT;                          // Start device state
        device.conn_status = CONN_STATUS_NOK;               // Connection status
        device.conn_type = STATE_DATA_TRANSMIT;             // Communication type - clasic

        radio1.op_modes = NRF_MODE_PWR_ON_RST;              // set default radio state
        radio2.op_modes = NRF_MODE_PWR_ON_RST;              // set default radio state
        device.flag_lost_connection = 0;
        radio1.irq_on_pipe = 0xFF;
        radio2.irq_on_pipe = 0xFF;

        device.init_done = 1;                               // Block init 

        __enable_irq();
    }


    while(1)
    {
        /* USB TRANSCODING HANDLING */
        if (buffers.flag_new_uart_rx_data == 1) 
        {
            buffers.flag_new_uart_rx_data = 0;

            // TODO: Decode
            uint8_t ret = UART_decode(buffers.buffer_UART, &data_packets, &buffers.flag_new_rf_tx_data);
            //UART_buffer_clear();
            memset(buffers.buffer_UART, 0, sizeof(buffers.buffer_UART));

            // Based on return values trigger events
            switch (ret) 
            {
                case TRANSCODE_BOOT_PKT:
                    system_reset_trigger();             // Bootloader reboot detection
                    break;
                
                case TRANSCODE_BROADCAST:
                    device.state = STATE_CONN_START;    // Trigger new reconections
                    break;
                
                case TRANSCODE_VER_ERR:                 // Wrong version of packet
                    break;
                
                case TRANSCODE_DEST_ERR:                // Wrong destination address
                    break;

                case TRANSCODE_CRC_ERR:                 // Corupted frame / CRC
                    break;
                    
                case TRANSCODE_OK:                      // Packet OK
                    break;
            }
        }


        /* RF TRANSCODING HANDLING */
        if (buffers.flag_new_rf_rx_data == 1) 
        {
            buffers.flag_new_rf_rx_data = 0;

            // TODO: Decode
            uint8_t ret = RF_decode(radio2.buffers.RX_FIFO, &data_packets, &buffers.flag_new_uart_tx_data);

            // Based on return values trigger events
            switch (ret) 
            {
                case TRANSCODE_BOOT_PKT:                // Wrong destination address
                    break;
                
                case TRANSCODE_BROADCAST:               // Unavailable command
                    break;
                
                case TRANSCODE_VER_ERR:                 // Wrong version of packet
                    break;
                
                case TRANSCODE_DEST_ERR:                // Wrong destination address
                    break;

                case TRANSCODE_CRC_ERR:                 // Corupted frame / CRC
                    break;
                    
                case TRANSCODE_OK:                      // Packet OK
                    break;
            }
        }


        /* RADIO IRQ HANDLING */
        if (radio1.irq_flag == 1)
        {   
            // TX
            NRF24_HandleIRQ(&radio1);
            radio1.buffers.pipe_data = radio1.irq_on_pipe;
            radio1.irq_flag = 0;
            radio1.irq_on_pipe = 0xFF;
        }

        if (radio2.irq_flag == 1)
        {
            // RX
            NRF24_HandleIRQ(&radio2);
            radio2.buffers.pipe_data = radio2.irq_on_pipe;
            radio2.irq_flag = 0;
            radio2.irq_on_pipe = 0xFF;
        }
        


        /* MAIN STATE MACHINE */
        switch(device.state)
        {

            // STATE: Initialize all devices on the PCB
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
                NRF24_pin_config(&radio1, SPI1, GPIOC, NRF_CS1, GPIOC, NRF_CE1);        // Map pins for radio 1
                NRF24_pin_config(&radio2, SPI1, GPIOD, NRF_CS2, GPIOC, NRF_CE2);        // Map pins for radio 2

                uint8_t status = 0;
                if (NRF24_ReadStatus(&radio1, &status) == 0) 
                {
                    if (status != 0x0E) 
                    {
                        radio1.radioErr = NRF_ERR_BOOT;
                    }
                    else
                    {
                        radio1.radioErr = NRF_ERR_NONE;
                        radio1.op_modes = NRF_MODE_PWR_DOWN;
                    }
                }

                if (NRF24_ReadStatus(&radio2, &status) == 0) 
                {
                    if (status != 0x0E) 
                    {
                        radio2.radioErr = NRF_ERR_BOOT;
                    }
                    else
                    {
                        radio2.radioErr = NRF_ERR_NONE;
                        radio2.op_modes = NRF_MODE_PWR_DOWN;
                    }
                }

                // Set radio configurations and init
                radio1.role     = NRF_ROLE_PTX;
                radio1.config   = &radio_tx_cfg;
                radio1.address  = &radio_tx_addr;
                radio1.id       = NRF_ID_1;
                NRF24_init(&radio1);
                NRF24_SetTXAddress(&radio1, radio1.address->tx_addr);

                radio2.role     = NRF_ROLE_PRX;
                radio2.config   = &radio_rx_cfg;
                radio2.address  = &radio_rx_addr;
                radio2.id       = NRF_ID_2;
                NRF24_init(&radio2);
                NRF24_SetRXAddress(&radio2, 0, radio2.address->pipe0_rx_addr);

                // End of initialization
                GPIO_WriteBit(GPIOC, LED_RED, Bit_RESET);
                GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);

                // Check radio initialization
                if (radio1.radioErr == 1 || radio2.radioErr == 1)
                {
                    device.state = STATE_FAIL;          // Trigger init fail
                } 
                else
                {
                    device.state = STATE_CONN_START;    // OK... continue
                }
                                                
                break;



            // STATE: Start pairing routine to connect device with a drone
            case STATE_CONN_START:

                // TODO: pairing routine
                device.conn_status = CONN_STATUS_OK;   // TEST

                // Pairing is successfull
                if (device.conn_status == CONN_STATUS_OK) 
                {
                    device.flag_lost_connection = 0;
                    GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);
                    device.state = STATE_CONN_OK;
                }
                
                // Pairing is NOT successfull 
                //  -> IRQ timeout trigger (device.conn_status == CONN_STATUS_NOK)

                break;



            // STATE: Device and drone are connected, data can be send/received
            case STATE_CONN_OK:

                // ### Experimental - only option to get signal quality
                /*
                {
                    uint8_t arc = NRF24_GetLinkQuality(&radio1);

                    if (arc <= 1)       radio1.strength = LINK_EXCELLENT;
                    else if (arc <= 4)  radio1.strength = LINK_GOOD;
                    else if (arc <= 8)  radio1.strength = LINK_WEAK;
                    else                radio1.strength = LINK_BAD;
                }
                */

                // Select what type of communication will be used (Drone to PC)
                switch (device.conn_type) 
                {
                    
                    // Clasic data transmition - IRQ, parameters to/from drone
                    case STATE_DATA_TRANSMIT:
                        {
                            // ### New data available to send -> send
                            // # RF #
                            if (buffers.flag_new_rf_tx_data)
                            {
                                RF_encode(&data_packets, radio1.buffers.TX_FIFO, &radio1.buffers.tx_lenght);
                                NRF24_Send(&radio1);
                                buffers.flag_new_rf_tx_data = 0;
                            }

                            // # UART #
                            if (buffers.flag_new_uart_tx_data)
                            {
                                UART_encode(&data_packets, buffers.buffer_UART);
                                UartSendBuffer(buffers.buffer_UART, sizeof(buffers.buffer_UART));
                                buffers.flag_new_uart_tx_data = 0;
                            }

                        }


                        // ### Transmited data
                        if (radio1.buffers.flag_tx_done) 
                        {
                            // success
                            device.pct_tx_cnt++;
                            radio1.buffers.flag_tx_done = 0;
                        }

                        if (radio1.buffers.flag_max_rxs_reached) 
                        {
                            device.pct_fail_cnt++;
                            radio1.buffers.flag_max_rxs_reached = 0;

                            if (device.pct_fail_cnt >= 5)
                            {
                                device.flag_lost_connection = 1;
                                device.state = STATE_CONN_FAIL;
                            } 
                        }

                        // ### Receive data
                        if (radio2.buffers.flag_new_rx)
                        {
                            NRF24_ReadRXPayload(&radio2);
                            buffers.flag_new_uart_tx_data = 1;
                        }
                        
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
                


            // STATE: Pairing routine failed, reconnection must be triggered
            case STATE_CONN_FAIL:
                {
                    // Report failed connection
                    char statusLabel[] = "Connection FAILED!\r\n";
                    UartSendBuffer((uint8_t*)statusLabel, strlen(statusLabel));
                    GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);                     // Indicate error
                    device.state = STATE_IDLE;                                  // Go in idle mode and wait on user response

                    break;
                }
                



            // STATE: Idle mode where device just waits on user response
            case STATE_IDLE:
                // DO NOTHING... Wait on user (remote or physical)
                break;



            // STATE: Fail state if device / radion fail to initialize
            case STATE_FAIL:
                // DO NOTHING... Fast red LED blinks
                break;



            // STATE: Critical error in software
            default:
                // Problem... - sw error
            break;
        }
    }
}




/*###########################################################################################################################################################*/
/* Functions Init and code */

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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
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
    hspi1.Init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;                 // Two line SPI
    hspi1.Init.SPI_Mode = SPI_Mode_Master;                                      // SPI Mode - Master
    hspi1.Init.SPI_DataSize = SPI_DataSize_8b;                                  // Data lenght
    hspi1.Init.SPI_CPOL = SPI_CPOL_Low;                                         // Idle low
    hspi1.Init.SPI_CPHA = SPI_CPHA_1Edge;                                       // 1st edge
    hspi1.Init.SPI_NSS = SPI_NSS_Soft;                                          // Software NSS
    hspi1.Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;                // fPCLK/16
    hspi1.Init.SPI_FirstBit = SPI_FirstBit_MSB;                                 // MSB bit first
    //hspi1.SPI_CRCPolynomial = 7;                                              // CRC lenght

    // Apply config and enable
    SPI_Init(SPI1, &hspi1.Init);                                                // Apply SPI configurations
    //SPI_CalculateCRC(SPI_InitStructure.Instance, DISABLE);         
    SPI_Cmd(SPI1, ENABLE);                                                      // Enable SPI

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
    DMA_Cmd(DMA1_Channel4, DISABLE);                                            // Disable DMA

    for(uint16_t cnt = 0; cnt < length; cnt++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)             // Waiting for sending finish
        {
            // Wait, do nothing
        }
        USART_SendData(USART1, buffer[cnt]);
    }

    //memcpy(buffers.buffer_UART, 0, sizeof(buffers.buffer_UART));
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
        if (data == SIG_SOF && len_new_rx_data == 0)    // No flag for new packet and SOA packet
        {
            buffers.flag_USB_RX_new = 1;                                        // Indicate new data received
            buffers.flag_new_uart_rx_data = 0;                                        // Clear end of packet flag
            len_new_rx_data = 0;                                                // Clear packet counter
        }
        else if (buffers.flag_USB_RX_new == 1 && len_new_rx_data == 0)          // Flag for new packet, but no lenght of packet yet
        {
            len_new_rx_data = data;                                             // Save packet lenght
            buffers.flag_USB_RX_new = 0;                                        // Clear flag for new data
        }
        else if (cntBuffer_UART >= (len_new_rx_data + 2))                       // Detect end of complete packet
        {
            buffers.flag_new_uart_rx_data = 1;                                  // Indicate end of packet
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
void UART_buffer_clear(void)
{
    // Clear buffer
    for (int i = 0; i < sizeof(buffers.buffer_UART); i++)
    {
        buffers.buffer_UART[i] = 0;
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
        radio1.irq_flag = 1;
        EXTI_ClearITPendingBit(EXTI_Line3);     // Clear Flag
    }

    // Radio 2 IRQ pin
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        radio2.irq_flag = 1;
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
                device.state = STATE_CONN_FAIL;                                 // failed connection
                device.conn_status = CONN_STATUS_NOK;                           // discnnected
                GPIO_WriteBit(GPIOD, LED_BLUE, Bit_RESET);                      // Turn off blue LED
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



/*********************************************************************
 * @fcn     flash_read_boot_flag
 *
 * @param *data: pointer to metadata struct
 *
 * @brief   Read value of a boot flag from flash
 *
 * @return  none
 */
void flash_read_boot_flag(s_meta_data *data)
{
    data->flags = *(volatile uint8_t*)(APP_METADATA_ADDR);
}



/*********************************************************************
 * @fcn     flash_read_crc32
 *
 * @param *data: pointer to metadata struct pointer
 *
 * @brief   Read CRC32 value of current firmware used
 *
 * @return  none
 */
void flash_read_crc32(s_meta_data *data)
{
    data->fw_crc32 = *(volatile uint32_t*)(APP_METADATA_ADDR + 4);
}



/*********************************************************************
 * @fcn     flash_save_metadata
 *
 * @param   Metadata struct pointer
 *
 * @brief   Write metadata values to flash
 *
 * @return  none
 */
void flash_save_metadata(s_meta_data *data)
{
    // Unlock, clear all flags that may be set, errase page you want to write to
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR);
    FLASH_ErasePage_Fast(APP_METADATA_ADDR);
    while(FLASH_GetStatus() == FLASH_BUSY);

    FLASH_ProgramWord(APP_METADATA_ADDR + 0,  (uint32_t)data->flags);
    while(FLASH_GetStatus() == FLASH_BUSY);
    FLASH_ProgramWord(APP_METADATA_ADDR + 4,  data->fw_crc32);
    while(FLASH_GetStatus() == FLASH_BUSY);

    FLASH_Lock_Fast();
}



/*********************************************************************
 * @fn      system_reset_trigger
 *
 * @brief   trigger system/mcu reset 
 *          ! Only to enter bootloader
 *
 * @return  none
 */
void system_reset_trigger(void)
{
    // Send a response 
    buffers.buffer_UART[0] = SIG_SOF;
    buffers.buffer_UART[1] = 11;
    buffers.buffer_UART[2] = ID_PC;
    buffers.buffer_UART[3] = CMD_ACK;
    buffers.buffer_UART[4] = CODE_BOOT_VER;
    buffers.buffer_UART[5] = BOOT_VER;
    buffers.buffer_UART[6] = CODE_SW_CRC;
    buffers.buffer_UART[7] = (uint8_t)(metadata.fw_crc32 >> 24);
    buffers.buffer_UART[8] = (uint8_t)(metadata.fw_crc32 >> 16);
    buffers.buffer_UART[9] = (uint8_t)(metadata.fw_crc32 >> 8);
    buffers.buffer_UART[10] = (uint8_t)(metadata.fw_crc32);
    uint16_t crc_temp = crc16_cal(&buffers.buffer_UART[1], 10);
    buffers.buffer_UART[11] = (uint8_t)(crc_temp);
    buffers.buffer_UART[12] = (uint8_t)(crc_temp >> 8);

    // Send
    UartSendBuffer(buffers.buffer_UART, sizeof(buffers.buffer_UART));

    __disable_irq();

    metadata.flags = 1; // stay in bootloader
    flash_save_metadata(&metadata);

    NVIC_SystemReset();
    while(1); // wait for reset
}


