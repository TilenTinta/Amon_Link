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
static void UART_decode(uint8_t* raw_uart_data);
static void UART_buffer_clear(void);
static void UART_packet_parse(uint8_t* raw_data);
static void UART_packetDataReset(void);
static void flash_erase_app(void);
static void flash_write_chunk(uint32_t address, uint8_t *data, uint16_t length);
static uint16_t crc16_cal(const uint8_t *data, uint16_t length);

static void jump_to_app(void);


/* Structs */
buffers_t buffers;
packet_t packet;
typedef void (*app_entry_t)(void);

int main(void)
{
    // Init //
    SystemCoreClockUpdate(); 
    LinkPinout_Init();
    USART1_Init();

    buffers.flag_update_OK = 0;
    buffers.flag_update_NOK = 0;

    // Check if button is pressed, TODO: or reset flag set 
    if (GPIO_ReadInputDataBit(GPIOD, BTN_PIN) == Bit_SET) 
    {
        // Turn on BLUE LED
        GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);

        while(1)
        {
            if (buffers.flag_USB_RX_end == 1) 
            {
                UART_decode(buffers.buffer_UART);               
            }

            if (buffers.flag_update_OK == 1) jump_to_app();     // Exit bootloader
            if (buffers.flag_update_NOK == 1) GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);   // Indicate an error


        };
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
 * @fn      UART_packetDataReset
 *
 * @brief   Clear data in UART struct
 *
 * @return  none
 */
static void UART_packetDataReset(void)
{
    // Clear data struct
    packet.sof = 0;
    packet.plen = 0;
    packet.addr = 0;
    packet.cmd = 0;
    for (int i = 0; i < packet.plen; i++)
    {
        packet.payload[i] = 0;
    }
    packet.crc16 = 0;
}



/*********************************************************************
 * @fcn     UART_decode
 *
 * @brief   Initialises the GPIOs
 *
 * @return  none
 */

static void UART_decode(uint8_t* raw_uart_data)
{

    // Parse received buffer
    UART_packetDataReset();
    UART_packet_parse(raw_uart_data);

    // CRC check
    uint16_t cal_CRC = crc16_cal(&raw_uart_data[1], packet.plen - 2);

    if (cal_CRC != packet.crc16)
    {
        // Send a response - error
        buffers.buffer_UART[0] = SIG_SOF;
        buffers.buffer_UART[1] = 5;
        buffers.buffer_UART[2] = ID_PC;
        buffers.buffer_UART[3] = CMD_ERR;
        buffers.buffer_UART[4] = CODE_BAD_CRC;
        buffers.buffer_UART[5] = crc16_cal(&buffers.buffer_UART[1], 4);

        // Send
        UartSendBuffer(buffers.buffer_UART,sizeof(buffers.buffer_UART));
        return;
    } 

    // Clear uart buffer - used for transmit
    UART_buffer_clear();
   
    // Check destination address
    switch (packet.addr) 
    {
        // Destination device: PC - error
        case ID_PC:
            return;
            break;

        // Destination device: link
        case ID_LINK:
            
            // Do action based on commands
            switch (packet.cmd)
            {
                case CMD_GET_INFO:

                    // Send a response 
                    buffers.buffer_UART[0] = SIG_SOF;
                    buffers.buffer_UART[1] = 5;
                    buffers.buffer_UART[2] = ID_PC;
                    buffers.buffer_UART[3] = CMD_ACK;
                    buffers.buffer_UART[4] = CODE_SW_VER;
                    buffers.buffer_UART[5] = SW_VER;
                    buffers.buffer_UART[6] = crc16_cal(&buffers.buffer_UART[1], 5);

                    // Send
                    UartSendBuffer(buffers.buffer_UART,sizeof(buffers.buffer_UART));

                    break;


                case CMD_ERASE_APP:

                    flash_erase_app();

                    // Send a response 
                    buffers.buffer_UART[0] = SIG_SOF;
                    buffers.buffer_UART[1] = 5;
                    buffers.buffer_UART[2] = ID_PC;
                    buffers.buffer_UART[3] = CMD_ACK;
                    buffers.buffer_UART[4] = crc16_cal(&buffers.buffer_UART[1], 3);

                    // Send
                    UartSendBuffer(buffers.buffer_UART,sizeof(buffers.buffer_UART));

                    break;


                case CMD_WRITE:
                    {
                        uint32_t addr = packet.payload[0]        |
                                   (packet.payload[1] << 8)  |
                                   (packet.payload[2] << 16) |
                                   (packet.payload[3] << 24);

                        flash_write_chunk(addr, &packet.payload[4], sizeof(packet.payload) - 4);

                        // Send a response 
                        buffers.buffer_UART[0] = SIG_SOF;
                        buffers.buffer_UART[1] = 5;
                        buffers.buffer_UART[2] = ID_PC;
                        buffers.buffer_UART[3] = CMD_ACK;
                        buffers.buffer_UART[4] = CODE_DATA_WRITEN;
                        buffers.buffer_UART[5] = crc16_cal(&buffers.buffer_UART[1], 4);

                        // Send
                        UartSendBuffer(buffers.buffer_UART,sizeof(buffers.buffer_UART));
                        break;
                    }
                    

                case CMD_JUMP_APP:

                    // Send a respnse 
                    buffers.buffer_UART[0] = SIG_SOF;
                    buffers.buffer_UART[1] = 5;
                    buffers.buffer_UART[2] = ID_PC;
                    buffers.buffer_UART[3] = CMD_ACK;
                    buffers.buffer_UART[4] = CODE_EXIT_BOOT;
                    buffers.buffer_UART[5] = crc16_cal(&buffers.buffer_UART[1], 4);

                    // Send
                    UartSendBuffer(buffers.buffer_UART,sizeof(buffers.buffer_UART));

                    jump_to_app();

                    break;
            }

            break;

        default:
            // Unavailable
            break;
    
    }
}



 /*********************************************************************
 * @fcn     UART_packet_parse
 *
 * @brief   Spliting packet in to information parts
 *
 * @return  none
 */

static void UART_packet_parse(uint8_t* raw_data)
{
    packet.sof = *raw_data;
    packet.plen = *(raw_data + 1);
    packet.addr = *(raw_data + 2);
    packet.cmd = *(raw_data + 3);
    for (int i = 0; i < packet.plen; i++)
    {
        packet.payload[i] = *(raw_data + HEADER_SHIFT + i);
    }
    packet.crc16 = *(raw_data + (packet.plen + 1));
}



/*********************************************************************
 * @fcn     erase_app_space
 *
 * @brief   Erase part of flash where app is places
 *
 * @return  none
 */
static void flash_erase_app(void)
{
    FLASH_Unlock();

    for(uint32_t addr = APP_START_ADDRESS; addr < APP_END_ADDRESS; addr += PAGE_SIZE)
    {
        FLASH_ErasePage(addr);
        while(FLASH_GetStatus() == FLASH_BUSY); // wait 
    }

    FLASH_Lock();
}



/*********************************************************************
 * @fcn     flash_write_chunk
 *
 * @brief   Write single page to flash
 *
 * @return  none
 */
static void flash_write_chunk(uint32_t address, uint8_t *data, uint16_t length)
{
    FLASH_Unlock();

    for(uint16_t i = 0; i < length; i += 4)
    {
        uint32_t word =  data[i] |
                        (data[i+1] << 8) |
                        (data[i+2] << 16) |
                        (data[i+3] << 24);

        FLASH_ProgramWord(address + i, word);
        while(FLASH_GetStatus() == FLASH_BUSY);
    }

    FLASH_Lock();
}



/*********************************************************************
 * @fcn     crc16_cal
 *
 * @brief   Calculates 2 byte crc over data
 *          Modbus RTU-style CRC-16 algorithm
 *          https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
 *
 * @return  crc value
 */
static uint16_t crc16_cal(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;                  // don¡¯t start with zero, this would make some leading zeros in data invisible

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];                     // XOR byte into least sig. byte of crc

        for (uint8_t j = 8; j != 0; j--)    // Loop over each bit
        {
            if ((crc & 0x0001) != 0)        // If the LSB is set
            {
                crc >>= 1;                  // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }       
            else                            // Else LSB is not set
            {
                crc >>= 1;                  // Just shift right
            }                            
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly
    return crc;
}



/*********************************************************************
 * @fn      jump_to_app
 *
 * @brief   Jump from bootloader to main program
 *
 * @return  none
 */
static void jump_to_app(void)
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




