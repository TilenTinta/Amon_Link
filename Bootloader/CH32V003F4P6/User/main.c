/*****************************************************************
 * File Name          : main.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/10/25
 * Description        : Custom bootloader for updating firmware 
 *                      over UART (USB)
*****************************************************************/

#include "debug.h"
#include "main.h"
#include "core_riscv.h"

/* IRQs */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Functions */
void USART1_Init(void);
void LinkPinout_Init(void);
void UartSendBuffer(uint8_t* buffer, uint16_t length);
void UART_decode(uint8_t* raw_uart_data);
void UART_buffer_clear(void);
void UART_packet_parse(uint8_t* raw_data);
void UART_packetDataReset(void);
void flash_write_page(s_buffers *address, uint8_t *data);
void flash_read_boot_flag(s_meta_data *data);
void flash_read_crc32(s_meta_data *data);
void flash_save_metadata(s_meta_data *data);
uint16_t crc16_cal(const uint8_t *data, uint16_t length);
void jump_to_app(void);


/* Structs */
s_buffers buffers;
s_packet packet;
s_meta_data metadata;
typedef void (*app_entry_t)(void);

/* Variables */
//uint32_t device_id = 0;


int main(void)
{
    // Init //
    SystemCoreClockUpdate(); 
    LinkPinout_Init();
    USART1_Init();

    //device_id = DBGMCU_GetCHIPID();             // Get unic device ID

    /* For test/development */
    // metadata.flags = 0x01;
    // metadata.fw_crc32 = 0x12345678;
    // flash_save_metadata(&metadata);

    flash_read_crc32(&metadata);                  // Read current firmware crc32 value 
    flash_read_boot_flag(&metadata);              // Read flag from flash - select staying in bootloader or not

    buffers.flag_USB_RX_end = 0;
    buffers.flag_new_uart_rx_data = 0;
    buffers.flash_address_cnt = APP_FW_START_ADDR;  // Start address where firmware will be written

    // Check if button is pressed to stay in bootloader
    if ((GPIO_ReadInputDataBit(GPIOD, BTN_PIN) == Bit_SET) || metadata.flags == 1)
    {
        // Turn on BLUE LED
        GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);

        while(1)
        {
            /* Wait for received data */
            if (buffers.flag_USB_RX_end == 1) 
            {
                UART_decode(&buffers.buffer_UART[0]); 
                buffers.flag_USB_RX_end = 0;   
                UART_buffer_clear();           
            }

            //GPIO_WriteBit(GPIOC, LED_RED, Bit_SET);   // Indicate an error
        };
    }
    else
    {
        /* Jump to main application */
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
void LinkPinout_Init(void)
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
 * @fcn     UartSendBuffer
 *
 * @param *buffer: pointer to data you want to send
 * @param lenght: lenght of data bytes to send
 *
 * @brief   Send data over USART - USB comunication (parameters, setup...)
 *
 * @return  none
 */
void UartSendBuffer(uint8_t *buffer, uint16_t length)
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
        if (data == SIG_SOF && len_new_rx_data == 0 )                           // - No flag for new packet and SOA packet     
        {
            buffers.flag_new_uart_rx_data = 1;                                  // Indicate new data received
            buffers.flag_USB_RX_end = 0;                                        // Clear end of packet flag
        }
        else if (buffers.flag_new_uart_rx_data == 1 && len_new_rx_data == 0)    // - Flag for new packet, but no lenght of packet
        {
            len_new_rx_data = data;                                             // Save packet lenght
            buffers.flag_new_uart_rx_data = 0;                                  // Clear flag for new data
        }
        else if (cntBuffer_UART >= (len_new_rx_data + 2) && len_new_rx_data != 0) // - Detect end of complete packet
        {
            buffers.flag_USB_RX_end = 1;                                        // Indicate end of packet
            cntBuffer_UART = 0;                                                 // Clear UART counter
            len_new_rx_data = 0;                                                // Set expected data to 0
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
 * @fn      UART_packetDataReset
 *
 * @brief   Clear data in UART struct
 *
 * @return  none
 */
void UART_packetDataReset(void)
{
    // Clear data struct
    // packet.sof = 0;
    packet.plen = 0;
    packet.addr = 0;
    packet.cmd = 0;
    for (int i = 0; i < sizeof(packet.payload); i++)
    {
        packet.payload[i] = 0;
    }
    packet.crc16 = 0;
}



/*********************************************************************
 * @fcn     UART_decode
 *
 * @param *raw_uart_data: pointer ro raw data packet received over UART
 * 
 * @brief   Initialises the GPIOs
 *
 * @return  none
 */

void UART_decode(uint8_t* raw_uart_data)
{

    // Parse received buffer
    UART_packetDataReset();
    UART_packet_parse(raw_uart_data);

    // CRC check
    uint16_t cal_CRC = crc16_cal(&raw_uart_data[1], packet.plen - 1);   // CRC calculate over: PLEN + ADDR + CMD + payload (-1 because of PLEN)

    if (cal_CRC != packet.crc16)
    {
        // Send a response - CRC error 
        buffers.buffer_UART[0] = SIG_SOF;
        buffers.buffer_UART[1] = 5;
        buffers.buffer_UART[2] = ID_PC;
        buffers.buffer_UART[3] = CMD_ERR;
        buffers.buffer_UART[4] = CODE_BAD_CRC;
        uint16_t crc_temp = crc16_cal(&buffers.buffer_UART[1], 4);
        buffers.buffer_UART[5] = (uint8_t)(crc_temp);
        buffers.buffer_UART[6] = (uint8_t)(crc_temp >> 8);

        // Send
        UartSendBuffer(buffers.buffer_UART, sizeof(buffers.buffer_UART));
        UART_buffer_clear();
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

        // Destination device: LINK main software - error
        case ID_LINK_SW:
            return;
            break;

        // Destination device: Drone - error
        case ID_DRONE:
            return;
            break;

        // Destination device: Broadcast - error
        case ID_BROADCAST:
            return;
            break;

        // Destination device: link
        case ID_LINK_BOOT:
            
            // Do action based on commands
            switch (packet.cmd)
            {
                case CMD_INFO:

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

                    break;

                case CMD_WRITE:
                    {
                        flash_write_page(&buffers, packet.payload);

                        // Send a response 
                        buffers.buffer_UART[0] = SIG_SOF;
                        buffers.buffer_UART[1] = 5;
                        buffers.buffer_UART[2] = ID_PC;
                        buffers.buffer_UART[3] = CMD_ACK;
                        buffers.buffer_UART[4] = CODE_DATA_WRITEN;
                        uint16_t crc_temp = crc16_cal(&buffers.buffer_UART[1], 4);
                        buffers.buffer_UART[5] = (uint8_t)(crc_temp);
                        buffers.buffer_UART[6] = (uint8_t)(crc_temp >> 8);

                        // Send
                        UartSendBuffer(buffers.buffer_UART, sizeof(buffers.buffer_UART));
                        break;
                    }

                case CMD_END_OF_FW:
                    {
                        // Save a flag and replace CRC32 for new firmware
                        metadata.flags = packet.payload[0];              // Next boot do not stay in bootloader
                        metadata.fw_crc32 = (packet.payload[1] << 24 |
                                             packet.payload[2] << 16 |
                                             packet.payload[3] << 8  |
                                             packet.payload[4]);        // Assemble new CRC32 value
                                             
                        flash_save_metadata(&metadata);

                        // Send a response 
                        buffers.buffer_UART[0] = SIG_SOF;
                        buffers.buffer_UART[1] = 5;
                        buffers.buffer_UART[2] = ID_PC;
                        buffers.buffer_UART[3] = CMD_ACK;
                        buffers.buffer_UART[4] = CODE_EXIT_BOOT;
                        uint16_t crc_temp = crc16_cal(&buffers.buffer_UART[1], 4);
                        buffers.buffer_UART[5] = (uint8_t)(crc_temp);
                        buffers.buffer_UART[6] = (uint8_t)(crc_temp >> 8);

                        // Send
                        UartSendBuffer(buffers.buffer_UART, sizeof(buffers.buffer_UART));

                        // Jump to main application
                        jump_to_app();

                        break;
                    }
                    

                case CMD_JUMP_APP:
                    {
                        // Send a response 
                        buffers.buffer_UART[0] = SIG_SOF;
                        buffers.buffer_UART[1] = 5;
                        buffers.buffer_UART[2] = ID_PC;
                        buffers.buffer_UART[3] = CMD_ACK;
                        buffers.buffer_UART[4] = CODE_EXIT_BOOT;
                        uint16_t crc_temp = crc16_cal(&buffers.buffer_UART[1], 4);
                        buffers.buffer_UART[5] = (uint8_t)(crc_temp);
                        buffers.buffer_UART[6] = (uint8_t)(crc_temp >> 8);

                        // Send
                        UartSendBuffer(buffers.buffer_UART, sizeof(buffers.buffer_UART));

                        jump_to_app();

                        break;
                    }
                    
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
 * @param *raw_data: pointer to raw data packet that you want to decode in fields
 *
 * @brief   Spliting packet in to information parts
 *
 * @return  none
 */

void UART_packet_parse(uint8_t *raw_data)
{
    // Example: 
    //      Packet structure: SOF=0xaa, PLEN=0x4, ADDR=0x10, CMD=0x1, PAYLOAD=0xXX, CRC16=0x1234
    //      CRC16: calculated over entire packet excluding SOF and CRC16 itself
    //      PLEN: does not count itself in its value
    //      Full packet bytes: ['0xaa', '0x4', '0x10', '0x1', '0xfc', '0x1']

    // packet.sof = *raw_data;
    packet.plen = *(raw_data + 1);      // Without CRC16 (and SOF)
    packet.addr = *(raw_data + 2);
    packet.cmd = *(raw_data + 3);

    if (packet.plen > 4)                // 4 bytes does not have payload (only: ADDR, CMD, CRC16_1, CRC16_2)
    {
        for (int i = 0; i < packet.plen - 4; i++)
        {
            packet.payload[i] = *(raw_data + HEADER_SHIFT + i);
        }
    }

    uint16_t crc1_tmp = (uint16_t)*(raw_data + packet.plen);
    uint16_t crc2_tmp = (uint16_t)*(raw_data + packet.plen + 1) << 8;
    packet.crc16 = crc1_tmp | crc2_tmp;

    //packet.crc16 = (uint16_t)*(raw_data + (packet.plen)) | ((uint16_t)*(raw_data + (packet.plen - 1)) << 8);
}



/*********************************************************************
 * @fcn     flash_write_page
 *
 * @param *address: pointer to buffers struct
 * @param *data: pointer to payload data of firmware
 *
 * @brief   Write single page to flash
 *
 * @return  none
 */
void flash_write_page(s_buffers *address, uint8_t *data)
{
    // Do not allow firmware update bo write over metadata page
    if (address->flash_address_cnt >= APP_METADATA_ADDR) return;

    // Unlock, clear all flags that may be set, errase page you want to write to
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR);
    FLASH_ErasePage_Fast(address->flash_address_cnt);
    while(FLASH_GetStatus() == FLASH_BUSY);

    // Write all data
    for(uint8_t i = 0; i < PAGE_SIZE; i += 4)
    {
        uint32_t word = (uint32_t)data[i]     |
                ((uint32_t)data[i + 1] << 8)  |
                ((uint32_t)data[i + 2] << 16) |
                ((uint32_t)data[i + 3] << 24);

        FLASH_ProgramWord(address->flash_address_cnt + i, word);
        while(FLASH_GetStatus() == FLASH_BUSY);
    }

    // Lock flash and increase counter to next page
    FLASH_Lock_Fast();
    address->flash_address_cnt += PAGE_SIZE;
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
 * @fcn     crc16_cal
 *
 * @param *data: pointer to data over which you want to calculate crc
 * @param lenght: lenght of data you provide to function
 *
 * @brief   Calculates 2 byte crc over data
 *          Modbus RTU-style CRC-16 algorithm
 *          x^16 + x^15 + x^2 + 1
 *          https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
 *
 * @return  crc value
 */
uint16_t crc16_cal(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;                  // don't start with zero, this would make some leading zeros in data invisible

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= *(data+i);                   // XOR byte into least sig. byte of crc

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
void jump_to_app(void)
{
    const uint32_t app_entry = APP_START_ADDRESS; // entry (_start) of the app

    // Disable interrupts while switching context
    __disable_irq();

    // Clear pending IRQs you used (at least USART1)
    NVIC_ClearPendingIRQ(USART1_IRQn);

    // Deinit peripherals used by the bootloader
    USART_DeInit(USART1);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOC);

    // Disable peripheral clocks used by the bootloader
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_USART1 |
        RCC_APB2Periph_GPIOD  |
        RCC_APB2Periph_GPIOC  |
        RCC_APB2Periph_AFIO,
        DISABLE);

    // Set the next PC to the app entry (_start) and return to it in M-mode
    __set_MEPC(app_entry);
    __asm volatile("mret");

    // Should never reach here
    while (1) { }

}




