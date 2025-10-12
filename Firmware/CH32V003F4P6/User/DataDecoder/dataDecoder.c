/*****************************************************************
 * File Name          : dataDecoder.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/09
 * Description        : Decoding logic for received and transmited data
*****************************************************************/

#include "dataDecoder.h"

UART_PACKET uart_packet;    // UART packet parts

/* Local function */
static void UART_packetDataReset(void);
static void UART_packetDataReset(void);


 /*********************************************************************
 * @fcn     UART_packet_parse
 *
 * @brief   Spliting packet in to information parts
 *
 * @return  none
 */

void UART_packet_parse(uint8_t* raw_data)
{
    uart_packet.sof = *raw_data;
    uart_packet.len = *(raw_data + 1);
    uart_packet.version = *(raw_data + 2);
    uart_packet.flags = *(raw_data + 3);
    uart_packet.src_id = *(raw_data + 4);
    uart_packet.dist_id = *(raw_data + 5);
    uart_packet.opcode = *(raw_data + 6);
    uart_packet.plen = *(raw_data + 7);
    for (int i = 0; i < uart_packet.plen; i++)
    {
        uart_packet.payload[i] = *(raw_data + HEADER_SHIFT + i);
    }
    uart_packet.CRC = *(raw_data + (uart_packet.plen + 1));

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
    uart_packet.sof = 0;
    uart_packet.len = 0;
    uart_packet.version = 0;
    uart_packet.flags = 0;
    uart_packet.src_id = 0;
    uart_packet.dist_id = 0;
    uart_packet.opcode = 0;
    for (int i = 0; i < uart_packet.plen; i++)
    {
        uart_packet.payload[i] = 0;
    }
    uart_packet.plen = 0;
    uart_packet.CRC = 0;
}


/*********************************************************************
 * @fcn     UART_decode
 *
 * @brief   Initialises the GPIOs
 *
 * @return  none
 */

void UART_decode(uint8_t* raw_uart_data, uint8_t* raw_rf_data, uint8_t* rf_tx_flag)
{
    // Parse received buffer
    UART_packetDataReset();
    UART_packet_parse(raw_uart_data);

    // Check version
    if (uart_packet.version != DRIVER_VER) return;

    // CRC check
    uint16_t cal_CRC = 0;

    if (cal_CRC != uart_packet.CRC) return;


    // Check destination address
    switch (uart_packet.dist_id) 
    {
        // Destination device: PC - error
        case ID_PC:

            return;

            break;

        // Destination device: link
        case ID_LINK:
            //...
            break;

        // Destination device: drone
        case ID_DRONE:

            *rf_tx_flag = 1;    // indicate that new data to send is available
            
            // Asamble RF packet
            *raw_rf_data = uart_packet.version;
            *(raw_rf_data + 1) = uart_packet.flags;
            *(raw_rf_data + 2) = uart_packet.src_id;
            *(raw_rf_data + 3) = uart_packet.dist_id;
            *(raw_rf_data + 4) = uart_packet.opcode;
            *(raw_rf_data + 5) = uart_packet.plen;
            for (int i = 5; i < uart_packet.plen + 5; i++)
            {
                *(raw_rf_data + i) = uart_packet.payload[i];
            }

            break;

        // Destination device: broadcast - triger connecting/search...
        case ID_BROADCAST:

            break;

        default:
            // Unavailable
            break;
    
    }
}


/*********************************************************************
 * @fcn     CRC_calculate
 *
 * @brief   calculate CRC of received data over UART
 *
 * @return  none
 */



