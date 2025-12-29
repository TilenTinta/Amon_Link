/*****************************************************************
 * File Name          : dataDecoder.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/09
 * Description        : Decoding logic for received and transmited data
*****************************************************************/

#include "dataDecoder.h"

/* Structs */
s_uart_packet uart_packet;      // UART packet - application format
s_boot_packet boot_packet;      // UART packet - bootloader format


/* Local function */
static void UART_packetDataReset(void);


 /*********************************************************************
 * @fcn     UART_packet_parse
 *
 * @param *raw_data: pointer to raw data received over uart
 *
 * @brief   Spliting packet in to information parts
 *
 * @return  none
 */

void UART_packet_parse(s_uart_packet *data, uint8_t *raw_data)
{
    data->sof       = *raw_data;
    data->len       = *(raw_data + 1);
    data->version   = *(raw_data + 2);
    data->flags     = *(raw_data + 3);
    data->src_id    = *(raw_data + 4);
    data->dist_id   = *(raw_data + 5);
    data->opcode    = *(raw_data + 6);
    data->plen      = *(raw_data + 7);

    if (data->plen > 0)                 // Detect if payload is even present
    {
        for (int i = 0; i < data->plen; i++)
        {
            data->payload[i] = *(raw_data + HEADER_SHIFT + i);
        }
    }

    uint16_t crc1_tmp = (uint16_t)*(raw_data + data->len);
    uint16_t crc2_tmp = (uint16_t)*(raw_data + data->len + 1) << 8;
    data->CRC = crc1_tmp | crc2_tmp;

}



 /*********************************************************************
 * @fcn     UART_boot_packet_parse
 *
 * @param *raw_data: pointer to raw data packet that you want to decode in fields
 * @param *data: pointer to packet struct
 *
 * @brief   Spliting packet in to information parts
 *
 * @return  none
 */

void UART_boot_packet_parse(s_boot_packet *data, uint8_t *raw_data)
{
    // Example: 
    //      Packet structure: SOF=0xaa, PLEN=0x4, ADDR=0x10, CMD=0x1, PAYLOAD=0xXX, CRC16=0x1234
    //      CRC16: calculated over entire packet excluding SOF and CRC16 itself
    //      PLEN: does not count itself in its value
    //      Full packet bytes: ['0xaa', '0x4', '0x10', '0x1', '0xfc', '0x1']

    data->sof = *raw_data;
    data->plen = *(raw_data + 1);      // Without CRC16 (and SOF)
    data->addr = *(raw_data + 2);
    data->cmd = *(raw_data + 3);

    // if (data->plen > 4)                // 4 bytes does not have payload (only: ADDR, CMD, CRC16_1, CRC16_2)
    // {
    //     for (int i = 0; i < data->plen - 4; i++)
    //     {
    //         data->payload[i] = *(raw_data + HEADER_SHIFT + i);
    //     }
    // }

    uint16_t crc1_tmp = (uint16_t)*(raw_data + data->plen);
    uint16_t crc2_tmp = (uint16_t)*(raw_data + data->plen + 1) << 8;
    data->crc16 = crc1_tmp | crc2_tmp;

    //packet.crc16 = (uint16_t)*(raw_data + (packet.plen)) | ((uint16_t)*(raw_data + (packet.plen - 1)) << 8);
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
    UART_packet_parse(&uart_packet, &raw_uart_data[0]);

    // Check version
    if (uart_packet.version != PROTOCOL_VER) return;

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

        case ID_LINK_BOOT:
            return;
            break;

        // Destination device: link
        case ID_LINK_SW:
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



