/*****************************************************************
 * File Name          : dataDecoder.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/09
 * Description        : Decoding logic for received and transmited data
*****************************************************************/

#include "data_transcode.h"

/* Structs */
s_packets packets;


/* Local function */
static void UART_packet_parse(s_packets *data, uint8_t *raw_data);
static void RF_packet_parse(s_packets *data, uint8_t *raw_data);
static void packetDataReset(void);
//uint16_t crc16_cal(const uint8_t *data, uint16_t length);



/*********************************************************************
 * @fn      UART_packetDataReset
 *
 * @brief   Clear data in all packet structs
 *
 * @return  none
 */
void packetDataReset(void)
{
    // Clear data struct - uart
    packets.uart_packet.sof = 0;
    packets.uart_packet.len = 0;
    packets.uart_packet.version = 0;
    packets.uart_packet.flags = 0;
    packets.uart_packet.src_id = 0;
    packets.uart_packet.dest_id = 0;
    packets.uart_packet.opcode = 0;
    for (int i = 0; i < packets.uart_packet.plen; i++)
    {
        packets.uart_packet.payload[i] = 0;
    }
    packets.uart_packet.plen = 0;
    packets.uart_packet.CRC = 0;

    // Clear data struct - boot
    packets.boot_packet.sof = 0;
    packets.boot_packet.addr = 0;
    packets.boot_packet.cmd = 0;
    packets.boot_packet.plen = 0;
    packets.boot_packet.crc16 = 0;

    // Clear data struct - rf
    packets.rf_packet.version = 0;
    packets.rf_packet.flags = 0;
    packets.rf_packet.src_id = 0;
    packets.rf_packet.dest_id = 0;
    packets.rf_packet.opcode = 0;
    packets.rf_packet.plen = 0;
    for (int i = 0; i < sizeof(packets.rf_packet.payload); i++)
    {
        packets.rf_packet.payload[i] = 0;
    }
}



 /*********************************************************************
 * @fn     UART_packet_parse
 *
 * @param *data: pointer containing all packets data structure
 * @param *raw_data: pointer to raw data received over uart
 *
 * @brief   Spliting uart packet in to information fields
 *
 * @return  none
 */

void UART_packet_parse(s_packets *data, uint8_t *raw_data)
{
    data->uart_packet.sof       = *raw_data;
    data->uart_packet.len       = *(raw_data + 1);

    // Check if packet is bootloader format -> if it is decode different
    // Example: 
        //      Packet structure: SOF=0xaa, PLEN=0x4, ADDR=0x10, CMD=0x1, PAYLOAD=0xXX, CRC16=0x1234
        //      CRC16: calculated over entire packet excluding SOF and CRC16 itself
        //      PLEN: does not count itself in its value
        //      Full packet bytes: ['0xaa', '0x4', '0x10', '0x1', '0xfc', '0x1']

    if (data->uart_packet.len == 4) 
    {   
        data->boot_packet.sof = data->uart_packet.sof;
        data->boot_packet.plen = data->uart_packet.len;               // Without CRC16 (and SOF)
        data->boot_packet.addr = *(raw_data + 2);
        data->boot_packet.cmd = *(raw_data + 3);

        // if (data->plen > 4)                // 4 bytes does not have payload (only: ADDR, CMD, CRC16_1, CRC16_2)
        // {
        //     for (int i = 0; i < data->plen - 4; i++)
        //     {
        //         data->payload[i] = *(raw_data + HEADER_SHIFT + i);
        //     }
        // }

        uint16_t crc1_tmp = (uint16_t)*(raw_data + data->uart_packet.len);
        uint16_t crc2_tmp = (uint16_t)*(raw_data + data->uart_packet.len + 1) << 8;
         data->boot_packet.crc16 = crc1_tmp | crc2_tmp;
        return;
    }

    data->uart_packet.version   = *(raw_data + 2);
    data->uart_packet.flags     = *(raw_data + 3);
    data->uart_packet.src_id    = *(raw_data + 4);
    data->uart_packet.dest_id   = *(raw_data + 5);
    data->uart_packet.opcode    = *(raw_data + 6);
    data->uart_packet.plen      = *(raw_data + 7);

    if (data->uart_packet.plen > 0)                 // Detect if payload is even present
    {
        for (int i = 0; i < data->uart_packet.plen; i++)
        {
            data->uart_packet.payload[i] = *(raw_data + HEADER_SHIFT_UART + i);
        }
    }

    uint16_t crc1_tmp = (uint16_t)*(raw_data + data->uart_packet.len);
    uint16_t crc2_tmp = (uint16_t)*(raw_data + data->uart_packet.len + 1) << 8;
    data->uart_packet.CRC = crc1_tmp | crc2_tmp;

}



 /*********************************************************************
 * @fn     RF_packet_parse
 *
 * @param *data: pointer containing all packets data structure
 * @param *raw_data: pointer to raw data received over uart
 *
 * @brief   Spliting rf packet in to information fields
 *
 * @return  none
 */

void RF_packet_parse(s_packets *data, uint8_t *raw_data)
{
    data->rf_packet.version    = *raw_data;
    data->rf_packet.flags      = *(raw_data + 1);
    data->rf_packet.src_id     = *(raw_data + 2);
    data->rf_packet.dest_id    = *(raw_data + 3);
    data->rf_packet.opcode     = *(raw_data + 4);
    data->rf_packet.plen       = *(raw_data + 5);

    if (data->rf_packet.plen > 0)                 // Detect if payload is even present
    {
        for (int i = 0; i < data->rf_packet.plen; i++)
        {
            data->rf_packet.payload[i] = *(raw_data + HEADER_SHIFT_RF + i);
        }
    }
}



/*********************************************************************
 * @fn     UART_decode
 *
 * @param *raw_uart_data: pointer to raw data received over uart
 * @param *packets: pointer to struct of all packets data
 * @param *rf_tx_flag: pointer to flag for new rf data avalable to transmit
 *
 * @brief   Decode received UART packet on its fields
 *
 * @return  error code - watch defines
 */
uint8_t UART_decode(uint8_t *raw_uart_data, s_packets *packets, uint8_t *rf_tx_flag)
{
    // Parse received buffer
    packetDataReset();
    UART_packet_parse(packets, &raw_uart_data[0]);

    // CRC check
    if (packets->boot_packet.plen != 0)
    {
        // Bootloader format
        if (crc16_cal(&raw_uart_data[1], packets->boot_packet.plen - 1) != packets->boot_packet.crc16) return TRANSCODE_CRC_ERR;
        if (packets->boot_packet.addr == ID_LINK_BOOT) return TRANSCODE_BOOT_PKT;
    }
    else 
    {
        // UART format
        if (crc16_cal(&raw_uart_data[1], packets->uart_packet.len - 1) != packets->uart_packet.CRC) return TRANSCODE_CRC_ERR;
    }

    // Check version
    if (packets->uart_packet.version != PROTOCOL_VER) return TRANSCODE_VER_ERR;

    // Check destination address
    switch (packets->uart_packet.dest_id) 
    {
        // Destination device: PC - error
        case ID_PC:
            return TRANSCODE_DEST_ERR;
            break;

        // Destination device: link bootloader - handled before
        case ID_LINK_BOOT:
            return TRANSCODE_DEST_ERR;
            break;

        // Destination device: link main application
        case ID_LINK_SW:
            //...
            return TRANSCODE_OK;
            break;

        // Destination device: drone
        case ID_DRONE:

            *rf_tx_flag = 1;    // indicate that new data to send is available
            
            // save in fields of RF packet
            packets->rf_packet.version = packets->uart_packet.version;
            packets->rf_packet.flags = packets->uart_packet.flags;
            packets->rf_packet.src_id = packets->uart_packet.src_id;
            packets->rf_packet.dest_id = packets->uart_packet.dest_id;
            packets->rf_packet.opcode = packets->uart_packet.opcode;
            packets->rf_packet.plen = packets->uart_packet.plen;
            for (int i = 0; i < packets->uart_packet.plen; i++)
            {
                packets->rf_packet.payload[i] = packets->uart_packet.payload[i];
            }

            return TRANSCODE_OK;
            break;

        // Destination device: broadcast - triger connecting/search...
        case ID_BROADCAST:
            return TRANSCODE_BROADCAST;
            break;

        default:
            // Unavailable
            break;
    
    }

    return 0;
}



/*********************************************************************
 * @fn     RF_decode
 *
 * @param *raw_rf_data: pointer to struct of rf data
 * @param *packets: pointer to struct of all packets data
 * @param *uart_tx_flag: pointer to flag for new uart data avalable to transmit
 *
 * @brief   Decode RF packet on its fields
 *
 * @return  error code - watch defines
 */
uint8_t RF_decode(uint8_t *raw_rf_data, s_packets *packets,  uint8_t *uart_tx_flag)
{
    // Parse received buffer
    packetDataReset();
    RF_packet_parse(packets, &raw_rf_data[0]);

    // Check version
    if (packets->rf_packet.version != PROTOCOL_VER) return TRANSCODE_VER_ERR; 

    // Check destination address
    switch (packets->rf_packet.dest_id) 
    {
        // Destination device: Drone - error
        case ID_DRONE:
            return TRANSCODE_DEST_ERR;
            break;

        // Destination device: link bootloader - error
        case ID_LINK_BOOT:
            return TRANSCODE_DEST_ERR;
            break;

        // Destination device: link main application
        case ID_LINK_SW:
            //...
            return TRANSCODE_OK;
            break;

        // Destination device: PC
        case ID_PC:

            *uart_tx_flag = 1;    // indicate that new data to send is available
            
            // save in fields of UART packet
            packets->uart_packet.sof       = SIG_SOF;
            packets->uart_packet.len       = HEADER_SHIFT_UART + packets->rf_packet.plen; // (header[6] + CRC[2]) + payload 
            packets->uart_packet.version   = packets->rf_packet.version;
            packets->uart_packet.flags     = packets->rf_packet.flags;
            packets->uart_packet.src_id    = packets->rf_packet.src_id;
            packets->uart_packet.dest_id   = packets->rf_packet.dest_id;
            packets->uart_packet.opcode    = packets->rf_packet.opcode;
            packets->uart_packet.plen      = packets->rf_packet.plen;

            for (int i = 0; i < packets->rf_packet.plen; i++)
            {
                packets->uart_packet.payload[i] = packets->rf_packet.payload[i];
            }

            return TRANSCODE_OK;
            break;

        // Destination device: broadcast
        case ID_BROADCAST:
            return TRANSCODE_BROADCAST;
            break;

        default:
            // Unavailable
            break;
    }


    return 0;
}



/*********************************************************************
 * @fn     UART_encode
 *
 * @param *packets: pointer to struct of all packets data
 * @param *raw_uart_data: pointer to raw data received over uart
 *
 * @brief   Encode data to be send over UART
 *
 * @return none
 */
void UART_encode(s_packets *packets, uint8_t *raw_uart_data)
{
    raw_uart_data[0]    = packets->uart_packet.sof;
    raw_uart_data[1]    = packets->uart_packet.len;
    raw_uart_data[2]    = packets->uart_packet.version;
    raw_uart_data[3]    = packets->uart_packet.flags;
    raw_uart_data[4]    = packets->uart_packet.src_id;
    raw_uart_data[5]    = packets->uart_packet.dest_id;
    raw_uart_data[6]    = packets->uart_packet.opcode;
    raw_uart_data[7]    = packets->uart_packet.plen;

    if (packets->uart_packet.plen > 0)
    {
        for (int i = 0; i < packets->uart_packet.plen; i++)
        {
            raw_uart_data[HEADER_SHIFT_UART + i] = packets->uart_packet.payload[i];
        }
    }
    
    uint8_t eof_num = (uint8_t)(HEADER_SHIFT_UART + packets->uart_packet.plen);

    uint16_t crc_temp = crc16_cal(&raw_uart_data[1], (uint16_t)eof_num);
    raw_uart_data[eof_num] = (uint8_t)(crc_temp);
    raw_uart_data[eof_num + 1] = (uint8_t)(crc_temp >> 8);
}



/*********************************************************************
 * @fn     RF_encode
 *
 * @param *packets: pointer to struct of all data packets 
 * @param *raw_rf_data: pointer to raw data you want to send
 *
 * @brief   Encode data to be send over RF
 *
 * @return  none
 */
void RF_encode(s_packets *packets, uint8_t *raw_rf_data, uint8_t *tx_lenght)
{
    raw_rf_data[0]    = packets->rf_packet.version;
    raw_rf_data[1]    = packets->rf_packet.flags;
    raw_rf_data[2]    = packets->rf_packet.src_id;
    raw_rf_data[3]    = packets->rf_packet.dest_id;
    raw_rf_data[4]    = packets->rf_packet.opcode;
    raw_rf_data[5]    = packets->rf_packet.plen;
    if (packets->rf_packet.plen > 0)
    {
        for (int i = 0; i < packets->rf_packet.plen; i++)
        {
            raw_rf_data[HEADER_SHIFT_RF + i] = packets->rf_packet.payload[i];
        }
    }

    *tx_lenght = HEADER_SHIFT_RF + packets->rf_packet.plen;
}



/*********************************************************************
 * @fn     crc16_cal
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
