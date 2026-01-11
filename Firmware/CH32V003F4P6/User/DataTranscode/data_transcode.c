/*****************************************************************
 * File Name          : dataDecoder.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/09
 * Description        : Decoding logic for received and transmited data
*****************************************************************/

#include "data_transcode.h"

/* Structs */
s_uart_packet uart_packet;      // UART packet - application format
s_boot_packet boot_packet;      // UART packet - bootloader format


/* Local function */
void UART_packet_parse(s_uart_packet *data, s_boot_packet *boot, uint8_t *raw_data);
void UART_packetDataReset(void);
//uint16_t crc16_cal(const uint8_t *data, uint16_t length);




 /*********************************************************************
 * @fcn     UART_packet_parse
 *
 * @param *data: pointer containing uart data structure
 * @param *boot: pointer containing bootloader data structure
 * @param *raw_data: pointer to raw data received over uart
 *
 * @brief   Spliting packet in to information parts
 *
 * @return  none
 */

void UART_packet_parse(s_uart_packet *data, s_boot_packet *boot, uint8_t *raw_data)
{
    data->sof       = *raw_data;
    data->len       = *(raw_data + 1);

    // Check if packet is bootloader format -> if it is decode different
    // Example: 
        //      Packet structure: SOF=0xaa, PLEN=0x4, ADDR=0x10, CMD=0x1, PAYLOAD=0xXX, CRC16=0x1234
        //      CRC16: calculated over entire packet excluding SOF and CRC16 itself
        //      PLEN: does not count itself in its value
        //      Full packet bytes: ['0xaa', '0x4', '0x10', '0x1', '0xfc', '0x1']
    if (data->len == 4) 
    {   
        boot->sof = data->sof;
        boot->plen = data->len;               // Without CRC16 (and SOF)
        boot->addr = *(raw_data + 2);
        boot->cmd = *(raw_data + 3);

        // if (data->plen > 4)                // 4 bytes does not have payload (only: ADDR, CMD, CRC16_1, CRC16_2)
        // {
        //     for (int i = 0; i < data->plen - 4; i++)
        //     {
        //         data->payload[i] = *(raw_data + HEADER_SHIFT + i);
        //     }
        // }

        uint16_t crc1_tmp = (uint16_t)*(raw_data + data->len);
        uint16_t crc2_tmp = (uint16_t)*(raw_data + data->len + 1) << 8;
        boot->crc16 = crc1_tmp | crc2_tmp;

        return;
    }

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
 * @fcn     UART_decode
 *
 * @param *raw_uart_data: pointer to raw data received over uart
 * @param *raw_rf_data: pointer to struct of rf data
 * @param *rf_tx_flag: pointer to flag for new rf data avalable to transmit
 *
 * @brief   Initialises the GPIOs
 *
 * @return  error code - watch defines
 */
uint8_t UART_decode(uint8_t *raw_uart_data, uint8_t *raw_rf_data, uint8_t *rf_tx_flag)
{
    // Parse received buffer
    UART_packetDataReset();
    UART_packet_parse(&uart_packet, &boot_packet, &raw_uart_data[0]);

    // CRC check
    if (boot_packet.plen != 0)
    {
        // Bootloader format
        if (crc16_cal(&raw_uart_data[1], boot_packet.plen - 1) != boot_packet.crc16) return TRANSCODE_CRC_ERR;
        if (boot_packet.addr == ID_LINK_BOOT) return TRANSCODE_BOOT_PKT;
    }
    else 
    {
        // UART format
        if (crc16_cal(&raw_uart_data[1], uart_packet.len - 1) != uart_packet.CRC) return TRANSCODE_CRC_ERR;
    }

    // Check version
    if (uart_packet.version != PROTOCOL_VER) return TRANSCODE_VER_ERR;

    // Check destination address
    switch (uart_packet.dist_id) 
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
 * @fcn     UART_encode
 *
 * @param *raw_uart_data: pointer to raw data received over uart
 * @param *raw_rf_data: pointer to struct of rf data
 * @param *rf_tx_flag: pointer to flag for new rf data avalable to transmit
 *
 * @brief   Initialises the GPIOs
 *
 * @return  error code - watch defines
 */
uint8_t UART_encode(uint8_t *raw_uart_data, uint8_t *raw_rf_data, uint8_t *rf_tx_flag)
{

    return 0;
}




