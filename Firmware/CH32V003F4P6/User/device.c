/*****************************************************************
 * File Name          : device.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/01/21
 * Description        : Device function to set or report values
*****************************************************************/

#include "device.h"


/*###########################################################################################################################################################*/
/* Functions */

/*********************************************************************
 * @fn      UART_packetDataReset
 *
 * @brief   Clear data in all packet structs
 *
 * @return  none
 */
void UART_report_err_tx(s_packets *data)
{
    
    uint8_t temp_buff[] = {};
    data->uart_packet.sof       = SIG_SOF;
    data->uart_packet.len       = 8;
    temp_buff[0] = 8;
    data->uart_packet.version   = PROTOCOL_VER;
    temp_buff[1] = PROTOCOL_VER; 
    data->uart_packet.flags     = FLAG_ERR;
    temp_buff[2] = FLAG_ERR;
    data->uart_packet.src_id    = ID_LINK_SW;
    temp_buff[3] = ID_LINK_SW;
    data->uart_packet.dest_id   = ID_PC;
    temp_buff[4] = ID_PC;
    data->uart_packet.opcode    = OPT_ERROR_TX;
    temp_buff[5] = OPT_ERROR_TX;
    data->uart_packet.plen      = 0;
    temp_buff[6] = 0;

    // CRC calculation
    uint16_t crc_temp = crc16_cal(temp_buff, sizeof(temp_buff));
    uint16_t crc1_tmp = (uint8_t)(crc_temp);
    uint16_t crc2_tmp = (uint8_t)(crc_temp >> 8);

    data->uart_packet.CRC = crc1_tmp | crc2_tmp;
}


/*********************************************************************
 * @fn      UART_device_set_get
 *
 * @brief   Clear data in all packet structs
 *
 * @return  none
 */
void UART_device_set_get(s_packets *data)
{
    
    // uint8_t temp_buff[] = {};
    // data->uart_packet.sof       = SIG_SOF;
    // data->uart_packet.len       = 8;
    // temp_buff[0] = 8;
    // data->uart_packet.version   = PROTOCOL_VER;
    // temp_buff[1] = PROTOCOL_VER; 
    // data->uart_packet.flags     = FLAG_ERR;
    // temp_buff[2] = FLAG_ERR;
    // data->uart_packet.src_id    = ID_LINK_SW;
    // temp_buff[3] = ID_LINK_SW;
    // data->uart_packet.dest_id   = ID_PC;
    // temp_buff[4] = ID_PC;
    // data->uart_packet.opcode    = OPT_ERROR_TX;
    // temp_buff[5] = OPT_ERROR_TX;
    // data->uart_packet.plen      = 0;
    // temp_buff[6] = 0;

    // // CRC calculation
    // uint16_t crc_temp = crc16_cal(temp_buff, sizeof(temp_buff));
    // uint16_t crc1_tmp = (uint8_t)(crc_temp);
    // uint16_t crc2_tmp = (uint8_t)(crc_temp >> 8);

    // data->uart_packet.CRC = crc1_tmp | crc2_tmp;
}