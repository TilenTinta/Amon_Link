/*****************************************************************
 * File Name          : dataDecoder.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/09
 * Description        : Decoding logic for received and transmited data
*****************************************************************/

#ifndef USER_DATADECODER_DATADECODER_H_
#define USER_DATADECODER_DATADECODER_H_

/* Include */
#include <stdio.h>

/* --- Quick description of comm protocol --- */

/* --------------------- UART ---------------------
 * + Relation: PC - Link
 * + Basic framing: SOF | LEN | HEADER | PAYLOAD | CRC16.
 * + Layout (little-endian for multibyte fields):
 *	  - SOF    (1B): 0xAA
 *	  - LEN    (1B): number of bytes in HEADER...PAYLOAD (0..255) - LEN not included
 *	  - HEADER (6B):
 *		- VER    (1B) : protocol version (start with 0x01)
 *		- FLAGS  (1B) : bitfield - "type of frame"
 *		- SRC    (1B) : source address
 *		- DST    (1B) : destination address
 *		- OPCODE (1B) : command
 *		- PLEN   (1B) : payload length (0..N)
 *	  - PAYLOAD (PLEN bytes)
 *	  - CRC16  (2B): CRC-16/CCITT (poly 0x1021, init 0xFFFF), calculated over LEN...PAYLOAD
 */


/* --------------------- RF ---------------------
 * + Relation: Link - Drone
 * + Compact layout (fits nRF DPL ¡Ü 32):
 *      - VER(1) | FLAGS(1) | SRC(1) | DST(1) | OPCODE(1) | PLEN(1) | PAYLOAD(0..26)
 */

/*###########################################################################################################################################################*/

// Driver defines
#define PROTOCOL_VER            0x01    // Version of current driver
#define BOOT_VER                0x01    // Version of software - botloader
#define HEADER_SHIFT_UART       0x08    // Number of bytes before payload - used in code
#define HEADER_SHIFT_RF         0x06    // Number of bytes before payload - used in code

// Signaling (1 byte)
#define SIG_SOF                 0xAA    // Start-Of-Frame

// BOOTLOADER - Commands/Responses (1 byte)
#define CMD_INFO                0x01    // e.g. bootloader version [CMD_READ]
#define CMD_WRITE               0x03    // Write a chunk: [addr(4B) + data...]
#define CMD_ERASE               0x02    // Erase application area (0x08001000 .. end)
#define CMD_VERIFY              0x04    // Optional: verify CRC of whole app
#define CMD_JUMP_APP            0x05    // Jump to application
#define CMD_END_OF_FW           0x06    // End of firmware update, jump to application
#define CMD_ACK                 0x80    // Response - Replies with info
#define CMD_ERR                 0x81    // Response - Error reply with error code           

// BOOTLOADER - Codes/payload (1 byte)
#define CODE_BAD_CRC            0x01    // calculated CRC and received CRC doesnt match
#define CODE_BOOT_VER           0x02    // Return bootloader version
#define CODE_SW_CRC             0x02    // Return current CRC of main firmware
#define CODE_EXIT_BOOT          0x03    // Exiting bootloader, jumping to application
#define CODE_DATA_WRITEN        0x10    // Succesfuly writen data packet

// TRANSCODE RETURN
#define TRANSCODE_NaN           0x00    // No error / no response / not used
#define TRANSCODE_CRC_ERR       0x01    // CRC error
#define TRANSCODE_VER_ERR       0x02    // Packet version error
#define TRANSCODE_DEST_ERR      0x03    // Destination address error
#define TRANSCODE_BROADCAST     0x04    // Broadcast command
#define TRANSCODE_DEST_LINK     0x05    // Link command
#define TRANSCODE_DEST_RF       0x06    // Packet for RF transmition
#define TRANSCODE_DEST_PC       0x07    // Packet for PC-UART transmition
#define TRANSCODE_BOOT_PKT      0x0A    // Bootloader packet received

// Address / IDs (1 byte)
#define ID_PC                   0x01    // Address: PC
#define ID_LINK_BOOT            0x10    // Address: Link board - bootloader
#define ID_LINK_SW              0x11    // Address: Link board - software
#define ID_DRONE                0x20    // Address: Drone (can be multiple)
#define ID_BROADCAST            0xFF    // Address: Broadcast

// Flags (1 byte) - combined with Opcodes
#define FLAG_ACK                0x01    // frame is a reply
#define FLAG_ERR                0x02    // processing error; payload carries error code
#define FLAG_STREAM             0x03    // telemetry stream frame
#define FLAG_FRAG               0x04    // fragmented payload; optional future use
#define FLAG_DATA               0x05    // data frame - non telemetry data
#define FLAG_NaN                0xF0    // bit4..7: reserved

// Opcodes (1 byte) - combined with Flags
#define OPT_NOP                 0x00    // NOP / Reserved
#define OPT_PING                0x01    // PING (req) / PONG (reply with status TLVs)
#define OPT_ERROR_REPORT	    0x02    // Device error report (payload: error code + info)
#define OPT_ERROR_TX            0x03    // Error when transmiting packet
#define OPT_PAIR_STATUS         0x10    // Status of pairing procedure
#define OPT_PAIR_START          0x11    // Trigger pairing procedure
#define OPT_TELEMETRY_STREAM    0x12    // Trigger telemetry data stream
#define OPT_LINK_GET_PARAMS     0x20    // Get parameters from link device
#define OPT_LINK_SET_PARAMS     0x21    // Set parameters on link device
#define OPT_DRONE_GET_PARAMS    0x30    // Get parameters from drone
#define OPT_DRONE_SET_PARAMS    0x31    // Set parameters on drone
#define OPT_DRONE_SET_STATE 	0x32    // Change rurrent state of drone (arm/disarm/modes)
#define OPT_DRONE_COMMAND       0x33    // Send command to drone (calibrate, save¡­)
#define OPT_TELEMETRY           0x40    // Telemetry data from drone (STREAM; sub-type via TLVs)

// Payload format ¡ª TLV (Type-Length-Value): T(1B), L(1B), V(L bytes)
#define TVL_BAT                 0x01    // Battery mV (u16 LE)
#define TVL_RSSI                0x02    // RSSI/Link quality (u8)
#define TVL_FW_VER              0x03    // FW version (ascii)
#define TVL_RF_CH               0x10    // RF Channel (u8)
#define TVL_ADDR_LEN            0x11    // Address length (u8=3/4/5)
#define TVL_DATA_RATE           0x12    // Data rate (u8: 0=250kbps,1=1Mbps,2=2Mbps)
#define TVL_PWR_LVL             0x13    // Power level (u8)
#define TVL_DRONE_MODE          0x20    // Drone mode (u8)
#define TLV_CALIB_TARG          0x21    // Calib target (u8)
#define TVL_ALT_ANGL            0x30    // Attitude roll/pitch/yaw (i16 each, deg*100)
#define TVL_IMU                 0x31    // IMU raw ax/ay/az,gx/gy/gz (i16 each)
#define TVL_GPS                 0x32    // GPS lat(i32 1e-7deg), lon(i32), alt_cm(i32)
#define TVL_ERR                 0x7F    // Error code (u8) + detail (optional ascii)
// TODO: TBD

/*###########################################################################################################################################################*/
/* Structs */

typedef struct {
    uint8_t     sof;                    // Start of frame
    uint8_t     len;                    // Lenght of the packet
    uint8_t     version;                // Version (only for logic)
    uint8_t     flags;                  // Flags of packet
    uint8_t     src_id;                 // Source address
    uint8_t     dest_id;                // Destination address
    uint8_t     opcode;                 // Code for the command
    uint8_t     plen;                   // Payload lenght
    uint16_t    CRC;                    // CRC calculated over len-payload
    uint8_t     payload[64];            // Payload data

} s_uart_packet;


typedef struct {
    uint8_t sof;                        // Start of frame
    uint8_t plen;                       // Packet lenght
    uint8_t addr;                       // Address of targeted device
    uint8_t cmd;                        // Command
    // uint8_t payload[64];                // Payload data (64bytes is one page)                            
    uint16_t crc16;                     // CRC calculated over: plen + addr + cmd + payload

} s_boot_packet;


typedef struct {
    uint8_t     version;                // Version (only for logic)
    uint8_t     flags;                  // Flags of packet
    uint8_t     src_id;                 // Source address
    uint8_t     dest_id;                // Destination address
    uint8_t     opcode;                 // Code for the command
    uint8_t     plen;                   // Payload lenght
    uint8_t     payload[26];            // Payload data [32-6]

} s_rf_packet;


typedef struct {
    uint8_t     version;                // Version (only for logic)
    uint8_t     flags;                  // Flags of packet
    uint8_t     src_id;                 // Source address
    uint8_t     dest_id;                // Destination address
    uint8_t     opcode;                 // Code for the command
    uint8_t     plen;                   // Payload lenght
    uint8_t     payload[26];            // Payload data [32-6]

} s_rf_packet_drone;


typedef struct {
    s_boot_packet       boot_packet;
    s_rf_packet         rf_packet;
    s_rf_packet_drone 	rf_packet_drone;
    s_uart_packet       uart_packet;

} s_packets;


/*###########################################################################################################################################################*/
/* Functions */
uint8_t UART_decode(uint8_t *raw_uart_data, s_packets* packets, uint8_t *rf_tx_flag);
uint8_t RF_decode(uint8_t *raw_rf_data, s_packets *packets, uint8_t *uart_tx_flag, uint8_t *stream_flag);
void UART_encode(s_packets *packets, uint8_t *raw_uart_data);
void RF_encode(s_packets *packets, uint8_t *raw_rf_data, uint8_t *tx_lenght);
uint16_t crc16_cal(const uint8_t *data, uint16_t length);




#endif /* USER_DATADECODER_DATADECODER_H_ */
