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

/* --------------------- UART ---------------------
 * + Relation: PC - Link
 * + Framing: SOF | LEN | HEADER | PAYLOAD | CRC16.
 * + Layout (little-endian for multibyte fields):
 *	  - SOF    (1B): 0xAA
 *	  - LEN    (1B): number of bytes in HEADER+PAYLOAD (0..255)
 *	  - HEADER (6B):
 *		- VER    (1B) : protocol version (start with 0x01)
 *		- FLAGS  (1B) : bitfield (see 4)
 *		- SRC    (1B) : logical source ID
 *		- DST    (1B) : logical destination ID
 *		- OPCODE (1B) : command (see 5)
 *		- PLEN   (1B) : payload length (0..N)
 *	  - PAYLOAD (PLEN bytes)
 *	  - CRC16  (2B): CRC-16/CCITT (poly 0x1021, init 0xFFFF) over LEN..PAYLOAD
 */


/* --------------------- RF ---------------------
 * + Relation: Link - Drone
 * + Compact layout (fits nRF DPL ¡Ü 32):
 *      - VER(1) | FLAGS(1) | SRC(1) | DST(1) | OPCODE(1) | PLEN(1) | PAYLOAD(0..26)
 */

/*###########################################################################################################################################################*/

// Driver defines
#define DRIVER_VER              0x01    // Version of current driver

// Entities / IDs (1 byte)
#define ID_PC                   0x01    // Address: PC
#define ID_LINK_BOOT            0x10    // Address: Link board - bootloader
#define ID_LINK_SW              0x11    // Address: Link board - software
#define ID_DRONE                0x20    // Address: Drone (can be multiple)
#define ID_BROADCAST            0xFF    // Address: Broadcast

// Signaling (1 byte)
#define HEADER_SHIFT            0x08    // Number of bytes before payload
#define SIG_SOF                 0xAA    // Start-Of-Frame

// Flags (1 byte) (bit on this place is set or not)
#define FLAG_ACK                0x01    // frame is a reply
#define FLAG_ERR                0x02    // processing error; payload carries error code
#define FLAG_STREAM             0x03    // telemetry stream frame
#define FLAG_FRAG               0x04    // fragmented payload; optional future use
#define FLAG_NaN                0xF0    // bit4..7: reserved

// Opcodes (1 byte)
#define OPT_NOP                 0x00    // NOP / Reserved
#define OPT_PING                0x01    // PING (req) / PONG (reply with status TLVs)
#define OPT_ERROR_REPORT	    0x02    // Device error report (payload: error code + info)
#define OPT_PAIR_STATUS         0x10    // Status of pairing procedure
#define OPT_PAIR_START          0x11    // Trigger pairing procedure
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


/*###########################################################################################################################################################*/
/* Structs */

// Struct where each packet get saved
typedef struct {
    uint8_t     sof;                    // Start of frame
    uint8_t     len;                    // Lenght of the packet
    uint8_t     version;                // Version (only for logic)
    uint8_t     flags;                  // Flags of packet
    uint8_t     src_id;                 // Source address
    uint8_t     dist_id;                // Destination address
    uint8_t     opcode;                 // Code for the command
    uint8_t     plen;                   // Payload lenght
    uint16_t    CRC;                    // CRC calculated over len-payload
    uint8_t     payload[];              // Payload data [must be last to allow scaling]
} UART_PACKET;


/*###########################################################################################################################################################*/
/* Functions */
void UART_decode(uint8_t* raw_uart_data, uint8_t* raw_rf_data, uint8_t* rf_tx_flag);




#endif /* USER_DATADECODER_DATADECODER_H_ */
