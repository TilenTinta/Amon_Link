/*****************************************************************
 * File Name          : dataDecoder.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2025/03/09
 * Description        : Decoding logic for received and transmited data
*****************************************************************/

#ifndef USER_DATADECODER_DATADECODER_H_
#define USER_DATADECODER_DATADECODER_H_

// Communication
#define PACKET_SIZE 16                  // size of received-send packet (SPI-UART DMA)

// Communication - USB and RF //
// - Byte 0: Address and command
//   [8-4: address]
#define B0_1_NULL           0b0000      // Reserved / null
#define B0_1_PC             0b0001      // PC
#define B0_1_LINK           0b0010      // Link (0011 is still link no. 2)
#define B0_1_DRONE          0b0100      // Drone (all higher numbers can be other drones)

//   [4-0: code of the command]
#define B0_0_NULL           0b0000      // Reserved / null
#define B0_0_PING           0b0001      // (Link/Drone) Check connection / status report (ping)
#define B0_0_ERR            0b0010      // (Link/Drone) Error report - it depands on address
#define B0_0_LINK_CONN      0b0100      // (Link) Pair status (link to drone)
#define B0_0_LINK_PAIR      0b0101      // (Link) Start pairing (if not allready paired)
#define B0_0_LINK_PARAM     0b0110      // (Link) Current parameters
#define B0_0_LINK_MOD       0b0111      // (Link) Change parameters on link
#define B0_0_DRONE_PARAM    0b1000      // (Drone) Current parameters
#define B0_0_DRONE_MOD      0b1001      // (Drone) Change parameters on drone
#define B0_0_DRONE_STATUS   0b1010      // (Drone) Change status on drone
#define B0_0_DRONE_TASK     0b1011      // (Drone) Drone commands (calibrate, save to SD...)
#define B0_0_DRONE_TELEM    0b1100      // (Drone) Real time data - telemetry

// - Byte 1: Values and parameters







// Defines - error report
#define ERROR_NaN           0           // Error code - no error
#define ERROR_BOOT          1           // Error code - on boot
#define ERROR_RADIO_1       2           // Error code - problem with radio 1
#define ERROR_RADIO_2       3           // Error code - problem with radio 2
#define ERROR_DRONE         4           // Error code - indicate problem with drone if cause is unknown   

/*###########################################################################################################################################################*/
/* Functions */





#endif /* USER_DATADECODER_DATADECODER_H_ */
