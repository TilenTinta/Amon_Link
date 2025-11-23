##################################################################
# File Name          : application.py
# Author             : Tinta T.
# Version            : V1.0.0
# Date               : 2025/11/09
# Description        : Terminal application for updating firmware 
#                      over UART (USB)
##################################################################

import sys
import queue
import time
from serial_comm import SerialComm

# Constants from your main.h
SOF = 0xAA
ID_LINK = 0x10
ID_PC = 0x01
CMD_GET_INFO = 0x01
HEADER_SHIFT = 0x08

def crc16_cal(data):
    """CRC16 calculation matching your C code (Modbus RTU-style)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def build_packet(addr, cmd, payload=b""):
    """Build packet matching your modified C struct packet_t (with uint8_t plen)"""
    # Calculate packet length: addr(1) + cmd(1) + payload + crc16(2)
    plen = 1 + 1 + len(payload) + 2
    
    if plen > 255:
        raise ValueError("Packet too long for uint8_t plen")
    
    # Build packet
    packet = bytearray()
    
    # SOF (1 byte)
    packet.append(SOF)
    
    # Packet length (1 byte - since you changed to uint8_t)
    packet.append(plen & 0xFF)
    
    # Address (1 byte)
    packet.append(addr)
    
    # Command (1 byte)
    packet.append(cmd)
    
    # Payload
    packet.extend(payload)
    
    # Calculate CRC over: plen + addr + cmd + payload
    crc_data = packet[1:1 + plen]  # From plen byte to end of payload
    crc = crc16_cal(crc_data)
    
    # Append CRC (2 bytes - little endian)
    packet.append(crc & 0xFF)        # Low byte
    packet.append((crc >> 8) & 0xFF) # High byte
    
    return bytes(packet)

def parse_packet(frame):
    """Parse received packet matching your modified C struct"""
    if len(frame) < 6:  # Minimum packet size: SOF(1) + PLEN(1) + ADDR(1) + CMD(1) + CRC(2)
        raise ValueError("Frame too short")
    
    if frame[0] != SOF:
        raise ValueError("Invalid SOF")
    
    # Extract packet length (1 byte - since uint8_t)
    plen = frame[1]
    
    # Verify frame length
    expected_length = 1 + 1 + plen  # SOF + plen(1) + data(plen bytes)
    if len(frame) != expected_length:
        raise ValueError(f"Frame length mismatch: expected {expected_length}, got {len(frame)}")
    
    # Extract fields
    addr = frame[2]
    cmd = frame[3]
    
    # Payload length = plen - (addr + cmd + crc) = plen - 4
    payload_length = plen - 4
    payload = frame[4:4 + payload_length] if payload_length > 0 else b""
    
    # Extract CRC (last 2 bytes - little endian)
    received_crc = frame[-2] | (frame[-1] << 8)
    
    # Calculate CRC for verification (over plen + addr + cmd + payload)
    crc_data = frame[1:-2]
    calculated_crc = crc16_cal(crc_data)
    
    if received_crc != calculated_crc:
        raise ValueError(f"CRC mismatch: received {hex(received_crc)}, calculated {hex(calculated_crc)}")
    
    return addr, cmd, payload

def main():
    if len(sys.argv) != 2:
        print("Usage: python application.py <COM_PORT>")
        sys.exit(1)

    port = sys.argv[1]
    resp_q = queue.Queue()
    comm = SerialComm(port, baudrate=115200, response_queue=resp_q)
    
    try:
        comm.open()
        print(f"Opened {port} at 115200 baud")

        # Build CMD_GET_INFO packet for LINK device
        packet = build_packet(ID_LINK, CMD_GET_INFO)
        print(f"Sending CMD_GET_INFO packet: {packet.hex()}")
        print(f"Packet structure: SOF={hex(packet[0])}, PLEN={hex(packet[1])}, ADDR={hex(packet[2])}, CMD={hex(packet[3])}")
        print(f"Full packet bytes: {[hex(b) for b in packet]}")
        
        comm.send_bytes(packet)

        try:
            resp = resp_q.get(timeout=2.0)
        except queue.Empty:
            print("Timeout waiting for response")
            return

        if isinstance(resp, tuple) and resp[0] == "DISCONNECT":
            print("Serial disconnected:", resp[1])
            return

        print(f"Received raw frame: {resp.hex()}")
        
        try:
            addr, cmd, payload = parse_packet(resp)
            print(f"Parsed - Addr: {hex(addr)}, Cmd: {hex(cmd)}, Payload: {payload.hex()}")
            
            # Interpret response
            if cmd == 0x80:  # CMD_ACK
                if payload and len(payload) >= 1:
                    if payload[0] == 0x02:  # CODE_SW_VER
                        if len(payload) >= 2:
                            print(f"Bootloader version: {payload[1]}")
                        else:
                            print("ACK: Software version request")
                    else:
                        print(f"ACK with code: {hex(payload[0])}")
            elif cmd == 0x81:  # CMD_ERR
                print(f"Error response with code: {hex(payload[0]) if payload else 'unknown'}")
                
        except ValueError as e:
            print(f"Error parsing response: {e}")

    finally:
        comm.close()
        print("Closed serial port")

if __name__ == "__main__":
    main()