##################################################################
# File Name          : protocol.py
# Description        : Packet format & CRC16 helpers for Amon Link
##################################################################

from __future__ import annotations

# Constants from your original main.h / application.py
SOF = 0xAA
ID_LINK_BOOT = 0x10
ID_LINK_SW = 0x11
ID_PC = 0x01
CMD_GET_INFO = 0x01
CMD_WRITE = 0x03
CMD_JUMP_APP = 0x05
CMD_END_OF_FW = 0x06
CMD_ACK = 0x80
CMD_ERR = 0x81
CODE_BOOT_VER = 0x02
CODE_SW_CRC = 0x02
CODE_DATA_WRITEN = 0x10
CODE_BAD_CRC = 0x01
CODE_EXIT_BOOT = 0x03
HEADER_SHIFT = 0x08


def crc16_cal(data: bytes | bytearray) -> int:
    """CRC16 calculation matching your C code (Modbus RTU-style)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_packet(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    """Build packet matching your modified C struct packet_t (with uint8_t plen)."""
    plen = 1 + 1 + len(payload) + 2
    if plen > 255:
        raise ValueError("Packet too long for uint8_t plen")

    packet = bytearray()
    packet.append(SOF)  # SOF
    packet.append(plen & 0xFF)  # Packet length
    packet.append(addr)  # Address
    packet.append(cmd)  # Command
    packet.extend(payload)

    crc_data = packet[1 : 1 + plen]  # plen + addr + cmd + payload
    crc = crc16_cal(crc_data)
    packet.append(crc & 0xFF)  # Low byte
    packet.append((crc >> 8) & 0xFF)  # High byte
    return bytes(packet)


def parse_packet(frame: bytes) -> tuple[int, int, bytes]:
    """Parse received packet matching your modified C struct."""
    if len(frame) < 6:
        raise ValueError("Frame too short")
    if frame[0] != SOF:
        raise ValueError("Invalid SOF")

    plen = frame[1]
    expected_length = 1 + 1 + plen  # SOF + plen + data
    if len(frame) != expected_length:
        raise ValueError(f"Frame length mismatch: expected {expected_length}, got {len(frame)}")

    addr = frame[2]
    cmd = frame[3]
    payload_length = plen - 4
    payload = frame[4 : 4 + payload_length] if payload_length > 0 else b""

    received_crc = frame[-2] | (frame[-1] << 8)
    crc_data = frame[1:-2]
    calculated_crc = crc16_cal(crc_data)
    if received_crc != calculated_crc:
        raise ValueError(
            f"CRC mismatch: received {hex(received_crc)}, calculated {hex(calculated_crc)}"
        )

    return addr, cmd, payload


__all__ = [
    "SOF",
    "ID_LINK_BOOT",
    "ID_LINK_SW",
    "ID_PC",
    "CMD_GET_INFO",
    "CMD_WRITE",
    "CMD_JUMP_APP",
    "CMD_END_OF_FW",
    "CMD_ACK",
    "CMD_ERR",
    "CODE_BOOT_VER",
    "CODE_SW_CRC",
    "CODE_DATA_WRITEN",
    "CODE_BAD_CRC",
    "CODE_EXIT_BOOT",
    "HEADER_SHIFT",
    "crc16_cal",
    "build_packet",
    "parse_packet",
]

