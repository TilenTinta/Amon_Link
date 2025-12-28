import argparse
import sys
import time
from pathlib import Path

import serial
import zlib

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from application.protocol import (  # noqa: E402
    CMD_ACK,
    CMD_END_OF_FW,
    CMD_GET_INFO,
    CMD_WRITE,
    CODE_BOOT_VER,
    CODE_SW_CRC,
    ID_LINK_BOOT,
    SOF,
    build_packet,
    parse_packet,
)


def _decode_info_payload(payload: bytes) -> dict[str, str] | None:
    if len(payload) < 7:
        return None
    if payload[0] != CODE_BOOT_VER or payload[2] != CODE_SW_CRC:
        return None
    boot_ver = payload[1]
    device_crc32 = int.from_bytes(payload[3:7], "big")
    return {
        "code_boot_ver": f"0x{payload[0]:02X}",
        "boot_ver": f"0x{boot_ver:02X}",
        "code_sw_crc": f"0x{payload[2]:02X}",
        "device_crc32": f"0x{device_crc32:08X}",
    }


def _read_frames(ser: serial.Serial, listen_s: float) -> list[bytes]:
    frames: list[bytes] = []
    buf = bytearray()
    deadline = time.time() + listen_s
    while time.time() < deadline:
        data = ser.read(1)
        if data:
            buf.extend(data)
            while len(buf) >= 4:
                if buf[0] != SOF:
                    buf.pop(0)
                    continue
                if len(buf) < 2:
                    break
                plen = buf[1]
                total_len = 1 + 1 + plen
                if len(buf) < total_len:
                    break
                frame = bytes(buf[:total_len])
                buf = buf[total_len:]
                frames.append(frame)
        else:
            time.sleep(0.01)
    return frames


def _hex_to_bin(data: bytes, start_addr: int | None = None) -> bytes:
    base_addr = 0
    chunks: dict[int, bytes] = {}
    min_addr: int | None = None
    max_addr = 0

    for raw_line in data.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line[:1] != b":":
            raise ValueError("Invalid HEX line start")
        try:
            byte_count = int(line[1:3], 16)
            address = int(line[3:7], 16)
            record_type = int(line[7:9], 16)
            payload = bytes.fromhex(line[9:9 + byte_count * 2].decode("ascii"))
        except Exception as exc:  # noqa: BLE001
            raise ValueError("Malformed HEX record") from exc

        if record_type == 0x00:
            abs_addr = base_addr + address
            if start_addr is not None:
                if abs_addr + len(payload) <= start_addr:
                    continue
                if abs_addr < start_addr:
                    offset = start_addr - abs_addr
                    payload = payload[offset:]
                    abs_addr = start_addr
            if not payload:
                continue
            chunks[abs_addr] = payload
            if min_addr is None or abs_addr < min_addr:
                min_addr = abs_addr
            max_addr = max(max_addr, abs_addr + len(payload))
        elif record_type == 0x01:
            break
        elif record_type == 0x04:
            if len(payload) != 2:
                raise ValueError("Invalid HEX extended address record")
            base_addr = int.from_bytes(payload, "big") << 16
        else:
            continue

    if not chunks or min_addr is None:
        return b""
    span = max_addr - min_addr
    blob = bytearray(b"\xFF" * span)
    for addr, payload in chunks.items():
        offset = addr - min_addr
        blob[offset:offset + len(payload)] = payload
    return bytes(blob)


def _crc32_hex(data: bytes) -> str:
    return f"0x{zlib.crc32(data) & 0xFFFFFFFF:08X}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Serial probe for Amon Link")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen", type=float, default=2.0)
    parser.add_argument("--dtr", action="store_true", help="Set DTR on open")
    parser.add_argument("--rts", action="store_true", help="Set RTS on open")
    parser.add_argument("--hex", help="Path to Intel HEX file to build write packet")
    parser.add_argument("--start-addr", type=lambda x: int(x, 0))
    parser.add_argument("--write", action="store_true", help="Send CMD_WRITE packet")
    parser.add_argument("--end", action="store_true", help="Send CMD_END_OF_FW packet")
    parser.add_argument("--crc32", type=lambda x: int(x, 0), help="CRC32 value for end packet")
    parser.add_argument(
        "--skip-info",
        action="store_true",
        help="Skip sending the initial GET_INFO packet",
    )
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}")
        return 1

    ser.dtr = args.dtr
    ser.rts = args.rts
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"Opened {args.port} @ {args.baud}")
    if not args.skip_info:
        packet = build_packet(ID_LINK_BOOT, CMD_GET_INFO)
        print(f"TX {packet.hex()}")
        ser.write(packet)

    if args.hex:
        hex_path = Path(args.hex)
        if not hex_path.exists():
            print(f"HEX file not found: {hex_path}")
        else:
            data = _hex_to_bin(hex_path.read_bytes(), start_addr=args.start_addr)
            if not data:
                print("HEX data is empty after trimming")
            else:
                chunk = data[:64]
                if len(chunk) < 64:
                    chunk = chunk + b"\xFF" * (64 - len(chunk))
                write_packet = build_packet(ID_LINK_BOOT, CMD_WRITE, payload=chunk)
                print(f"WRITE payload {chunk.hex()}")
                print(f"WRITE packet {write_packet.hex()}")
                if args.write:
                    ser.write(write_packet)

    if args.end:
        crc32_value = args.crc32
        if crc32_value is None and args.hex:
            hex_path = Path(args.hex)
            if not hex_path.exists():
                print(f"HEX file not found: {hex_path}")
            else:
                data = _hex_to_bin(hex_path.read_bytes(), start_addr=args.start_addr)
                if not data:
                    print("HEX data is empty after trimming")
                else:
                    crc32_value = int(_crc32_hex(data), 16)
                    print(f"Computed CRC32 {_crc32_hex(data)}")
        if crc32_value is None:
            print("Missing --crc32 for end packet (or provide --hex)")
        else:
            end_payload = bytes(
                [
                    0x00,
                    (crc32_value >> 24) & 0xFF,
                    (crc32_value >> 16) & 0xFF,
                    (crc32_value >> 8) & 0xFF,
                    crc32_value & 0xFF,
                ]
            )
            end_packet = build_packet(ID_LINK_BOOT, CMD_END_OF_FW, payload=end_payload)
            print(f"END payload {end_payload.hex()}")
            print(f"END packet {end_packet.hex()}")
            ser.write(end_packet)

    frames = _read_frames(ser, args.listen)
    if not frames:
        print("No frames received")
        ser.close()
        return 2

    for frame in frames:
        print(f"RX {frame.hex()}")
        try:
            addr, cmd, payload = parse_packet(frame)
            print(f"Parsed addr={hex(addr)} cmd={hex(cmd)} payload={payload.hex()}")
            if cmd == CMD_ACK:
                info = _decode_info_payload(payload)
                if info:
                    print("Decoded fields:")
                    for key, value in info.items():
                        print(f"  {key}: {value}")
        except Exception as exc:  # noqa: BLE001
            print(f"Parse error: {exc}")

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
