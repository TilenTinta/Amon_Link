import queue
import threading
import time
from pathlib import Path
from typing import Optional

import serial.tools.list_ports

from .protocol import (
    CMD_ACK,
    CMD_ERR,
    CMD_GET_INFO,
    CODE_BAD_CRC,
    CODE_DATA_WRITEN,
    ID_LINK_BOOT,
    build_packet,
    parse_packet,
)
from .serial_comm import SerialComm


class SerialBackend:
    """Thread-safe wrapper around the existing SerialComm for UI use."""

    def __init__(self):
        self._lock = threading.RLock()
        self._resp_q: queue.Queue[bytes | tuple[str, str]] = queue.Queue()
        self._comm: Optional[SerialComm] = None
        self.connected_port: Optional[str] = None
        self.baudrate: int = 115200

    @staticmethod
    def list_ports() -> list[str]:
        """Return a list of available serial port device names."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port: str, baudrate: int) -> None:
        """Open the serial connection."""
        with self._lock:
            self.disconnect()
            self._resp_q = queue.Queue()
            self._comm = SerialComm(port, baudrate=baudrate, response_queue=self._resp_q)
            self._comm.open()
            self.connected_port = port
            self.baudrate = baudrate

    def disconnect(self) -> None:
        """Close the serial connection if open."""
        with self._lock:
            if self._comm:
                self._comm.close()
            self._comm = None
            self.connected_port = None

    def is_connected(self) -> bool:
        return self._comm is not None

    def send_get_info(self) -> bytes:
        """Send the GET_INFO command using the existing packet format."""
        if not self._comm:
            raise RuntimeError("Serial port not open")
        packet = build_packet(ID_LINK_BOOT, CMD_GET_INFO)
        self._comm.send_bytes(packet)
        return packet

    def send_bytes(self, payload: bytes) -> None:
        if not self._comm:
            raise RuntimeError("Serial port not open")
        self._comm.send_bytes(payload)

    def wait_for_ack(
        self,
        *,
        expected_code: int = CODE_DATA_WRITEN,
        expected_crc16: int | None = None,
        timeout_s: float = 2.0,
    ) -> bool:
        if not self._comm:
            raise RuntimeError("Serial port not open")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            try:
                resp = self._resp_q.get(timeout=remaining)
            except queue.Empty:
                return False
            if isinstance(resp, tuple) and resp[0] == "DISCONNECT":
                raise RuntimeError(f"Disconnected: {resp[1]}")
            try:
                _addr, cmd, payload = parse_packet(resp)
            except ValueError:
                continue
            if cmd == CMD_ERR and payload:
                if payload[0] == CODE_BAD_CRC and len(payload) >= 3:
                    err_crc = payload[1] | (payload[2] << 8)
                    raise RuntimeError(f"Device reported bad CRC (0x{err_crc:04X})")
                raise RuntimeError("Device reported error")
            if cmd != CMD_ACK or not payload:
                continue
            if payload[0] != expected_code:
                continue
            if expected_crc16 is not None and len(payload) >= 3:
                ack_crc = payload[1] | (payload[2] << 8)
                if ack_crc != expected_crc16:
                    continue
            return True
        return False

    def stream_file(self, data: bytes, chunk_size: int = 256, delay_s: float = 0.01) -> None:
        """Send file contents in small chunks to avoid overrunning the device."""
        if not self._comm:
            raise RuntimeError("Serial port not open")

        for start in range(0, len(data), chunk_size):
            chunk = data[start : start + chunk_size]
            self._comm.send_bytes(chunk)
            time.sleep(delay_s)

    def pop_all_responses(self) -> list[bytes | tuple[str, str]]:
        """Drain the response queue without blocking."""
        responses: list[bytes | tuple[str, str]] = []
        while True:
            try:
                responses.append(self._resp_q.get_nowait())
            except queue.Empty:
                break
        return responses

    @staticmethod
    def save_upload(file_name: str, data: bytes, upload_dir: Path | str = "uploads") -> Path:
        """Persist an uploaded file for later transmission."""
        path = Path(upload_dir)
        path.mkdir(parents=True, exist_ok=True)
        dest = path / file_name
        dest.write_bytes(data)
        return dest


backend = SerialBackend()

