import queue
import threading
import time
from pathlib import Path
from typing import Optional

import serial.tools.list_ports

from .protocol import CMD_GET_INFO, ID_LINK_BOOT, build_packet, parse_packet
from .serial_comm import SerialComm


class SerialBackend:
    """Thread-safe wrapper around the existing SerialComm for UI use."""

    def __init__(self):
        self._lock = threading.Lock()
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

    def send_get_info(self) -> None:
        """Send the GET_INFO command using the existing packet format."""
        if not self._comm:
            raise RuntimeError("Serial port not open")
        packet = build_packet(ID_LINK_BOOT, CMD_GET_INFO)
        self._comm.send_bytes(packet)

    def send_bytes(self, payload: bytes) -> None:
        if not self._comm:
            raise RuntimeError("Serial port not open")
        self._comm.send_bytes(payload)

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

