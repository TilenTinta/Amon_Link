from __future__ import annotations

import threading
import time
from pathlib import Path
import zlib
from typing import Any

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from application.backend import backend
from application.protocol import (
    CMD_ACK,
    CMD_END_OF_FW,
    CMD_JUMP_APP,
    CMD_WRITE,
    CODE_BOOT_VER,
    CODE_DATA_WRITEN,
    CODE_EXIT_BOOT,
    CODE_SW_CRC,
    ID_LINK_BOOT,
    build_packet,
    crc16_cal,
    parse_packet,
)


class AppState:
    def __init__(self) -> None:
        self.connection_status = "Disconnected"
        self.last_message = ""
        self.upload_path = ""
        self.upload_name = ""
        self.upload_progress = 0
        self.is_uploading = False
        self.upload_crc32 = ""
        self.device_boot_ver = ""
        self.device_crc32 = ""
        self.crc_match = ""


state = AppState()
state_lock = threading.Lock()

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class ConnectRequest(BaseModel):
    port: str
    baud_rate: int


class UploadStartRequest(BaseModel):
    chunk_size: int | None = None
    delay_ms: int | None = None


def _state_snapshot(extra: dict[str, Any] | None = None) -> dict[str, Any]:
    with state_lock:
        snapshot = {
            "connection_status": state.connection_status,
            "last_message": state.last_message,
            "upload_path": state.upload_path,
            "upload_name": state.upload_name,
            "upload_progress": state.upload_progress,
            "is_uploading": state.is_uploading,
            "upload_crc32": state.upload_crc32,
            "device_boot_ver": state.device_boot_ver,
            "device_crc32": state.device_crc32,
            "crc_match": state.crc_match,
        }
    if extra:
        snapshot.update(extra)
    return snapshot


def _set_status(message: str) -> None:
    with state_lock:
        state.connection_status = message


def _set_last_message(message: str) -> None:
    with state_lock:
        state.last_message = message


def _set_upload_state(
    *,
    path: str | None = None,
    name: str | None = None,
    progress: int | None = None,
    is_uploading: bool | None = None,
    crc32: str | None = None,
    ) -> None:
        with state_lock:
            if path is not None:
                state.upload_path = path
        if name is not None:
            state.upload_name = name
        if progress is not None:
            state.upload_progress = progress
        if is_uploading is not None:
            state.is_uploading = is_uploading
        if crc32 is not None:
            state.upload_crc32 = crc32


def _set_device_state(
    *,
    boot_ver: str | None = None,
    crc32: str | None = None,
    crc_match: str | None = None,
) -> None:
    with state_lock:
        if boot_ver is not None:
            state.device_boot_ver = boot_ver
        if crc32 is not None:
            state.device_crc32 = crc32
        if crc_match is not None:
            state.crc_match = crc_match


def _crc32_hex(data: bytes) -> str:
    return f"0x{zlib.crc32(data) & 0xFFFFFFFF:08X}"


def _normalize_crc32(value: str | None) -> str | None:
    if not value:
        return None
    text = value.strip()
    if text.lower().startswith("0x"):
        text = text[2:]
    if not text:
        return None
    lowered = text.lower()
    if any(char not in "0123456789abcdef" for char in lowered):
        return None
    return lowered.zfill(8).upper()


def _update_crc_match() -> None:
    with state_lock:
        upload_crc32 = _normalize_crc32(state.upload_crc32)
        device_crc32 = _normalize_crc32(state.device_crc32)
        if upload_crc32 and device_crc32:
            state.crc_match = "match" if upload_crc32 == device_crc32 else "mismatch"
        else:
            state.crc_match = ""


def _hex_to_bin(data: bytes, start_addr: int | None = None) -> bytes:
    """Parse Intel HEX into raw binary bytes, optionally trimming below start_addr."""
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


def _looks_like_hex(data: bytes) -> bool:
    """Heuristic to detect Intel HEX content even if extension is wrong."""
    for raw_line in data.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if not line.startswith(b":"):
            return False
        return True
    return False


def _format_response(resp: bytes | tuple[str, str]) -> str:
    if isinstance(resp, tuple) and resp[0] == "DISCONNECT":
        _set_status(f"Disconnected: {resp[1]}")
        return f"Disconnected: {resp[1]}"

    try:
        addr, cmd, payload = parse_packet(resp)
        if cmd == CMD_ACK:
            info = _parse_info_payload(payload)
            if info:
                boot_ver = info["boot_ver"]
                device_crc32 = info["device_crc32"]
                message = (
                    f"Info: boot ver 0x{boot_ver:02X}, device CRC32 {device_crc32}"
                )
                _set_device_state(
                    boot_ver=f"0x{boot_ver:02X}",
                    crc32=device_crc32,
                    crc_match="",
                )
                _update_crc_match()
                with state_lock:
                    crc_match = state.crc_match
                if crc_match == "match":
                    message += " (matches upload, no update needed)"
                elif crc_match == "mismatch":
                    message += " (differs from upload, update needed)"
                return message

        payload_hex = payload.hex() or "0x00"
        return f"Addr {hex(addr)} Cmd {hex(cmd)} Payload {payload_hex}"
    except ValueError:
        return f"Raw: {resp.hex()}"


def _parse_info_payload(payload: bytes) -> dict[str, int | str] | None:
    if len(payload) < 7:
        return None
    if payload[0] != CODE_BOOT_VER or payload[2] != CODE_SW_CRC:
        return None
    boot_ver = payload[1]
    device_crc32 = int.from_bytes(payload[3:7], "big")
    return {
        "boot_ver": boot_ver,
        "device_crc32": f"0x{device_crc32:08X}",
    }


@app.get("/ports")
def list_ports() -> dict[str, Any]:
    ports = backend.list_ports()
    status = (
        f"Connected to {backend.connected_port}" if backend.is_connected() else "Disconnected"
    )
    _set_status(status)
    return _state_snapshot({"ports": ports})


@app.post("/connect")
def connect(request: ConnectRequest) -> dict[str, Any]:
    if not request.port:
        raise HTTPException(status_code=400, detail="Port is required.")
    error = None
    try:
        print(f"CONNECT start {request.port} @ {request.baud_rate}", flush=True)
        _set_device_state(boot_ver="", crc32="", crc_match="")
        _set_last_message(f"Opening {request.port} @ {request.baud_rate}")
        backend.connect(request.port, int(request.baud_rate))
        print("CONNECT opened port", flush=True)
        time.sleep(0.25)
        packet = backend.send_get_info()
        print(f"CONNECT sent GET_INFO {packet.hex()}", flush=True)
        _set_last_message(f"Sent GET_INFO {packet.hex()}")
        _set_status(f"Connected to {request.port}")
    except Exception as exc:  # noqa: BLE001
        error = str(exc)
        print(f"CONNECT failed: {exc}", flush=True)
        _set_last_message(f"Connect failed: {exc}")
        _set_status(f"Failed to connect: {exc}")
    return _state_snapshot({"error": error})


@app.post("/disconnect")
def disconnect() -> dict[str, Any]:
    backend.disconnect()
    _set_status("Disconnected")
    _set_device_state(boot_ver="", crc32="", crc_match="")
    return _state_snapshot()


@app.get("/responses")
def poll_responses() -> dict[str, Any]:
    responses = backend.pop_all_responses()
    messages: list[str] = []
    for resp in responses:
        message = _format_response(resp)
        messages.append(message)
        _set_last_message(message)
    return _state_snapshot({"messages": messages})


@app.post("/upload")
def upload(file: UploadFile = File(...)) -> dict[str, Any]:
    if not file.filename:
        raise HTTPException(status_code=400, detail="File is required.")
    data = file.file.read()
    if not data:
        raise HTTPException(status_code=400, detail="File is empty.")
    file_name = file.filename
    is_hex = file.filename.lower().endswith(".hex") or _looks_like_hex(data)
    if is_hex:
        try:
            data = _hex_to_bin(data)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        if not data:
            raise HTTPException(status_code=400, detail="HEX file contains no data.")
        file_name = Path(file.filename).with_suffix(".bin").name

    crc32 = _crc32_hex(data)
    dest = backend.save_upload(file_name, data, upload_dir="uploaded_files")
    _set_upload_state(
        path=str(dest),
        name=file_name,
        progress=0,
        is_uploading=False,
        crc32=crc32,
    )
    _update_crc_match()
    _set_last_message(f"Loaded {file_name} ({len(data)} bytes)")
    return _state_snapshot()


def _upload_worker(path: str, chunk_size: int, delay_s: float) -> None:
    _set_upload_state(progress=0, is_uploading=True)
    try:
        data = Path(path).read_bytes()
        if not data:
            _set_last_message("File is empty")
            return

        sent = 0
        total = len(data)
        padded_chunk_size = max(1, chunk_size)
        for index, start in enumerate(range(0, total, padded_chunk_size), start=1):
            chunk = data[start : start + padded_chunk_size]
            if len(chunk) < padded_chunk_size:
                chunk = chunk + b"\xFF" * (padded_chunk_size - len(chunk))

            packet = build_packet(ID_LINK_BOOT, CMD_WRITE, payload=chunk)
            backend.send_bytes(packet)
            expected_crc = crc16_cal(chunk)
            if not backend.wait_for_ack(
                expected_code=CODE_DATA_WRITEN,
                expected_crc16=expected_crc,
                timeout_s=2.0,
            ):
                _set_status(f"Upload failed: no ACK at chunk {index}")
                return

            sent += min(padded_chunk_size, total - start)
            progress = int((sent / total) * 100)
            _set_upload_state(progress=progress)
            if delay_s:
                time.sleep(delay_s)

        with state_lock:
            upload_crc32 = state.upload_crc32
        if not upload_crc32:
            _set_status("Upload failed: missing CRC32 for end packet")
            return
        crc32_value = int(upload_crc32, 16)
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
        backend.send_bytes(end_packet)
        expected_crc = crc16_cal(end_payload)
        if not backend.wait_for_ack(
            expected_code=CODE_EXIT_BOOT,
            expected_crc16=expected_crc,
            timeout_s=2.0,
        ):
            _set_status("Upload failed: no ACK for end packet")
            return
        _set_status(f"Uploaded {state.upload_name}")
    except Exception as exc:  # noqa: BLE001
        _set_status(f"Upload failed: {exc}")
    finally:
        _set_upload_state(is_uploading=False)


@app.post("/start_upload")
def start_upload(request: UploadStartRequest | None = None) -> dict[str, Any]:
    if not backend.is_connected():
        raise HTTPException(status_code=400, detail="Connect to a device first.")

    with state_lock:
        path = state.upload_path
        is_uploading = state.is_uploading

    if not path:
        raise HTTPException(status_code=400, detail="Select a file first.")
    if is_uploading:
        raise HTTPException(status_code=409, detail="Upload already in progress.")

    chunk_size = 64
    delay_s = 0.01
    if request:
        if request.chunk_size:
            chunk_size = max(64, min(256, request.chunk_size))
        if request.delay_ms is not None:
            delay_s = max(0.0, request.delay_ms / 1000.0)

    thread = threading.Thread(
        target=_upload_worker,
        args=(path, chunk_size, delay_s),
        daemon=True,
    )
    thread.start()
    return _state_snapshot()


@app.post("/jump_app")
def jump_app() -> dict[str, Any]:
    if not backend.is_connected():
        raise HTTPException(status_code=400, detail="Connect to a device first.")
    packet = build_packet(ID_LINK_BOOT, CMD_JUMP_APP)
    backend.send_bytes(packet)
    _set_last_message(f"Sent JUMP_APP {packet.hex()}")
    return _state_snapshot()


@app.get("/status")
def status() -> dict[str, Any]:
    return _state_snapshot()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("backend_server:app", host="127.0.0.1", port=8000, log_level="info")
