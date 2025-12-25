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
from application.protocol import parse_packet


class AppState:
    def __init__(self) -> None:
        self.connection_status = "Disconnected"
        self.last_message = ""
        self.upload_path = ""
        self.upload_name = ""
        self.upload_progress = 0
        self.is_uploading = False
        self.upload_crc32 = ""


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


def _crc32_hex(data: bytes) -> str:
    return f"0x{zlib.crc32(data) & 0xFFFFFFFF:08X}"


def _hex_to_bin(data: bytes) -> bytes:
    """Parse Intel HEX into raw binary bytes."""
    base_addr = 0
    chunks: dict[int, bytes] = {}
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
            chunks[abs_addr] = payload
            max_addr = max(max_addr, abs_addr + len(payload))
        elif record_type == 0x01:
            break
        elif record_type == 0x04:
            if len(payload) != 2:
                raise ValueError("Invalid HEX extended address record")
            base_addr = int.from_bytes(payload, "big") << 16
        else:
            continue

    if not chunks:
        return b""
    blob = bytearray(b"\xFF" * max_addr)
    for addr, payload in chunks.items():
        blob[addr:addr + len(payload)] = payload
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
        payload_hex = payload.hex() or "0x00"
        return f"Addr {hex(addr)} Cmd {hex(cmd)} Payload {payload_hex}"
    except ValueError:
        return f"Raw: {resp.hex()}"


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
        backend.connect(request.port, int(request.baud_rate))
        backend.send_get_info()
        _set_status(f"Connected to {request.port}")
    except Exception as exc:  # noqa: BLE001
        error = str(exc)
        _set_status(f"Failed to connect: {exc}")
    return _state_snapshot({"error": error})


@app.post("/disconnect")
def disconnect() -> dict[str, Any]:
    backend.disconnect()
    _set_status("Disconnected")
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
        for start in range(0, total, chunk_size):
            chunk = data[start : start + chunk_size]
            backend.send_bytes(chunk)
            sent += len(chunk)
            progress = int((sent / total) * 100)
            _set_upload_state(progress=progress)
            time.sleep(delay_s)
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

    chunk_size = 256
    delay_s = 0.01
    if request:
        if request.chunk_size:
            chunk_size = max(64, min(1024, request.chunk_size))
        if request.delay_ms is not None:
            delay_s = max(0.0, request.delay_ms / 1000.0)

    thread = threading.Thread(
        target=_upload_worker,
        args=(path, chunk_size, delay_s),
        daemon=True,
    )
    thread.start()
    return _state_snapshot()


@app.get("/status")
def status() -> dict[str, Any]:
    return _state_snapshot()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("backend_server:app", host="127.0.0.1", port=8000, log_level="info")
