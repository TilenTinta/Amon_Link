##################################################################
# File Name          : serial_comm.py
# Description        : Serial driver for updating firmware over UART (USB)
##################################################################
import serial
import threading
import queue
import time


class SerialComm:
    def __init__(self, port, baudrate, response_queue, timeout=0.5, open_timeout_s=2.0):
        self.port_name = port
        self.baudrate = baudrate
        self.response_queue = response_queue
        self.timeout = timeout
        self.open_timeout_s = open_timeout_s
        self.ser = None
        self.alive = threading.Event()
        self.thread = threading.Thread(target=self._reader_thread, daemon=True)

    def _open_serial(self):
        self.ser = serial.Serial(
            self.port_name,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

    def open(self):
        try:
            open_thread = threading.Thread(target=self._open_serial, daemon=True)
            open_thread.start()
            open_thread.join(self.open_timeout_s)
            if open_thread.is_alive():
                raise RuntimeError(
                    f"Timed out opening {self.port_name} after {self.open_timeout_s}s"
                )
            if self.ser is None:
                raise RuntimeError(f"Failed to open {self.port_name}")
        except serial.SerialException as exc:
            raise RuntimeError(f"Failed to open {self.port_name}: {exc}") from exc
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.alive.set()
        self.thread.start()

    def close(self):
        self.alive.clear()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_bytes(self, data_bytes):
        if self.ser and self.ser.is_open:
            self.ser.write(data_bytes)
        else:
            raise RuntimeError("Serial port not open")

    def _reader_thread(self):
        buf = bytearray()
        while self.alive.is_set():
            try:
                data = self.ser.read(1)
                if data:
                    buf.extend(data)

                    # Look for complete packets based on your modified C structure
                    while len(buf) >= 4:  # Minimum: SOF(1) + PLEN(1) + ADDR(1) + CMD(1)
                        if buf[0] != 0xAA:
                            buf.pop(0)
                            continue

                        if len(buf) < 2:
                            break
                        plen = buf[1]

                        total_len = 1 + 1 + plen  # SOF + PLEN + data
                        if len(buf) < total_len:
                            break  # Wait for more data

                        frame = bytes(buf[:total_len])
                        buf = buf[total_len:]
                        self.response_queue.put(frame)
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as e:
                self.alive.clear()
                self.response_queue.put(("DISCONNECT", str(e)))
                break


