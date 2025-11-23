##################################################################
# File Name          : serial_comm.py
# Author             : Tinta T.
# Version            : V1.0.0
# Date               : 2025/11/09
# Description        : Serial driver for updating firmware 
#                      over UART (USB)
##################################################################
import serial
import threading
import queue
import time

class SerialComm:
    def __init__(self, port, baudrate, response_queue, timeout=0.5):
        self.port_name = port
        self.baudrate = baudrate
        self.response_queue = response_queue
        self.timeout = timeout
        self.ser = None
        self.alive = threading.Event()
        self.thread = threading.Thread(target=self._reader_thread, daemon=True)

    def open(self):
        self.ser = serial.Serial(self.port_name, baudrate=self.baudrate, timeout=self.timeout)
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
                        
                        # Extract packet length (1 byte - uint8_t)
                        if len(buf) < 2:
                            break
                        plen = buf[1]
                        
                        # Total packet length = SOF(1) + PLEN(1) + plen bytes
                        total_len = 1 + 1 + plen
                        
                        if len(buf) < total_len:
                            break  # Wait for more data
                        
                        # Extract complete packet
                        frame = bytes(buf[:total_len])
                        buf = buf[total_len:]
                        self.response_queue.put(frame)
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as e:
                self.alive.clear()
                self.response_queue.put(("DISCONNECT", str(e)))
                break

