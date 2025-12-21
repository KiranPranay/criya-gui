from __future__ import annotations
import sys
import logging
from typing import List, Optional
from PySide6 import QtCore

# Helper for Serial imports
try:
    import serial
    import serial.tools.list_ports
    HAVE_SERIAL = True
except Exception:
    serial = None
    HAVE_SERIAL = False

class SerialLink(QtCore.QObject):
    statusChanged = QtCore.Signal(str, str) # msg, color
    portsChanged  = QtCore.Signal(list)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ser: Optional[serial.Serial] = None
        self.port_name = ""
        self.baud = 9600
        self.connected = False
        self.dummy_mode = False # Logic Flag

    def available_ports(self):
        ports = []
        if HAVE_SERIAL:
            try:
                pts = serial.tools.list_ports.comports()
                ports = [p.device for p in pts]
            except Exception as e:
                logging.error(f"Serial scan error: {e}")
        
        # Always allow Dummy if enabled or strictly requested
        if self.dummy_mode:
            ports.append("Simulated Arm")
        return ports

    def refresh_ports(self):
        pts = self.available_ports()
        self.portsChanged.emit(pts)

    def connect(self, port: str, baud: int):
        self.port_name = port
        self.baud = baud
        
        # Handle Dummy Connection
        if port == "Simulated Arm":
            self.connected = True
            self.dummy_mode = True # Ensure flag is set
            self.statusChanged.emit(f"Simulated: vRobot @ {baud}", "blue")
            return

        if not HAVE_SERIAL:
            self.statusChanged.emit("PySerial not installed", "red")
            return
            
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                
            self.ser = serial.Serial(port, baud, timeout=1)
            self.connected = True
            self.statusChanged.emit(f"Connected: {port} @ {baud}", "green")
            
        except Exception as e:
            self.connected = False
            self.ser = None
            logging.error(f"Connect error: {e}")
            self.statusChanged.emit(f"Error: {e}", "red")

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None
        self.connected = False
        self.statusChanged.emit("Disconnected", "orange")

    def force_reconnect(self):
        if self.port_name:
            self.connect(self.port_name, self.baud)

    def _write_raw(self, b: bytes):
        if self.connected:
            # If Dummy, just pretend we sent it
            if self.port_name == "Simulated Arm":
                # logging.info(f"SIM TX: {b}")
                return

            if self.ser:
                try:
                    self.ser.write(b)
                except Exception as e:
                    logging.error(f"Write error: {e}")
                    self.statusChanged.emit("Write Error", "red")
                    self.disconnect()

    def send_angle(self, idx: int, angle: int):
        # Firmware expects: "ID ANGLE" (e.g. "1 90")
        cmd = f"{idx+1} {angle}\n"
        self._write_raw(cmd.encode('utf-8'))

    def send_all(self, angles: List[int]):
        # Firmware expects: "M A1 A2 A3 A4 A5 A6 A7"
        # e.g. "M 90 90 90 90 90 90 90"
        vals = " ".join(map(str, angles))
        cmd = f"M {vals}\n"
        self._write_raw(cmd.encode('utf-8'))
