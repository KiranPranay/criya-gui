
import sys
import os
from PySide6 import QtWidgets

# Add project root
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from criya.ui.main_window import MainWindow

def run_test():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    
    print("--- Test Start: Demo Mode ---")
    
    # 1. Verify Default state
    print(f"Initial Dummy Mode: {window.link.dummy_mode}")
    ports = window.link.available_ports()
    print(f"Initial Ports: {ports}")
    if "Simulated Arm" in ports:
        print("FAIL: Simulated Arm present by default")
        return
        
    # 2. Toggle Demo Mode ON
    print("Action: Toggling Demo Mode ON")
    window.chkDemo.setChecked(True) # Triggers _toggle_demo_mode
    
    # 3. Verify Simulated Port appears
    ports = window.link.available_ports()
    print(f"Ports after Enable: {ports}")
    if "Simulated Arm" not in ports:
        print("FAIL: Simulated Arm did not appear")
        return
        
    # 4. Connect to Simulated Arm
    print("Action: Connecting to 'Simulated Arm'...")
    window.link.connect("Simulated Arm", 115200)
    
    if window.link.connected:
        print("SUCCESS: Connected to Simulated Arm")
    else:
        print("FAIL: Could not connect to Simulated Arm")
        return
        
    # 5. Send Command (Should not crash)
    print("Action: Sending Angle Command...")
    window.link.send_angle(0, 45) # Should log valid but do nothing
    print("Command sent (no crash expected)")
    
    # 6. Disconnect
    print("Action: Disconnecting...")
    window.link.disconnect()
    if not window.link.connected:
        print("SUCCESS: Disconnected")
    else:
        print("FAIL: Failed to disconnect")

if __name__ == "__main__":
    run_test()
