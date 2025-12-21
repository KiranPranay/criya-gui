
import sys
import os
from PySide6 import QtWidgets

# Add project root
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from criya.ui.main_window import MainWindow

def run_test():
    # Create Qt Application (required for QMainWindow)
    app = QtWidgets.QApplication(sys.argv)
    
    # Instantiate Main Window (this runs __init__ and loads config)
    window = MainWindow()
    
    # Force the "slider" state to match the "Calibration" state
    # This simulates the fix I made earlier (self.angles = calibration)
    print(f"Initial Window Angels: {window.angles}")
    
    # Set Inputs for Z=300 Target
    # User Context: "I have asked the z to be 300"
    window.ikX.setText("178.5")
    window.ikY.setText("19.9")
    window.ikZ.setText("300.0")
    
    # Ensure Checkboxes are checked
    window.chkX.setChecked(True)
    window.chkY.setChecked(True)
    window.chkZ.setChecked(True)
    
    print("Simulating 'Move to Point' click...")
    window._solve_ik()
    
    print("Done. Checking log file...")
    if os.path.exists("resultlog.txt"):
        with open("resultlog.txt", "r") as f:
            print("\n--- CONTENT OF resultlog.txt ---")
            print(f.read())
            print("--------------------------------")
    else:
        print("Error: resultlog.txt was not created.")

if __name__ == "__main__":
    run_test()
