
import sys
import os
import json
import logging

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from criya.core.kinematics import Kinematics
from criya.core.config import DEFAULT_CONFIG

CONFIG_FILE = "config.json"

def load_real_config():
    cfg = DEFAULT_CONFIG.copy()
    if os.path.exists(CONFIG_FILE):
        try: 
            loaded = json.load(open(CONFIG_FILE))
            cfg.update(loaded)
            print(f"Loaded Config from {CONFIG_FILE}")
        except: 
            print("Failed to load config.json")
    else:
        print("config.json not found")
    return cfg

def run_test():
    # 1. Load Real Config
    cfg = load_real_config()
    print(f"Calibration: {cfg.get('calibration')}")
    
    kin = Kinematics(cfg)
    
    # 2. Simulate UI Call
    # User Target: Z=300 (from context)
    # Let's say user entered (178.5, 19.9, 300.0)
    tx, ty, tz = 178.5, 19.9, 300.0
    
    # Context: The UI passes 'current_angles' to IK.
    # If the robot is at Home (90s), let's simulate that.
    # Note: UI keeps angles in 'self.angles' which are RAW values (Sliders).
    
    # Scenario A: Robot is at Home (Raw 90s? Or Calibrated?)
    # In UI __init__: self.angles = [90]*7.
    # If the user hasn't moved sliders, they are [90,90,90,90,90,90,90].
    # BUT wait, the User's calibration is [90, 130, 110, 100, 95, 95, 90].
    # If the UI sends [90]*7 to the robot, the robot is NOT at geometric home.
    # It is at a completely weird position!
    
    current_angles_ui = [90]*7 
    
    print("\n--- Test A: UI Default State (Angles=[90]*7) ---")
    print(f"UI Current Angles: {current_angles_ui}")
    
    # Check FK of this state:
    fk_pos = kin.forward_kinematics(current_angles_ui)
    print(f"Robot Actual Position (FK of [90]*7): {fk_pos}")
    # If the robot is physically here, IK might try to find a solution "close" to this weird state.
    
    print(f"\nAttempting IK to ({tx}, {ty}, {tz})...")
    sol = kin.inverse_kinematics(tx, ty, tz, current_angles=current_angles_ui)
    
    if sol:
        print(f"IK Solution: {sol}")
        print("Action: Applying Solution -> FK")
        res = kin.forward_kinematics(sol)
        print(f"Resultant Cartesian: {res}")
    else:
        print("IK Failed in Test A")

    # Scenario B: Robot is at "Calibrated Home"
    # To be at geometric home, the sliders should be at Calibration Values.
    # S1=90, S2=130, S3=110...
    calib_home = cfg.get('calibration', [90]*7)
    
    print("\n--- Test B: Robot at Calibrated Home ---")
    print(f"UI Current Angles: {calib_home}")
    
    sol_b = kin.inverse_kinematics(tx, ty, tz, current_angles=calib_home)
    if sol_b:
        print(f"IK Solution: {sol_b}")
        print("Action: Applying Solution -> FK")
        res_b = kin.forward_kinematics(sol_b)
        print(f"Resultant Cartesian: {res_b}")
    else:
        print("IK Failed in Test B")

if __name__ == "__main__":
    run_test()
