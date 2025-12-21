
import sys
import os
import math
import logging

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from criya.core.kinematics import Kinematics

# User's Config from screenshot/context
TEST_CONFIG = {
    "geometry": {
        "L_BASE": 120,
        "L_HUMERUS": 120,
        "L_SEG1": 70,
        "L_SEG2": 85,
        "L_SEG3": 75,
        "L_TOOL": 120
    },
    "calibration": [90, 130, 110, 100, 95, 95, 90] 
}

def fmt_list(lst):
    return "(" + ", ".join([f"{v:.1f}" for v in lst]) + ")"

def fmt_xyz(xyz):
    return f"({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})"

def run_simulation():
    kin = Kinematics(TEST_CONFIG)
    print("Running Calibrated IK Simulation...\n")
    
    # --- 1. Initial Position (Geometric Home) ---
    # User calls this "(90,90...)" which implies Geometric Ideal.
    # We must convert Ideal 90s to Raw Calibrated to get the physical robot state.
    ideal_home = [90.0] * 7
    raw_home = []
    for i in range(7):
        # raw = ideal + (calib - 90)
        raw = kin.get_raw(ideal_home[i], i)
        raw_home.append(raw)
        
    # Cartesian of the physical robot (using raw values)
    home_xyz = kin.forward_kinematics(raw_home)
    
    # Define Targets
    targets = [
        (178.5, 19.9, 300.0), 
        (178.5, 19.9, 395.0), 
        (178.5, 19.9, 150.0), 
        (250.0, 0.0, 200.0),
        (120.0, 0.0, 200.0),
        (200.0, 50.0, 200.0),
        (200.0, -50.0, 200.0),
        (150.0, 100.0, 350.0),
        (220.0, -80.0, 100.0),
        (0.0, 200.0, 250.0),
        (280.0, 0.0, 50.0),
        (50.0, 50.0, 400.0)
    ]
    
    for tx, ty, tz in targets:
        target = (tx, ty, tz)
        
        print("Initial Position:")
        print(f"Cartesian     : {fmt_xyz(home_xyz)}")
        print(f"Angles (Ideal): {fmt_list(ideal_home)}")
        print(f"Angles (Raw)  : {fmt_list(raw_home)}")
        print("")
        
        print("Target Position:")
        print(f"Cartesian : {fmt_xyz(target)}")
        
        # Solve IK -> Returns RAW angles
        raw_sol = kin.inverse_kinematics(tx, ty, tz)
        
        if raw_sol:
            # Back-calculate Ideal for display
            ideal_sol = []
            for i in range(7):
                # ideal = raw - (calib - 90)
                ideal = kin.get_ideal(raw_sol[i], i)
                ideal_sol.append(ideal)
                
            print(f"Computed angles (Ideal): {fmt_list(ideal_sol)}")
            print(f"Computed angles (Raw)  : {fmt_list(raw_sol)}")
            
            # Verify FK
            print("Action: Applying Calibrated Computed Angles (Raw) -> FK...")
            result_xyz = kin.forward_kinematics(raw_sol)
            print(f"Resultant Cartesian    : {fmt_xyz(result_xyz)}")
        else:
            print("Computed angles: UNREACHABLE")
            
        print("-" * 50 + "\n")

if __name__ == "__main__":
    run_simulation()
