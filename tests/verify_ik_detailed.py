
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
    # Calibration from User Screenshot
    "calibration": [90, 130, 110, 100, 95, 95, 90] 
}

def fmt_angles(angles):
    return "(" + ", ".join([f"{a:.1f}" for a in angles]) + ")"

def fmt_xyz(xyz):
    return f"({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})"

def run_simulation():
    kin = Kinematics(TEST_CONFIG)
    
    print("Running Detailed IK Simulation...\n")
    
    # 1. Start at Home
    current_angles_raw = [90, 130, 110, 100, 95, 95, 90] # Calibrated 90s
    current_pos_xyz = kin.forward_kinematics(current_angles_raw)
    
    # Define Comprehensive Targets
    targets = [
        # 1. Pure Z Changes
        (178.5, 19.9, 300.0), 
        (178.5, 19.9, 395.0), 
        (178.5, 19.9, 150.0), 
        
        # 2. Pure X Changes
        (250.0, 0.0, 200.0),  
        (120.0, 0.0, 200.0),  
        
        # 3. Pure Y Changes
        (200.0, 50.0, 200.0),  
        (200.0, -50.0, 200.0), 
        
        # 4. Complex
        (150.0, 100.0, 350.0), 
        (220.0, -80.0, 100.0), 
        (0.0, 200.0, 250.0),  
        
        # 5. Extreme
        (280.0, 0.0, 50.0),    
        (50.0, 50.0, 400.0)    
    ]
    
    for tx, ty, tz in targets:
        target = (tx, ty, tz)
        
        print("Initial Position:")
        print(f"Cartesian: {fmt_xyz(current_pos_xyz)}")
        print(f"Angles   : {fmt_angles(current_angles_raw)}")
        print("")
        
        print("Target Position:")
        print(f"Cartesian: {fmt_xyz(target)}")
        
        # Solve IK
        sol_angles = kin.inverse_kinematics(tx, ty, tz)
        
        if sol_angles:
            print(f"Computed angles: {fmt_angles(sol_angles)}")
            
            # Apply angles -> FK
            current_angles_raw = sol_angles
            current_pos_xyz = kin.forward_kinematics(sol_angles)
            
            print(f"Resultant Cartesian with computed angles: {fmt_xyz(current_pos_xyz)}")
        else:
            print("Computed angles: UNREACHABLE")
            print("Resultant Cartesian: N/A")
            
        print("-" * 40 + "\n")

    print("Simulation Complete.")

if __name__ == "__main__":
    run_simulation()
