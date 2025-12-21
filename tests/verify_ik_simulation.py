
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

def log_movement(current, target, reached):
    print(f"Current : ({current[0]:.2f}, {current[1]:.2f}, {current[2]:.2f})")
    print(f"Target : ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})")
    print(f"Reached : ({reached[0]:.2f}, {reached[1]:.2f}, {reached[2]:.2f})")
    print("-" * 20)

def run_simulation():
    kin = Kinematics(TEST_CONFIG)
    
    print(f"Running Simulation... Logging to STDOUT")
    
    # 1. Start at Home
    # Home Raw = Calibration Values (Ideal 90s)
    home_raw = [90, 130, 110, 100, 95, 95, 90]
    home_pos = kin.forward_kinematics(home_raw)
    
    current_pos = home_pos
    
    # Define Comprehensive Targets
    targets = [
        # 1. Pure Z Changes (Vertical)
        (178.5, 19.9, 300.0), # Mid Height
        (178.5, 19.9, 395.0), # High Height (Limits)
        (178.5, 19.9, 150.0), # Low Height
        
        # 2. Pure X Changes (Radial distance)
        (250.0, 0.0, 200.0),  # Far reach
        (120.0, 0.0, 200.0),  # Close in (Crowded)
        
        # 3. Pure Y Changes (Lateral/Base Rotation)
        (200.0, 50.0, 200.0),  # Left
        (200.0, -50.0, 200.0), # Right
        
        # 4. Complex Combinations
        (150.0, 100.0, 350.0), # High, Side, Close
        (220.0, -80.0, 100.0), # Low, Far, Right
        (0.0, 200.0, 250.0),   # Directly Side (90 deg base)
        
        # 5. Extreme/Edge Cases
        (280.0, 0.0, 50.0),    # Max Reach Low
        (50.0, 50.0, 400.0)    # High and Close (Hard for kinematics)
    ]
    
    for tx, ty, tz in targets:
        target = (tx, ty, tz)
        
        # Solve IK
        sol = kin.inverse_kinematics(tx, ty, tz)
        
        if sol:
            # Calculate Reached Position via FK
            reached_pos_list = kin.forward_kinematics(sol)
            reached_pos = (reached_pos_list[0], reached_pos_list[1], reached_pos_list[2])
            
            log_movement(current_pos, target, reached_pos)
            
            # Update current for next move
            current_pos = reached_pos
        else:
            print(f"Current : ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
            print(f"Target : ({tx:.2f}, {ty:.2f}, {tz:.2f})")
            print("Reached : UNREACHABLE")
            print("-" * 20)

    print("Simulation Complete.")

if __name__ == "__main__":
    run_simulation()
