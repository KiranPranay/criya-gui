
import sys
import os
import math
import logging

# Configure logging to stdout
logging.basicConfig(level=logging.INFO, format='%(message)s')

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
    "calibration": [90, 130, 110, 100, 95, 95, 90] # S1...S7
}

def verify_point(kin, x, y, z):
    print(f"\n--- Testing Target: X={x}, Y={y}, Z={z} ---")
    
    # 1. Solve IK
    sol = kin.inverse_kinematics(x, y, z)
    if not sol:
        print("RESULT: Unreachable")
        return
        
    print(f"IK Result (Servo Values): {sol}")
    
    # 2. Check Forward Kinematics
    # FK expects list of [S1...S7]
    # Note: sol is [S1, S2, S3, S4, S5, S6, S7]
    fk_res = kin.forward_kinematics(sol)
    
    fx, fy, fz = fk_res
    print(f"FK Verification: X={fx:.2f}, Y={fy:.2f}, Z={fz:.2f}")
    
    dx = x - fx
    dy = y - fy
    dz = z - fz
    err = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    print(f"Error: {err:.4f} mm")
    if err < 1.0:
        print("SUCCESS: Accurate")
    else:
        print("FAILURE: Inaccurate")

def main():
    kin = Kinematics(TEST_CONFIG)
    print("Loaded Kinematics with Calibration:", kin.calib)
    
    # Test 1: High Point (Z=400) - Should require Tilt
    verify_point(kin, 178.5, 19.9, 395.4) # User's screenshot coordinates nearby
    
    # Test 2: Mid Point (Z=300) - Should be easy
    verify_point(kin, 178.5, 19.9, 300.0)
    
    # Test 3: Low Point
    verify_point(kin, 200, 0, 150)

if __name__ == "__main__":
    main()
