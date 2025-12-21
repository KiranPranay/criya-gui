
import sys
import os
import math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from criya.core.kinematics import Kinematics

TEST_CONFIG = {
    "geometry": {"L_BASE": 120, "L_HUMERUS": 120, "L_SEG1": 70, "L_SEG2": 85, "L_SEG3": 75, "L_TOOL": 120},
    "calibration": [90, 130, 110, 100, 95, 95, 90] 
}

def run_test():
    kin = Kinematics(TEST_CONFIG)
    
    # These are the "Weird" angles the user reported
    # (90, 130, 48, 100, 5, 150, 96)
    # RAW values.
    angles = [90, 130, 48, 100, 5, 150, 96]
    
    print(f"Testing Angles: {angles}")
    
    res = kin.forward_kinematics(angles)
    print(f"FK Result (New Logic): {res}")
    
    # Target was (178.5, 19.9, 300)
    tx, ty, tz = 178.5, 19.9, 300.0
    
    dx = res[0] - tx
    dy = res[1] - ty
    dz = res[2] - tz
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    print(f"Distance to Target: {dist:.2f}")
    
    if dist < 1.0:
        print("CONCLUSION: The angles were CORRECT. The previous FK Check was WRONG (Geometric Mismatch).")
    else:
        print("CONCLUSION: The angles are indeed pointing to the wrong location.")

if __name__ == "__main__":
    run_test()
