
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
    
    # Target: Z=300
    tx, ty, tz = 178.5, 19.9, 300.0
    
    # UI Default State (Uncalibrated 90s)
    current_angles = [90]*7
    
    print(f"Target: ({tx}, {ty}, {tz})")
    print(f"Current Angles (Bias): {current_angles}")
    
    # 1. Get ALL candidates manually (simulating IK loop)
    # Copied logic from kinematics.py roughly
    
    raw_base = kin.get_raw(math.degrees(math.atan2(ty, tx)) + 90, 6)
    
    # Scan pitches
    pitches = sorted(list(range(-90, 121, 5)), key=lambda x: abs(x))
    candidates = []
    
    for pitch in pitches:
        # Solve 2-link...
        # ... (Skipping full math re-implementation, calling IK directly but inspecting internals via logging would be better, 
        # but let's just see what the IK returns vs what we expect)
        pass

    # Actually, simpler: Use the class, it mimics the logic exactly.
    # The class only returns the BEST one.
    # We want to know if the BEST one is weird.
    
    sol = kin.inverse_kinematics(tx, ty, tz, current_angles=current_angles)
    
    with open("logs/bias_check.txt", "w") as f:
        f.write(f"Target: ({tx}, {ty}, {tz})\n")
        f.write(f"Current Angles (Bias): {current_angles}\n\n")
        
        sol = kin.inverse_kinematics(tx, ty, tz, current_angles=current_angles)
        
        f.write(f"Selected Solution: {sol}\n")
        if sol:
            res = kin.forward_kinematics(sol)
            f.write(f"Resultant XYZ: {res}\n")
            
            diff = sum(abs(sol[i] - current_angles[i]) for i in range(7))
            f.write(f"Distance to Current (Bias): {diff}\n")
            
            good_home = cfg.get('calibration')
            diff_good = sum(abs(good_home[i] - current_angles[i]) for i in range(7))
            f.write(f"Distance of Good Home to Current: {diff_good}\n")
            
            if diff < diff_good:
                f.write("CONCLUSION: The solver picked this solution because it is NUMERICALLY CLOSER to the uncalibrated start state than the correct vertical pose.\n")
            else:
                f.write("CONCLUSION: Bias did not affect selection.\n")
        else:
            f.write("No Solution Found\n")

if __name__ == "__main__":
    run_test()
