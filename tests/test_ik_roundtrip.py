
import sys
import os
import math

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from criya.core.kinematics import Kinematics
from criya.core.config import DEFAULT_CONFIG

def test_round_trip():
    print("=== IK Round Trip Verification ===")
    
    # Initialize with default config/geometry
    kin = Kinematics(DEFAULT_CONFIG)
    
    # Test Cases: [S1, S2, S3, S4, S5, S6, S7]
    # Note: S1 is Tip, S7 is Base.
    # Angles are 0-180.
    test_poses = [
        ("Home/Up",      [90, 90, 90, 90, 90, 90, 90]), # Straight up?
        ("Forward Reach", [90, 90, 90, 90, 90, 60, 90]), # Shoulder forward
        ("Low Reach",     [90, 90, 90, 90, 120, 45, 90]), 
        ("High Reach",    [90, 90, 150, 90, 90, 90, 90]), # Wrist up
        ("Side Reach",    [90, 90, 90, 90, 90, 50, 45]), # Base turn
        ("Testing 400Z",  [90, 90, 120, 90, 80, 80, 90]) # Custom random guess
    ]
    
    failed = 0
    passed = 0
    
    for name, angles in test_poses:
        print(f"\n--- Testing: {name} ---")
        print(f"Input Angles: {angles}")
        
        # 1. Forward Kinematics
        fk_pos = kin.forward_kinematics(angles)
        x, y, z = fk_pos
        print(f"FK Target: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        
        # 2. Inverse Kinematics
        print("Solving IK...")
        # Check if the internal math is sane by passing the exact angles as 'current'
        sol_angles = kin.inverse_kinematics(x, y, z, current_angles=angles)
        
        if not sol_angles:
            print("FAILED: No IK solution found for reachable point.")
            failed += 1
            continue
            
        print(f"IK Solution: {sol_angles}")
        
        # 3. Verify Solution with FK
        verify_pos = kin.forward_kinematics(sol_angles)
        vx, vy, vz = verify_pos
        print(f"Result FK: X={vx:.2f}, Y={vy:.2f}, Z={vz:.2f}")
        
        # 4. Check Error
        err_x = abs(vx - x)
        err_y = abs(vy - y)
        err_z = abs(vz - z)
        total_err = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        
        print(f"Errors: dX={err_x:.4f}, dY={err_y:.4f}, dZ={err_z:.4f}")
        
        if total_err < 1.0:
            print("STATUS: PASS ( < 1mm error )")
            passed += 1
        else:
            print("STATUS: FAIL ( > 1mm error )")
            failed += 1
            
    print(f"\n=== Summary: Passed {passed}, Failed {failed} ===")

if __name__ == "__main__":
    test_round_trip()
