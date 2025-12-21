
import sys
import os
import math
import logging

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from criya.core.kinematics import Kinematics

# Default Config (what the code falls back to)
TEST_CONFIG = {
    "geometry": {} # Empty to test defaults
}

def fmt(v): return f"{v:.2f}"

def run_debug():
    kin = Kinematics(TEST_CONFIG)
    
    # Target (from user log)
    tx, ty, tz = 86.90, 54.60, 500.00
    
    print(f"Target: ({tx}, {ty}, {tz})")
    
    # Run IK
    sol = kin.inverse_kinematics(tx, ty, tz, current_angles=[90]*7)
    
    if not sol:
        print("IK Failed to find solution")
        return

    print(f"IK Solution (Raw): {sol}")
    
    # Run FK
    res = kin.forward_kinematics(sol)
    print(f"FK Result: ({fmt(res[0])}, {fmt(res[1])}, {fmt(res[2])})")
    
    dx = res[0] - tx
    dy = res[1] - ty
    dz = res[2] - tz
    err = math.sqrt(dx*dx + dy*dy + dz*dz)
    print(f"Error: {fmt(err)} mm")
    
    # Deep Dive: Reconstruct IK Logic manually to find where it diverges
    L_BASE = kin.L_BASE
    L_HUMERUS = kin.L_HUMERUS
    L_SEG1 = kin.L_SEG1
    L_SEG2 = kin.L_SEG2
    L_SEG3 = kin.L_SEG3
    L_TOOL = kin.L_TOOL
    
    theta1 = math.atan2(ty, tx)
    r_total = math.sqrt(tx*tx + ty*ty)
    z_local = tz - L_BASE
    
    print(f"\n--- Manually Checking IK Logic ---")
    print(f"R_Total: {fmt(r_total)}")
    print(f"Z_Local: {fmt(z_local)}")
    
    # We don't know exact pitch selected, but let's reverse engineer from solution
    # Sol[5] is S6 (Shoulder). Sol[4] is S5 (Elbow).
    # Ideal S6, S5
    s6_ideal = kin.get_ideal(sol[5], 5)
    s5_ideal = kin.get_ideal(sol[4], 4)
    s3_ideal = kin.get_ideal(sol[2], 2) # Wrist2
    
    print(f"Ideal S6: {fmt(s6_ideal)}")
    print(f"Ideal S5: {fmt(s5_ideal)}")
    print(f"Ideal S3: {fmt(s3_ideal)}")
    
    q_should = s6_ideal
    q_elbow = s5_ideal + 90
    
    # Re-calculate wrist position based on these angles
    # Using 2-Link FK logic (Circle Intersection logic used in IK)
    L1 = L_HUMERUS
    L2_arm = L_SEG1 + L_SEG2
    
    # Angle from vertical
    # Beta + Alpha = q_should.
    # We need internal angles.
    # Let's project lengths.
    
    # Shoulder Joint is at (0,0, L_BASE) relative to base frame? No, (0,0,0) in "Plane" frame.
    # Elbow Joint:
    # x_e = L1 * cos(q_should) ?? q_should is 90 for vertical.
    # So angle from horizontal is q_should.
    elbow_r = L1 * math.cos(math.radians(q_should))
    elbow_z = L1 * math.sin(math.radians(q_should))
    
    # Wrist Joint (End of L2):
    # Angle of L2 absolute: q_should + q_elbow?
    # No, q_elbow is internal angle "gamma" in _solve_2link?
    # q_elbow = math.degrees(gamma).
    # In IK, gamma is internal angle.
    # If arm is straight, gamma=180.
    # Angle of L2 = Angle of L1 + (gamma - 180).
    # ang2 = q_should + (q_elbow - 180).
    
    ang2 = q_should + (q_elbow - 180)
    
    wrist_r = elbow_r + L2_arm * math.cos(math.radians(ang2))
    wrist_z = elbow_z + L2_arm * math.sin(math.radians(ang2))
    
    print(f"Reconstructed Wrist R: {fmt(wrist_r)}")
    print(f"Reconstructed Wrist Z: {fmt(wrist_z)}")
    
    # Now add Tool Vector
    # Tool Pitch. S3 is used to set pitch.
    # s3_ideal = 270 - (q_should + q_elbow) + pitch_deg
    # pitch_deg = s3_ideal - 270 + q_should + q_elbow
    pitch_deg = s3_ideal - 270 + q_should + q_elbow
    print(f"Derived Pitch: {fmt(pitch_deg)}")
    
    H_L = L_SEG3 + L_TOOL
    tool_r = wrist_r + H_L * math.cos(math.radians(pitch_deg))
    tool_z = wrist_z + H_L * math.sin(math.radians(pitch_deg))
    
    print(f"Reconstructed Tool R: {fmt(tool_r)}")
    print(f"Reconstructed Tool Z: {fmt(tool_z)}")
    print(f"Reconstructed Tool Z (Global): {fmt(tool_z + L_BASE)}")
    
    print(f"Original R: {fmt(r_total)}")
    print(f"Original Z: {fmt(tz)}")
    
    diff_r = tool_r - r_total
    diff_z = (tool_z + L_BASE) - tz
    print(f"Reconstruction Error: R={fmt(diff_r)}, Z={fmt(diff_z)}")

if __name__ == "__main__":
    run_debug()
