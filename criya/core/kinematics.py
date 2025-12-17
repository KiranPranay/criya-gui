from __future__ import annotations
import math
import numpy as np
from typing import List, Optional

# -------------------- Matrix Helpers --------------------
def rot_x(rad: float):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]])

def rot_y(rad: float):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0, s, 0],
                     [0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [0, 0, 0, 1]])

def rot_z(rad: float):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def trans(x: float, y: float, z: float):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

# -------------------- Kinematics --------------------
class Kinematics:
    def __init__(self, config: dict):
        self.config = config
        # Default Link Lengths (mm) - User should configure these
        # Custom 5-Link Geometry
        # Base -> Shoulder -> Elbow -> Wrist1 -> Wrist2 -> Tool
        self.L_BASE = 120    # Base Height (Z)
        self.L_HUMERUS = 120 # Shoulder to Elbow
        self.L_SEG1 = 70     # Elbow to Wrist1
        self.L_SEG2 = 85     # Wrist1 to Wrist2
        self.L_SEG3 = 75     # Wrist2 to Gripper Base
        self.L_TOOL = 120    # Gripper Length
        
        # Load from config if available (Geometry section)
        self.update_lengths(config.get("geometry", {}))
        
        # Calibration offsets (degrees)
        self.calib = [90, 130, 110, 100, 95, 95, 90]
        if "calibration" in config:
            self.calib = config["calibration"]

    def update_lengths(self, cfg: dict):
        self.L_BASE = float(cfg.get("L_BASE", 120))
        self.L_HUMERUS = float(cfg.get("L_HUMERUS", 120))
        self.L_SEG1 = float(cfg.get("L_SEG1", 70))
        self.L_SEG2 = float(cfg.get("L_SEG2", 85))
        self.L_SEG3 = float(cfg.get("L_SEG3", 75))
        self.L_TOOL = float(cfg.get("L_TOOL", 120))

    def get_ideal(self, raw, idx):
        # ideal = raw - (calib - 90)
        return raw - (self.calib[idx] - 90)

    def get_raw(self, ideal, idx):
        # ideal + (calib - 90)
        return ideal + (self.calib[idx] - 90)

    def forward_kinematics(self, angles: List[float]) -> List[float]:
        # FK using Homogenous Matrices (same as Viz3D) for accuracy with 3D rotations (Roll/Pitch)
        # S7=Pan, S6=Pitch, S5=Pitch, S4=ROLL, S3=Pitch, S2=Roll
        
        T = np.eye(4)
        
        # 1. Base (S7) - Pan (X-Forward)
        theta1 = math.radians(angles[6] - 90)
        T = T @ rot_z(theta1) @ trans(0, 0, self.L_BASE)
        
        # 2. Shoulder (S6) - PITCH (Rot Y)
        # User: "<90 is Forward". 0->+90(Forward).
        # Old: (angle - 90). 0->-90. 180->+90.
        # New: (90 - angle). 0->+90. 180->-90.
        theta2 = math.radians(90 - angles[5])
        T = T @ rot_y(theta2) @ trans(0, 0, self.L_HUMERUS)
        
        # 3. Elbow (S5) - PITCH (Rot Y)
        # User: "<90 is Forward".
        theta3 = math.radians(90 - angles[4])
        T = T @ rot_y(theta3) @ trans(0, 0, self.L_SEG1)
        
        # 4. Wrist 1 (S4) - LATERAL BEND (Rot X)
        # User: "Servo 4 in y direction" -> Y-Motion = Lateral = Rot X.
        theta4 = math.radians(angles[3] - 90)
        T = T @ rot_x(theta4) @ trans(0, 0, self.L_SEG2) 
        
        # 5. Wrist 2 (S3) - PITCH (Rot Y)
        # User: "<90 is Forward".
        theta5 = math.radians(90 - angles[2])
        T = T @ rot_y(theta5) @ trans(0, 0, self.L_SEG3)
        
        # 6. Wrist 3 (S2) - ROLL (Rot X / Parallel to Tool)
        # User: "Servo 2 motion is okay" (Roll along X).
        theta6 = math.radians(angles[1] - 90)
        T = T @ rot_x(theta6)
        
        # 7. Tool Offset (Fixed)
        # Forward (X).
        T = T @ trans(self.L_TOOL, 0, 0)
        
        # 8. S1 (Tip) - Removed Visualization request.
        # Just return current Tool Tip position.
        return list(T[:3, 3])

    def inverse_kinematics(self, x: float, y: float, z: float) -> Optional[List[float]]:
        # Geometric resolution for 3-DOF (Pan, Shoulder, Elbow)
        # Supports "Leaning Back" (Negative Radius) if target is behind the Y axis.
        
        candidates = []
        
        # Strategy 1: Forward Reach (r > 0)
        theta1_a = math.atan2(y, x)
        angle_base_a = math.degrees(theta1_a) + 90 # Map -90..90 to 0..180
        r_total_a = math.sqrt(x*x + y*y)
        # Wrist Target = Target - Tool
        r_wrist_a = r_total_a - self.L_TOOL 
        candidates.append((angle_base_a, r_wrist_a))
        
        # Strategy 2: Backward Reach (r < 0, Base flipped 180)
        theta1_b = theta1_a + math.pi # Flip 180
        # Normalize to -PI..PI
        theta1_b = math.atan2(math.sin(theta1_b), math.cos(theta1_b))
        angle_base_b = math.degrees(theta1_b) + 90 # Map to servo
        
        # For backward reach, the tool also points backward relative to base?
        # If Base rotates 180, Forward is now "Back".
        # So r_wrist should be calculated as if reaching 'backwards'
        # Actually, math.sqrt(x*x+y*y) is always positive.
        # But if we rotate base 180, the arm is effectively reaching into negative correlation?
        # No, the arm always reaches 'forward' in its own frame.
        # If we flip base, we need the arm to reach "The other way"?
        # Wait, if we flip base, x/y are reversed in local frame.
        # So r is still positive in local frame. 
        # But let's verify if "Leaning Back" (Negative Radius) is physically possible.
        # If we use negative radius in the shoulder/elbow calc, it means the arm bends backwards "overhead".
        # This is a valid strategy.
        r_wrist_b = -1 * (r_total_a - self.L_TOOL) # Negative Reach
        candidates.append((angle_base_a, r_wrist_b)) # Same Base Angle?? No.
        # If we use Negative Reach strategy, we might keep the Base Angle same (if we want to reach "behind" without panning)
        # OR we can Pan 180 and reach "Forward".
        # Let's support "Negative Reach" with SAME Base Angle.
        candidates.append((angle_base_a, -r_wrist_a))
        
        
        solutions = []
        
        for base_angle, r in candidates:
            # We treat the arm as a 3-link planar robot (Humerus, Forearm, Wrist) in the vertical plane.
            # R = Horizontal distance to wrist
            # Z = Vertical distance to wrist
            
            # Since our Kinematics is 5-DOF really, finding an analytic solution is hard.
            # Simplified approach: Treat S4, S3 as fixed extensions or part of the chain?
            # User wants S3/S5/S6 to be X-axis (Pitch). 
            # S4 is Y-axis (Lateral). 
            # If we keep S4=90 (Straight), it has no effect on R/Z except length.
            # So Effective Chain = Humerus + Seg1 + Seg2 + Seg3.
            # BUT S4 is in the middle.
            # If S4 is lateral, it bends out of plane. We assume S4=90 for IK.
            
            # Lengths projection
            L1 = self.L_HUMERUS
            L2 = self.L_SEG1 + self.L_SEG2 + self.L_SEG3 # Simplified: All inline if S4=90, S3=0??
            # Wait, S3 is also Pitch.
            # So Chain is: Humerus(S6) -> Elbow(S5) -> [S4-Lateral] -> Wrist2(S3-Pitch).
            # If S4=90, L_SEG2 is inline.
            # So Planar Chain: L1(S6) -> L2(S5) -> L3(S3) -> Tip.
            # This is 3-Link Planar IK. (Redundant).
            # To simplify: We can set S3=90 (Straight) and solve for S6, S5 (2-Link IK).
            # Or set S5=S3? 
            # Let's stick to the 2-Link approach where L2 = Sum(Remaining).
            # This is robust enough for simple Pick/Place.
            
            # Effective L2 for 2-link solver (Shoulder + Elbow)
            # We will actuate S6 and S5. S3 will be kept straight (90) relative to S5? 
            # Or we include it in L2.
            # Let's say L2 = L_SEG1 + L_SEG2 + L_SEG3.
            l2_eff = self.L_SEG1 + self.L_SEG2 + self.L_SEG3
            
            # Target for Shoulder/Elbow
            # Triangle SSS
            # h = hypotenuse (Distance from Shoulder to Wrist)
            # z_local = z - L_BASE
            z_local = z - self.L_BASE
            h_sq = r*r + z_local*z_local
            h = math.sqrt(h_sq)
            
            if h > (L1 + l2_eff) or h < abs(L1 - l2_eff):
                continue # Unreachable
            
            # Law of Cosines
            # Gamma (Elbow Angle internal)
            # h^2 = L1^2 + L2^2 - 2*L1*L2*cos(gamma)
            # cos(gamma) = (L1^2 + L2^2 - h^2) / (2*L1*L2)
            cos_gamma = (L1*L1 + l2_eff*l2_eff - h_sq) / (2 * L1 * l2_eff)
            # Clamp for float errors
            cos_gamma = max(-1.0, min(1.0, cos_gamma))
            gamma = math.acos(cos_gamma)
            
            # Elbow Servo Angle (180 - gamma) usually
            # But Check orientation.
            # If Arm is straight, gamma=180? No, cos(180)=-1. (L1+L2)^2 ... 
            # h^2 = L1^2 + L2^2 - 2L1L2(-1) = (L1+L2)^2. Correct.
            # So Gamma is angle AT elbow. 180 is straight.
            q_elbow = math.degrees(gamma)
            
            # Alpha (Shoulder angle from vector h)
            # L2^2 = L1^2 + h^2 - 2*L1*h*cos(alpha)
            cos_alpha = (L1*L1 + h_sq - l2_eff*l2_eff) / (2 * L1 * h)
            cos_alpha = max(-1.0, min(1.0, cos_alpha))
            alpha = math.acos(cos_alpha)
            
            # Beta (Angle of h above horizon)
            beta = math.atan2(z_local, r)
            
            # Shoulder Servo Angle = Beta + Alpha (Elbow Up) or Beta - Alpha (Elbow Down)
            # Let's try Beta + Alpha (Standard Elbow Up/Back)
            q_shoulder = math.degrees(beta + alpha)
            
            # Map coordinates to Servos (0-180)
            # S6 (Shoulder): 90 is Up. 180 is Forward (if X-axis).
            # beta+alpha gives angle from Horizon? 
            # If Z is huge, beta ~ 90. Alpha ~ 0. Shoulder ~ 90. Correct.
            # So S6 = q_shoulder?
            # User inverted logic: <90 is Forward.
            # If we want Forward (180 old), we need 0.
            # My q_shoulder is 0 (Back) to 180 (Forward) standard.
            # So IK output should be 180 - q_shoulder?
            # Wait, 90 is Up. 
            # If q_shoulder = 90. Output 90.
            # If q_shoulder = 180 (Forward). Output 0.
            # So yes: s6_out = 180 - q_shoulder? Or just keep it standard and let FK invert it?
            # The IK solver finds 'q` in standard frame.
            # We return standard angles (0-180 compliant with robot geometry).
            # The FK 'inverts' input. So IK must produce output that matches the invert.
            # Standard: 90=Up, 180=Forward.
            # Inverted: 90=Up, 0=Forward.
            # So if `q_shoulder` (Standard) says 180. We need 0.
            # If `q` says 0. We need 180.
            # So `s6 = 180 - q_shoulder`.
            
            q_s6 = 180 - q_shoulder 
            
            # S5 (Elbow):
            # Gamma is internal angle. 180 is straight. 90 is bent.
            # Servo 90 usually means 90 deg bend?
            # If 180 is straight.
            # q_elbow is Gamma (180 for straight).
            # Servo mapping: 90 is 90 bend. 180 is Straight?
            # Let's assume S5 90 is "Up/Bent 90". 180 is Straight out.
            # Then S5 = q_elbow?
            # Inverted: <90 is Forward.
            # If Straight (Forward reach), standard is 180. Inverted is 0.
            # So `s5 = 180 - q_elbow`.
            
            q_s5 = 180 - q_elbow
            
            # S3 (Wrist 2) - Part of the chain. 
            # We assumed it's straight extension. Standard: 180? Or 90?
            # Usually center is 90.
            # If straight extension, let's look at geometry.
            # If S3=90 is straight.
            # We just return 90.
            
            q_s3 = 90
            
            # S4 (Lateral) - Keep 90 (Straight)
            q_s4 = 90
            
            # S2 (Roll) - Keep existing or target orientation?
            # IK doesn't spec orientation. Keep 90.
            q_s2 = 90
            
            # S1 (Tip) - Keep 90.
            q_s1 = 90
            
            # Check range
            if 0 <= angle_base_a <= 180 and 0 <= q_s6 <= 180 and 0 <= q_s5 <= 180:
                # Found valid
                # Order: S1, S2, S3, S4, S5, S6, S7
                return [q_s1, q_s2, q_s3, q_s4, q_s5, q_s6, int(base_angle)]
                
        return None
