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



    def _solve_2link(self, r: float, z: float, L1: float, L2: float) -> Optional[tuple]:
        # Returns (alpha, gamma, beta) in radians
        # z is relative to shoulder
        h_sq = r*r + z*z
        h = math.sqrt(h_sq)
        
        if h > (L1 + L2) or h < abs(L1 - L2):
            return None
            
        try:
            cos_gamma = (L1*L1 + L2*L2 - h_sq) / (2 * L1 * L2)
            cos_gamma = max(-1.0, min(1.0, cos_gamma))
            gamma = math.acos(cos_gamma)
            
            cos_alpha = (L1*L1 + h_sq - L2*L2) / (2 * L1 * h)
            cos_alpha = max(-1.0, min(1.0, cos_alpha))
            alpha = math.acos(cos_alpha)
            
            beta = math.atan2(z, r)
            return (alpha, gamma, beta)
        except ValueError:
            return None

    def _is_valid_angle(self, deg: float) -> bool:
        # Allow slight leniency 
        return -5 <= deg <= 185

    def check_solution(self, s6, s5, s3):
        # 4. Check Limits (Strict)
        if self._is_valid_angle(s6) and self._is_valid_angle(s5) and self._is_valid_angle(s3):
             # Clamp output to 0-180
             s6 = max(0, min(180, s6))
             s5 = max(0, min(180, s5))
             s3 = max(0, min(180, s3))
             return [s6, s5, s3]
        return None

    def inverse_kinematics(self, x: float, y: float, z: float, current_angles: Optional[List[float]] = None) -> Optional[List[float]]:
        candidates_input = []
        
        theta1 = math.atan2(y, x)
        angle_base = math.degrees(theta1) + 90
        if not self._is_valid_angle(angle_base):
             # Try 180 flip?
             return None
        angle_base = max(0, min(180, angle_base))

        r_total = math.sqrt(x*x + y*y)
        z_local = z - self.L_BASE
        
        # Define Strategies
        # 1. Inline Tool (Max Reach) - S3 Fixed 90
        # 2. Horizontal Tool (Close Body) - S3 Variable
        
        # Adaptive Strategies: Try Pointing Tool at different pitches (0=Horiz, 90=Vert).
        # We start with 0 because user prefers horizontal.
        # If reachable, we stop and use it.
        # If not, we tilt up (30, 45...) until reachable.
        
        pitches = [0, 30, 45, 60, 90]
        
        # Hand Length (Pivot S3 to Tip) is Seg3 + Tool
        H_L = self.L_SEG3 + self.L_TOOL
        L1 = self.L_HUMERUS
        # L2 for 2-link is Seg1 + Seg2.
        L2_arm = self.L_SEG1 + self.L_SEG2
        
        valid_solutions = []
        
        for pitch_deg in pitches:
            pitch_rad = math.radians(pitch_deg)
            
            # Decompose Hand Vector
            H_x = H_L * math.cos(pitch_rad)
            H_z = H_L * math.sin(pitch_rad)
            
            # Wrist Target (S3 Position)
            # R_s3 = R_tip - H_x
            # Z_s3 = Z_tip - H_z
            # (Note: Z_tip is local relative to base shoulder)
            r_wrist = r_total - H_x
            z_wrist = z_local - H_z
            
            # Solve 2-Link for S3 position
            res = self._solve_2link(r_wrist, z_wrist, L1, L2_arm)
            if not res: 
                continue
                
            alpha, gamma, beta = res
            
            # Solve Angles (Standard Elbow Up)
            q_shoulder = math.degrees(beta + alpha)
            q_elbow = math.degrees(gamma)
            
            # Map to Servos
            s6 = q_shoulder
            s5 = q_elbow - 90
            
            # S3 Logic for Defined Pitch
            # We want Global Tool Pitch = pitch_deg (Angle from Horizon)
            # Global rotations Sum = 90 - pitch_deg.
            # Sum = (90-q_s) + (180-q_e) + (90-s3) = 360 - q_s - q_e - s3.
            # 90 - pitch = 360 - q_s - q_e - s3
            # s3 = 360 - 90 - q_s - q_e + pitch
            # s3 = 270 - (q_shoulder + q_elbow) + pitch_deg
            
            s3 = 270 - (q_shoulder + q_elbow) + pitch_deg
            
            # Check and Clamp
            cleaned = self.check_solution(s6, s5, s3)
            if cleaned:
                # Found a valid strategy!
                sol = [90]*7
                sol[6] = int(angle_base)
                sol[5] = int(cleaned[0])
                sol[4] = int(cleaned[1])
                sol[2] = int(cleaned[2]) # S3 is Index 2
                valid_solutions.append(sol)
                # Since we prioritize small pitch, we can break early if we just want "First Working"
                # But to maintain old structure of "valid_solutions", we can collect and break?
                # Actually, breaking here is best efficiently.
                break
                
        if not valid_solutions:
            return None
            
        # Optimization: Choose best solution
        if current_angles:
            best_sol = None
            min_cost = float('inf')
            weights = [1.0] * 7
            weights[6] = 2.0; weights[5] = 1.5
            
            for sol in valid_solutions:
                cost = 0
                for i in range(7):
                    diff = abs(sol[i] - current_angles[i])
                    cost += diff * weights[i]
                if cost < min_cost:
                    min_cost = cost
                    best_sol = sol
            return best_sol
            
        return valid_solutions[0]
