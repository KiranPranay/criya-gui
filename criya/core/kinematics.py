from __future__ import annotations
import math
import numpy as np
import logging
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
        
        # Load from config if available (Geometry section or Root)
        # Some configs have flat keys, some have "geometry" dict
        geo_config = config.get("geometry", config) 
        self.update_lengths(geo_config)
        
        # Calibration offsets (degrees)
        self.calib = [90, 130, 110, 100, 95, 95, 90]
        if "calibration" in config:
            self.calib = config["calibration"]

    def update_lengths(self, cfg: dict):
        # Base
        if "L_BASE" in cfg: self.L_BASE = float(cfg["L_BASE"])
        elif "link_base" in cfg: self.L_BASE = float(cfg["link_base"])
        
        # Humerus
        if "L_HUMERUS" in cfg: self.L_HUMERUS = float(cfg["L_HUMERUS"])
        elif "link_humerus" in cfg: self.L_HUMERUS = float(cfg["link_humerus"])
        
        # Seg1 (Forearm) - Prioritize link_forearm (75) over link_seg1 (70) if both exist?
        # Usually link_forearm is the intended user value in this older config format
        if "L_SEG1" in cfg: self.L_SEG1 = float(cfg["L_SEG1"])
        elif "link_forearm" in cfg: self.L_SEG1 = float(cfg["link_forearm"])
        elif "link_seg1" in cfg: self.L_SEG1 = float(cfg["link_seg1"])
        
        # Seg2
        if "L_SEG2" in cfg: self.L_SEG2 = float(cfg["L_SEG2"])
        elif "link_seg2" in cfg: self.L_SEG2 = float(cfg["link_seg2"])
        
        # Seg3
        if "L_SEG3" in cfg: self.L_SEG3 = float(cfg["L_SEG3"])
        elif "link_seg3" in cfg: self.L_SEG3 = float(cfg["link_seg3"])
        
        # Tool
        if "L_TOOL" in cfg: self.L_TOOL = float(cfg["L_TOOL"])
        elif "link_tool" in cfg: self.L_TOOL = float(cfg["link_tool"])

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
        # Use Ideal Angle
        val7 = self.get_ideal(angles[6], 6)
        theta1 = math.radians(val7 - 90)
        T = T @ rot_z(theta1) @ trans(0, 0, self.L_BASE)
        
        # 2. Shoulder (S6) - PITCH (Rot Y)
        val6 = self.get_ideal(angles[5], 5)
        theta2 = math.radians(90 - val6)
        T = T @ rot_y(theta2) @ trans(0, 0, self.L_HUMERUS)
        
        # 3. Elbow (S5) - PITCH (Rot Y)
        val5 = self.get_ideal(angles[4], 4)
        theta3 = math.radians(90 - val5)
        T = T @ rot_y(theta3) @ trans(0, 0, self.L_SEG1)
        
        # 4. Wrist 1 (S4) - LATERAL BEND (Rot X)
        val4 = self.get_ideal(angles[3], 3)
        theta4 = math.radians(val4 - 90)
        T = T @ rot_x(theta4) @ trans(0, 0, self.L_SEG2) 
        
        # 5. Wrist 2 (S3) - PITCH (Rot Y)
        val3 = self.get_ideal(angles[2], 2)
        theta5 = math.radians(90 - val3)
        T = T @ rot_y(theta5) @ trans(0, 0, self.L_SEG3)
        
        # 6. Wrist 3 (S2) - ROLL (Rot X / Parallel to Tool)
        val2 = self.get_ideal(angles[1], 1)
        theta6 = math.radians(val2 - 90)
        T = T @ rot_x(theta6)
        
        # 7. Tool Offset (Fixed)
        # Forward (Z-axis of the last frame, collinear with arm)
        T = T @ trans(0, 0, self.L_TOOL)
        
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



    def check_solution(self, s6, s5, s3):
        # 4. Check Limits (Strict)
        # We DO NOT clamp large errors. If the math says -5 deg, and we clamp to 0, 
        # the physical tip will be centimeters away from target.
        # Allow minute float error only.
        tolerance = 0.1
        if (-tolerance <= s6 <= 180+tolerance) and \
           (-tolerance <= s5 <= 180+tolerance) and \
           (-tolerance <= s3 <= 180+tolerance):
             # Safe to clamp tiny epsilon
             s6 = max(0, min(180, s6))
             s5 = max(0, min(180, s5))
             s3 = max(0, min(180, s3))
             return [s6, s5, s3]
        return None

    def inverse_kinematics(self, x: float, y: float, z: float, current_angles: Optional[List[float]] = None) -> Optional[List[float]]:
        candidates_input = []
        
        theta1 = math.atan2(y, x)
        angle_base = math.degrees(theta1) + 90
        if not (-0.1 <= angle_base <= 180.1):
             return None
        angle_base = max(0, min(180, angle_base))

        r_total = math.sqrt(x*x + y*y)
        z_local = z - self.L_BASE
        
        # Adaptive Strategies: Iterative Pitch Search
        # We scan a wide range of pitches (-90 Down to +120 Back)
        # We take the FIRST solution that strictly respects geometry and limits.
        
        # Sort pitches to prefer near-horizontal (0) first, then outward.
        # e.g. 0, 5, -5, 10, -10 ...
        scan = list(range(-90, 121, 5))
        pitches = sorted(scan, key=lambda x: abs(x))
        
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
            
            # Wrist Target
            r_wrist = r_total - H_x
            z_wrist = z_local - H_z
            
            # Solve 2-Link
            res = self._solve_2link(r_wrist, z_wrist, L1, L2_arm)
            if not res: 
                continue
                
            alpha, gamma, beta = res
            
            # Solve Angles
            q_shoulder = math.degrees(beta + alpha)
            q_elbow = math.degrees(gamma)
            
            s6 = q_shoulder
            s5 = q_elbow - 90
            s3 = 270 - (q_shoulder + q_elbow) + pitch_deg
            
            # Strictly Check
            cleaned = self.check_solution(s6, s5, s3)
            if cleaned:
                logging.info(f"IK Found Solution at Pitch {pitch_deg}: S6={cleaned[0]:.1f}, S5={cleaned[1]:.1f}, S3={cleaned[2]:.1f}")
                # Need to map geometric angles (ideal) to Raw Servo Values
                
                # Base (Idx 6)
                raw_base = self.get_raw(angle_base, 6)
                # Shoulder (Idx 5) - s6 computed is geometric
                raw_s6 = self.get_raw(cleaned[0], 5)
                # Elbow (Idx 4) - s5 computed is geometric
                raw_s5 = self.get_raw(cleaned[1], 4)
                # Wrist (Idx 2) - s3 computed is geometric
                raw_s3 = self.get_raw(cleaned[2], 2)
                
                # Fixed Joints (S4, S2, S1) should be at Geometric 90
                raw_s4 = self.get_raw(90, 3)
                raw_s2 = self.get_raw(90, 1)
                raw_s1 = self.get_raw(90, 0)
                
                # Construct Solution
                sol = [0] * 7
                sol[6] = int(raw_base)
                sol[5] = int(raw_s6)
                sol[4] = int(raw_s5)
                sol[3] = int(raw_s4)
                sol[2] = int(raw_s3)
                sol[1] = int(raw_s2)
                sol[0] = int(raw_s1)
                
                valid_solutions.append(sol)
                # Found the "flattest" valid pitch. Use it.
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
