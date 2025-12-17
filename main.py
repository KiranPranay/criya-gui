# main.py  — PySide6 light UI, scrollable columns, corrected backgrounds & alignment

from __future__ import annotations
import sys, json, datetime, math, os, subprocess, logging
import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Optional
from dataclasses import dataclass
from PySide6 import QtCore, QtGui, QtWidgets

# -------------------- optional qt-material (not required) --------------------
try:
    from qt_material import apply_stylesheet  # type: ignore
    HAVE_QT_MATERIAL = True
except Exception:
    HAVE_QT_MATERIAL = False

CONFIG_FILE = "config.json"

# -------------------- Matrix Helpers --------------------
def rot_x(rad: float) -> np.ndarray:
    c = math.cos(rad)
    s = math.sin(rad)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ])

def rot_y(rad: float) -> np.ndarray:
    c = math.cos(rad)
    s = math.sin(rad)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ])

def rot_z(rad: float) -> np.ndarray:
    c = math.cos(rad)
    s = math.sin(rad)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans(x: float, y: float, z: float) -> np.ndarray:
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

# -------------------- serial --------------------
try:
    import serial
    import serial.tools.list_ports
    HAVE_SERIAL = True
except Exception:
    serial = None
    HAVE_SERIAL = False


# -------------------- theme (Material-ish light) --------------------
PRIMARY   = "#5B6AC4"  # replace with your brand color
SECONDARY = "#00B8A9"
ACCENT    = "#FF8A00"
BG        = "#F9FAFB"  # page
SURFACE   = "#FFFFFF"  # cards
TEXT      = "#1F1F1F"
TEXT_MUT  = "#5F6368"
BORDER    = "#E6E8EB"
OK_GREEN  = "#2E7D32"
WARN_AMB  = "#F9A825"
ERR_RED   = "#D32F2F"

QSS = f"""
/* Basics */
QWidget {{
  background: {BG};
  color: {TEXT};
  font-size: 14px;
}}
QScrollArea, QScrollArea > QWidget, QScrollArea > QWidget > QWidget {{
  background: {BG};
}}
/* Cards */
QGroupBox {{
  background: {SURFACE};
  border: 1px solid {BORDER};
  border-radius: 10px;
  margin-top: 10px;
}}
QGroupBox::title {{
  subcontrol-origin: margin;
  left: 12px;
  padding: 4px 6px;
  color: {TEXT_MUT};
  font-weight: 600;
}}
/* Inputs */
QLineEdit, QComboBox {{
  background: #fff;
  border: 1px solid {BORDER};
  border-radius: 8px;
  padding: 8px 10px;
}}
QComboBox QAbstractItemView {{
  background: #fff;
  border: 1px solid {BORDER};
}}
/* Buttons */
QPushButton {{
  background: {PRIMARY};
  color: #fff;
  border: none;
  border-radius: 8px;
  padding: 9px 14px;
  min-height: 34px;
}}
QPushButton[flat="true"] {{    /* secondary/flat */
  background: #EEF0F3;
  color: {TEXT};
}}
QPushButton[warn="true"] {{    /* warning */
  background: {WARN_AMB};
  color: #000;
}}
QPushButton[danger="true"] {{  /* danger */
  background: {ERR_RED};
  color: #fff;
}}
QPushButton:disabled {{
  background: #CFD4DA; color: #fff;
}}
/* Sliders */
QSlider::groove:horizontal {{
  height: 6px; background: #E7E9EC; border-radius: 3px;
}}
QSlider::handle:horizontal {{
  background: {SECONDARY};
  width: 18px; height: 18px; margin: -6px 0; border-radius: 9px;
}}
/* Table */
QHeaderView::section {{
  background: #F3F5F8; color: {TEXT};
  padding: 8px; border: 1px solid {BORDER};
}}
QTableWidget {{
  background: #fff;
  gridline-color: {BORDER};
  selection-background-color: #D9F3EF;
  selection-color: {TEXT};
}}
/* Splitter */
QSplitter::handle {{
  background: {BG};      /* remove the dark bar */
  width: 8px;
}}
QStatusBar {{
  background: transparent; color: {TEXT};
}}
"""


# -------------------- data --------------------
@dataclass
class SeqStep:
    kind: str                   # "pos" or "angles"
    dwell_ms: int
    speed: int                  # 1..100 (lower = faster)
    pos_name: Optional[str] = None
    angles: Optional[List[int]] = None
    label: str = ""


# -------------------- serial helper --------------------
class SerialLink(QtCore.QObject):
    statusChanged = QtCore.Signal(str, str)
    portsChanged  = QtCore.Signal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ser: Optional[serial.Serial] = None
        self.debounce_timers = [None]*7
        self.last_sent = [None]*7
        self.debounce_ms = 25
        self.port = ""; self.baud = 115200

    def available_ports(self) -> List[str]:
        if not HAVE_SERIAL: return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        self.portsChanged.emit(self.available_ports())

    def connect(self, port: str, baud: int):
        if not HAVE_SERIAL:
            self.statusChanged.emit("pyserial not installed", ERR_RED); return
        try:
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05, write_timeout=0.05)
            self.port, self.baud = port, baud
            self.statusChanged.emit(f"Connected to {port} @ {baud}", OK_GREEN)
        except Exception as e:
            self.statusChanged.emit(f"Error: {e}", ERR_RED)

    def disconnect(self):
        try:
            if self.ser and self.ser.is_open: self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.statusChanged.emit("Disconnected", ERR_RED)

    def force_reconnect(self):
        if self.port and self.baud:
            self.disconnect()
            QtCore.QTimer.singleShot(300, lambda: self.connect(self.port, self.baud))
        else:
            self.statusChanged.emit("Pick a port & baud first", WARN_AMB)

    def _write_raw(self, b: bytes):
        if not self.ser or not self.ser.is_open: return
        try:
            self.ser.write(b)
        except Exception as e:
            self.statusChanged.emit(f"Write error: {e}. Reconnecting…", ERR_RED)
            self.force_reconnect()

    def send_angle(self, idx: int, angle: int):
        t: Optional[QtCore.QTimer] = self.debounce_timers[idx]
        if t: t.stop()
        t = QtCore.QTimer(self); t.setSingleShot(True); t.setInterval(self.debounce_ms)
        def _do():
            if self.last_sent[idx] != angle:
                self._write_raw(f"{idx+1} {angle}\n".encode()); self.last_sent[idx] = angle
        t.timeout.connect(_do); t.start()
        self.debounce_timers[idx] = t

    def send_all(self, angles: List[int]):
        # Use new Sync Write command 'M'
        # Format: M <a1> <a2> ... <a7>\n
        if not self.ser or not self.ser.is_open: return
        msg = "M " + " ".join(map(str, angles)) + "\n"
        self._write_raw(msg.encode())
        logging.info(f"TX: {msg.strip()}")
        # Update last_sent to avoid redundant individual updates immediately after
        for i, a in enumerate(angles):
            self.last_sent[i] = a


# -------------------- Kinematics & Safety --------------------
class Kinematics:
    def __init__(self, config: dict):
        self.config = config
        # Default Link Lengths (mm) - User should configure these
        # Custom 5-Link Geometry
        self.L_BASE = float(self.config.get("link_base", 120))      # Base Height
        self.L_HUMERUS = float(self.config.get("link_humerus", 120)) # Shoulder to Elbow
        
        # Forearm Segments
        self.L_SEG1 = float(self.config.get("link_seg1", 70))  # Elbow -> Wrist 1
        self.L_SEG2 = float(self.config.get("link_seg2", 85))  # Wrist 1 -> Wrist 2
        self.L_SEG3 = float(self.config.get("link_seg3", 75))  # Wrist 2 -> Tip
        
        # Calculated effective forearm for IK (sum of segments)
        self.L_FOREARM = self.L_SEG1 + self.L_SEG2 + self.L_SEG3 
        
        # Tool Offset (Gripper)
        self.L_TOOL = float(self.config.get("link_tool", 120))
        
        # Calibration: Value at 90 degrees
        self.calib = self.config.get("calib", [90, 130, 110, 100, 95, 95, 90])
        
    def update_lengths(self, cfg: dict):
        self.L_BASE = float(cfg.get("link_base", self.L_BASE))
        self.L_HUMERUS = float(cfg.get("link_humerus", self.L_HUMERUS))
        self.L_SEG1 = float(cfg.get("link_seg1", self.L_SEG1))
        self.L_SEG2 = float(cfg.get("link_seg2", self.L_SEG2))
        self.L_SEG3 = float(cfg.get("link_seg3", self.L_SEG3))
        self.L_TOOL = float(cfg.get("link_tool", self.L_TOOL))
        
        # Recalculate sum
        self.L_FOREARM = self.L_SEG1 + self.L_SEG2 + self.L_SEG3
        self.calib = cfg.get("calib", self.calib)

    def get_ideal(self, raw, idx):
        # raw - (calib - 90)
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
        # So Tool points "Back" relative to world.
        # So we want Wrist to be FURTHER away? 
        # r_total (negative sign) = r_wrist + L_TOOL?
        # Wait, r is radial distance from Z axis.
        # If Target is at Y = -100. r_target = 100.
        # We point base at -90 (South).
        # Tool points South.
        # So Wrist must be at 100 - 120 = -20 (Behind Z axis in the frame)?
        # Yes.
        r_wrist_b = -r_total_a - self.L_TOOL # ??? 
        # Let's check:
        # If Target Y=-100. Target R=100. Base Angle turns to -90.
        # In frame -90: Target is at +100.
        # Tool is +120. 
        # Wrist needs to be at 100 - 120 = -20.
        # So we pass -20 to Planar Solver.
        # Planar Solver treats -20 as "Leaning Back".
        # Yes.
        candidates.append((angle_base_b, r_total_a - self.L_TOOL)) 
        # WRONG. In Strategy 2 (Flipped), The Base points AWAY from target.
        # Target -100. Base +90 (North). 
        # Target is at -100 in Base Frame.
        # Tool is +120 in Base Frame? No, Tool rotates with Base. 
        # So Tool is +120 (North). 
        # We want Reach + Tool = -100? No.
        # Reach + 120 = -100 -> Reach = -220.
        
        # Let's simplify Strategy 2:
        # Turn Base to Target. (Strategy 1).
        # Then try to reach with Negative R. 
        # If R_wrist_a < 0, that IS strategy 2 naturally?
        
        # Actually, let's just stick to Strategy 1 (Turn to Target).
        # And allow r_wrist to be negative.
        # If r_wrist is -20, the planar solver handles it.
        # We don't need explicit Flip of base if "Leaning Back" covers it.
        
        # The only reason to Flip Base is if Servo 7 limits prevent turning to target.
        # But for 0-180 limits (Forward Only), we MUST Flip Base if Target Y < 0.
        
        # Case A: Target Y > 0. Base ~ 90. Tool Points Forward (Y+).
        # r_total = sqrt. r_wrist = r_total - Tool.
        
        # Case B: Target Y < 0. Base ~ 270 (-90). 
        # Servo 7 cannot go to -90.
        # We MUST use Base 90 (Face Forward) and Reach BACK (Negative R).
        # Tool Points Forward (Y+).
        # Target is Behind (-Y).
        # Wrist + Tool = Target.
        # Wrist + 120 = -100.
        # Wrist = -220.
        # Planar Solver needs to reach -220.
        
        # So:
        # If Base cannot face target (Y < 0), we face Forward/Nearest.
        # And solve for r_wrist.
        
        theta_target = math.atan2(y, x)
        angle_target = math.degrees(theta_target)
        
        final_candidates = []
        
        # Option 1: Face Target
        angle_1 = angle_target
        r_1 = math.sqrt(x*x + y*y)
        r_w1 = r_1 - self.L_TOOL
        final_candidates.append((angle_1, r_w1))
        
        # Option 2: Face Opposite
        angle_2 = angle_target + 180
        angle_2_norm = (angle_2 + 180) % 360 - 180
        r_2 = -r_1 # Target is "Behind"
        # Tool is "Forward" relative to base.
        # So Reach (Wrist) + Tool = Target (Negative).
        # Wrist + 120 = -r
        # Wrist = -r - 120.
        r_w2 = -r_1 - self.L_TOOL
        final_candidates.append((angle_2_norm, r_w2))
        
        for (angle_base, r_plane) in final_candidates:
            # Check Base Limit (0 to 180)
            if not (0 <= angle_base <= 180):
                continue
                
            # Solve planar (r_plane, z)
            dx = r_plane
            dy = z - self.L_BASE
            
            # Law of Cosines
            l1 = self.L_HUMERUS
            l2 = self.L_FOREARM
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > (l1 + l2) or dist == 0:
                 # Unreachable in this config
                 continue
            
            # alpha: angle between Humerus and line-to-target
            try:
                alpha = math.acos((l1*l1 + dist*dist - l2*l2) / (2 * l1 * dist))
                # beta: angle between Humerus and Forearm
                beta = math.acos((l1*l1 + l2*l2 - dist*dist) / (2 * l1 * l2))
            except ValueError:
                continue

            # gamma: angle of line-to-target vs horizon
            gamma = math.atan2(dy, dx)
            
            # Shoulder Angle (relative to horizon)
            theta_shoulder_rad = gamma + alpha # Elbow UP solution
            
            # Elbow Angle (relative to humerus)
            theta_elbow_rad = math.pi - beta
            
            angle_shoulder = math.degrees(theta_shoulder_rad)
            angle_elbow = math.degrees(theta_elbow_rad) + 90
            
            # Map to servos
            # Geometry: Gamma(0)=Forward Horizon. Gamma(90)=Up. Gamma(180)=Back Horizon.
            # Servo: 180=Forward, 90=Up, 0=Back.
            # Relation: Servo = 180 - Geometry.
            
            ideal_base = angle_base
            ideal_shoulder = 180 - angle_shoulder
            ideal_elbow = angle_elbow
            
            # Check limits (Ideal 0-180)
            if (0 <= ideal_shoulder <= 180) and (0 <= ideal_elbow <= 180):
                 # Valid Solution Found!
                 out = [90]*7
                 out[6] = int(ideal_base)
                 out[5] = int(ideal_shoulder)
                 out[4] = int(ideal_elbow)
                 return out
        
        # If no solution found
        logging.warning(f"IK Fail: No valid config for ({x},{y},{z})")
        return None

# -------------------- 3D Viz (Numpy) --------------------
class Viz3D(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111, projection='3d')
        super(Viz3D, self).__init__(self.fig)
        self.setParent(parent)

    def update_plot(self, ideal_angles: List[float], kin: Kinematics):
        self.ax.clear()
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(0, 500)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        # UI/Logic is now Ideal-based. No conversion needed here.
        
        # Helper: Rotation Matrices
        def rot_z(deg):
            rad = np.radians(deg)
            c, s = np.cos(rad), np.sin(rad)
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            
        def rot_y(deg):
            rad = np.radians(deg)
            c, s = np.cos(rad), np.sin(rad)
            return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

        def rot_x(deg):
            rad = np.radians(deg)
            c, s = np.cos(rad), np.sin(rad)
            return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

        def trans(x, y, z):
            return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

        # 0. Origin
        T = np.eye(4)
        points = [T[:3, 3]]
        
        # 1. Base (Servo 7 - Pan)
        # Rotates around Z. 90 -> 90 deg (Y axis).
        # CHANGED: 90 -> 0 deg (X axis/Forward).
        # Height: L_BASE (120)
        pan_angle = ideal_angles[6] - 90
        T = T @ rot_z(pan_angle) @ trans(0, 0, kin.L_BASE)
        points.append(T[:3, 3])
        
        # 2. Shoulder (Servo 6 - PITCH / Rot Y)
        # Inverted: 90 - Angle
        shoulder_angle = 90 - ideal_angles[5]
        T = T @ rot_y(shoulder_angle) @ trans(0, 0, kin.L_HUMERUS)
        points.append(T[:3, 3])
        
        # 3. Elbow (Servo 5 - PITCH / Rot Y)
        elbow_angle = 90 - ideal_angles[4]
        T = T @ rot_y(elbow_angle) @ trans(0, 0, kin.L_SEG1)
        points.append(T[:3, 3]) 
        
        # 4. Wrist 1 (Servo 4 - LATERAL / Rot X)
        w1_angle = ideal_angles[3] - 90
        T = T @ rot_x(w1_angle) @ trans(0, 0, kin.L_SEG2)
        points.append(T[:3, 3])
        
        # 5. Wrist 2 (Servo 3 - PITCH / Rot Y)
        w2_angle = 90 - ideal_angles[2]
        T = T @ rot_y(w2_angle) @ trans(0, 0, kin.L_SEG3)
        points.append(T[:3, 3])
        
        # 6. Wrist 3 (Servo 2 - Link/Gripper Base)
        # Rot X (Tool Roll).
        w3_angle = ideal_angles[1] - 90
        T = T @ rot_x(w3_angle) @ trans(0, 0, 0) 
        # No point append here (Same pos as S3 end).
        
        # 7. TOOL TIP (Fixed Offset)
        # X-Direction (Forward).
        T = T @ trans(kin.L_TOOL, 0, 0)
        points.append(T[:3, 3])
        
        # 8. S1 removed from Visualization.
        
        # Plot the Robot Arm Structure
        pts = np.array(points)
        print(f"DEBUG: Viz Points: {len(pts)}")
        
        # Draw Line (Skeleton) all the way to Tip
        self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-', linewidth=4, color='tab:blue')
        
        # Draw Markers (Joints) stopping before the Tip (remove last point)
        self.ax.plot(pts[:-1, 0], pts[:-1, 1], pts[:-1, 2], 'o', markersize=6, color='tab:blue') 
        
        # Plot Orientation Triad at the Tip
        R = T[:3, :3]
        P = T[:3, 3]
        scale = 40 
        
        # X-Axis (Red)
        px = P + R @ np.array([scale, 0, 0])
        self.ax.plot([P[0], px[0]], [P[1], px[1]], [P[2], px[2]], 'r-', linewidth=2)
        
        # Y-Axis (Green)
        py = P + R @ np.array([0, scale, 0])
        self.ax.plot([P[0], py[0]], [P[1], py[1]], [P[2], py[2]], 'g-', linewidth=2)
        
        # Z-Axis (Blue)
        pz = P + R @ np.array([0, 0, scale])
        self.ax.plot([P[0], pz[0]], [P[1], pz[1]], [P[2], pz[2]], 'b-', linewidth=2)
        
        # S1 Viz Removed.
        # If we stick a tool "Perpendicular" to the last segment?
        # The user says "impace X by 120".
        # If Arm is Up. Tool is X=120.
        # My Viz chain: Pan (Z) -> Pitch (Y). Forward is ... Y-axis world?
        # If Pitch=0. Local Z is Up. 
        # A Perpendicular Tool would be in Local Y (Forward) or X (Right)?
        # Let's assume Local X (which corresponds to World Y if Pan=0? No).
        # Let's align with the Base Radial direction.
        # If Base Angle = 90 (Y-axis). Tool should be Y direction.
        # Our FK code adds L_TOOL to 'r'.
        # 'r' is the "horizontal projection".
        # So Tool is effectively lengthening the Reach in the Plane of the arm.
        # In Viz: If arm bends, L_TOOL should bend with it?
        # Or is L_TOOL fixed to Servo 2?
        # "Impacting X by 120" usually means radial reach.
        # So it's effectively an extension along the gripper axis.
        # If S2 is Roll, and Gripper is attached to S2.
        # Let's draw it along Local Z? No that would be inline.
        # Let's draw it perpendicular... 
        # But math says r += 120. That means it acts like an extension of the arm reach, but 
        # "Tip is out of straight line" suggests it's NOT linear.
        # It's an OFFSET. 
        # But if it always adds 120 to Reach, it acts like a horizontal bar at the end.
        # Let's assume it's a horizontal offset relative to the *Ground* (parallel to XY)? 
        # OR perpendicular to the last link? 
        # Re-reading: "From (0,0,0) to tip... perpendicular distance is 120".
        # This implies horizontal distance from vertical axis.
        # If arm is vertical. Tip is at 120 horizontally.
        # If arm is horizontal. Tip is at 120.... from what?
        # If it adds to Reach, it implies it's ALONG the arm direction?
        # NO. "Tip is out of straight line".
        # This means L-shape.
        # If Arm is Up (Z), Tool is Sideways (Y or X).
        # If Arm is Forward (Y), Tool is Down (-Z)? Or Sideways (X)?
        # User: "Impacting X by 120" (when upright?).
        # I will assume it's a Right-Angle bracket.
        # And it points "Forward" relative to the arm plane?
        # Viz: Draw a line in Local Y? (If Local Z is arm axis).
        # Actually... if FK logic `r += L_TOOL`, this implicitly models it as a Horizontal Extension *Parallel to Ground*?
        # No, `r += L_TOOL` in my FK code treats it as `r` (Projected) + Offset.
        # If the offset is physically attached to the last link (S2), it rotates with S2.
        # If S2 is Up (90), Tool is Horizontal.
        # If S2 is Forward (0), Tool is Vertical Down? 
        # If it's vertical down, it does NOT add to reach.
        # BUT User IK Request implies they want to control value.
        # I will Model it in Viz as a green line 120mm "Sideways" from wrist.
        T = T @ trans(0, kin.L_TOOL, 0) # Try Local Y
        points.append(T[:3, 3])

        # Plot
        pts = np.array(points)
        self.ax.plot(pts[:,0], pts[:,1], pts[:,2], 'o-', linewidth=4, markersize=6, label='Arm')
        
        # Draw Base Ref
        self.ax.plot([0,0],[0,0],[0,0], 'k^') 
        # Draw Axes
        self.ax.plot([0,100],[0,0],[0,0], 'r-', label='X')
        self.ax.plot([0,0],[0,100],[0,0], 'g-', label='Y')
        self.draw()


# -------------------- main window --------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Criya – Stack Assembly Robot (Qt)")
        self.resize(1400, 900)
        self.setMinimumSize(1200, 800)

        # state
        self.link = SerialLink(self)
        self.angles = [90]*7
        self.reset_defaults = [0,90,90,90,90,90,90]
        self.saved_positions: Dict[str, List[int]] = {}
        self.sequence: List[SeqStep] = []
        
        # Config
        self.config = self.load_config()
        self.kin = Kinematics(self.config)
        
        # Smooth Manual Control State
        self.current_servo_pos = [90.0]*7
        self.control_timer = QtCore.QTimer(self)
        self.control_timer.timeout.connect(self._control_loop)
        self.control_timer.start(20) # 50Hz control loop

        # smooth move (timer-based) -> Visual Animation Only now
        self.smooth_timer = QtCore.QTimer(self)
        self.smooth_timer.timeout.connect(self._smooth_step)
        self.smooth_steps = 40
        self.smooth_step_idx = 0
        self.smooth_start = self.angles[:]
        self.smooth_target = self.angles[:]
        self.smooth_delay_ms = 120
        self.move_done_callbacks: List[callable] = []

        # sequence
        self.seq_running = False
        self.seq_index = 0
        self.dwell_timer = QtCore.QTimer(self); self.dwell_timer.setSingleShot(True)
        self.dwell_timer.timeout.connect(self._seq_next)

        # UI
        self._build_ui()

        # signals
        self.link.statusChanged.connect(self._set_status)
        self.link.portsChanged.connect(self._fill_ports)
        self._fill_ports(self.link.available_ports())
        self._set_status("Disconnected", ERR_RED)

        # connection monitor
        self._conn_timer = QtCore.QTimer(self)
        self._conn_timer.timeout.connect(self._check_connection)
        self._conn_timer.start(1000)

    def load_config(self) -> dict:
        if os.path.exists(CONFIG_FILE):
            try: return json.load(open(CONFIG_FILE))
            except: pass
        return {
            "arduino_path": r"D:\programs\arduino\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe", 
            "board": "arduino:avr:uno",
            "link_base": 120, "link_humerus": 120, 
            "link_base": 120, "link_humerus": 120, 
            "link_seg1": 70, "link_seg2": 85, "link_seg3": 75,
            "link_tool": 120,
            "calib": [90, 130, 110, 100, 95, 95, 90]
        }

    def save_config(self):
        json.dump(self.config, open(CONFIG_FILE, "w"), indent=2)
        if hasattr(self, 'kin'): self.kin.update_lengths(self.config)

    # ---------- ui ----------
    def _build_ui(self):
        splitter = QtWidgets.QSplitter(self)
        splitter.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.setCentralWidget(splitter)

        # LEFT SIDE LAYOUT (Tabs)
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget); left_layout.setContentsMargins(0,0,0,0)
        
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._tab_controls(), "Controls")
        self.tabs.addTab(self._tab_ik(), "Inverse Kinematics")
        self.tabs.addTab(self._tab_settings(), "Settings")
        
        left_layout.addWidget(self.tabs)

        # RIGHT SIDE LAYOUT (3D Viz + Servos)
        right_scroll = QtWidgets.QScrollArea(); right_scroll.setWidgetResizable(True)
        right = QtWidgets.QWidget(); right_scroll.setWidget(right)
        R = QtWidgets.QVBoxLayout(right); R.setContentsMargins(16,16,16,16); R.setSpacing(14)
        
        # 3D Viz
        self.viz = Viz3D(self, width=5, height=5, dpi=100)
        R.addWidget(self.viz)
        R.addWidget(self._grp_servos())
        R.addStretch(1)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_scroll)
        splitter.setStretchFactor(0, 1); splitter.setStretchFactor(1, 1)

        # status bar
        self.status_label = QtWidgets.QLabel("")
        self.statusBar().addWidget(self.status_label, 1)

    # --- Tabs ---
    def _tab_controls(self):
        page = QtWidgets.QWidget()
        L = QtWidgets.QVBoxLayout(page); L.setContentsMargins(16,16,16,16); L.setSpacing(14)
        
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget(); scroll.setWidget(content)
        v = QtWidgets.QVBoxLayout(content)
        
        v.addWidget(self._grp_connection())
        v.addWidget(self._grp_firmware())
        v.addWidget(self._grp_speed())
        v.addWidget(self._grp_defaults())
        v.addWidget(self._grp_actions())
        v.addWidget(self._grp_positions())
        v.addWidget(self._grp_sequence())
        v.addStretch(1)
        
        L.addWidget(scroll)
        return page

    def _tab_ik(self):
        page = QtWidgets.QWidget()
         # Form
        layout = QtWidgets.QVBoxLayout(page)
        
        box = QtWidgets.QGroupBox("Cartesian Control (XYZ)")
        form = QtWidgets.QGridLayout()
        
        self.ikX = QtWidgets.QLineEdit("0.0")
        self.ikY = QtWidgets.QLineEdit("0.0")
        self.ikZ = QtWidgets.QLineEdit("470.0") # Default HOME
        
        # Checkboxes for "Active" (If unchecked, treat as Optional/Hold Current)
        self.chkX = QtWidgets.QCheckBox("Use")
        self.chkY = QtWidgets.QCheckBox("Use")
        self.chkZ = QtWidgets.QCheckBox("Use")
        self.chkX.setChecked(True)
        self.chkY.setChecked(True)
        self.chkZ.setChecked(True)
        
        form.addWidget(QtWidgets.QLabel("X (mm)"), 0, 0)
        form.addWidget(self.ikX, 0, 1)
        form.addWidget(self.chkX, 0, 2)
        
        form.addWidget(QtWidgets.QLabel("Y (mm)"), 1, 0)
        form.addWidget(self.ikY, 1, 1)
        form.addWidget(self.chkY, 1, 2)
        
        form.addWidget(QtWidgets.QLabel("Z (mm)"), 2, 0)
        form.addWidget(self.ikZ, 2, 1)
        form.addWidget(self.chkZ, 2, 2)
        
        btn_go = QtWidgets.QPushButton("Move to Point")
        btn_go.clicked.connect(self._solve_ik)
        
        form.addWidget(btn_go, 3, 0, 1, 3)
        
        box.setLayout(form)
        layout.addWidget(box)
        layout.addStretch(1)
        return page

    def _tab_settings(self):
        page = QtWidgets.QWidget()
        L = QtWidgets.QVBoxLayout(page); L.setContentsMargins(16,16,16,16)
        
        g = self._card("Robot Geometry (mm)")
        form = QtWidgets.QFormLayout()
        
        self.stBase = QtWidgets.QLineEdit(str(self.config.get("link_base", 120)))
        self.stHum  = QtWidgets.QLineEdit(str(self.config.get("link_humerus", 120)))
        
        self.stSeg1 = QtWidgets.QLineEdit(str(self.config.get("link_seg1", 70)))
        self.stSeg2 = QtWidgets.QLineEdit(str(self.config.get("link_seg2", 85)))
        self.stSeg3 = QtWidgets.QLineEdit(str(self.config.get("link_seg3", 75)))
        self.stTool = QtWidgets.QLineEdit(str(self.config.get("link_tool", 120)))
        
        form.addRow("Base Height (L1)", self.stBase)
        form.addRow("Shoulder->Elbow (L2)", self.stHum)
        form.addRow("Elbow->Wrist1 (S1)", self.stSeg1)
        form.addRow("Wrist1->Wrist2 (S2)", self.stSeg2)
        form.addRow("Link 3 (S3)", self.stSeg3)
        form.addRow("Gripper Offset (Tool)", self.stTool)
        form.addRow("Base Height (L1)", self.stBase)
        form.addRow("Shoulder->Elbow (L2)", self.stHum)
        form.addRow("Elbow->Wrist1 (S1)", self.stSeg1)
        form.addRow("Wrist1->Wrist2 (S2)", self.stSeg2)
        form.addRow("Link 3 (Gripper Tip)", self.stSeg3)

        g2 = self._card("Calibration (Val at 90°)")
        form2 = QtWidgets.QFormLayout()
        self.calibEdits = []
        current_calib = self.config.get("calib", [90, 130, 110, 100, 95, 95, 90])
        
        # CHANGED: Order 1 to 7 (ascending) as per user request
        for i in range(7):
            idx = i # 0..6 (Servo 1..7)
            le = QtWidgets.QLineEdit(str(current_calib[idx]))
            form2.addRow(f"Servo {idx+1}", le)
            self.calibEdits.append(le)
        
        btn_save = QtWidgets.QPushButton("Save Settings")
        btn_save.clicked.connect(self._save_settings)
        
        g.layout().addLayout(form)
        g2.layout().addLayout(form2)
        
        L.addWidget(g)
        L.addWidget(g2)
        L.addWidget(btn_save)
        L.addStretch(1)
        return page

    # --- Actions ---
    def _save_settings(self):
        try:
            self.config["link_base"] = float(self.stBase.text())
            self.config["link_humerus"] = float(self.stHum.text())
            
            self.config["link_seg1"] = float(self.stSeg1.text())
            self.config["link_seg2"] = float(self.stSeg2.text())
            self.config["link_seg3"] = float(self.stSeg3.text())
            self.config["link_tool"] = float(self.stTool.text())
            
            # Save Calib
            # calibEdits: [S1, S2, S3, S4, S5, S6, S7] (Now ascending)
            new_calib = [90]*7
            for i, le in enumerate(self.calibEdits):
                new_calib[i] = int(float(le.text()))
            
            self.config["calib"] = new_calib
            
            self.save_config()
            self._set_status("Settings Saved", OK_GREEN)
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Invalid", "Values must be numbers")

    def _solve_ik(self):
        try:
            # Determine Targets
            # If Checked -> Use Field
            # If Unchecked -> Use Current Robot Position (Partial IK)
            
            # Get current position first (for defaults)
            cur_pos = [0,0,0]
            if hasattr(self, 'kin') and hasattr(self, 'current_servo_pos'):
                 # Need IDEAL angles for FK
                 # current_servo_pos are RAW. 
                 # FK in Kinematics expects IDEAL (if my recent change was correct).
                 # Wait, let's check `_control_loop`:
                 # cur_xyz = self.kin.forward_kinematics(rounded_pos) 
                 # and `rounded_pos` WAS converted to Ideal logic?
                 # No, `rounded_pos` in `_control_loop` comes from sliders which ARE Ideal now.
                 # So yes, `self.current_servo_pos` (which isn't strictly tracked, `self.angles` is)
                 # `self.angles` stores the Slider values (Ideal).
                 
                 cur_pos = self.kin.forward_kinematics(self.angles)

            x = float(self.ikX.text()) if self.chkX.isChecked() else cur_pos[0]
            y = float(self.ikY.text()) if self.chkY.isChecked() else cur_pos[1]
            z = float(self.ikZ.text()) if self.chkZ.isChecked() else cur_pos[2]
            
            logging.info(f"IK Request: X={x}, Y={y}, Z={z} (Flags: {self.chkX.isChecked()},{self.chkY.isChecked()},{self.chkZ.isChecked()})")
            
            sol = self.kin.inverse_kinematics(x, y, z)
            if sol:
                logging.info(f"IK Success: {sol}")
                self.smooth_move(sol) # Use smooth move to target
                self._set_status(f"Moved to ({x},{y},{z})", OK_GREEN)
            else:
                logging.warning(f"IK Failed: Unreachable ({x}, {y}, {z})")
                QtWidgets.QMessageBox.warning(self, "Unreachable", "Target out of reach")
        except ValueError:
             QtWidgets.QMessageBox.warning(self, "Error", "Invalid coordinates")

    def _card(self, title: str) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox(title)
        v = QtWidgets.QVBoxLayout(g)
        v.setContentsMargins(14,14,14,10)
        v.setSpacing(10)
        return g

    # --- groups
    def _grp_connection(self):
        g = self._card("Connection")

        # One clean row
        row = QtWidgets.QHBoxLayout()
        self.portCombo = QtWidgets.QComboBox(); self.portCombo.setMinimumWidth(220)
        self.baudCombo = QtWidgets.QComboBox(); self.baudCombo.addItems(["9600","19200","38400","57600","115200"]); self.baudCombo.setCurrentText("115200")
        btn_refresh = QtWidgets.QPushButton("Refresh"); btn_refresh.setProperty("flat", True)
        btn_connect = QtWidgets.QPushButton("Connect")
        btn_disconnect = QtWidgets.QPushButton("Disconnect"); btn_disconnect.setProperty("danger", True)
        btn_reconnect  = QtWidgets.QPushButton("Force Reconnect"); btn_reconnect.setProperty("warn", True)

        form = QtWidgets.QFormLayout(); form.setLabelAlignment(QtCore.Qt.AlignRight)
        form.addRow("Port", self.portCombo); form.addRow("Baud", self.baudCombo)

        row.addLayout(form, 1); row.addSpacing(8)
        row.addWidget(btn_refresh)
        row.addSpacing(8)
        row.addWidget(btn_connect)
        row.addWidget(btn_disconnect)
        row.addWidget(btn_reconnect)
        g.layout().addLayout(row)

        btn_refresh.clicked.connect(self.link.refresh_ports)
        btn_connect.clicked.connect(lambda: self.link.connect(self.portCombo.currentText(), int(self.baudCombo.currentText())))
        btn_disconnect.clicked.connect(self.link.disconnect)
        btn_reconnect.clicked.connect(self.link.force_reconnect)
        return g

        btn_reconnect.clicked.connect(self.link.force_reconnect)
        return g

    def _grp_firmware(self):
        g = self._card("Firmware Upload")
        
        # Tool Path
        row1 = QtWidgets.QHBoxLayout()
        self.arduinoPath = QtWidgets.QLineEdit(self.config.get("arduino_path",""))
        self.arduinoPath.setPlaceholderText("Path to arduino-cli.exe or arduino_debug.exe")
        btn_browse = QtWidgets.QPushButton("..."); btn_browse.setFixedWidth(30)
        btn_browse.setProperty("flat", True)
        row1.addWidget(QtWidgets.QLabel("Tool:")); row1.addWidget(self.arduinoPath); row1.addWidget(btn_browse)
        g.layout().addLayout(row1)

        # Board & Upload
        row2 = QtWidgets.QHBoxLayout()
        self.boardCombo = QtWidgets.QComboBox()
        self.boardCombo.addItems(["arduino:avr:uno", "arduino:avr:nano", "arduino:avr:mega", "arduino:avr:leonardo"])
        self.boardCombo.setEditable(True)
        self.boardCombo.setCurrentText(self.config.get("board", "arduino:avr:uno"))
        
        btn_upload = QtWidgets.QPushButton("Upload Firmware")
        btn_upload.setProperty("warn", True)
        
        row2.addWidget(QtWidgets.QLabel("Board:")); row2.addWidget(self.boardCombo, 1); row2.addWidget(btn_upload)
        g.layout().addLayout(row2)

        def _browse():
            path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select Arduino Tool", "", "Executables (*.exe)")
            if path: self.arduinoPath.setText(path); self.config["arduino_path"]=path; self.save_config()
        
        def _upload():
            self.config["board"] = self.boardCombo.currentText()
            self.config["arduino_path"] = self.arduinoPath.text()
            self.save_config()
            self.upload_firmware()

        btn_browse.clicked.connect(_browse)
        btn_upload.clicked.connect(_upload)
        return g

    def _grp_speed(self):
        g = self._card("Speed (lower = faster)")
        row = QtWidgets.QHBoxLayout()
        self.speedSlider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); self.speedSlider.setRange(1,100); self.speedSlider.setValue(30)
        self.speedLabel = QtWidgets.QLabel("30"); self.speedLabel.setMinimumWidth(32)
        self.speedSlider.valueChanged.connect(lambda v: self.speedLabel.setText(str(v)))
        row.addWidget(self.speedSlider, 1); row.addWidget(self.speedLabel)
        g.layout().addLayout(row); return g

    def _grp_defaults(self):
        g = self._card("Default Reset Positions")
        grid = QtWidgets.QGridLayout(); grid.setHorizontalSpacing(14); grid.setVerticalSpacing(8)
        self.defaultEdits: List[QtWidgets.QLineEdit] = []
        for i in range(7):
            grid.addWidget(QtWidgets.QLabel(f"Servo {i+1}"), i, 0)
            e = QtWidgets.QLineEdit(str(self.reset_defaults[i])); e.setFixedWidth(80)
            self.defaultEdits.append(e); grid.addWidget(e, i, 1)
        btns = QtWidgets.QHBoxLayout()
        btn_apply = QtWidgets.QPushButton("Apply Defaults")
        btn_factory = QtWidgets.QPushButton("Reset to Factory"); btn_factory.setProperty("flat", True)
        btns.addWidget(btn_apply); btns.addWidget(btn_factory)
        g.layout().addLayout(grid); g.layout().addLayout(btns)
        btn_apply.clicked.connect(self._apply_defaults)
        btn_factory.clicked.connect(self._factory_defaults)
        return g

    def _grp_actions(self):
        g = self._card("Actions")
        row = QtWidgets.QHBoxLayout()
        btn_reset = QtWidgets.QPushButton("Reset Positions")
        btn_stop  = QtWidgets.QPushButton("Stop All"); btn_stop.setProperty("danger", True)
        row.addWidget(btn_reset); row.addWidget(btn_stop)
        g.layout().addLayout(row)
        btn_reset.clicked.connect(self.reset_positions)
        btn_stop.clicked.connect(self.stop_all)
        return g

    def _grp_positions(self):
        g = self._card("Saved Positions")
        form = QtWidgets.QFormLayout()
        self.posName = QtWidgets.QLineEdit(); self.posName.setPlaceholderText("Name")
        form.addRow("Name", self.posName)
        g.layout().addLayout(form)

        rowTop = QtWidgets.QHBoxLayout()
        btn_save = QtWidgets.QPushButton("Save Current")
        btn_update = QtWidgets.QPushButton("Update Selected"); btn_update.setProperty("flat", True)
        rowTop.addWidget(btn_save); rowTop.addWidget(btn_update)
        g.layout().addLayout(rowTop)

        self.posList = QtWidgets.QListWidget(); self.posList.setMinimumHeight(150)
        g.layout().addWidget(self.posList)

        row = QtWidgets.QHBoxLayout()
        btn_move = QtWidgets.QPushButton("Move To")
        btn_delete= QtWidgets.QPushButton("Delete"); btn_delete.setProperty("danger", True)
        btn_export= QtWidgets.QPushButton("Export"); btn_export.setProperty("flat", True)
        btn_import= QtWidgets.QPushButton("Import"); btn_import.setProperty("flat", True)
        row.addWidget(btn_move); row.addWidget(btn_delete); row.addWidget(btn_export); row.addWidget(btn_import)
        g.layout().addLayout(row)

        btn_save.clicked.connect(self._save_current_pos)
        btn_update.clicked.connect(self._update_selected_pos)
        btn_move.clicked.connect(self._move_to_selected_pos)
        btn_delete.clicked.connect(self._delete_selected_pos)
        btn_export.clicked.connect(self._export_positions)
        btn_import.clicked.connect(self._import_positions)
        return g

    def _grp_sequence(self):
        g = self._card("Program / Sequence")
        self.seqTable = QtWidgets.QTableWidget(0, 4)
        self.seqTable.setHorizontalHeaderLabels(["#", "Step", "Dwell (ms)", "Speed"])
        self.seqTable.verticalHeader().setVisible(False)
        self.seqTable.horizontalHeader().setStretchLastSection(True)
        self.seqTable.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.seqTable.setMinimumHeight(220)
        g.layout().addWidget(self.seqTable)

        ctl1 = QtWidgets.QHBoxLayout()
        self.addPosCombo = QtWidgets.QComboBox()
        self.addDwell = QtWidgets.QLineEdit("500"); self.addDwell.setFixedWidth(90)
        self.addSpeed = QtWidgets.QLineEdit("30");  self.addSpeed.setFixedWidth(70)
        btn_add = QtWidgets.QPushButton("Add Step")
        btn_add_curr = QtWidgets.QPushButton("Add Current (no save)"); btn_add_curr.setProperty("flat", True)
        ctl1.addWidget(QtWidgets.QLabel("Use position:")); ctl1.addWidget(self.addPosCombo, 1)
        ctl1.addWidget(QtWidgets.QLabel("Dwell (ms)")); ctl1.addWidget(self.addDwell)
        ctl1.addWidget(QtWidgets.QLabel("Speed")); ctl1.addWidget(self.addSpeed)
        ctl1.addWidget(btn_add); ctl1.addWidget(btn_add_curr)
        g.layout().addLayout(ctl1)

        ctl2 = QtWidgets.QHBoxLayout()
        btn_up   = QtWidgets.QPushButton("Up"); btn_up.setProperty("flat", True)
        btn_down = QtWidgets.QPushButton("Down"); btn_down.setProperty("flat", True)
        btn_del  = QtWidgets.QPushButton("Delete Step"); btn_del.setProperty("danger", True)
        self.loopCheck = QtWidgets.QCheckBox("Loop")
        ctl2.addWidget(btn_up); ctl2.addWidget(btn_down); ctl2.addWidget(btn_del); ctl2.addStretch(1); ctl2.addWidget(self.loopCheck)
        g.layout().addLayout(ctl2)

        ctl3 = QtWidgets.QHBoxLayout()
        btn_run = QtWidgets.QPushButton("Run Sequence")
        btn_stop= QtWidgets.QPushButton("Stop Sequence"); btn_stop.setProperty("danger", True)
        btn_save= QtWidgets.QPushButton("Save Seq"); btn_save.setProperty("flat", True)
        btn_load= QtWidgets.QPushButton("Load Seq"); btn_load.setProperty("flat", True)
        ctl3.addWidget(btn_run); ctl3.addWidget(btn_stop); ctl3.addStretch(1); ctl3.addWidget(btn_save); ctl3.addWidget(btn_load)
        g.layout().addLayout(ctl3)

        btn_add.clicked.connect(self._add_step_from_saved)
        btn_add_curr.clicked.connect(self._add_step_from_current)
        btn_up.clicked.connect(self._seq_up)
        btn_down.clicked.connect(self._seq_down)
        btn_del.clicked.connect(self._seq_delete)
        btn_run.clicked.connect(self.run_sequence)
        btn_stop.clicked.connect(self.stop_sequence)
        btn_save.clicked.connect(self._export_sequence)
        btn_load.clicked.connect(self._import_sequence)
        return g

    def _grp_servos(self):
        g = self._card("Servo Controls")
        grid = QtWidgets.QGridLayout(); grid.setHorizontalSpacing(14); grid.setVerticalSpacing(10)
        self.sliders: List[QtWidgets.QSlider] = []; self.edits: List[QtWidgets.QLineEdit] = []
        for i in range(7):
            grid.addWidget(QtWidgets.QLabel(f"Servo {i+1}"), i, 0)
            s = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); s.setRange(0,180); s.setValue(self.angles[i])
            s.valueChanged.connect(lambda v, idx=i: self._on_slider(idx, v))
            self.sliders.append(s); grid.addWidget(s, i, 1)
            e = QtWidgets.QLineEdit(str(self.angles[i])); e.setFixedWidth(72)
            def _apply(idx=i, edit=e):
                try: v = max(0, min(180, int(edit.text())))
                except Exception: v = self.angles[idx]
                edit.setText(str(v)); self.angles[idx]=v; self.sliders[idx].setValue(v); self.link.send_angle(idx, v)
            e.returnPressed.connect(_apply); self.edits.append(e); grid.addWidget(e, i, 2)
        g.layout().addLayout(grid); return g

    # ---------- status & connection ----------
    @QtCore.Slot(list)
    def _fill_ports(self, items: List[str]):
        self.addPosCombo.clear()
        self.portCombo.clear(); self.portCombo.addItems(items)

    def _set_status(self, text: str, color: str):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color:{color};")

    def _check_connection(self):
        if self.link.ser and not self.link.ser.is_open:
            self._set_status("Lost connection. Reconnecting…", WARN_AMB)
            self.link.force_reconnect()

    # ---------- defaults ----------
    def _apply_defaults(self):
        try:
            vals = [max(0, min(180, int(e.text()))) for e in self.defaultEdits]
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Invalid", "All defaults must be integers 0..180"); return
        self.reset_defaults = vals; self._set_status(f"Defaults set: {self.reset_defaults}", OK_GREEN)

    def _factory_defaults(self):
        self.reset_defaults = [0,90,90,90,90,90,90]
        for i, e in enumerate(self.defaultEdits): e.setText(str(self.reset_defaults[i]))
        self._set_status("Factory defaults restored", OK_GREEN)

    # ---------- servo events ----------
    def _on_slider(self, idx: int, val: int):
        # Only update target. Control loop handles the move.
        self.angles[idx] = val
        self.edits[idx].setText(str(val))

    # ---------- smooth move (Sequence/UI Animation) ----------
    def smooth_move(self, target: List[int], speed_override: Optional[int] = None, on_done: Optional[callable] = None):
        if on_done: self.move_done_callbacks.append(on_done)
        if self.smooth_timer.isActive(): self.smooth_timer.stop()
        self.smooth_start = self.angles[:]
        self.smooth_target = [max(0, min(180, int(a))) for a in target]
        self.smooth_step_idx = 0
        
        # Adaptive resolution: At least 1 step per degree, min 20 steps
        max_diff = 0
        for i in range(7):
            max_diff = max(max_diff, abs(self.smooth_target[i] - self.smooth_start[i]))
        self.smooth_steps = max(max_diff, 20)
        
        # Speed calc: map slider 1..100 (fast..slow). 
        # New delay strategy: 'val' ms per step.
        # e.g. 30 -> 15ms/step. 100 -> 50ms/step.
        base = speed_override if speed_override is not None else self.speedSlider.value()
        self.smooth_delay_ms = max(3, int(base * 0.5))
        
        self.smooth_timer.start(self.smooth_delay_ms)

    def _smooth_step(self):
        self.smooth_step_idx += 1
        current_step = self.smooth_step_idx; total_steps = self.smooth_steps
        
        # Cosine Easing (S-Curve): 1/2 * (1 - cos(pi * t))
        # t goes from 0.0 to 1.0
        t = current_step / total_steps
        factor = (1 - math.cos(t * math.pi)) / 2.0
        
        new = []
        for i in range(7):
            # Interpolate
            val = self.smooth_start[i] + (self.smooth_target[i] - self.smooth_start[i]) * factor
            v = int(round(val))
            v = max(0, min(180, v)); new.append(v)
            self.sliders[i].blockSignals(True); self.sliders[i].setValue(v); self.sliders[i].blockSignals(False)
            self.edits[i].setText(str(v))
        
        # Update TARGETS only. Control loop will chase these targets.
        self.angles = new
        
        if current_step >= total_steps:
            self.smooth_timer.stop()
            cbs = self.move_done_callbacks[:]; self.move_done_callbacks.clear()
            for cb in cbs:
                try: cb()
                except Exception: pass

    # ---------- positions ----------
    def _refresh_pos_combo(self):
        self.addPosCombo.clear()
        self.addPosCombo.addItems(sorted(self.saved_positions.keys()))

    def _save_current_pos(self):
        name = self.posName.text().strip() or datetime.datetime.now().strftime("pos_%H%M%S")
        self.posName.setText(name)
        self.saved_positions[name] = self.angles[:]
        self._refresh_pos_list(); self._refresh_pos_combo()
        self._set_status(f"Saved '{name}'", OK_GREEN)

    def _refresh_pos_list(self):
        self.posList.clear()
        self.posList.addItems(sorted(self.saved_positions.keys()))

    def _selected_pos(self) -> Optional[str]:
        sel = self.posList.selectedItems()
        return sel[0].text() if sel else None

    def _update_selected_pos(self):
        name = self._selected_pos()
        if not name:
            QtWidgets.QMessageBox.information(self, "Select", "Pick a position to update"); return
        self.saved_positions[name] = self.angles[:]
        self._set_status(f"Updated '{name}'", OK_GREEN)

    def _move_to_selected_pos(self):
        name = self._selected_pos()
        if not name:
            QtWidgets.QMessageBox.information(self, "Select", "Pick a position to move to"); return
        self.smooth_move(self.saved_positions[name])

    def _delete_selected_pos(self):
        name = self._selected_pos()
        if not name: return
        del self.saved_positions[name]
        self._refresh_pos_list(); self._refresh_pos_combo()

    def _export_positions(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export Positions", "", "JSON (*.json)")
        if not path: return
        json.dump(self.saved_positions, open(path,"w"), indent=2)
        self._set_status(f"Positions saved to {path}", OK_GREEN)

    def _import_positions(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import Positions", "", "JSON (*.json)")
        if not path: return
        try:
            data = json.load(open(path,"r"))
            valid_data = {}
            for k, v in data.items():
                if isinstance(v, list):
                    if len(v) == 6:
                        v.append(90)  # Upgrade old format
                    if len(v) == 7:
                        valid_data[k] = v
            
            if not valid_data: raise ValueError("No valid positions found (need 6 or 7 angles)")
            self.saved_positions.update(valid_data)
            self._refresh_pos_list(); self._refresh_pos_combo()
            self._set_status(f"Imported positions from {path}", OK_GREEN)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Import error", str(e))

    # ---------- sequence ----------
    def _rebuild_seq_table(self):
        self.seqTable.setRowCount(0)
        for i, step in enumerate(self.sequence, start=1):
            self.seqTable.insertRow(self.seqTable.rowCount())
            label = step.pos_name+" (saved)" if step.kind=="pos" else (step.label or "angles")
            self.seqTable.setItem(self.seqTable.rowCount()-1, 0, QtWidgets.QTableWidgetItem(str(i)))
            self.seqTable.setItem(self.seqTable.rowCount()-1, 1, QtWidgets.QTableWidgetItem(label))
            self.seqTable.setItem(self.seqTable.rowCount()-1, 2, QtWidgets.QTableWidgetItem(str(step.dwell_ms)))
            self.seqTable.setItem(self.seqTable.rowCount()-1, 3, QtWidgets.QTableWidgetItem(str(step.speed)))

    def _add_step_from_saved(self):
        name = self.addPosCombo.currentText().strip()
        if not name:
            QtWidgets.QMessageBox.information(self, "No position", "Choose a saved position or use 'Add Current'"); return
        if name not in self.saved_positions:
            QtWidgets.QMessageBox.critical(self, "Unknown", "Selected position does not exist"); return
        try:
            dwell = int(self.addDwell.text()); speed = int(self.addSpeed.text())
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Invalid", "Dwell and Speed must be integers"); return
        self.sequence.append(SeqStep(kind="pos", pos_name=name, dwell_ms=dwell, speed=speed))
        self._rebuild_seq_table()

    def _add_step_from_current(self):
        try:
            dwell = int(self.addDwell.text()); speed = int(self.addSpeed.text())
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Invalid", "Dwell and Speed must be integers"); return
        label = datetime.datetime.now().strftime("current_%H%M%S")
        self.sequence.append(SeqStep(kind="angles", angles=self.angles[:], label=label, dwell_ms=dwell, speed=speed))
        self._rebuild_seq_table()

    def _selected_seq_index(self) -> Optional[int]:
        sel = self.seqTable.selectionModel().selectedRows()
        return sel[0].row() if sel else None

    def _seq_delete(self):
        idx = self._selected_seq_index()
        if idx is None: return
        del self.sequence[idx]; self._rebuild_seq_table()

    def _seq_up(self):
        idx = self._selected_seq_index()
        if idx is None or idx==0: return
        self.sequence[idx-1], self.sequence[idx] = self.sequence[idx], self.sequence[idx-1]
        self._rebuild_seq_table(); self.seqTable.selectRow(idx-1)

    def _seq_down(self):
        idx = self._selected_seq_index()
        if idx is None or idx >= len(self.sequence)-1: return
        self.sequence[idx+1], self.sequence[idx] = self.sequence[idx], self.sequence[idx+1]
        self._rebuild_seq_table(); self.seqTable.selectRow(idx+1)

    def _export_sequence(self):
        if not self.sequence:
            QtWidgets.QMessageBox.information(self, "Empty", "Nothing to save"); return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Sequence", "", "JSON (*.json)")
        if not path: return
        data = []
        for s in self.sequence:
            d = dict(kind=s.kind, dwell=s.dwell_ms, speed=s.speed)
            if s.kind == "pos": d["pos"] = s.pos_name
            else: d["angles"] = s.angles; d["label"] = s.label
            data.append(d)
        json.dump(data, open(path, "w"), indent=2)
        self._set_status(f"Sequence saved to {path}", OK_GREEN)

    def _import_sequence(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Sequence", "", "JSON (*.json)")
        if not path: return
        try:
            seq = json.load(open(path, "r")); new: List[SeqStep] = []
            for s in seq:
                if "kind" not in s or "dwell" not in s or "speed" not in s:
                    raise ValueError("Invalid sequence file")
                if s["kind"] == "pos":
                    new.append(SeqStep(kind="pos", pos_name=s.get("pos",""), dwell_ms=int(s["dwell"]), speed=int(s["speed"])))
                else:
                    arr = s.get("angles", [90]*7)
                    if len(arr) == 6: arr.append(90)
                    new.append(SeqStep(kind="angles", angles=arr, label=s.get("label","angles"), dwell_ms=int(s["dwell"]), speed=int(s["speed"])))
            self.sequence = new; self._rebuild_seq_table()
            self._set_status(f"Loaded sequence from {path}", OK_GREEN)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Import error", str(e))

    # sequencing with timers (no threads)
    def run_sequence(self):
        if self.seq_running: return
        if not self.sequence:
            QtWidgets.QMessageBox.information(self, "Empty", "Add steps first"); return
        self.seq_running = True; self.seq_index = 0
        self._set_status("Sequence running…", WARN_AMB)
        self._seq_next()

    def stop_sequence(self):
        self.seq_running = False
        self.dwell_timer.stop()
        if self.smooth_timer.isActive(): self.smooth_timer.stop(); self.move_done_callbacks.clear()
        self._set_status("Stopped", ERR_RED)

    def _seq_next(self):
        if not self.seq_running: return
        if self.seq_index >= len(self.sequence):
            if True and self.findChild(QtWidgets.QCheckBox, "", options=QtCore.Qt.FindDirectChildrenOnly):
                pass
            if False: pass
            if getattr(self, "loopCheck", None) and self.loopCheck.isChecked():
                self.seq_index = 0
            else:
                self.seq_running = False; self._set_status("Sequence finished", OK_GREEN); return

        step = self.sequence[self.seq_index]
        if step.kind == "pos":
            if step.pos_name not in self.saved_positions:
                self._set_status(f"Missing saved position '{step.pos_name}'", ERR_RED)
                self.seq_running = False; return
            target = self.saved_positions[step.pos_name]; label = step.pos_name
        else:
            target = step.angles or self.angles[:]; label = step.label or "angles"

        self._set_status(f"Moving to {label}", WARN_AMB)
        def after_move():
            self._set_status(f"Dwell {step.dwell_ms} ms", WARN_AMB)
            self.dwell_timer.start(max(0, int(step.dwell_ms)))
            self.seq_index += 1
        self.smooth_move(target, speed_override=step.speed, on_done=after_move)

    # top actions
    def reset_positions(self):
        self.smooth_move(self.reset_defaults)

    def stop_all(self):
        self.stop_sequence()
        # Force stop: set target to current to stop any drift
        self.angles = [int(p) for p in self.current_servo_pos]
        for i, s in enumerate(self.sliders): 
            s.blockSignals(True); s.setValue(self.angles[i]); s.blockSignals(False)
            self.edits[i].setText(str(self.angles[i]))

    # ---------- background control loop ----------
    def _control_loop(self):
        # Runs at 50Hz. Moves current_servo_pos -> angles
        if not self.link.ser or not self.link.ser.is_open: return
        
        # Improved Speed Scaling (Non-linear)
        # Slider: 1-100
        # We want fine control at low speeds.
        # Speed 1 (Fast): ~500 deg/s ?
        # Speed 100 (Slow): ~5 deg/s
        
        s = self.speedSlider.value()
        
        # Quadratic curve for better "slow" resolution
        # inverse: 0 (Fast) -> 1 (Slow)
        inv_s = (s - 1) / 99.0 
        
        # factor: 0.01 (Slow) -> 1.0 (Fast)
        # Use (1 - inv_s) for "Fastness"
        fastness = 1.0 - inv_s
        
        # Curve: fastness^2
        # value range: 0.1 deg/tick to 10 deg/tick
        max_step = 0.1 + (fastness * fastness) * 12.0

        updated_phys = False
        rounded_pos = []
        
        for i in range(7):
            curr = self.current_servo_pos[i]
            targ = float(self.angles[i])
            diff = targ - curr
            
            if abs(diff) > 0.05:
                # Move towards target
                step = diff
                if abs(step) > max_step:
                    step = math.copysign(max_step, diff)
                
                self.current_servo_pos[i] += step
                updated_phys = True
            else:
                self.current_servo_pos[i] = targ
            
            rounded_pos.append(int(round(self.current_servo_pos[i])))

        # VITAL: Only send if the INTEGER values have actually changed 
        # vs what was last sent to the hardware.
        # This prevents flooding the serial line with "90, 90, 90..." 50 times a second.
        
        if updated_phys:
             # Convert IDEAL (rounded_pos) to RAW/PHYSICAL
             if hasattr(self, 'kin'):
                 raw_pos = []
                 for i, p in enumerate(rounded_pos):
                     val = self.kin.get_raw(p, i)
                     clamped = int(max(0, min(180, val)))
                     if abs(val - clamped) > 1:
                         logging.warning(f"Servo {i+1} Clamped: {int(val)} -> {clamped}")
                     raw_pos.append(clamped)
             else:
                 raw_pos = rounded_pos

             # Check distinct
             if raw_pos != self.link.last_sent:
                 self.link.send_all(raw_pos)
             
             # Updates XYZ in IK tab (Forward Kinematics)
             # We rely on the stored config/kinematics
             if hasattr(self, 'kin'):
                 # kin.forward_kinematics now expects IDEAL angles (rounded_pos)
                 cur_xyz = self.kin.forward_kinematics(rounded_pos)
                 
                 # Only update if user is NOT typing
                 if not self.ikX.hasFocus(): self.ikX.setText(f"{cur_xyz[0]:.1f}")
                 if not self.ikY.hasFocus(): self.ikY.setText(f"{cur_xyz[1]:.1f}")
                 if not self.ikZ.hasFocus(): self.ikZ.setText(f"{cur_xyz[2]:.1f}")

        # Update 3D Viz with the INTERPOLATED (Physical/Ideal) position
        # Viz expects Ideal now.
        if self.viz:
             self.viz.update_plot(rounded_pos, self.kin) # rounded_pos is Ideal

    # ---------- firmware upload ----------
    def upload_firmware(self):
        tool = self.arduinoPath.text().strip()
        if not tool or not os.path.exists(tool):
            QtWidgets.QMessageBox.critical(self, "Error", "Arduino tool not found. Please select arduino-cli.exe or arduino_debug.exe")
            return
            
        board = self.boardCombo.currentText().strip()
        port = self.portCombo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.critical(self, "Error", "Select a COM port first")
            return

        ino_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware", "criya_firmware", "criya-v2_firmware.ino")
        if not os.path.exists(ino_path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Firmware file not found at:\n{ino_path}")
            return
        
        # Disconnect serial before upload
        was_connected = False
        if self.link.ser and self.link.ser.is_open:
            self.link.disconnect()
            was_connected = True

        self._set_status("Uploading firmware… please wait", WARN_AMB)
        QtWidgets.QApplication.processEvents()
        
        try:
            # Detect tool type
            cmd = []
            if "arduino-cli" in os.path.basename(tool).lower():
                # arduino-cli compile --fqbn {board} --upload -p {port} "{ino}"
                cmd = [tool, "compile", "--fqbn", board, "--upload", "-p", port, ino_path]
            else:
                # Legacy: arduino_debug --upload --board {board} --port {port} "{ino}"
                cmd = [tool, "--upload", "--board", board, "--port", port, ino_path]

            # Run
            # We use startupinfo to hide console window on Windows
            si = subprocess.STARTUPINFO()
            si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            
            p = subprocess.run(cmd, capture_output=True, text=True, startupinfo=si)
            
            if p.returncode == 0:
                QtWidgets.QMessageBox.information(self, "Success", "Firmware uploaded successfully!")
                self._set_status("Upload complete", OK_GREEN)
            else:
                err = p.stderr or p.stdout
                QtWidgets.QMessageBox.critical(self, "Upload Failed", f"Exit Code: {p.returncode}\n\n{err[-500:]}")
                self._set_status("Upload failed", ERR_RED)
                
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e))
            self._set_status(f"Error: {e}", ERR_RED)
        
        if was_connected:
            self.link.connect(port, 115200)

# -------------------- app bootstrap --------------------
def apply_light_theme(app: QtWidgets.QApplication):
    if HAVE_QT_MATERIAL:
        try:
            apply_stylesheet(app, theme='light_cyan_500.xml', invert_secondary=False, extra={
                'primaryColor': PRIMARY,
                'secondaryColor': SECONDARY,
                'warning': WARN_AMB,
                'danger': ERR_RED,
                'success': OK_GREEN,
            })
            return
        except Exception:
            pass
    app.setStyleSheet(QSS)

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Criya Robot")
    apply_light_theme(app)
    win = MainWindow(); win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    # Setup Logging to botlog.txt
    logging.basicConfig(
        filename="botlog.txt",
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        filemode='a' # Overwrite each run? Or 'a' append. User said "logs of whats happening". 'a' is safer for history, 'w' cleaner. Let's use 'w' for now or 'a'. 'a' is better.
    )
    logging.info("=== Application Started ===")
    
    app = QtWidgets.QApplication(sys.argv)
    
    # Palette/Style setup (existing)
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    # ... (rest of palette)
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(240, 240, 240))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.black)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(245, 245, 245))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.ToolTipText, QtCore.Qt.black)
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(240, 240, 240))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.black)
    palette.setColor(QtGui.QPalette.BrightText, QtCore.Qt.red)
    palette.setColor(QtGui.QPalette.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.white)
    app.setPalette(palette)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())
