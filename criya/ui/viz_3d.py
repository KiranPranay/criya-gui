from __future__ import annotations
import numpy as np
from typing import List

from PySide6 import QtWidgets, QtCore
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from criya.core.kinematics import Kinematics, rot_x, rot_y, rot_z, trans

class Viz3D(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111, projection='3d')
        super(Viz3D, self).__init__(self.fig)
        self.setParent(parent)
        
        # Initial View
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(0, 500)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

    def update_plot(self, ideal_angles: List[float], kin: Kinematics):
        self.ax.cla()
        
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(0, 500)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        
        # Draw Origin
        self.ax.plot([0, 50], [0, 0], [0, 0], 'r-')
        self.ax.plot([0, 0], [0, 50], [0, 0], 'g-')
        self.ax.plot([0, 0], [0, 0], [0, 50], 'k-')
        
        # Build Points using Homogenous Transforms
        # Matches core/kinematics logic exactly
        
        T = np.eye(4)
        points = [T[:3, 3]]
        
        # 1. Base (S7) - Pan
        theta1 = np.radians(ideal_angles[6] - 90)
        T = T @ rot_z(theta1) @ trans(0, 0, kin.L_BASE)
        points.append(T[:3, 3])
        
        # 2. Shoulder (S6) - PITCH (Rot Y) -> Inverted (90 - Angle)
        # User: "<90 is Forward". 
        theta2 = np.radians(90 - ideal_angles[5])
        T = T @ rot_y(theta2) @ trans(0, 0, kin.L_HUMERUS)
        points.append(T[:3, 3])
        
        # 3. Elbow (S5) - PITCH (Rot Y) -> Inverted (90 - Angle)
        theta3 = np.radians(90 - ideal_angles[4])
        T = T @ rot_y(theta3) @ trans(0, 0, kin.L_SEG1)
        points.append(T[:3, 3]) 
        
        # 4. Wrist 1 (S4) - LATERAL (Rot X)
        theta4 = np.radians(ideal_angles[3] - 90)
        T = T @ rot_x(theta4) @ trans(0, 0, kin.L_SEG2)
        points.append(T[:3, 3])
        
        # 5. Wrist 2 (S3) - PITCH (Rot Y) -> Inverted
        theta5 = np.radians(90 - ideal_angles[2])
        T = T @ rot_y(theta5) @ trans(0, 0, kin.L_SEG3)
        points.append(T[:3, 3])
        
        # 6. Wrist 3 (S2) - ROLL (Rot X)
        theta6 = np.radians(ideal_angles[1] - 90)
        T = T @ rot_x(theta6) @ trans(0, 0, 0)
        # No point append here
        
        # 7. Tool
        T = T @ trans(kin.L_TOOL, 0, 0)
        points.append(T[:3, 3])
        
        # 8. S1 removed
        
        # Draw Structure
        pts = np.array(points)
        print(f"DEBUG: Viz Points: {len(pts)}")
        
        # Line (Blue) - Full Structure
        self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-', linewidth=4, color='tab:blue')
        
        # Markers (Blue) - Skip Tip (Index 6)
        # pts length is 7 (0..6). pts[:-1] is 0..5 (6 items).
        self.ax.plot(pts[:-1, 0], pts[:-1, 1], pts[:-1, 2], 'o', markersize=6, color='tab:blue') 
        
        # Orientation Triad
        R = T[:3, :3]
        P = T[:3, 3]
        scale = 40 
        
        px = P + R @ np.array([scale, 0, 0])
        self.ax.plot([P[0], px[0]], [P[1], px[1]], [P[2], px[2]], 'r-', linewidth=2)
        
        py = P + R @ np.array([0, scale, 0])
        self.ax.plot([P[0], py[0]], [P[1], py[1]], [P[2], py[2]], 'g-', linewidth=2)
        
        pz = P + R @ np.array([0, 0, scale])
        self.ax.plot([P[0], pz[0]], [P[1], pz[1]], [P[2], pz[2]], 'b-', linewidth=2)
        
        self.draw() # Trigger canvas refresh