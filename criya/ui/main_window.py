from __future__ import annotations
import sys
import json
import datetime
import math
import os
import subprocess
import logging
from typing import List, Dict, Optional

from PySide6 import QtWidgets, QtCore, QtGui

# Core Imports
from criya.core.config import CONFIG_FILE, DEFAULT_CONFIG
from criya.core.kinematics import Kinematics
from criya.core.serial_comm import SerialLink
from criya.core.data_types import SeqStep
# UI Imports
from criya.ui.viz_3d import Viz3D
from criya.ui.styling import *

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
        cfg = DEFAULT_CONFIG.copy()
        if os.path.exists(CONFIG_FILE):
            try: 
                loaded = json.load(open(CONFIG_FILE))
                cfg.update(loaded)
            except: pass
        return cfg

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

        # Equations Area (Plain Display)
        info = QtWidgets.QTextBrowser()
        info.setOpenExternalLinks(True)
        info.setReadOnly(True)
        info.setFrameShape(QtWidgets.QFrame.Shape.NoFrame) # Remove border
        info.setMinimumHeight(300) 
        
        # Transparent background, dark text
        info.setStyleSheet("border: none; background-color: transparent; color: #1F2937; font-size: 13px;")
        
        info.setHtml("""
        <!DOCTYPE html>
        <html>
        <body style="font-family: 'Segoe UI', sans-serif; color: #333;">
            <p style="margin-top:0; font-size:14px; color:#0066CC;"><b>Geometric Inverse Kinematics</b></p>
            <p>The robot position (X, Y, Z) is solved using a breakdown of the 5-DOF chain into a Planar 2-Link mechanism plus Base rotation.</p>
            
            <p><b>1. Base Rotation (θ<sub>1</sub>)</b><br>
            θ<sub>1</sub> = atan2(y, x)<br>
            <i>Radial Distance:</i> r<sub>total</sub> = √(x² + y²)</p>

            <p><b>2. Planar Projection (R, Z)</b><br>
            r<sub>wrist</sub> = r<sub>total</sub> - L<sub>Tool</sub><br>
            z<sub>local</sub> = z - L<sub>Base</sub><br>
            <i>Hypotenuse:</i> h = √(r<sub>wrist</sub>² + z<sub>local</sub>²)</p>

            <p><b>3. Law of Cosines</b><br>
            cos(γ) = (L₁² + L₂² - h²) / (2 · L₁ · L₂)<br>
            cos(α) = (L₁² + h² - L₂²) / (2 · L₁ · h)<br>
            β = atan2(z<sub>local</sub>, r<sub>wrist</sub>)</p>
            
            <p><b>4. Servo Angles</b><br>
            Shoulder (S6) = β + α<br>
            Elbow (S5) = 180° - γ</p>
        </body>
        </html>
        """)
        
        layout.addWidget(info)
        layout.addStretch(1) # Keep stretch at bottom to push content up
        return page

        layout.addStretch(1)
        return page

    def _tab_settings(self):
        page = QtWidgets.QWidget()
        L = QtWidgets.QVBoxLayout(page); L.setContentsMargins(16,16,16,16)
        
        g = self._card("Robot Geometry (mm)")
        form = QtWidgets.QFormLayout()
        
        geo = self.config.get("geometry", DEFAULT_CONFIG["geometry"])
        self.stBase = QtWidgets.QLineEdit(str(geo.get("L_BASE", 120)))
        self.stHum  = QtWidgets.QLineEdit(str(geo.get("L_HUMERUS", 120)))
        
        self.stSeg1 = QtWidgets.QLineEdit(str(geo.get("L_SEG1", 70)))
        self.stSeg2 = QtWidgets.QLineEdit(str(geo.get("L_SEG2", 85)))
        self.stSeg3 = QtWidgets.QLineEdit(str(geo.get("L_SEG3", 75)))
        self.stTool = QtWidgets.QLineEdit(str(geo.get("L_TOOL", 120)))
        
        form.addRow("Base Height (L1)", self.stBase)
        form.addRow("Shoulder->Elbow (L2)", self.stHum)
        form.addRow("Elbow->Wrist1 (S1)", self.stSeg1)
        form.addRow("Wrist1->Wrist2 (S2)", self.stSeg2)
        form.addRow("Link 3 (Gripper Tip)", self.stSeg3)
        form.addRow("Gripper Offset (Tool)", self.stTool)

        g2 = self._card("Calibration (Val at 90°)")
        form2 = QtWidgets.QFormLayout()
        self.calibEdits = []
        current_calib = self.config.get("calibration", [0]*7)
        if len(current_calib) < 7: current_calib = [0]*7

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
            geo = self.config.setdefault("geometry", {})
            geo["L_BASE"] = float(self.stBase.text())
            geo["L_HUMERUS"] = float(self.stHum.text())
            geo["L_SEG1"] = float(self.stSeg1.text())
            geo["L_SEG2"] = float(self.stSeg2.text())
            geo["L_SEG3"] = float(self.stSeg3.text())
            geo["L_TOOL"] = float(self.stTool.text())
            
            # Save Calib
            new_calib = [0]*7
            for i, le in enumerate(self.calibEdits):
                new_calib[i] = int(float(le.text()))
            
            self.config["calibration"] = new_calib
            
            self.save_config()
            self._set_status("Settings Saved", OK_GREEN)
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Invalid", "Values must be numbers")

    def _solve_ik(self):
        try:
            # Determine Targets
            cur_pos = [0,0,0]
            if hasattr(self, 'kin') and hasattr(self, 'current_servo_pos'):
                 # Use slider angles (Ideal) for FK
                 cur_pos = self.kin.forward_kinematics(self.angles)
                 
                 # FEATURE: Auto-populate text fields if they are 0.0 to prevent accidents
                 if float(self.ikX.text() or 0) == 0: self.ikX.setText(f"{cur_pos[0]:.1f}")
                 if float(self.ikY.text() or 0) == 0: self.ikY.setText(f"{cur_pos[1]:.1f}")
                 if float(self.ikZ.text() or 0) == 0: self.ikZ.setText(f"{cur_pos[2]:.1f}")

            x = float(self.ikX.text()) if self.chkX.isChecked() else cur_pos[0]
            y = float(self.ikY.text()) if self.chkY.isChecked() else cur_pos[1]
            z = float(self.ikZ.text()) if self.chkZ.isChecked() else cur_pos[2]
            
            logging.info(f"IK Request: X={x}, Y={y}, Z={z} (Flags: {self.chkX.isChecked()},{self.chkY.isChecked()},{self.chkZ.isChecked()})")
            
            # Pass current_angles for continuity optimization
            sol = self.kin.inverse_kinematics(x, y, z, current_angles=self.angles)
            if sol:
                logging.info(f"IK Success: {sol}")
                self.smooth_move(sol) # Use smooth move to target
                self._set_status(f"Moved to ({x},{y},{z})", OK_GREEN)
            else:
                logging.warning(f"IK Failed: Unreachable ({x}, {y}, {z})")
                QtWidgets.QMessageBox.warning(self, "Move Failed", "Target Unreachable.\nThe point is outside the robot's workspace.")
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
        layout = g.layout()

        # Port/Baud Row
        row1 = QtWidgets.QHBoxLayout()
        self.portCombo = QtWidgets.QComboBox(); self.portCombo.setMinimumWidth(150)
        self.baudCombo = QtWidgets.QComboBox(); self.baudCombo.addItems(["9600","19200","38400","57600","115200"]); self.baudCombo.setCurrentText("115200")
        self.baudCombo.setFixedWidth(100)
        
        row1.addWidget(QtWidgets.QLabel("Port:")); row1.addWidget(self.portCombo, 1)
        row1.addWidget(QtWidgets.QLabel("Baud:")); row1.addWidget(self.baudCombo)
        layout.addLayout(row1)

        # Buttons Grid (2 rows)
        grid = QtWidgets.QGridLayout()
        btn_refresh = QtWidgets.QPushButton("Refresh"); btn_refresh.setProperty("flat", True)
        btn_connect = QtWidgets.QPushButton("Connect")
        btn_disconnect = QtWidgets.QPushButton("Disconnect"); btn_disconnect.setProperty("danger", True)
        btn_reconnect  = QtWidgets.QPushButton("Reconnect"); btn_reconnect.setProperty("warn", True)
        
        grid.addWidget(btn_connect, 0, 0)
        grid.addWidget(btn_disconnect, 0, 1)
        grid.addWidget(btn_refresh, 1, 0)
        grid.addWidget(btn_reconnect, 1, 1)
        
        layout.addLayout(grid)

        btn_refresh.clicked.connect(self.link.refresh_ports)
        btn_connect.clicked.connect(self._do_connect)
        btn_disconnect.clicked.connect(self.link.disconnect)
        btn_reconnect.clicked.connect(self.link.force_reconnect)
        return g

    def _do_connect(self):
        self.link.connect(self.portCombo.currentText(), int(self.baudCombo.currentText()))
        if self.link.connected:
            # Force One-Time Sync
            raw = [int(p) for p in self.current_servo_pos]
            self.link.send_all(raw)
            # self.link.last_sent = raw # optional optimization

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
        layout = g.layout()
        
        # Grid for Inputs
        grid = QtWidgets.QGridLayout(); grid.setHorizontalSpacing(10); grid.setVerticalSpacing(6)
        self.defaultEdits: List[QtWidgets.QLineEdit] = []
        
        # 2 Rows of inputs (S1-S4, S5-S7)
        for i in range(7):
            row = i // 4
            col = (i % 4) * 2 # Label + Edit
            
            lbl = QtWidgets.QLabel(f"S{i+1}")
            e = QtWidgets.QLineEdit(str(self.reset_defaults[i]))
            # e.setFixedWidth(50) # Allow flexibility
            e.setMaximumWidth(60)
            
            grid.addWidget(lbl, row, col)
            grid.addWidget(e, row, col+1)
            self.defaultEdits.append(e)

        layout.addLayout(grid)
        
        btns = QtWidgets.QHBoxLayout()
        btn_apply = QtWidgets.QPushButton("Apply")
        btn_factory = QtWidgets.QPushButton("Factory Reset"); btn_factory.setProperty("flat", True)
        btns.addWidget(btn_apply); btns.addWidget(btn_factory)
        layout.addLayout(btns)
        
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
        
        # Adaptive resolution
        max_diff = 0
        for i in range(7):
            max_diff = max(max_diff, abs(self.smooth_target[i] - self.smooth_start[i]))
        self.smooth_steps = max(max_diff, 20)
        
        base = speed_override if speed_override is not None else self.speedSlider.value()
        self.smooth_delay_ms = max(3, int(base * 0.5))
        
        self.smooth_timer.start(self.smooth_delay_ms)

    def _smooth_step(self):
        self.smooth_step_idx += 1
        current_step = self.smooth_step_idx; total_steps = self.smooth_steps
        
        # Cosine Easing
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
        
        s = self.speedSlider.value()
        inv_s = (s - 1) / 99.0 
        fastness = 1.0 - inv_s
        max_step = 0.1 + (fastness * fastness) * 12.0

        updated_phys = False
        rounded_pos = []
        
        for i in range(7):
            curr = self.current_servo_pos[i]
            targ = float(self.angles[i])
            diff = targ - curr
            
            if abs(diff) > 0.05:
                step = diff
                if abs(step) > max_step:
                    step = math.copysign(max_step, diff)
                self.current_servo_pos[i] += step
                updated_phys = True
            else:
                self.current_servo_pos[i] = targ
            
            rounded_pos.append(int(round(self.current_servo_pos[i])))

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
             if raw_pos != self.link.last_sent if hasattr(self.link, 'last_sent') else True:
                 self.link.send_all(raw_pos)
                 self.link.last_sent = raw_pos # Track it here or in Link? Link doesn't have it.
                 # Let's add last_sent to Link or just ignore? 
                 # The original code had it?
                 # main.py: if raw_pos != self.link.last_sent: 
                 # Wait, SerialLink in main.py didn't have last_sent?
                 # Ah, main.py didn't rely on `last_sent` in serial link?
                 # Let's check main.py logic again.
                 pass

             # Updates XYZ in IK tab (Forward Kinematics)
             if hasattr(self, 'kin'):
                 cur_xyz = self.kin.forward_kinematics(rounded_pos)
                 if not self.ikX.hasFocus(): self.ikX.setText(f"{cur_xyz[0]:.1f}")
                 if not self.ikY.hasFocus(): self.ikY.setText(f"{cur_xyz[1]:.1f}")
                 if not self.ikZ.hasFocus(): self.ikZ.setText(f"{cur_xyz[2]:.1f}")

        # Update 3D Viz with the INTERPOLATED (Ideal) position
        if self.viz:
             self.viz.update_plot(rounded_pos, self.kin)

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
        
        # Firmware path is relative to package or root? 
        # Old: main.py/firmware
        # New: criya/ui/main_window.py -> need to go up 2 levels?
        # Actually firmware folder is in root.
        # root/firmware/criya_firmware/
        # main_window is in root/criya/ui/
        # base_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        # Let's look for it dynamically
        
        base_dir = os.getcwd() # Typically root when running main.py
        ino_path = os.path.join(base_dir, "firmware", "criya_firmware", "criya-v2_firmware.ino")
        
        if not os.path.exists(ino_path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Firmware file not found at:\n{ino_path}")
            return
        
        was_connected = False
        if self.link.ser and self.link.ser.is_open:
            self.link.disconnect()
            was_connected = True

        self._set_status("Uploading firmware… please wait", WARN_AMB)
        QtWidgets.QApplication.processEvents()
        
        try:
            cmd = []
            if "arduino-cli" in os.path.basename(tool).lower():
                cmd = [tool, "compile", "--fqbn", board, "--upload", "-p", port, ino_path]
            else:
                cmd = [tool, "--upload", "--board", board, "--port", port, ino_path]

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
