# main.py  — PySide6 light UI, scrollable columns, corrected backgrounds & alignment

from __future__ import annotations
import sys, json, datetime
from typing import List, Dict, Optional
from dataclasses import dataclass
from PySide6 import QtCore, QtGui, QtWidgets

# -------------------- optional qt-material (not required) --------------------
try:
    from qt_material import apply_stylesheet  # type: ignore
    HAVE_QT_MATERIAL = True
except Exception:
    HAVE_QT_MATERIAL = False

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
        self.debounce_timers = [None]*6
        self.last_sent = [None]*6
        self.debounce_ms = 25
        self.port = ""; self.baud = 9600

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
                self._write_raw(f"{idx+1}:{angle}\n".encode()); self.last_sent[idx] = angle
        t.timeout.connect(_do); t.start()
        self.debounce_timers[idx] = t

    def send_all(self, angles: List[int]):
        for i, a in enumerate(angles):
            self.send_angle(i, a)


# -------------------- main window --------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Criya – Stack Assembly Robot (Qt)")
        self.resize(1280, 860)
        self.setMinimumSize(1020, 720)

        # state
        self.link = SerialLink(self)
        self.angles = [90]*6
        self.reset_defaults = [0,90,90,90,90,90]
        self.saved_positions: Dict[str, List[int]] = {}
        self.sequence: List[SeqStep] = []

        # smooth move (timer-based)
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

    # ---------- ui ----------
    def _build_ui(self):
        splitter = QtWidgets.QSplitter(self)
        splitter.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.setCentralWidget(splitter)

        # LEFT scroll column
        left_scroll = QtWidgets.QScrollArea(); left_scroll.setWidgetResizable(True)
        left = QtWidgets.QWidget(); left_scroll.setWidget(left)
        L = QtWidgets.QVBoxLayout(left); L.setContentsMargins(16,16,16,16); L.setSpacing(14)

        # RIGHT scroll column
        right_scroll = QtWidgets.QScrollArea(); right_scroll.setWidgetResizable(True)
        right = QtWidgets.QWidget(); right_scroll.setWidget(right)
        R = QtWidgets.QVBoxLayout(right); R.setContentsMargins(16,16,16,16); R.setSpacing(14)

        splitter.addWidget(left_scroll); splitter.addWidget(right_scroll)
        splitter.setStretchFactor(0, 2); splitter.setStretchFactor(1, 1)

        # left groups
        L.addWidget(self._grp_connection())
        L.addWidget(self._grp_speed())
        L.addWidget(self._grp_defaults())
        L.addWidget(self._grp_actions())
        L.addWidget(self._grp_positions())
        L.addWidget(self._grp_sequence())
        L.addStretch(1)

        # right group
        R.addWidget(self._grp_servos()); R.addStretch(1)

        # status bar
        self.status_label = QtWidgets.QLabel("")
        self.statusBar().addWidget(self.status_label, 1)

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
        self.baudCombo = QtWidgets.QComboBox(); self.baudCombo.addItems(["9600","19200","38400","57600","115200"]); self.baudCombo.setCurrentText("9600")
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
        for i in range(6):
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
        for i in range(6):
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
        self.reset_defaults = [0,90,90,90,90,90]
        for i, e in enumerate(self.defaultEdits): e.setText(str(self.reset_defaults[i]))
        self._set_status("Factory defaults restored", OK_GREEN)

    # ---------- servo events ----------
    def _on_slider(self, idx: int, val: int):
        self.angles[idx]=val; self.edits[idx].setText(str(val)); self.link.send_angle(idx, val)

    # ---------- smooth move ----------
    def smooth_move(self, target: List[int], speed_override: Optional[int] = None, on_done: Optional[callable] = None):
        if on_done: self.move_done_callbacks.append(on_done)
        if self.smooth_timer.isActive(): self.smooth_timer.stop()
        self.smooth_start = self.angles[:]
        self.smooth_target = [max(0, min(180, int(a))) for a in target]
        self.smooth_step_idx = 0
        base = speed_override if speed_override is not None else self.speedSlider.value()
        self.smooth_delay_ms = max(1, int(base/200.0*1000))   # lower = faster
        self.smooth_timer.start(self.smooth_delay_ms)

    def _smooth_step(self):
        self.smooth_step_idx += 1
        s = self.smooth_step_idx; steps = self.smooth_steps
        new = []
        for i in range(6):
            v = int(self.smooth_start[i] + (self.smooth_target[i]-self.smooth_start[i]) * s / steps)
            v = max(0, min(180, v)); new.append(v)
            self.sliders[i].blockSignals(True); self.sliders[i].setValue(v); self.sliders[i].blockSignals(False)
            self.edits[i].setText(str(v))
        self.angles = new; self.link.send_all(new)
        if s >= steps:
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
            for k,v in data.items():
                if not (isinstance(v,list) and len(v)==6): raise ValueError("Invalid positions file")
            self.saved_positions.update(data)
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
                    new.append(SeqStep(kind="angles", angles=s.get("angles",[90]*6), label=s.get("label","angles"), dwell_ms=int(s["dwell"]), speed=int(s["speed"])))
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
    main()
