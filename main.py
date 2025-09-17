import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import time, threading, json, datetime
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class ScrollableFrame(ttk.Frame):
    """Reusable vertical scrollable frame using Canvas + interior frame."""
    def __init__(self, container, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0)
        self.vscroll = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vscroll.set)
        self.inner = ttk.Frame(self.canvas)
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas_frame = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.vscroll.pack(side="right", fill="y")
        self.bind("<Configure>", self._resize)

    def _resize(self, event):
        # make inner frame match canvas width
        self.canvas.itemconfig(self.canvas_frame, width=event.width - self.vscroll.winfo_width())


class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Criya - Stack Assembly Robot")
        self.root.minsize(1080, 650)

        # --- runtime state ---
        self.ser = None
        self.serial_lock = threading.Lock()
        self.runner_thread = None
        self.runner_stop = threading.Event()
        self.is_running_sequence = False
        self._move_done = threading.Event()          # <-- signals smooth_move completion

        # status
        self.status_var = tk.StringVar(value="Disconnected")

        # angles
        self.angles = [90, 90, 90, 90, 90, 90]

        # reset defaults (Servo1=0 others 90)
        self.reset_defaults = [0, 90, 90, 90, 90, 90]

        # saved positions {name: [a1..a6]}
        self.saved_positions = {}

        # sequence: list of dicts -> supports either named or raw angles
        #   {kind: 'pos', pos: name, dwell: ms, speed: 1..100}
        #   {kind: 'angles', angles: [..], label: str, dwell: ms, speed: 1..100}
        self.sequence = []

        # send throttling per-servo
        self._pending_send_after = {}
        self._last_sent_angles = self.angles[:]
        self._smooth_move_job = None

        # ----- layout: PanedWindow with two scrollable sides -----
        paned = ttk.Panedwindow(root, orient=tk.HORIZONTAL)
        paned.pack(fill="both", expand=True)

        left_scroll = ScrollableFrame(paned)
        right_scroll = ScrollableFrame(paned)
        paned.add(left_scroll, weight=1)
        paned.add(right_scroll, weight=1)

        left = left_scroll.inner
        right = right_scroll.inner

        # Connection box
        conn_box = ttk.LabelFrame(left, text="Connection")
        conn_box.pack(fill="x", pady=6, padx=6)

        self.port_cb = ttk.Combobox(conn_box, values=self.get_ports(), state="readonly", width=18)
        self.port_cb.pack(side="left", padx=6, pady=6)
        ttk.Button(conn_box, text="Refresh", command=self.refresh_ports).pack(side="left", padx=4)

        ttk.Label(conn_box, text="Baud").pack(side="left", padx=(8, 0))
        self.baud_cb = ttk.Combobox(conn_box, values=[9600, 19200, 38400, 57600, 115200], width=8)
        self.baud_cb.set(9600)  # default to match your Arduino sketch
        self.baud_cb.pack(side="left", padx=6)

        self.connect_btn = ttk.Button(conn_box, text="Connect", command=self.connect_serial)
        self.connect_btn.pack(side="left", padx=4)
        self.disconnect_btn = ttk.Button(conn_box, text="Disconnect", command=self.disconnect_serial, state="disabled")
        self.disconnect_btn.pack(side="left", padx=4)
        self.reconnect_btn = ttk.Button(conn_box, text="Force Reconnect", command=self.force_reconnect)
        self.reconnect_btn.pack(side="left", padx=4)

        # speed box
        spd_box = ttk.LabelFrame(left, text="Speed (lower = faster)")
        spd_box.pack(fill="x", pady=6, padx=6)
        self.speed_var = tk.DoubleVar(value=30)
        self.speed_scale = ttk.Scale(spd_box, from_=1, to=100, variable=self.speed_var, orient="horizontal")
        self.speed_scale.pack(fill="x", padx=6, pady=6)
        ttk.Label(spd_box, textvariable=self.speed_var).pack()

        # Defaults editor
        defaults_box = ttk.LabelFrame(left, text="Default Reset Positions")
        defaults_box.pack(fill="x", pady=6, padx=6)
        self.default_entries = []
        for i in range(6):
            row = ttk.Frame(defaults_box)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=f"Servo {i+1}").pack(side="left")
            var = tk.StringVar(value=str(self.reset_defaults[i]))
            ttk.Entry(row, textvariable=var, width=5).pack(side="left", padx=4)
            self.default_entries.append(var)
        ttk.Button(defaults_box, text="Apply Defaults", command=self.apply_defaults).pack(fill="x", pady=2)
        ttk.Button(defaults_box, text="Reset to Factory Defaults", command=self.reset_factory_defaults).pack(fill="x", pady=2)

        # Action buttons (Reset/Stop)
        action_box = ttk.Frame(left)
        action_box.pack(fill="x", pady=6, padx=6)
        ttk.Button(action_box, text="Reset Positions", command=self.reset_positions).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(action_box, text="Stop All", command=self.stop_all).pack(side="left", expand=True, fill="x", padx=2)

        # Saved Positions
        pos_box = ttk.LabelFrame(left, text="Saved Positions")
        pos_box.pack(fill="both", expand=False, pady=6, padx=6)

        name_row = ttk.Frame(pos_box)
        name_row.pack(fill="x", padx=4, pady=2)
        ttk.Label(name_row, text="Name:").pack(side="left")
        self.pos_name_var = tk.StringVar()
        ttk.Entry(name_row, textvariable=self.pos_name_var, width=18).pack(side="left", padx=4)
        ttk.Button(name_row, text="Save Current", command=self.save_current_position).pack(side="left", padx=2)
        ttk.Button(name_row, text="Update", command=self.update_selected_position).pack(side="left", padx=2)

        list_row = ttk.Frame(pos_box)
        list_row.pack(fill="both", expand=True, padx=4, pady=2)
        self.pos_list = tk.Listbox(list_row, height=6)
        self.pos_list.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(list_row, command=self.pos_list.yview)
        self.pos_list.config(yscrollcommand=sb.set)
        sb.pack(side="left", fill="y")

        btn_row = ttk.Frame(pos_box)
        btn_row.pack(fill="x", padx=4, pady=2)
        ttk.Button(btn_row, text="Move To", command=self.move_to_selected_position).pack(side="left", padx=2)
        ttk.Button(btn_row, text="Delete", command=self.delete_selected_position).pack(side="left", padx=2)
        ttk.Button(btn_row, text="Export", command=self.export_positions).pack(side="left", padx=2)
        ttk.Button(btn_row, text="Import", command=self.import_positions).pack(side="left", padx=2)

        # Program Sequencer
        prog_box = ttk.LabelFrame(left, text="Program / Sequence")
        prog_box.pack(fill="both", expand=True, pady=6, padx=6)

        # sequence table + scrollbar
        tree_frame = ttk.Frame(prog_box)
        tree_frame.pack(fill="both", expand=True)
        self.seq_tree = ttk.Treeview(tree_frame, columns=("#", "Step", "Dwell(ms)", "Speed"), show="headings", height=6)
        for col, w in (("#", 60), ("Step", 220), ("Dwell(ms)", 100), ("Speed", 80)):
            self.seq_tree.heading(col, text=col)
            self.seq_tree.column(col, width=w, anchor="center")
        self.seq_tree.pack(side="left", fill="both", expand=True)
        seq_sb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.seq_tree.yview)
        self.seq_tree.configure(yscrollcommand=seq_sb.set)
        seq_sb.pack(side="left", fill="y")

        # Add-step controls
        controls = ttk.Frame(prog_box)
        controls.pack(fill="x", padx=4, pady=4)
        ttk.Label(controls, text="Use position:").pack(side="left")
        self.add_pos_cb = ttk.Combobox(controls, values=[], width=18, state="readonly")
        self.add_pos_cb.pack(side="left", padx=4)
        ttk.Label(controls, text="Dwell(ms)").pack(side="left")
        self.add_dwell = tk.StringVar(value="500")
        ttk.Entry(controls, textvariable=self.add_dwell, width=7).pack(side="left", padx=2)
        ttk.Label(controls, text="Speed").pack(side="left")
        self.add_speed = tk.StringVar(value="30")
        ttk.Entry(controls, textvariable=self.add_speed, width=5).pack(side="left", padx=2)
        ttk.Button(controls, text="Add Step", command=self.add_step).pack(side="left", padx=6)
        ttk.Button(controls, text="Add Current (no save)", command=self.add_step_current).pack(side="left", padx=6)

        row2 = ttk.Frame(prog_box)
        row2.pack(fill="x", padx=4, pady=2)
        ttk.Button(row2, text="Up", command=self.step_up).pack(side="left", padx=2)
        ttk.Button(row2, text="Down", command=self.step_down).pack(side="left", padx=2)
        ttk.Button(row2, text="Delete Step", command=self.delete_step).pack(side="left", padx=2)
        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row2, text="Loop", variable=self.loop_var).pack(side="left", padx=10)

        row3 = ttk.Frame(prog_box)
        row3.pack(fill="x", padx=4, pady=2)
        ttk.Button(row3, text="Run Sequence", command=self.run_sequence).pack(side="left", padx=2)
        ttk.Button(row3, text="Stop Sequence", command=self.stop_sequence).pack(side="left", padx=2)
        ttk.Button(row3, text="Save Seq", command=self.export_sequence).pack(side="left", padx=2)
        ttk.Button(row3, text="Load Seq", command=self.import_sequence).pack(side="left", padx=2)

        # Servo sliders on right (also scrollable)
        servo_box = ttk.LabelFrame(right, text="Servo Controls")
        servo_box.pack(fill="both", expand=True, padx=6, pady=6)

        self.servo_scales = []
        self.servo_entries = []
        for i in range(6):
            frame = ttk.Frame(servo_box)
            frame.pack(fill="x", pady=4)
            ttk.Label(frame, text=f"Servo {i+1}").pack(side="left")
            scale = ttk.Scale(frame, from_=0, to=180, orient="horizontal")
            scale.set(self.angles[i])
            scale.pack(side="left", fill="x", expand=True, padx=6)
            scale.bind("<B1-Motion>", lambda e, idx=i: self.update_angle(idx))
            scale.bind("<ButtonRelease-1>", lambda e, idx=i: self.update_angle(idx))
            self.servo_scales.append(scale)

            entry_var = tk.StringVar(value=str(self.angles[i]))
            entry = ttk.Entry(frame, textvariable=entry_var, width=5)
            entry.pack(side="left", padx=4)
            entry.bind("<Return>", lambda e, idx=i, var=entry_var: self.set_angle_from_entry(idx, var))
            self.servo_entries.append(entry_var)

            ttk.Label(frame, textvariable=entry_var).pack(side="left")

        # Status bar
        self.status_label = tk.Label(root, textvariable=self.status_var, relief="sunken", anchor="w")
        self.status_label.pack(side="bottom", fill="x")
        self.update_status("Disconnected", "red")

        # Background checker
        self.root.after(1000, self.check_connection)

    # ------------------- serial / status -------------------
    def get_ports(self):
        if serial:
            return [p.device for p in serial.tools.list_ports.comports()]
        return []

    def refresh_ports(self):
        self.port_cb["values"] = self.get_ports()

    def connect_serial(self):
        if not serial:
            messagebox.showwarning("pyserial missing", "pyserial not installed.")
            return
        port = self.port_cb.get()
        if not port:
            messagebox.showinfo("Select port", "Choose a COM port")
            return
        baud = int(self.baud_cb.get())
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05, write_timeout=0.05)
            self.update_status(f"Connected to {port} @ {baud}", "green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.send_all_angles(self.reset_defaults)  # auto reset servos
        except Exception as e:
            self.update_status(f"Error: {e}", "red")

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.update_status("Disconnected", "red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")

    def force_reconnect(self):
        self.update_status("Reconnecting...", "orange")
        self.disconnect_serial()
        self.root.after(500, self.connect_serial)

    def check_connection(self):
        if self.ser and not self.ser.is_open:
            self.update_status("Lost connection. Reconnecting...", "orange")
            self.force_reconnect()
        self.root.after(1000, self.check_connection)

    def update_status(self, message, color):
        self.status_var.set(message)
        self.status_label.config(fg=color)

    # ------------------- servo I/O -------------------
    def send_angle(self, idx, angle):
        """Debounced send: coalesce rapid slider updates to ~40 FPS (25ms)."""
        if idx in self._pending_send_after:
            self.root.after_cancel(self._pending_send_after[idx])

        def _do_send():
            self._pending_send_after.pop(idx, None)
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    try:
                        if self._last_sent_angles[idx] != angle:
                            msg = f"{idx+1}:{angle}\n".encode()
                            self.ser.write(msg)
                            self._last_sent_angles[idx] = angle
                    except Exception:
                        self.update_status("Write error. Reconnecting...", "red")
                        self.force_reconnect()

        self._pending_send_after[idx] = self.root.after(25, _do_send)

    def send_all_angles(self, angles):
        for i, a in enumerate(angles):
            self.send_angle(i, a)

    def update_angle(self, idx):
        angle = int(self.servo_scales[idx].get())
        self.angles[idx] = angle
        self.servo_entries[idx].set(str(angle))
        self.send_angle(idx, angle)

    def set_angle_from_entry(self, idx, var):
        try:
            val = int(var.get())
            val = max(0, min(180, val))
            self.servo_scales[idx].set(val)
            self.angles[idx] = val
            var.set(str(val))
            self.send_angle(idx, val)
        except ValueError:
            var.set(str(self.angles[idx]))

    def smooth_move(self, target, speed_override=None, wait=False):
        """Non-blocking smooth move using Tk 'after'.
           If wait=True, block the caller until the move completes (safe from non-UI thread)."""
        if self._smooth_move_job:
            self.root.after_cancel(self._smooth_move_job)
            self._smooth_move_job = None

        self._move_done.clear()

        start = self.angles[:]
        steps = 40
        base_speed = float(speed_override) if speed_override is not None else float(self.speed_var.get())
        delay = max(1, int(base_speed / 200.0 * 1000))  # ms per step

        t0 = {"step": 0}

        def stepper():
            s = t0["step"] + 1
            t0["step"] = s
            new = []
            for i in range(6):
                val = int(start[i] + (target[i] - start[i]) * s / steps)
                val = max(0, min(180, val))
                new.append(val)
                self.servo_scales[i].set(val)
                self.servo_entries[i].set(str(val))
            self.angles = new
            self.send_all_angles(new)

            if s < steps and not self.runner_stop.is_set():
                self._smooth_move_job = self.root.after(delay, stepper)
            else:
                self._smooth_move_job = None
                self._move_done.set()

        self._smooth_move_job = self.root.after(0, stepper)

        if wait:
            self._move_done.wait()

    def reset_positions(self):
        # From UI button: non-blocking so the UI doesn't freeze
        self.smooth_move(self.reset_defaults, wait=False)

    def stop_all(self):
        self.runner_stop.set()
        self.is_running_sequence = False
        self.update_status("Stopped", "red")

    # ------------------- defaults editor -------------------
    def apply_defaults(self):
        try:
            self.reset_defaults = [int(var.get()) for var in self.default_entries]
            self.update_status(f"Applied reset defaults: {self.reset_defaults}", "green")
        except ValueError:
            messagebox.showerror("Error", "Invalid reset default values")

    def reset_factory_defaults(self):
        self.reset_defaults = [0, 90, 90, 90, 90, 90]
        for i, var in enumerate(self.default_entries):
            var.set(str(self.reset_defaults[i]))
        self.update_status("Factory defaults restored", "green")

    # ------------------- saved positions -------------------
    def refresh_pos_widgets(self):
        self.pos_list.delete(0, tk.END)
        for name in sorted(self.saved_positions.keys()):
            self.pos_list.insert(tk.END, name)
        self.add_pos_cb["values"] = sorted(self.saved_positions.keys())

    def save_current_position(self):
        name = self.pos_name_var.get().strip()
        if not name:
            name = datetime.datetime.now().strftime("pos_%H%M%S")
            self.pos_name_var.set(name)
        self.saved_positions[name] = self.angles[:]
        self.refresh_pos_widgets()
        self.update_status(f"Saved position '{name}' = {self.saved_positions[name]}", "green")

    def get_selected_pos_name(self):
        try:
            idx = self.pos_list.curselection()[0]
            return self.pos_list.get(idx)
        except Exception:
            return None

    def update_selected_position(self):
        name = self.get_selected_pos_name()
        if not name:
            messagebox.showinfo("Select", "Select a position to update")
            return
        self.saved_positions[name] = self.angles[:]
        self.update_status(f"Updated '{name}'", "green")

    def move_to_selected_position(self):
        name = self.get_selected_pos_name()
        if not name:
            messagebox.showinfo("Select", "Select a position to move to")
            return
        target = self.saved_positions[name]
        # From UI: don't block the Tk thread
        self.smooth_move(target, wait=False)

    def delete_selected_position(self):
        name = self.get_selected_pos_name()
        if not name:
            return
        del self.saved_positions[name]
        self.refresh_pos_widgets()

    def export_positions(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "w") as f:
            json.dump(self.saved_positions, f, indent=2)
        self.update_status(f"Positions saved to {path}", "green")

    def import_positions(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            for k, v in data.items():
                if not (isinstance(v, list) and len(v) == 6):
                    raise ValueError("Invalid positions file")
            self.saved_positions.update(data)
            self.refresh_pos_widgets()
            self.update_status(f"Imported positions from {path}", "green")
        except Exception as e:
            messagebox.showerror("Import error", str(e))

    # ------------------- program / sequence -------------------
    def rebuild_seq_tree(self):
        for iid in self.seq_tree.get_children():
            self.seq_tree.delete(iid)
        for i, step in enumerate(self.sequence, start=1):
            if step["kind"] == "pos":
                label = f"{step['pos']} (saved)"
            else:
                label = step.get("label", "current") + " (angles)"
            self.seq_tree.insert("", tk.END, values=(i, label, step["dwell"], step["speed"]))

    def add_step(self):
        pos_name = self.add_pos_cb.get()
        if not pos_name:
            messagebox.showinfo("No position", "Choose a saved position or use 'Add Current (no save)'")
            return
        if pos_name not in self.saved_positions:
            messagebox.showerror("Unknown position", "Selected position does not exist")
            return
        try:
            dwell = int(self.add_dwell.get())
            speed = int(self.add_speed.get())
        except ValueError:
            messagebox.showerror("Invalid", "Dwell and Speed must be integers")
            return
        self.sequence.append({"kind": "pos", "pos": pos_name, "dwell": dwell, "speed": speed})
        self.rebuild_seq_tree()

    def add_step_current(self):
        try:
            dwell = int(self.add_dwell.get())
            speed = int(self.add_speed.get())
        except ValueError:
            messagebox.showerror("Invalid", "Dwell and Speed must be integers")
            return
        label = datetime.datetime.now().strftime("current_%H%M%S")
        self.sequence.append({"kind": "angles", "angles": self.angles[:], "label": label, "dwell": dwell, "speed": speed})
        self.rebuild_seq_tree()

    def _selected_step_index(self):
        sel = self.seq_tree.selection()
        if not sel:
            return None
        return self.seq_tree.index(sel[0])

    def delete_step(self):
        idx = self._selected_step_index()
        if idx is None:
            return
        self.sequence.pop(idx)
        self.rebuild_seq_tree()

    def step_up(self):
        idx = self._selected_step_index()
        if idx is None or idx == 0:
            return
        self.sequence[idx - 1], self.sequence[idx] = self.sequence[idx], self.sequence[idx - 1]
        self.rebuild_seq_tree()
        self.seq_tree.selection_set(self.seq_tree.get_children()[idx - 1])

    def step_down(self):
        idx = self._selected_step_index()
        if idx is None or idx >= len(self.sequence) - 1:
            return
        self.sequence[idx + 1], self.sequence[idx] = self.sequence[idx], self.sequence[idx + 1]
        self.rebuild_seq_tree()
        self.seq_tree.selection_set(self.seq_tree.get_children()[idx + 1])

    def _run_sequence_worker(self):
        try:
            self.is_running_sequence = True
            self.runner_stop.clear()
            while not self.runner_stop.is_set():
                for step in self.sequence:
                    if self.runner_stop.is_set():
                        break
                    if step["kind"] == "pos":
                        if step["pos"] not in self.saved_positions:
                            self.update_status(f"Missing position '{step['pos']}'", "red")
                            self.runner_stop.set()
                            break
                        target = self.saved_positions[step["pos"]]
                        msg = f"Moving to {step['pos']}"
                    else:
                        target = step["angles"]
                        msg = f"Moving to {step.get('label','angles')}"
                    self.update_status(msg, "orange")
                    # Wait for move to finish here (non-UI thread)
                    self.smooth_move(target, speed_override=step["speed"], wait=True)
                    self.update_status(f"Dwell {step['dwell']} ms", "orange")
                    time.sleep(max(0, step["dwell"]) / 1000.0)
                if not self.loop_var.get():
                    break
            self.update_status("Sequence finished", "green")
        finally:
            self.is_running_sequence = False
            self.runner_stop.clear()

    def run_sequence(self):
        if self.is_running_sequence:
            return
        if not self.sequence:
            messagebox.showinfo("Empty", "Add steps to the sequence")
            return
        self.runner_thread = threading.Thread(target=self._run_sequence_worker, daemon=True)
        self.runner_thread.start()

    def stop_sequence(self):
        self.runner_stop.set()

    def export_sequence(self):
        if not self.sequence:
            messagebox.showinfo("Empty", "Nothing to save")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "w") as f:
            json.dump(self.sequence, f, indent=2)
        self.update_status(f"Sequence saved to {path}", "green")

    def import_sequence(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, "r") as f:
                seq = json.load(f)
            for s in seq:
                if not (isinstance(s, dict) and "kind" in s and "dwell" in s and "speed" in s):
                    raise ValueError("Invalid sequence file")
            self.sequence = seq
            self.rebuild_seq_tree()
            self.update_status(f"Loaded sequence from {path}", "green")
        except Exception as e:
            messagebox.showerror("Import error", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.mainloop()
