import json
import threading
import queue
import time
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

BAUD = 115200

class SerialWorker:
    def __init__(self):
        self.ser = None
        self.thread = None
        self.stop_evt = threading.Event()
        self.rx_q = queue.Queue()

    def ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_evt.set()
        try:
            if self.thread:
                self.thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.thread = None

    def send_line(self, line: str):
        if not self.ser:
            return
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode("utf-8", errors="ignore"))

    def _reader(self):
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("utf-8", errors="ignore").strip()
                    if s:
                        self.rx_q.put(s)
            except Exception as e:
                self.rx_q.put(json.dumps({"type":"info","msg":f"Serial read error: {e}"}))
                break


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MotorNode Controller (via MasterNode)")
        self.geometry("1100x700")

        self.sw = SerialWorker()

        # Latest known status/config
        self.status = {}
        self.master_info = {}
        self.connected_port = tk.StringVar(value="")

        # Node tracking (multi MotorNode support)
        # mac_str -> {"id": int, "last_seen": float, "status": dict|None}
        self.nodes = {}
        self._tree_item_by_mac = {}
        self.active_mac = None
        self._config_pull_pending = False
        self._config_pull_target_mac = None

        self._build_ui()
        self._refresh_ports()

        self.after(100, self._poll_rx)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=8)

        ttk.Label(top, text="Serial Port:").pack(side=tk.LEFT)
        self.cb_ports = ttk.Combobox(top, width=18, state="readonly")
        self.cb_ports.pack(side=tk.LEFT, padx=6)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=4)
        self.btn_conn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.btn_conn.pack(side=tk.LEFT, padx=6)

        self.lbl_conn = ttk.Label(top, text="Disconnected")
        self.lbl_conn.pack(side=tk.LEFT, padx=12)

        ttk.Button(top, text="GET_STATUS (refresh now)", command=lambda: self._cmd("GET_STATUS")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="PING (discover nodes)", command=lambda: self._cmd("PING")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="HELP", command=lambda: self._cmd("HELP")).pack(side=tk.LEFT, padx=4)

        # Main layout
        main = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left = ttk.Frame(main)
        right = ttk.Frame(main)
        main.add(left, weight=3)
        main.add(right, weight=2)

        # Split left side into tabs so Controls stay visible
        left_nb = ttk.Notebook(left)
        left_nb.pack(fill=tk.BOTH, expand=True)

        tab_ctrl = ttk.Frame(left_nb)
        tab_nodes = ttk.Frame(left_nb)
        left_nb.add(tab_ctrl, text="Control")
        left_nb.add(tab_nodes, text="Nodes/Targets")

        # Nodes panel (nodes tab)
        nodes_fr = ttk.LabelFrame(tab_nodes, text="Nodes")
        nodes_fr.pack(fill=tk.X, pady=6)

        self.tree_nodes = ttk.Treeview(nodes_fr, columns=("id", "mac", "last"), show="headings", height=5)
        self.tree_nodes.heading("id", text="ID")
        self.tree_nodes.heading("mac", text="MAC")
        self.tree_nodes.heading("last", text="Last seen")
        self.tree_nodes.column("id", width=40, anchor="center", stretch=False)
        self.tree_nodes.column("mac", width=150, anchor="w", stretch=True)
        self.tree_nodes.column("last", width=90, anchor="center", stretch=False)
        self.tree_nodes.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)
        self.tree_nodes.bind("<<TreeviewSelect>>", self._on_node_selected)

        id_set_fr = ttk.Frame(nodes_fr)
        id_set_fr.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)

        ttk.Label(id_set_fr, text="New ID (1-32):").pack(side=tk.LEFT)
        self.ent_new_id = ttk.Entry(id_set_fr, width=6)
        self.ent_new_id.insert(0, "1")
        self.ent_new_id.pack(side=tk.LEFT, padx=6)
        ttk.Button(id_set_fr, text="Set ID for selected MAC", command=self._set_id_for_selected).pack(side=tk.LEFT, padx=4)

        # Target IDs panel (nodes tab)
        targets_fr = ttk.LabelFrame(tab_nodes, text="Target IDs (messages go to these MotorNode IDs)")
        targets_fr.pack(fill=tk.X, pady=6)

        self.target_id_vars = [tk.BooleanVar(value=False) for _ in range(32)]
        grid = ttk.Frame(targets_fr)
        grid.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)
        cols = 8
        for i in range(32):
            r = i // cols
            c = i % cols
            ttk.Checkbutton(grid, text=str(i + 1), variable=self.target_id_vars[i]).grid(row=r, column=c, padx=6, pady=2, sticky="w")

        btns = ttk.Frame(targets_fr)
        btns.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)
        ttk.Button(btns, text="All", command=self._targets_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="None", command=self._targets_none).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Active only", command=self._targets_active_only).pack(side=tk.LEFT, padx=4)

        # Status panel (control tab)
        status_fr = ttk.LabelFrame(tab_ctrl, text="Live Status")
        status_fr.pack(fill=tk.X, pady=6)

        self.status_vars = {}
        status_fr.grid_columnconfigure(1, weight=1)
        status_fr.grid_columnconfigure(3, weight=1)

        def add_stat(row, key, label):
            rows_per_col = 10
            col = 0 if row < rows_per_col else 2
            r = row if row < rows_per_col else (row - rows_per_col)
            ttk.Label(status_fr, text=label).grid(row=r, column=col, sticky="w", padx=6, pady=2)
            v = tk.StringVar(value="—")
            ttk.Label(status_fr, textvariable=v, font=("Consolas", 11)).grid(row=r, column=col + 1, sticky="w", padx=6, pady=2)
            self.status_vars[key] = v

        add_stat(0, "from", "Motor MAC")
        add_stat(1, "uptime_ms", "Uptime (ms)")
        add_stat(2, "motor_pos", "Motor pos (user steps)")
        add_stat(3, "enc_pos", "Encoder pos (user steps)")
        add_stat(4, "err", "Error (steps)")
        add_stat(5, "thr", "Threshold (steps)")
        add_stat(6, "missed", "Missed events (latched count)")
        add_stat(7, "temp_c", "Temp (°C)")
        add_stat(8, "moving", "Moving")
        add_stat(9, "en", "Outputs enabled")
        add_stat(10, "cl", "Closed-loop mode")
        add_stat(11, "keep", "Keep enabled")
        add_stat(12, "speed", "Speed (steps/s)")
        add_stat(13, "accel", "Accel (steps/s²)")
        add_stat(14, "usteps", "Microsteps")
        add_stat(15, "curr", "Currents (irun/ihold/ihd)")
        add_stat(16, "drv", "DRV_STATUS")
        add_stat(17, "ioin", "IOIN")
        add_stat(18, "ifcnt", "IFCNT")

        # Controls panel (control tab)
        ctrl_fr = ttk.LabelFrame(tab_ctrl, text="Motion Controls")
        ctrl_fr.pack(fill=tk.X, pady=6)

        row = 0
        ttk.Button(ctrl_fr, text="Enable", command=lambda: self._cmd_motor("ENABLE 1")).grid(row=row, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(ctrl_fr, text="Disable", command=lambda: self._cmd_motor("ENABLE 0")).grid(row=row, column=1, padx=5, pady=5, sticky="ew")
        ttk.Button(ctrl_fr, text="STOP (decel)", command=lambda: self._cmd_motor("STOP")).grid(row=row, column=2, padx=5, pady=5, sticky="ew")
        ttk.Button(ctrl_fr, text="FSTOP (hard)", command=lambda: self._cmd_motor("FSTOP")).grid(row=row, column=3, padx=5, pady=5, sticky="ew")

        row += 1
        ttk.Label(ctrl_fr, text="MoveTo:").grid(row=row, column=0, padx=5, pady=5, sticky="e")
        self.ent_moveto = ttk.Entry(ctrl_fr, width=12)
        self.ent_moveto.insert(0, "0")
        self.ent_moveto.grid(row=row, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(ctrl_fr, text="Send", command=self._send_moveto).grid(row=row, column=2, padx=5, pady=5, sticky="ew")

        ttk.Label(ctrl_fr, text="MoveBy:").grid(row=row, column=3, padx=5, pady=5, sticky="e")
        self.ent_moveby = ttk.Entry(ctrl_fr, width=12)
        self.ent_moveby.insert(0, "0")
        self.ent_moveby.grid(row=row, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(ctrl_fr, text="Send", command=self._send_moveby).grid(row=row, column=5, padx=5, pady=5, sticky="ew")

        row += 1
        ttk.Label(ctrl_fr, text="Velocity (signed):").grid(row=row, column=0, padx=5, pady=5, sticky="e")
        self.ent_vel = ttk.Entry(ctrl_fr, width=12)
        self.ent_vel.insert(0, "0")
        self.ent_vel.grid(row=row, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(ctrl_fr, text="VEL", command=self._send_vel).grid(row=row, column=2, padx=5, pady=5, sticky="ew")

        # Toggles (control tab)
        togg_fr = ttk.LabelFrame(tab_ctrl, text="Toggles")
        togg_fr.pack(fill=tk.X, pady=6)

        self.var_keep = tk.BooleanVar(value=False)
        self.var_cl   = tk.BooleanVar(value=False)

        ttk.Checkbutton(togg_fr, text="Keep enabled (don’t disable after moves)",
                        variable=self.var_keep, command=self._toggle_keep).grid(row=0, column=0, padx=6, pady=6, sticky="w")
        ttk.Checkbutton(togg_fr, text="Closed-loop mode (CL=1 active recovery + correction)",
                        variable=self.var_cl, command=self._toggle_cl).grid(row=0, column=1, padx=6, pady=6, sticky="w")

        # Config panel (right)
        cfg = ttk.LabelFrame(right, text="Configuration")
        cfg.pack(fill=tk.BOTH, expand=True)

        # Speed/accel
        f1 = ttk.Frame(cfg); f1.pack(fill=tk.X, pady=6, padx=6)
        ttk.Label(f1, text="Speed (steps/s):").grid(row=0, column=0, sticky="e")
        self.ent_speed = ttk.Entry(f1, width=12); self.ent_speed.insert(0, "20000")
        self.ent_speed.grid(row=0, column=1, padx=6)
        ttk.Label(f1, text="Accel (steps/s²):").grid(row=0, column=2, sticky="e")
        self.ent_accel = ttk.Entry(f1, width=12); self.ent_accel.insert(0, "200000")
        self.ent_accel.grid(row=0, column=3, padx=6)
        ttk.Button(f1, text="Apply", command=self._apply_speed_accel).grid(row=0, column=4, padx=6)

        # Microsteps, currents
        f2 = ttk.Frame(cfg); f2.pack(fill=tk.X, pady=6, padx=6)
        ttk.Label(f2, text="Microsteps:").grid(row=0, column=0, sticky="e")
        self.cb_usteps = ttk.Combobox(f2, width=10, state="readonly",
                                      values=["1","2","4","8","16","32","64","128","256"])
        self.cb_usteps.set("32")
        self.cb_usteps.grid(row=0, column=1, padx=6)
        ttk.Button(f2, text="Set", command=self._set_usteps).grid(row=0, column=2, padx=6)

        ttk.Label(f2, text="irun:").grid(row=1, column=0, sticky="e")
        self.ent_irun = ttk.Entry(f2, width=8); self.ent_irun.insert(0, "20")
        self.ent_irun.grid(row=1, column=1, sticky="w", padx=6)
        ttk.Label(f2, text="ihold:").grid(row=1, column=2, sticky="e")
        self.ent_ihold = ttk.Entry(f2, width=8); self.ent_ihold.insert(0, "0")
        self.ent_ihold.grid(row=1, column=3, sticky="w", padx=6)
        ttk.Label(f2, text="ihd:").grid(row=1, column=4, sticky="e")
        self.ent_ihd = ttk.Entry(f2, width=8); self.ent_ihd.insert(0, "1")
        self.ent_ihd.grid(row=1, column=5, sticky="w", padx=6)
        ttk.Button(f2, text="Set Currents", command=self._set_currents).grid(row=1, column=6, padx=6)

        # Mismatch thresholds
        f3 = ttk.LabelFrame(cfg, text="Mismatch / Closed-loop thresholds")
        f3.pack(fill=tk.X, pady=6, padx=6)

        ttk.Label(f3, text="Base (fullsteps):").grid(row=0, column=0, sticky="e", padx=6, pady=4)
        self.ent_thr_base = ttk.Entry(f3, width=10); self.ent_thr_base.insert(0, "2.0")
        self.ent_thr_base.grid(row=0, column=1, padx=6, pady=4)
        ttk.Button(f3, text="Set", command=self._set_thr_base).grid(row=0, column=2, padx=6, pady=4)

        ttk.Label(f3, text="Gain (fullsteps/rps):").grid(row=1, column=0, sticky="e", padx=6, pady=4)
        self.ent_thr_gain = ttk.Entry(f3, width=10); self.ent_thr_gain.insert(0, "0.0")
        self.ent_thr_gain.grid(row=1, column=1, padx=6, pady=4)
        ttk.Button(f3, text="Set", command=self._set_thr_gain).grid(row=1, column=2, padx=6, pady=4)

        ttk.Label(f3, text="Check period (us):").grid(row=2, column=0, sticky="e", padx=6, pady=4)
        self.ent_thr_us = ttk.Entry(f3, width=10); self.ent_thr_us.insert(0, "2000")
        self.ent_thr_us.grid(row=2, column=1, padx=6, pady=4)
        ttk.Button(f3, text="Set", command=self._set_thr_us).grid(row=2, column=2, padx=6, pady=4)

        # Thermistor
        f4 = ttk.LabelFrame(cfg, text="Thermistor params (MotorNode ADC GPIO8)")
        f4.pack(fill=tk.X, pady=6, padx=6)

        ttk.Label(f4, text="Rfixed (ohm):").grid(row=0, column=0, sticky="e", padx=6, pady=3)
        self.ent_rfixed = ttk.Entry(f4, width=10); self.ent_rfixed.insert(0, "4700")
        self.ent_rfixed.grid(row=0, column=1, padx=6, pady=3)

        ttk.Label(f4, text="R0 (ohm):").grid(row=0, column=2, sticky="e", padx=6, pady=3)
        self.ent_r0 = ttk.Entry(f4, width=10); self.ent_r0.insert(0, "47000")
        self.ent_r0.grid(row=0, column=3, padx=6, pady=3)

        ttk.Label(f4, text="Beta:").grid(row=1, column=0, sticky="e", padx=6, pady=3)
        self.ent_beta = ttk.Entry(f4, width=10); self.ent_beta.insert(0, "3950")
        self.ent_beta.grid(row=1, column=1, padx=6, pady=3)

        ttk.Label(f4, text="T0 (°C):").grid(row=1, column=2, sticky="e", padx=6, pady=3)
        self.ent_t0 = ttk.Entry(f4, width=10); self.ent_t0.insert(0, "25.0")
        self.ent_t0.grid(row=1, column=3, padx=6, pady=3)

        ttk.Label(f4, text="Samples:").grid(row=2, column=0, sticky="e", padx=6, pady=3)
        self.ent_samples = ttk.Entry(f4, width=10); self.ent_samples.insert(0, "8")
        self.ent_samples.grid(row=2, column=1, padx=6, pady=3)

        ttk.Button(f4, text="Apply Therm Params", command=self._set_therm).grid(row=2, column=3, padx=6, pady=3, sticky="e")

        # Sync/Zero helpers (features you will want)
        f5 = ttk.LabelFrame(cfg, text="Sync / Zero helpers")
        f5.pack(fill=tk.X, pady=6, padx=6)

        ttk.Button(f5, text="ENC_TO_MOTOR", command=lambda: self._cmd_motor("ENC_TO_MOTOR")).grid(row=0, column=0, padx=6, pady=4)
        ttk.Button(f5, text="MOTOR_TO_ENC", command=lambda: self._cmd_motor("MOTOR_TO_ENC")).grid(row=0, column=1, padx=6, pady=4)
        ttk.Button(f5, text="MOTOR_ZERO", command=lambda: self._cmd_motor("MOTOR_ZERO")).grid(row=1, column=0, padx=6, pady=4)
        ttk.Button(f5, text="ENC_ZERO", command=lambda: self._cmd_motor("ENC_ZERO")).grid(row=1, column=1, padx=6, pady=4)

        ttk.Button(f5, text="APPLY (re-write TMC regs)", command=lambda: self._cmd_motor("APPLY")).grid(row=2, column=0, padx=6, pady=4)
        ttk.Button(f5, text="REQ_STATUS (targets)", command=lambda: self._cmd_motor("REQ_STATUS")).grid(row=2, column=1, padx=6, pady=4)

        # Save / load config (extra useful feature)
        f6 = ttk.Frame(cfg); f6.pack(fill=tk.X, pady=8, padx=6)
        ttk.Button(f6, text="Save config…", command=self._save_config).pack(side=tk.LEFT, padx=6)
        ttk.Button(f6, text="Load config…", command=self._load_config).pack(side=tk.LEFT, padx=6)
        ttk.Button(f6, text="Send config to node", command=self._send_full_config).pack(side=tk.LEFT, padx=6)
        ttk.Button(f6, text="Update config from node (active)", command=self._pull_config_from_active).pack(side=tk.LEFT, padx=6)
        ttk.Button(f6, text="Save config to flash (targets)", command=self._save_config_to_flash).pack(side=tk.LEFT, padx=6)

        # Log
        log_fr = ttk.LabelFrame(right, text="Log")
        log_fr.pack(fill=tk.BOTH, expand=True, pady=6, padx=6)
        self.txt_log = tk.Text(log_fr, height=12, wrap="none")
        self.txt_log.pack(fill=tk.BOTH, expand=True)
        self._log("GUI ready. Connect to MasterNode COM port.\n")

    # ---------------- actions ----------------
    def _refresh_ports(self):
        ports = self.sw.ports()
        self.cb_ports["values"] = ports
        if ports:
            if self.cb_ports.get() not in ports:
                self.cb_ports.set(ports[0])

    def _toggle_connect(self):
        if self.sw.ser:
            self.sw.disconnect()
            self.btn_conn.config(text="Connect")
            self.lbl_conn.config(text="Disconnected")
            return

        port = self.cb_ports.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a COM port")
            return
        try:
            self.sw.connect(port)
            self.btn_conn.config(text="Disconnect")
            self.lbl_conn.config(text=f"Connected: {port}")
            self._log(f"Connected to {port}\n")
            self._cmd("GET_STATUS")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open {port}\n{e}")

    def _cmd(self, s: str):
        if not self.sw.ser:
            self._log("Not connected.\n")
            return
        self.sw.send_line(s)
        self._log(f"> {s}\n")

    def _target_mask(self) -> int:
        mask = 0
        for i, v in enumerate(self.target_id_vars):
            if v.get():
                mask |= (1 << i)  # bit0 -> ID1
        return mask

    def _cmd_motor(self, cmd: str, mask: int = None):
        if mask is None:
            mask = self._target_mask()
        if not mask:
            self._log("No target IDs selected.\n")
            return
        self._cmd(f"IDS 0x{mask:08X} {cmd}")

    def _send_moveto(self):
        self._cmd_motor(f"MOVE_TO {self.ent_moveto.get().strip()}")

    def _send_moveby(self):
        self._cmd_motor(f"MOVE_BY {self.ent_moveby.get().strip()}")

    def _send_vel(self):
        self._cmd_motor(f"VEL {self.ent_vel.get().strip()}")

    def _toggle_keep(self):
        self._cmd_motor(f"KEEP {1 if self.var_keep.get() else 0}")

    def _toggle_cl(self):
        self._cmd_motor(f"CL {1 if self.var_cl.get() else 0}")

    def _apply_speed_accel(self):
        self._cmd_motor(f"SPEED {self.ent_speed.get().strip()}")
        self._cmd_motor(f"ACCEL {self.ent_accel.get().strip()}")

    def _set_usteps(self):
        self._cmd_motor(f"USTEPS {self.cb_usteps.get().strip()}")

    def _set_currents(self):
        self._cmd_motor(f"CUR {self.ent_irun.get().strip()} {self.ent_ihold.get().strip()} {self.ent_ihd.get().strip()}")

    def _set_thr_base(self):
        self._cmd_motor(f"THR_BASE {self.ent_thr_base.get().strip()}")

    def _set_thr_gain(self):
        self._cmd_motor(f"THR_GAIN {self.ent_thr_gain.get().strip()}")

    def _set_thr_us(self):
        self._cmd_motor(f"THR_US {self.ent_thr_us.get().strip()}")

    def _set_therm(self):
        self._cmd_motor(
            f"THERM {self.ent_rfixed.get().strip()} {self.ent_r0.get().strip()} "
            f"{self.ent_beta.get().strip()} {self.ent_t0.get().strip()} {self.ent_samples.get().strip()}"
        )

    def _save_config(self):
        cfg = self._current_config_dict()
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json")]
        )
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        self._log(f"Saved config: {path}\n")

    def _load_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self._apply_config_to_ui(cfg)
            self._log(f"Loaded config: {path}\n")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load config:\n{e}")

    def _send_full_config(self):
        # Sends all current UI settings to node in a reasonable order
        self._cmd_motor(f"KEEP {1 if self.var_keep.get() else 0}")
        self._cmd_motor(f"CL {1 if self.var_cl.get() else 0}")
        self._cmd_motor(f"USTEPS {self.cb_usteps.get().strip()}")
        self._cmd_motor(f"CUR {self.ent_irun.get().strip()} {self.ent_ihold.get().strip()} {self.ent_ihd.get().strip()}")
        self._cmd_motor(f"SPEED {self.ent_speed.get().strip()}")
        self._cmd_motor(f"ACCEL {self.ent_accel.get().strip()}")
        self._cmd_motor(f"THR_BASE {self.ent_thr_base.get().strip()}")
        self._cmd_motor(f"THR_GAIN {self.ent_thr_gain.get().strip()}")
        self._cmd_motor(f"THR_US {self.ent_thr_us.get().strip()}")
        self._set_therm()
        self._cmd_motor("APPLY")
        self._cmd("GET_STATUS")

    def _save_config_to_flash(self):
        # Push full config, then request NVS save on MotorNode(s)
        self._send_full_config()
        self._cmd_motor("SAVE_CFG")
        self._cmd_motor("REQ_STATUS")

    def _pull_config_from_active(self):
        if not self.active_mac:
            messagebox.showerror("Error", "Select a node first (in the Nodes list).")
            return
        node = self.nodes.get(self.active_mac, {})
        node_id = int(node.get("id", 0) or 0)
        if node_id < 1 or node_id > 32:
            messagebox.showerror("Error", "Active node has no valid ID (1-32). Set an ID first.")
            return
        self._config_pull_pending = True
        self._config_pull_target_mac = self.active_mac
        mask = 1 << (node_id - 1)
        self._cmd_motor("REQ_STATUS", mask=mask)

    def _current_config_dict(self):
        return {
            "keep": bool(self.var_keep.get()),
            "cl": bool(self.var_cl.get()),
            "usteps": int(self.cb_usteps.get()),
            "irun": int(self.ent_irun.get()),
            "ihold": int(self.ent_ihold.get()),
            "ihd": int(self.ent_ihd.get()),
            "speed": int(self.ent_speed.get()),
            "accel": int(self.ent_accel.get()),
            "thr_base": float(self.ent_thr_base.get()),
            "thr_gain": float(self.ent_thr_gain.get()),
            "thr_us": int(self.ent_thr_us.get()),
            "therm": {
                "rfixed": int(self.ent_rfixed.get()),
                "r0": int(self.ent_r0.get()),
                "beta": int(self.ent_beta.get()),
                "t0": float(self.ent_t0.get()),
                "samples": int(self.ent_samples.get()),
            }
        }

    def _apply_config_to_ui(self, cfg):
        try:
            self.var_keep.set(bool(cfg.get("keep", False)))
            self.var_cl.set(bool(cfg.get("cl", False)))
            self.cb_usteps.set(str(cfg.get("usteps", 32)))
            self.ent_irun.delete(0, tk.END); self.ent_irun.insert(0, str(cfg.get("irun", 20)))
            self.ent_ihold.delete(0, tk.END); self.ent_ihold.insert(0, str(cfg.get("ihold", 0)))
            self.ent_ihd.delete(0, tk.END); self.ent_ihd.insert(0, str(cfg.get("ihd", 1)))
            self.ent_speed.delete(0, tk.END); self.ent_speed.insert(0, str(cfg.get("speed", 20000)))
            self.ent_accel.delete(0, tk.END); self.ent_accel.insert(0, str(cfg.get("accel", 200000)))
            self.ent_thr_base.delete(0, tk.END); self.ent_thr_base.insert(0, str(cfg.get("thr_base", 2.0)))
            self.ent_thr_gain.delete(0, tk.END); self.ent_thr_gain.insert(0, str(cfg.get("thr_gain", 0.0)))
            self.ent_thr_us.delete(0, tk.END); self.ent_thr_us.insert(0, str(cfg.get("thr_us", 2000)))

            therm = cfg.get("therm", {})
            self.ent_rfixed.delete(0, tk.END); self.ent_rfixed.insert(0, str(therm.get("rfixed", 4700)))
            self.ent_r0.delete(0, tk.END); self.ent_r0.insert(0, str(therm.get("r0", 47000)))
            self.ent_beta.delete(0, tk.END); self.ent_beta.insert(0, str(therm.get("beta", 3950)))
            self.ent_t0.delete(0, tk.END); self.ent_t0.insert(0, str(therm.get("t0", 25.0)))
            self.ent_samples.delete(0, tk.END); self.ent_samples.insert(0, str(therm.get("samples", 8)))
        except Exception:
            pass

    # ---------------- RX processing ----------------
    def _poll_rx(self):
        try:
            while True:
                line = self.sw.rx_q.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass
        self.after(100, self._poll_rx)

    def _handle_line(self, line: str):
        # Master sends JSON lines; but keep log readable if it's not JSON.
        if not line:
            return
        try:
            msg = json.loads(line)
        except Exception:
            self._log(line + "\n")
            return

        t = msg.get("type", "")
        if t == "status":
            self._record_node_msg(msg)
            if not self.active_mac:
                self.active_mac = msg.get("from")
            if msg.get("from") == self.active_mac:
                self.status = msg
                self._update_status_ui(msg)
            if self._config_pull_pending and msg.get("from") == self._config_pull_target_mac:
                self._apply_status_config_to_ui(msg)
                self._config_pull_pending = False
                self._config_pull_target_mac = None
        elif t == "ack":
            self._record_node_msg(msg)
            self._log(f"ACK id={msg.get('id','?')} cmd={msg.get('cmd')} seq={msg.get('seq')} code={msg.get('code')}\n")
        elif t == "info":
            self._log(f"[INFO] {msg.get('msg','')}\n")
        else:
            self._log(line + "\n")

    def _record_node_msg(self, msg: dict):
        mac = msg.get("from") or msg.get("mac")
        if not mac:
            return
        now = time.time()
        node = self.nodes.get(mac)
        if not node:
            node = {"id": 0, "last_seen": now, "status": None}
            self.nodes[mac] = node
        node["last_seen"] = now
        if "id" in msg and msg.get("id") is not None:
            try:
                node["id"] = int(msg.get("id") or 0)
            except Exception:
                pass
        if msg.get("type") == "status":
            node["status"] = msg
        self._upsert_node_row(mac)

    def _upsert_node_row(self, mac: str):
        node = self.nodes.get(mac)
        if not node:
            return
        node_id = node.get("id", 0) or 0
        last_seen = node.get("last_seen", 0.0) or 0.0
        last_txt = time.strftime("%H:%M:%S", time.localtime(last_seen)) if last_seen else "â€”"

        item = self._tree_item_by_mac.get(mac)
        values = (str(node_id), mac, last_txt)
        if item and self.tree_nodes.exists(item):
            self.tree_nodes.item(item, values=values)
        else:
            item = self.tree_nodes.insert("", tk.END, values=values)
            self._tree_item_by_mac[mac] = item

    def _on_node_selected(self, _evt=None):
        sel = self.tree_nodes.selection()
        if not sel:
            return
        item = sel[0]
        vals = self.tree_nodes.item(item, "values")
        if not vals or len(vals) < 2:
            return
        mac = vals[1]
        self.active_mac = mac
        st = self.nodes.get(mac, {}).get("status")
        if st:
            self.status = st
            self._update_status_ui(st)

    def _targets_all(self):
        for v in self.target_id_vars:
            v.set(True)

    def _targets_none(self):
        for v in self.target_id_vars:
            v.set(False)

    def _targets_active_only(self):
        self._targets_none()
        if not self.active_mac:
            return
        node_id = int(self.nodes.get(self.active_mac, {}).get("id", 0) or 0)
        if 1 <= node_id <= 32:
            self.target_id_vars[node_id - 1].set(True)

    def _set_id_for_selected(self):
        if not self.active_mac:
            messagebox.showerror("Error", "Select a node first (in the Nodes list).")
            return
        try:
            node_id = int(self.ent_new_id.get().strip())
        except Exception:
            messagebox.showerror("Error", "ID must be a number (1-32).")
            return
        if node_id < 1 or node_id > 32:
            messagebox.showerror("Error", "ID must be 1-32.")
            return
        self._cmd(f"SET_ID {self.active_mac} {node_id}")

    def _apply_status_config_to_ui(self, s: dict):
        # Pull a subset of config from the node status. Only called on explicit user action.
        try:
            self.var_keep.set(bool(int(s.get("keep", 0))))
            self.var_cl.set(bool(int(s.get("cl", 0))))
        except Exception:
            pass

        try:
            if "usteps" in s:
                self.cb_usteps.set(str(s.get("usteps")))
            if "irun" in s:
                self.ent_irun.delete(0, tk.END); self.ent_irun.insert(0, str(s.get("irun")))
            if "ihold" in s:
                self.ent_ihold.delete(0, tk.END); self.ent_ihold.insert(0, str(s.get("ihold")))
            if "ihd" in s:
                self.ent_ihd.delete(0, tk.END); self.ent_ihd.insert(0, str(s.get("ihd")))
            if "speed" in s:
                self.ent_speed.delete(0, tk.END); self.ent_speed.insert(0, str(s.get("speed")))
            if "accel" in s:
                self.ent_accel.delete(0, tk.END); self.ent_accel.insert(0, str(s.get("accel")))
        except Exception:
            pass

    def _update_status_ui(self, s):
        # Display fields
        def setv(k, val):
            if k in self.status_vars:
                self.status_vars[k].set(str(val))

        setv("from", s.get("from", "—"))
        setv("uptime_ms", s.get("uptime_ms", "—"))
        setv("motor_pos", s.get("motor_pos", "—"))
        setv("enc_pos", s.get("enc_pos", "—"))
        setv("err", s.get("err", "—"))
        setv("thr", s.get("thr", "—"))
        setv("missed", s.get("missed", "—"))

        temp = s.get("temp_c", None)
        setv("temp_c", "—" if temp is None else temp)

        setv("moving", s.get("moving", "—"))
        setv("en", s.get("en", "—"))
        setv("cl", s.get("cl", "—"))
        setv("keep", s.get("keep", "—"))

        setv("speed", s.get("speed", "—"))
        setv("accel", s.get("accel", "—"))
        setv("usteps", s.get("usteps", "—"))
        setv("curr", f"{s.get('irun','—')}/{s.get('ihold','—')}/{s.get('ihd','—')}")
        setv("drv", s.get("drv", "—"))
        setv("ioin", s.get("ioin", "—"))
        setv("ifcnt", s.get("ifcnt", "—"))

        # Also update toggles from device state (but don’t fight user too hard)
        try:
            self.var_keep.set(bool(int(s.get("keep", 0))))
            self.var_cl.set(bool(int(s.get("cl", 0))))
        except Exception:
            pass

    # ---------------- logging ----------------
    def _log(self, txt: str):
        self.txt_log.insert(tk.END, txt)
        self.txt_log.see(tk.END)
        # keep log bounded
        if int(self.txt_log.index("end-1c").split(".")[0]) > 400:
            self.txt_log.delete("1.0", "50.0")

    def on_close(self):
        # Safety: try to disable motor on exit
        try:
            if self.sw.ser:
                self._cmd("STOP")
                self._cmd("ENABLE 0")
                time.sleep(0.1)
        except Exception:
            pass
        self.sw.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
