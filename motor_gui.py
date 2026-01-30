import json
import threading
import queue
import binascii
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports


# ---------------- Config registry (from your ConfigDefs.h) ----------------
# type uses your Proto::ValueType ints: VT_U8=1, VT_U16=2, VT_U32=3, VT_I16=4, VT_I32=5, VT_BOOL=6
# NOTE: TMC IRUN/IHOLD IDs corrected to your motor node:
P_TMC_IRUN  = 0x0013
P_TMC_IHOLD = 0x0014
P_ENCODER_INVERT = 0x0500

PARAMS = {
    0x0001: ("node_id", 1, ""),
    0x0010: ("microsteps", 2, ""),
    0x0011: ("max_speed_rev_s_q100", 3, "rev/s*100"),
    0x0012: ("accel_rev_s2_q100", 3, "rev/s^2*100"),

    # ---- NEW (your IDs) ----
    P_TMC_IRUN:  ("tmc_irun",  1, "0..31"),
    P_TMC_IHOLD: ("tmc_ihold", 1, "0..31"),

    0x0100: ("home_angle_deg_q100", 5, "deg*100"),
    0x0101: ("home_window_deg_q100", 2, "deg*100"),
    0x0102: ("home_settle_ms", 2, "ms"),
    0x0103: ("home_timeout_ms", 3, "ms"),
    0x0104: ("home_speed_rev_s_q100", 3, "rev/s*100"),
    0x0105: ("home_accel_rev_s2_q100", 3, "rev/s^2*100"),

    0x0200: ("loss_thr_hold_fs", 2, "fullsteps"),
    0x0201: ("loss_thr_run_fs", 2, "fullsteps"),
    0x0202: ("loss_thr_fast_fs", 2, "fullsteps"),
    0x0203: ("loss_speed_hold_sps", 3, "steps/s"),
    0x0204: ("loss_speed_fast_sps", 3, "steps/s"),
    0x0205: ("loss_speed_hyst_sps", 3, "steps/s"),
    0x0206: ("loss_confirm_hold_ms", 2, "ms"),
    0x0207: ("loss_confirm_run_ms", 2, "ms"),
    0x0208: ("loss_confirm_fast_ms", 2, "ms"),
    0x0209: ("recover_time_ms", 2, "ms"),
    0x020A: ("disable_time_ms", 2, "ms"),

    0x0300: ("ntc_r25_ohm", 3, "ohm"),
    0x0301: ("r_fixed_ohm", 3, "ohm"),
    0x0302: ("ntc_beta", 3, ""),
    0x0303: ("therm_interval_ms", 2, "ms"),

    0x0400: ("telem_interval_ms", 2, "ms"),
    P_ENCODER_INVERT: ("encoder_invert", 6, "bool"),
}

TYPE_NAMES = {1: "U8", 2: "U16", 3: "U32", 4: "I16", 5: "I32", 6: "BOOL"}


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# ---------------- TLV decode (from RSP_CONFIG) ----------------
def decode_tlvs(hex_str: str):
    b = binascii.unhexlify(hex_str)
    i = 0
    items = []
    while i + 4 <= len(b):
        pid = int.from_bytes(b[i:i+2], "little")
        vtype = b[i+2]
        vlen = b[i+3]
        i += 4
        if i + vlen > len(b):
            break
        raw = b[i:i+vlen]
        i += vlen

        if vtype == 1 and vlen == 1:
            val = raw[0]
        elif vtype == 2 and vlen == 2:
            val = int.from_bytes(raw, "little", signed=False)
        elif vtype == 3 and vlen == 4:
            val = int.from_bytes(raw, "little", signed=False)
        elif vtype == 4 and vlen == 2:
            val = int.from_bytes(raw, "little", signed=True)
        elif vtype == 5 and vlen == 4:
            val = int.from_bytes(raw, "little", signed=True)
        elif vtype == 6 and vlen == 1:
            val = 1 if raw[0] else 0
        else:
            val = raw.hex()

        name, _, unit = PARAMS.get(pid, (f"0x{pid:04X}", vtype, ""))
        items.append((pid, name, vtype, vlen, val, unit))
    return items


# ---------------- Serial bridge client ----------------
class BridgeClient:
    def __init__(self):
        self.ser = None
        self.rx_thread = None
        self.running = False
        self.q = queue.Queue()

    def connect(self, port: str, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def disconnect(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=0.5)
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None

    def send_line(self, line: str):
        if not self.ser:
            return
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode("utf-8"))

    def _rx_loop(self):
        buf = b""
        while self.running and self.ser:
            try:
                chunk = self.ser.read(4096)
            except:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line.decode("utf-8", errors="replace"))
                    self.q.put(msg)
                except:
                    pass


# ---------------- GUI ----------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MotorNode ESP-NOW Controller")
        self.geometry("1100x700")

        self.bridge = BridgeClient()
        self.telemetry = {}

        self._build_ui()
        self._poll_messages()

    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)

        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18, values=list_serial_ports())
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")

        self.conn_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=8)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=10)

        ttk.Label(top, text="Channel:").pack(side="left")
        self.chan_var = tk.StringVar(value="6")
        ttk.Entry(top, textvariable=self.chan_var, width=5).pack(side="left", padx=4)
        ttk.Button(top, text="Set", command=self._set_channel).pack(side="left", padx=4)

        ttk.Label(top, text="Motor MAC:").pack(side="left", padx=(12, 0))
        self.mac_var = tk.StringVar(value="AA:BB:CC:DD:EE:FF")
        ttk.Entry(top, textvariable=self.mac_var, width=18).pack(side="left", padx=4)
        ttk.Button(top, text="Set", command=self._set_mac).pack(side="left", padx=4)

        main = ttk.Frame(self)
        main.pack(fill="both", expand=True, padx=8, pady=6)

        left = ttk.Frame(main)
        left.pack(side="left", fill="y")

        right = ttk.Frame(main)
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))

        # ----- Command controls -----
        cmdf = ttk.LabelFrame(left, text="Commands")
        cmdf.pack(fill="x", pady=6)

        ttk.Button(cmdf, text="Ping", command=lambda: self._send("PING")).pack(fill="x", padx=6, pady=3)

        ef = ttk.Frame(cmdf)
        ef.pack(fill="x", padx=6, pady=3)
        ttk.Button(ef, text="Enable", command=lambda: self._send("ENABLE 1")).pack(side="left", expand=True, fill="x")
        ttk.Button(ef, text="Disable", command=lambda: self._send("ENABLE 0")).pack(side="left", expand=True, fill="x", padx=4)

        ttk.Button(cmdf, text="STOP", command=self._stop_motor).pack(fill="x", padx=6, pady=3)
        ttk.Button(cmdf, text="HOME", command=lambda: self._send("HOME")).pack(fill="x", padx=6, pady=3)
        ttk.Button(cmdf, text="Zero Position", command=lambda: self._send("ZERO")).pack(fill="x", padx=6, pady=3)

        rawf = ttk.Frame(cmdf)
        rawf.pack(fill="x", padx=6, pady=6)
        self.raw_cmd_var = tk.StringVar()
        ttk.Entry(rawf, textvariable=self.raw_cmd_var, width=24).pack(side="left", expand=True, fill="x")
        ttk.Button(rawf, text="Send", command=self._send_raw_cmd).pack(side="left", padx=4)
        ttk.Button(rawf, text="Help", command=lambda: self._send("HELP")).pack(side="left")

        mf = ttk.LabelFrame(left, text="Move / Velocity")
        mf.pack(fill="x", pady=6)

        self.move_deg_var = tk.StringVar(value="0.0")
        row = ttk.Frame(mf); row.pack(fill="x", padx=6, pady=3)
        ttk.Label(row, text="Move abs deg:").pack(side="left")
        ttk.Entry(row, textvariable=self.move_deg_var, width=10).pack(side="left", padx=6)
        ttk.Button(row, text="Send", command=self._send_move).pack(side="left")

        self.vel_dps_var = tk.StringVar(value="0.0")
        row = ttk.Frame(mf); row.pack(fill="x", padx=6, pady=3)
        ttk.Label(row, text="Velocity dps:").pack(side="left")
        ttk.Entry(row, textvariable=self.vel_dps_var, width=10).pack(side="left", padx=6)
        ttk.Button(row, text="Send", command=self._send_vel).pack(side="left")

        encf = ttk.LabelFrame(left, text="Encoder")
        encf.pack(fill="x", pady=6)

        self.encoder_invert_var = tk.IntVar(value=0)
        ttk.Checkbutton(encf, text="Invert encoder reading", variable=self.encoder_invert_var).pack(anchor="w", padx=6, pady=3)
        ttk.Button(encf, text="Apply", command=self._set_encoder_invert).pack(fill="x", padx=6, pady=(0, 6))

        # ----- TMC Current (IRUN/IHOLD) -----
        tmc = ttk.LabelFrame(left, text="TMC2209 Current (IRUN/IHOLD)")
        tmc.pack(fill="x", pady=6)

        row = ttk.Frame(tmc); row.pack(fill="x", padx=6, pady=3)
        ttk.Label(row, text="IRUN (0..31):").pack(side="left")
        self.irun_var = tk.StringVar(value="20")
        ttk.Entry(row, textvariable=self.irun_var, width=6).pack(side="left", padx=6)
        ttk.Button(row, text="Set", command=self._set_irun).pack(side="left")

        row = ttk.Frame(tmc); row.pack(fill="x", padx=6, pady=3)
        ttk.Label(row, text="IHOLD (0..31):").pack(side="left")
        self.ihold_var = tk.StringVar(value="8")
        ttk.Entry(row, textvariable=self.ihold_var, width=6).pack(side="left", padx=6)
        ttk.Button(row, text="Set", command=self._set_ihold).pack(side="left")

        row = ttk.Frame(tmc); row.pack(fill="x", padx=6, pady=6)
        ttk.Button(row, text="Apply now", command=self._apply_tmc_now).pack(side="left", expand=True, fill="x")
        ttk.Button(row, text="Apply + Save", command=self._apply_tmc_save).pack(side="left", expand=True, fill="x", padx=6)

        hint = ttk.Label(tmc, text=f"Param IDs: IRUN=0x{P_TMC_IRUN:04X}, IHOLD=0x{P_TMC_IHOLD:04X}")
        hint.pack(fill="x", padx=6, pady=(0, 6))

        # ----- Config controls -----
        cf = ttk.LabelFrame(left, text="Config")
        cf.pack(fill="x", pady=6)

        ttk.Button(cf, text="Get ALL", command=lambda: self._send("GETCFG ALL")).pack(fill="x", padx=6, pady=3)

        ids_row = ttk.Frame(cf); ids_row.pack(fill="x", padx=6, pady=3)
        self.get_ids_var = tk.StringVar(value="0x0010,0x0011,0x0012,0x0013,0x0014,0x0400,0x0500")
        ttk.Entry(ids_row, textvariable=self.get_ids_var, width=24).pack(side="left", expand=True, fill="x")
        ttk.Button(ids_row, text="Get IDs", command=self._send_get_ids).pack(side="left", padx=4)

        flagsf = ttk.LabelFrame(cf, text="SET flags")
        flagsf.pack(fill="x", padx=6, pady=6)
        self.flag_apply_now = tk.IntVar(value=1)
        self.flag_when_idle = tk.IntVar(value=0)
        self.flag_save_after = tk.IntVar(value=0)
        self.flag_strict = tk.IntVar(value=0)
        ttk.Checkbutton(flagsf, text="Apply now", variable=self.flag_apply_now).pack(anchor="w")
        ttk.Checkbutton(flagsf, text="Apply when idle", variable=self.flag_when_idle).pack(anchor="w")
        ttk.Checkbutton(flagsf, text="Save after apply", variable=self.flag_save_after).pack(anchor="w")
        ttk.Checkbutton(flagsf, text="Strict", variable=self.flag_strict).pack(anchor="w")

        setf = ttk.LabelFrame(left, text="Set one param (TLV)")
        setf.pack(fill="x", pady=6)

        self.param_choices = [f"0x{pid:04X} {PARAMS[pid][0]}" for pid in sorted(PARAMS.keys())]
        self.param_sel = tk.StringVar(value=self.param_choices[0])
        ttk.Combobox(setf, textvariable=self.param_sel, values=self.param_choices, width=28).pack(fill="x", padx=6, pady=3)

        self.param_value = tk.StringVar(value="128")
        row = ttk.Frame(setf); row.pack(fill="x", padx=6, pady=3)
        ttk.Label(row, text="Value:").pack(side="left")
        ttk.Entry(row, textvariable=self.param_value, width=12).pack(side="left", padx=6)
        ttk.Button(row, text="Send SETCFG", command=self._send_setcfg).pack(side="left")

        ttk.Button(left, text="SAVE CONFIG (NVS)", command=lambda: self._send("SAVE")).pack(fill="x", pady=6)

        # ----- Right side: Telemetry + logs + config table -----
        telem = ttk.LabelFrame(right, text="Telemetry (from RSP_STATUS)")
        telem.pack(fill="x")

        self.telem_text = tk.Text(telem, height=10, wrap="none")
        self.telem_text.pack(fill="x", padx=6, pady=6)
        self.telem_text.configure(state="disabled")

        mid = ttk.Frame(right)
        mid.pack(fill="both", expand=True, pady=8)

        cfg_frame = ttk.LabelFrame(mid, text="Config values (decoded from RSP_CONFIG)")
        cfg_frame.pack(side="left", fill="both", expand=True)

        cols = ("pid", "name", "type", "len", "value", "unit")
        self.cfg_tree = ttk.Treeview(cfg_frame, columns=cols, show="headings", height=14)
        for c in cols:
            self.cfg_tree.heading(c, text=c)
            self.cfg_tree.column(c, width=120 if c != "name" else 220, anchor="w")
        self.cfg_tree.pack(fill="both", expand=True, padx=6, pady=6)

        logf = ttk.LabelFrame(mid, text="Log / ACKs")
        logf.pack(side="left", fill="both", expand=True, padx=(10, 0))

        self.log_text = tk.Text(logf, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=6)
        self._log("Ready. Connect to controller ESP32 and set MAC/channel.")

    # ---------- bridge send helpers ----------
    def _send(self, line: str):
        if not self.bridge.ser:
            messagebox.showerror("Not connected", "Connect to the controller ESP32 first.")
            return
        self.bridge.send_line(line)
        self._log(f"> {line}")

    def _refresh_ports(self):
        self.port_combo["values"] = list_serial_ports()

    def _toggle_connect(self):
        if self.bridge.ser:
            self.bridge.disconnect()
            self.conn_btn.config(text="Connect")
            self._log("Disconnected.")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Port", "Select a serial port.")
            return
        try:
            self.bridge.connect(port)
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))
            return
        self.conn_btn.config(text="Disconnect")
        self._log(f"Connected to {port}")
        self._set_channel()
        self._set_mac()

    def _set_channel(self):
        ch = self.chan_var.get().strip()
        if ch:
            self._send(f"CHAN {ch}")

    def _set_mac(self):
        mac = self.mac_var.get().strip()
        if mac:
            self._send(f"MAC {mac}")

    def _send_move(self):
        self._send(f"MOVE_DEG {self.move_deg_var.get().strip()}")

    def _send_vel(self):
        self._send(f"VEL_DPS {self.vel_dps_var.get().strip()}")

    def _stop_motor(self):
        self._send("STOP")
        self._send("VEL_DPS 0")

    def _send_raw_cmd(self):
        cmd = self.raw_cmd_var.get().strip()
        if not cmd:
            messagebox.showerror("Command", "Enter a command to send.")
            return
        self._send(cmd)

    def _set_encoder_invert(self):
        flags = self._flags_int()
        val = "1" if self.encoder_invert_var.get() else "0"
        self._send_setcfg_pid(P_ENCODER_INVERT, 6, val, flags)

    def _send_get_ids(self):
        ids = self.get_ids_var.get().strip()
        self._send(f"GETCFG IDS {ids}")

    def _flags_int(self, apply_now=None, save_after=None):
        f = 0
        an = self.flag_apply_now.get() if apply_now is None else (1 if apply_now else 0)
        sa = self.flag_save_after.get() if save_after is None else (1 if save_after else 0)

        if an: f |= 1
        if self.flag_when_idle.get(): f |= 2
        if sa: f |= 4
        if self.flag_strict.get(): f |= 8
        return f

    def _send_setcfg_pid(self, pid: int, vtype: int, val_str: str, flags: int):
        self._send(f"SETCFG {flags} 0x{pid:04X} {vtype} {val_str}")

    def _parse_0_31(self, s: str, name: str):
        try:
            v = int(str(s).strip(), 0)
        except:
            messagebox.showerror("Value", f"{name} must be an integer 0..31")
            return None
        if v < 0 or v > 31:
            messagebox.showerror("Value", f"{name} must be 0..31")
            return None
        return v

    def _set_irun(self):
        v = self._parse_0_31(self.irun_var.get(), "IRUN")
        if v is None:
            return
        flags = self._flags_int()
        self._send_setcfg_pid(P_TMC_IRUN, 1, str(v), flags)   # VT_U8

    def _set_ihold(self):
        v = self._parse_0_31(self.ihold_var.get(), "IHOLD")
        if v is None:
            return
        flags = self._flags_int()
        self._send_setcfg_pid(P_TMC_IHOLD, 1, str(v), flags)  # VT_U8

    def _apply_tmc_now(self):
        ir = self._parse_0_31(self.irun_var.get(), "IRUN")
        ih = self._parse_0_31(self.ihold_var.get(), "IHOLD")
        if ir is None or ih is None:
            return
        flags = self._flags_int(apply_now=True, save_after=False)
        self._send_setcfg_pid(P_TMC_IRUN, 1, str(ir), flags)
        self._send_setcfg_pid(P_TMC_IHOLD, 1, str(ih), flags)

    def _apply_tmc_save(self):
        ir = self._parse_0_31(self.irun_var.get(), "IRUN")
        ih = self._parse_0_31(self.ihold_var.get(), "IHOLD")
        if ir is None or ih is None:
            return
        flags = self._flags_int(apply_now=True, save_after=True)
        self._send_setcfg_pid(P_TMC_IRUN, 1, str(ir), flags)
        self._send_setcfg_pid(P_TMC_IHOLD, 1, str(ih), flags)
        self._send("SAVE")

    def _send_setcfg(self):
        sel = self.param_sel.get()
        try:
            pid = int(sel.split()[0], 16)
        except:
            messagebox.showerror("Param", "Bad param selection.")
            return

        _name, vtype, _unit = PARAMS.get(pid, (None, None, None))
        if vtype is None:
            messagebox.showerror("Param", "Unknown param type.")
            return

        val = self.param_value.get().strip()
        flags = self._flags_int()
        self._send(f"SETCFG {flags} 0x{pid:04X} {vtype} {val}")

    # ---------- message handling ----------
    def _poll_messages(self):
        while True:
            try:
                msg = self.bridge.q.get_nowait()
            except queue.Empty:
                break
            self._handle_msg(msg)
        self.after(50, self._poll_messages)

    def _handle_msg(self, msg: dict):
        mtype = msg.get("type")
        if mtype == "bridge":
            self._log(f"[bridge] {msg}")
            return

        if mtype == "ack":
            self._log(f"[ACK] from={msg.get('from')} ack_seq={msg.get('ack_seq')} result={msg.get('result')}")
            return

        if mtype == "status":
            self.telemetry = msg
            self._update_telem()
            return

        if mtype == "config":
            hex_str = msg.get("tlv_hex", "")
            items = decode_tlvs(hex_str)
            self._update_config_table(items)
            self._log(f"[CONFIG] received {len(items)} TLVs")
            return

        self._log(f"[RX] {msg}")

    def _update_telem(self):
        t = self.telemetry
        temp_c = t.get("temp_c_q10", 0) / 10.0 if "temp_c_q10" in t else None

        lines = [
            f"from: {t.get('from')}",
            f"ms: {t.get('ms')}  state: {t.get('state')}  faultCode: {t.get('faultCode')}",
            f"enabled: {t.get('motorEnabled')} moving: {t.get('moving')} lossActive: {t.get('lossActive')}",
            f"pos_steps: {t.get('pos_steps')}  target_steps: {t.get('target_steps')}  home_steps: {t.get('home_steps')}",
            f"enc_deg_q100: {t.get('enc_deg_q100')}  enc_abs_deg_q100: {t.get('enc_abs_deg_q100')}",
            f"error_steps: {t.get('error_steps')}  temp_C: {temp_c}  config_rev: {t.get('config_revision')}",
        ]

        self.telem_text.configure(state="normal")
        self.telem_text.delete("1.0", "end")
        self.telem_text.insert("end", "\n".join(lines))
        self.telem_text.configure(state="disabled")

    def _update_config_table(self, items):
        for x in self.cfg_tree.get_children():
            self.cfg_tree.delete(x)

        # auto-fill IRUN/IHOLD fields when seen
        for (pid, name, vtype, vlen, val, unit) in items:
            self.cfg_tree.insert("", "end", values=(f"0x{pid:04X}", name, TYPE_NAMES.get(vtype, str(vtype)), vlen, val, unit))
            if pid == P_TMC_IRUN and isinstance(val, int):
                self.irun_var.set(str(val))
            if pid == P_TMC_IHOLD and isinstance(val, int):
                self.ihold_var.set(str(val))
            if pid == P_ENCODER_INVERT and isinstance(val, int):
                self.encoder_invert_var.set(1 if val else 0)

    def _log(self, s: str):
        self.log_text.insert("end", s + "\n")
        self.log_text.see("end")


if __name__ == "__main__":
    app = App()
    app.mainloop()
