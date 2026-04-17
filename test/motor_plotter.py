#!/usr/bin/env python3
"""
Motor Serial Plotter - Speed vs Encoder & Speed vs Input Characterization
For Pico 2 W Motor Controller

Flow:
  1. PPR Calibration  - manually rotate each wheel 1 full turn, estimate PPR
  2. Speed Testing    - sweep input values, measure encoder velocity (forward & back)
  3. Curve Fitting    - fit polynomial/linear model to speed vs input data
  4. Live Plotting    - real-time encoder plots during any manual test
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy import stats
from scipy.optimize import curve_fit
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────
NUM_MOTORS = 4
MOTOR_NAMES = ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]
COLORS = ["#00d4ff", "#ff6b35", "#7fff6b", "#ff3bff"]
BG      = "#0d0f14"
BG2     = "#13161e"
CARD    = "#1a1e2a"
BORDER  = "#2a2e3e"
TEXT    = "#e8ecf4"
MUTED   = "#6b7280"
ACCENT  = "#00d4ff"

SWEEP_STEPS        = 20   # number of input values to test per direction
SWEEP_DWELL_S      = 1.5  # seconds at each speed before measuring
MEASURE_WINDOW_S   = 0.8  # seconds of encoder data to average
SERIAL_TIMEOUT     = 2.0


# ─────────────────────────────────────────────
#  Serial Communication
# ─────────────────────────────────────────────
class MotorSerial:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._response_queue: list[str] = []
        self._response_event = threading.Event()

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baud: int = 115200) -> bool:
        try:
            self.ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self._running = True
            self._read_thread = threading.Thread(target=self._reader, daemon=True)
            self._read_thread.start()
            return True
        except Exception as e:
            print(f"Serial connect error: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.send("S")
            time.sleep(0.1)
            self.ser.close()

    def _reader(self):
        buf = ""
        while self._running:
            try:
                if self.ser and self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="ignore")
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            with self._lock:
                                self._response_queue.append(line)
                            self._response_event.set()
                else:
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.01)

    def send(self, cmd: str):
        if self.ser and self.ser.is_open:
            with self._lock:
                self.ser.write((cmd + "\n").encode())

    def get_encoders(self) -> Optional[list[int]]:
        with self._lock:
            self._response_queue.clear()
        self._response_event.clear()
        self.send("EA")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            self._response_event.wait(0.1)
            self._response_event.clear()
            with self._lock:
                for line in self._response_queue:
                    if line.startswith("ENCODERS:"):
                        vals = line.replace("ENCODERS:", "").split()
                        try:
                            return [int(v) for v in vals]
                        except ValueError:
                            pass
        return None

    def reset_encoders(self):
        self.send("RA")
        time.sleep(0.05)

    def set_motor(self, motor_id: int, speed: int):
        """speed: -200..200 mapped to -255..255"""
        pwm = int(np.clip(speed * 255 / 200, -255, 255))
        self.send(f"M{motor_id} {pwm}")

    def stop_all(self):
        self.send("S")


# ─────────────────────────────────────────────
#  Data Structures
# ─────────────────────────────────────────────
@dataclass
class MotorCalibration:
    ppr: float = 0.0
    calibrated: bool = False

@dataclass
class SpeedPoint:
    input_val: int    # -200..200
    direction: str    # 'fwd' or 'rev'
    rpm: float
    rps: float        # rotations per second

@dataclass
class MotorProfile:
    motor_id: int
    calibration: MotorCalibration = field(default_factory=MotorCalibration)
    speed_points: list[SpeedPoint] = field(default_factory=list)
    fit_fwd: Optional[np.poly1d] = None  # poly fit fwd: input → rps
    fit_rev: Optional[np.poly1d] = None
    fit_fwd_r2: float = 0.0
    fit_rev_r2: float = 0.0


# ─────────────────────────────────────────────
#  Main Application
# ─────────────────────────────────────────────
class MotorPlotterApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Motor Characterization Suite")
        self.configure(bg=BG)
        self.geometry("1400x900")
        self.minsize(1100, 700)

        self.mserial = MotorSerial()
        self.profiles = [MotorProfile(i) for i in range(NUM_MOTORS)]
        self.connected = False

        # Live encoder tracking
        self._enc_history = [deque(maxlen=300) for _ in range(NUM_MOTORS)]
        self._enc_times   = [deque(maxlen=300) for _ in range(NUM_MOTORS)]
        self._live_running = False
        self._last_enc = [0]*NUM_MOTORS

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI Build ──────────────────────────────
    def _build_ui(self):
        self._apply_theme()

        # Top bar
        topbar = tk.Frame(self, bg=BG, height=56)
        topbar.pack(fill=tk.X, padx=0, pady=0)
        topbar.pack_propagate(False)

        tk.Label(topbar, text="⚙  MOTOR CHARACTERIZATION SUITE",
                 font=("Courier New", 13, "bold"), fg=ACCENT, bg=BG
                 ).pack(side=tk.LEFT, padx=24, pady=16)

        # Connection bar
        conn_frame = tk.Frame(topbar, bg=BG)
        conn_frame.pack(side=tk.RIGHT, padx=16)

        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(conn_frame, textvariable=self.port_var, width=14,
                                    style="Dark.TCombobox")
        self.port_cb.pack(side=tk.LEFT, padx=4)
        self._refresh_ports()

        ttk.Button(conn_frame, text="↺", style="Icon.TButton",
                   command=self._refresh_ports).pack(side=tk.LEFT, padx=2)
        self.conn_btn = ttk.Button(conn_frame, text="Connect", style="Accent.TButton",
                                   command=self._toggle_connect)
        self.conn_btn.pack(side=tk.LEFT, padx=4)

        self.status_dot = tk.Label(conn_frame, text="●", font=("Arial", 14),
                                   fg="#444", bg=BG)
        self.status_dot.pack(side=tk.LEFT, padx=2)

        sep = tk.Frame(self, bg=BORDER, height=1)
        sep.pack(fill=tk.X)

        # Notebook
        self.nb = ttk.Notebook(self, style="Dark.TNotebook")
        self.nb.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)

        self._build_calibration_tab()
        self._build_sweep_tab()
        self._build_results_tab()
        self._build_live_tab()

    def _apply_theme(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure(".", background=BG, foreground=TEXT, fieldbackground=CARD,
                        troughcolor=BORDER, selectbackground=ACCENT,
                        selectforeground=BG, bordercolor=BORDER)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("TLabelframe", background=CARD, foreground=TEXT,
                        bordercolor=BORDER)
        style.configure("TLabelframe.Label", background=CARD, foreground=ACCENT,
                        font=("Courier New", 9, "bold"))

        style.configure("Dark.TNotebook", background=BG, bordercolor=BORDER,
                        tabmargins=[0,0,0,0])
        style.configure("Dark.TNotebook.Tab", background=BG2, foreground=MUTED,
                        padding=[20,8], font=("Courier New", 9, "bold"),
                        bordercolor=BORDER)
        style.map("Dark.TNotebook.Tab",
                  background=[("selected", CARD)],
                  foreground=[("selected", ACCENT)])

        style.configure("Accent.TButton", background=ACCENT, foreground=BG,
                        font=("Courier New", 9, "bold"), padding=[10,4],
                        bordercolor=ACCENT)
        style.map("Accent.TButton",
                  background=[("active", "#00aacf"), ("disabled", BORDER)])
        style.configure("Danger.TButton", background="#ff4455", foreground=BG,
                        font=("Courier New", 9, "bold"), padding=[10,4])
        style.map("Danger.TButton", background=[("active","#cc3344")])
        style.configure("Icon.TButton", background=BG2, foreground=TEXT,
                        font=("Courier New", 10, "bold"), padding=[4,2])
        style.configure("TButton", background=BG2, foreground=TEXT,
                        font=("Courier New", 9), padding=[8,4],
                        bordercolor=BORDER)
        style.map("TButton", background=[("active", BORDER)])

        style.configure("Dark.TCombobox", fieldbackground=CARD, background=CARD,
                        foreground=TEXT, arrowcolor=ACCENT)
        style.configure("TProgressbar", troughcolor=BORDER, background=ACCENT,
                        bordercolor=BORDER)
        style.configure("TScale", background=BG, troughcolor=BORDER,
                        sliderlength=16, sliderrelief="flat")

    # ── Calibration Tab ───────────────────────
    def _build_calibration_tab(self):
        frame = ttk.Frame(self.nb)
        self.nb.add(frame, text="  1 · PPR CALIBRATION  ")

        intro = tk.Frame(frame, bg=CARD, bd=0)
        intro.pack(fill=tk.X, padx=16, pady=(16,8))
        tk.Label(intro,
                 text="Manually rotate each wheel exactly ONE full rotation.\n"
                      "The encoder count change is the Pulses Per Revolution (PPR).\n"
                      "Click START, rotate wheel, click DONE.",
                 font=("Courier New", 10), fg=MUTED, bg=CARD,
                 justify=tk.LEFT).pack(padx=16, pady=10, anchor="w")

        grid = tk.Frame(frame, bg=BG)
        grid.pack(fill=tk.BOTH, expand=True, padx=16, pady=8)

        self._cal_frames = []
        for i in range(NUM_MOTORS):
            col = i % 2
            row = i // 2
            self._build_cal_card(grid, i, row, col)

        # Bottom actions
        bot = tk.Frame(frame, bg=BG)
        bot.pack(fill=tk.X, padx=16, pady=12)
        ttk.Button(bot, text="Reset All Encoders", command=self._reset_all_enc
                   ).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="Proceed to Sweep  →", style="Accent.TButton",
                   command=lambda: self.nb.select(1)).pack(side=tk.RIGHT, padx=4)

    def _build_cal_card(self, parent, motor_id, row, col):
        card = tk.LabelFrame(parent, text=f"  {MOTOR_NAMES[motor_id]}  ",
                             bg=CARD, fg=COLORS[motor_id],
                             font=("Courier New", 10, "bold"),
                             bd=1, relief=tk.FLAT, highlightthickness=1,
                             highlightbackground=BORDER)
        card.grid(row=row, column=col, padx=8, pady=8, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)
        parent.grid_rowconfigure(row, weight=1)

        # Encoder display
        enc_lbl = tk.Label(card, text="Encoder: 0", font=("Courier New", 20, "bold"),
                           fg=COLORS[motor_id], bg=CARD)
        enc_lbl.pack(pady=(16,4))

        ppr_lbl = tk.Label(card, text="PPR: —", font=("Courier New", 13),
                           fg=TEXT, bg=CARD)
        ppr_lbl.pack(pady=4)

        status_lbl = tk.Label(card, text="Not calibrated", font=("Courier New", 9),
                              fg=MUTED, bg=CARD)
        status_lbl.pack(pady=2)

        # Buttons
        btn_frame = tk.Frame(card, bg=CARD)
        btn_frame.pack(pady=12)

        state = {"active": False, "start_count": 0}

        def start_cal():
            if not self.connected:
                messagebox.showwarning("Not Connected", "Connect to serial port first.")
                return
            self.mserial.reset_encoders()
            time.sleep(0.1)
            state["active"] = True
            state["start_count"] = 0
            status_lbl.config(text="● RECORDING — rotate wheel one full turn",
                              fg="#ffcc00")
            start_btn.config(state=tk.DISABLED)
            done_btn.config(state=tk.NORMAL)
            self._start_cal_poll(motor_id, enc_lbl)

        def done_cal():
            state["active"] = False
            encs = self.mserial.get_encoders()
            if encs:
                ppr = abs(encs[motor_id])
                if ppr > 0:
                    self.profiles[motor_id].calibration.ppr = ppr
                    self.profiles[motor_id].calibration.calibrated = True
                    ppr_lbl.config(text=f"PPR: {ppr}", fg=COLORS[motor_id])
                    status_lbl.config(text="✓ Calibrated", fg="#7fff6b")
                else:
                    status_lbl.config(text="⚠ No movement detected", fg="#ff4444")
            start_btn.config(state=tk.NORMAL)
            done_btn.config(state=tk.DISABLED)

        start_btn = ttk.Button(btn_frame, text="▶  START", style="Accent.TButton",
                               command=start_cal)
        start_btn.pack(side=tk.LEFT, padx=6)
        done_btn = ttk.Button(btn_frame, text="■  DONE", style="Danger.TButton",
                              command=done_cal, state=tk.DISABLED)
        done_btn.pack(side=tk.LEFT, padx=6)

        # Manual PPR override
        ovr_frame = tk.Frame(card, bg=CARD)
        ovr_frame.pack(pady=(0,12))
        tk.Label(ovr_frame, text="Manual PPR:", font=("Courier New", 9),
                 fg=MUTED, bg=CARD).pack(side=tk.LEFT, padx=4)
        ppr_entry = tk.Entry(ovr_frame, width=8, bg=BG2, fg=TEXT,
                             insertbackground=ACCENT, relief=tk.FLAT,
                             font=("Courier New", 10), highlightthickness=1,
                             highlightbackground=BORDER)
        ppr_entry.pack(side=tk.LEFT, padx=2)

        def set_manual():
            try:
                v = float(ppr_entry.get())
                self.profiles[motor_id].calibration.ppr = v
                self.profiles[motor_id].calibration.calibrated = True
                ppr_lbl.config(text=f"PPR: {v:.1f}", fg=COLORS[motor_id])
                status_lbl.config(text="✓ Manual PPR set", fg="#7fff6b")
            except ValueError:
                pass

        ttk.Button(ovr_frame, text="Set", command=set_manual
                   ).pack(side=tk.LEFT, padx=2)

        self._cal_frames.append({
            "enc_lbl": enc_lbl, "ppr_lbl": ppr_lbl,
            "status_lbl": status_lbl, "state": state
        })

    def _start_cal_poll(self, motor_id, enc_lbl):
        def poll():
            if not self._cal_frames[motor_id]["state"]["active"]:
                return
            encs = self.mserial.get_encoders()
            if encs:
                enc_lbl.config(text=f"Encoder: {encs[motor_id]}")
            self.after(150, poll)
        poll()

    def _reset_all_enc(self):
        if self.connected:
            self.mserial.reset_encoders()

    # ── Sweep Tab ─────────────────────────────
    def _build_sweep_tab(self):
        frame = ttk.Frame(self.nb)
        self.nb.add(frame, text="  2 · SPEED SWEEP  ")

        # Config panel
        cfg = tk.LabelFrame(frame, text="  Sweep Configuration  ",
                            bg=CARD, fg=ACCENT,
                            font=("Courier New", 9, "bold"),
                            bd=1, relief=tk.FLAT,
                            highlightthickness=1, highlightbackground=BORDER)
        cfg.pack(fill=tk.X, padx=16, pady=(16,8))

        row1 = tk.Frame(cfg, bg=CARD)
        row1.pack(fill=tk.X, padx=12, pady=8)

        def cfg_field(parent, label, default, width=6):
            tk.Label(parent, text=label, font=("Courier New", 9),
                     fg=MUTED, bg=CARD).pack(side=tk.LEFT, padx=(12,2))
            var = tk.StringVar(value=str(default))
            e = tk.Entry(parent, textvariable=var, width=width, bg=BG2,
                         fg=TEXT, insertbackground=ACCENT, relief=tk.FLAT,
                         font=("Courier New", 10), highlightthickness=1,
                         highlightbackground=BORDER)
            e.pack(side=tk.LEFT, padx=2)
            return var

        self.sweep_steps_var  = cfg_field(row1, "Steps:", SWEEP_STEPS)
        self.sweep_dwell_var  = cfg_field(row1, "Dwell (s):", SWEEP_DWELL_S)
        self.sweep_window_var = cfg_field(row1, "Measure window (s):", MEASURE_WINDOW_S)

        # Motor selection
        row2 = tk.Frame(cfg, bg=CARD)
        row2.pack(fill=tk.X, padx=12, pady=(0,8))
        tk.Label(row2, text="Motors:", font=("Courier New", 9),
                 fg=MUTED, bg=CARD).pack(side=tk.LEFT, padx=(12,4))
        self._sweep_motor_vars = []
        for i in range(NUM_MOTORS):
            v = tk.BooleanVar(value=True)
            self._sweep_motor_vars.append(v)
            cb = tk.Checkbutton(row2, text=MOTOR_NAMES[i], variable=v,
                                bg=CARD, fg=COLORS[i], selectcolor=BG2,
                                activebackground=CARD, activeforeground=COLORS[i],
                                font=("Courier New", 9, "bold"))
            cb.pack(side=tk.LEFT, padx=8)

        # Progress area
        prog_frame = tk.Frame(frame, bg=BG)
        prog_frame.pack(fill=tk.X, padx=16, pady=4)

        self.sweep_prog = ttk.Progressbar(prog_frame, mode="determinate",
                                          style="TProgressbar")
        self.sweep_prog.pack(fill=tk.X, pady=4)
        self.sweep_status = tk.Label(prog_frame, text="Ready", bg=BG,
                                     fg=MUTED, font=("Courier New", 9))
        self.sweep_status.pack(anchor="w")

        # Live sweep plot
        fig_frame = tk.Frame(frame, bg=BG)
        fig_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=8)

        self._sweep_fig = Figure(figsize=(10, 3.5), facecolor=BG)
        self._sweep_axes = []
        for i in range(NUM_MOTORS):
            ax = self._sweep_fig.add_subplot(1, 4, i+1)
            ax.set_facecolor(CARD)
            ax.set_title(MOTOR_NAMES[i], color=COLORS[i],
                         fontsize=8, fontfamily="monospace")
            ax.tick_params(colors=MUTED, labelsize=7)
            for sp in ax.spines.values():
                sp.set_color(BORDER)
            ax.set_xlabel("Input", color=MUTED, fontsize=7)
            ax.set_ylabel("RPS", color=MUTED, fontsize=7)
            self._sweep_axes.append(ax)
        self._sweep_fig.tight_layout(pad=1.5)

        canvas = FigureCanvasTkAgg(self._sweep_fig, fig_frame)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._sweep_canvas = canvas

        # Buttons
        bot = tk.Frame(frame, bg=BG)
        bot.pack(fill=tk.X, padx=16, pady=8)
        self._sweep_btn = ttk.Button(bot, text="▶  Run Sweep", style="Accent.TButton",
                                     command=self._start_sweep)
        self._sweep_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="■  Stop", style="Danger.TButton",
                   command=self._stop_sweep).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="View Results  →", style="Accent.TButton",
                   command=lambda: self.nb.select(2)).pack(side=tk.RIGHT, padx=4)

        self._sweep_stop = False
        self._sweep_points: list[list] = [[] for _ in range(NUM_MOTORS)]  # per-motor

    def _start_sweep(self):
        if not self.connected:
            messagebox.showwarning("Not Connected", "Connect to serial port first.")
            return
        uncal = [i for i in range(NUM_MOTORS)
                 if self._sweep_motor_vars[i].get()
                 and not self.profiles[i].calibration.calibrated]
        if uncal:
            names = ", ".join(MOTOR_NAMES[i] for i in uncal)
            if not messagebox.askyesno("Uncalibrated Motors",
                                       f"{names} not calibrated. Use default PPR=360?"):
                return
            for i in uncal:
                self.profiles[i].calibration.ppr = 360
                self.profiles[i].calibration.calibrated = True

        self._sweep_stop = False
        self._sweep_btn.config(state=tk.DISABLED)
        t = threading.Thread(target=self._sweep_worker, daemon=True)
        t.start()

    def _stop_sweep(self):
        self._sweep_stop = True
        self.mserial.stop_all()

    def _sweep_worker(self):
        try:
            steps = int(self.sweep_steps_var.get())
            dwell = float(self.sweep_dwell_var.get())
            window = float(self.sweep_window_var.get())
        except ValueError:
            steps, dwell, window = SWEEP_STEPS, SWEEP_DWELL_S, MEASURE_WINDOW_S

        motors_to_test = [i for i in range(NUM_MOTORS) if self._sweep_motor_vars[i].get()]
        # Forward: 1..200, Reverse: -1..-200 in steps
        fwd_vals = np.linspace(20, 200, steps, dtype=int).tolist()
        rev_vals = (-np.linspace(20, 200, steps, dtype=int)).tolist()
        all_vals = fwd_vals + rev_vals
        total = len(motors_to_test) * len(all_vals)
        done = 0

        for motor_id in motors_to_test:
            if self._sweep_stop:
                break
            ppr = self.profiles[motor_id].calibration.ppr
            self.profiles[motor_id].speed_points.clear()
            self._sweep_points[motor_id].clear()

            for inp in all_vals:
                if self._sweep_stop:
                    break
                direction = "fwd" if inp > 0 else "rev"
                self._update_sweep_status(
                    f"Motor {motor_id+1} | Input: {inp:+d} ({direction})", done/total)

                self.mserial.reset_encoders()
                self.mserial.set_motor(motor_id, inp)
                time.sleep(dwell - window)

                # Measure
                self.mserial.reset_encoders()
                t0 = time.time()
                time.sleep(window)
                encs = self.mserial.get_encoders()
                elapsed = time.time() - t0
                self.mserial.set_motor(motor_id, 0)
                time.sleep(0.1)

                if encs and ppr > 0:
                    pulses = abs(encs[motor_id])
                    rps = pulses / ppr / elapsed
                    if inp < 0:
                        rps = -rps
                    rpm = rps * 60
                    pt = SpeedPoint(int(inp), direction, rpm, rps)
                    self.profiles[motor_id].speed_points.append(pt)
                    self._sweep_points[motor_id].append((inp, rps))
                    self._update_sweep_plot(motor_id)

                done += 1

        self.mserial.stop_all()
        self._update_sweep_status("Sweep complete!", 1.0)
        self._fit_all()
        self.after(100, self._update_results_tab)
        self.after(200, lambda: self._sweep_btn.config(state=tk.NORMAL))

    def _update_sweep_status(self, msg, frac):
        def _do():
            self.sweep_status.config(text=msg, fg=ACCENT)
            self.sweep_prog["value"] = frac * 100
        self.after(0, _do)

    def _update_sweep_plot(self, motor_id):
        def _do():
            ax = self._sweep_axes[motor_id]
            ax.cla()
            ax.set_facecolor(CARD)
            ax.set_title(MOTOR_NAMES[motor_id], color=COLORS[motor_id],
                         fontsize=8, fontfamily="monospace")
            ax.tick_params(colors=MUTED, labelsize=7)
            for sp in ax.spines.values():
                sp.set_color(BORDER)
            ax.set_xlabel("Input", color=MUTED, fontsize=7)
            ax.set_ylabel("RPS", color=MUTED, fontsize=7)

            pts = self._sweep_points[motor_id]
            if pts:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                fwd_x = [x for x, y in zip(xs, ys) if x > 0]
                fwd_y = [y for x, y in zip(xs, ys) if x > 0]
                rev_x = [x for x, y in zip(xs, ys) if x < 0]
                rev_y = [y for x, y in zip(xs, ys) if x < 0]
                if fwd_x:
                    ax.scatter(fwd_x, fwd_y, s=20, color=COLORS[motor_id], alpha=0.8,
                               zorder=3, label="fwd")
                if rev_x:
                    ax.scatter(rev_x, rev_y, s=20, color="#aaa", alpha=0.6,
                               zorder=3, label="rev")
                ax.axhline(0, color=BORDER, linewidth=0.5)
                ax.axvline(0, color=BORDER, linewidth=0.5)

            self._sweep_fig.tight_layout(pad=1.5)
            self._sweep_canvas.draw_idle()
        self.after(0, _do)

    # ── Curve Fitting ──────────────────────────
    def _fit_all(self):
        for prof in self.profiles:
            if not prof.speed_points:
                continue
            fwd = [(p.input_val, p.rps) for p in prof.speed_points if p.direction == "fwd"]
            rev = [(p.input_val, p.rps) for p in prof.speed_points if p.direction == "rev"]

            for pts, attr_fit, attr_r2 in [(fwd, "fit_fwd", "fit_fwd_r2"),
                                            (rev, "fit_rev", "fit_rev_r2")]:
                if len(pts) < 2:
                    continue
                xs = np.array([p[0] for p in pts])
                ys = np.array([p[1] for p in pts])
                # Force through origin with linear + quadratic
                try:
                    coeffs = np.polyfit(xs, ys, 2)
                    fit = np.poly1d(coeffs)
                    ys_pred = fit(xs)
                    ss_res = np.sum((ys - ys_pred)**2)
                    ss_tot = np.sum((ys - np.mean(ys))**2)
                    r2 = 1 - ss_res/ss_tot if ss_tot > 0 else 0
                    setattr(prof, attr_fit, fit)
                    setattr(prof, attr_r2, round(r2, 4))
                except Exception:
                    pass

    # ── Results Tab ────────────────────────────
    def _build_results_tab(self):
        frame = ttk.Frame(self.nb)
        self.nb.add(frame, text="  3 · RESULTS & CURVES  ")

        self._results_frame = frame
        self._results_fig = Figure(figsize=(12, 7), facecolor=BG)
        canvas = FigureCanvasTkAgg(self._results_fig, frame)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self._results_canvas = canvas

        bot = tk.Frame(frame, bg=BG)
        bot.pack(fill=tk.X, padx=16, pady=4)
        ttk.Button(bot, text="↺  Refresh Plots", command=self._update_results_tab
                   ).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="Export Coefficients",
                   command=self._export_coefficients).pack(side=tk.LEFT, padx=4)

    def _update_results_tab(self):
        self._results_fig.clf()
        axes = []
        for i in range(NUM_MOTORS):
            ax = self._results_fig.add_subplot(2, 4, i+1)
            axes.append(ax)
        for i in range(NUM_MOTORS):
            ax = self._results_fig.add_subplot(2, 4, i+5)
            axes.append(ax)

        for i, prof in enumerate(self.profiles):
            # Top row: raw scatter
            ax_top = axes[i]
            ax_top.set_facecolor(CARD)
            ax_top.set_title(f"{MOTOR_NAMES[i]}\nRPS vs Input",
                             color=COLORS[i], fontsize=8, fontfamily="monospace")
            ax_top.tick_params(colors=MUTED, labelsize=7)
            for sp in ax_top.spines.values():
                sp.set_color(BORDER)

            if prof.speed_points:
                fwd = [(p.input_val, p.rps) for p in prof.speed_points if p.direction=="fwd"]
                rev = [(p.input_val, p.rps) for p in prof.speed_points if p.direction=="rev"]

                if fwd:
                    fx, fy = zip(*fwd)
                    ax_top.scatter(fx, fy, s=18, color=COLORS[i], alpha=0.9,
                                   label="Forward", zorder=3)
                    if prof.fit_fwd:
                        xs = np.linspace(0, 200, 100)
                        ax_top.plot(xs, prof.fit_fwd(xs), color=COLORS[i],
                                    linewidth=1.5, linestyle="--", alpha=0.7)
                if rev:
                    rx, ry = zip(*rev)
                    ax_top.scatter(rx, ry, s=18, color="#aaaaaa", alpha=0.7,
                                   label="Reverse", zorder=3)
                    if prof.fit_rev:
                        xs = np.linspace(-200, 0, 100)
                        ax_top.plot(xs, prof.fit_rev(xs), color="#aaaaaa",
                                    linewidth=1.5, linestyle="--", alpha=0.7)

                ax_top.axhline(0, color=BORDER, linewidth=0.5)
                ax_top.axvline(0, color=BORDER, linewidth=0.5)
                ax_top.set_xlabel("Input (-200..200)", color=MUTED, fontsize=6)
                ax_top.set_ylabel("RPS", color=MUTED, fontsize=7)
                leg = ax_top.legend(fontsize=6, framealpha=0.3,
                                    labelcolor=TEXT, facecolor=BG2)

            # Bottom row: RPM + fit info
            ax_bot = axes[i+4]
            ax_bot.set_facecolor(CARD)
            ax_bot.set_title(f"RPM vs Input", color=MUTED, fontsize=8,
                             fontfamily="monospace")
            ax_bot.tick_params(colors=MUTED, labelsize=7)
            for sp in ax_bot.spines.values():
                sp.set_color(BORDER)

            if prof.speed_points:
                all_inp = [p.input_val for p in prof.speed_points]
                all_rpm = [p.rpm for p in prof.speed_points]
                ax_bot.scatter(all_inp, all_rpm, s=14, color=COLORS[i], alpha=0.7, zorder=3)
                ax_bot.axhline(0, color=BORDER, linewidth=0.5)
                ax_bot.axvline(0, color=BORDER, linewidth=0.5)
                ax_bot.set_xlabel("Input", color=MUTED, fontsize=6)
                ax_bot.set_ylabel("RPM", color=MUTED, fontsize=7)

                # Fit annotation
                info = f"PPR: {prof.calibration.ppr:.0f}\n"
                if prof.fit_fwd:
                    c = prof.fit_fwd.coeffs
                    info += f"Fwd: {c[0]:.4f}x²+{c[1]:.4f}x+{c[2]:.4f}\nR²={prof.fit_fwd_r2:.4f}\n"
                if prof.fit_rev:
                    c = prof.fit_rev.coeffs
                    info += f"Rev: R²={prof.fit_rev_r2:.4f}"
                ax_bot.text(0.02, 0.97, info, transform=ax_bot.transAxes,
                            fontsize=6, va="top", color=MUTED,
                            fontfamily="monospace",
                            bbox=dict(facecolor=BG, alpha=0.5, edgecolor=BORDER, pad=2))

        self._results_fig.tight_layout(pad=1.5)
        self._results_canvas.draw_idle()

    def _export_coefficients(self):
        lines = ["# Motor Speed Characterization Coefficients",
                 "# Generated by Motor Plotter",
                 "# Polynomial fit: RPS = a*input^2 + b*input + c",
                 ""]
        for i, prof in enumerate(self.profiles):
            lines.append(f"# {MOTOR_NAMES[i]}")
            lines.append(f"  PPR: {prof.calibration.ppr:.2f}")
            if prof.fit_fwd:
                c = prof.fit_fwd.coeffs
                lines.append(f"  Forward fit  (R²={prof.fit_fwd_r2:.4f}): "
                              f"a={c[0]:.6f}, b={c[1]:.6f}, c={c[2]:.6f}")
            if prof.fit_rev:
                c = prof.fit_rev.coeffs
                lines.append(f"  Reverse fit  (R²={prof.fit_rev_r2:.4f}): "
                              f"a={c[0]:.6f}, b={c[1]:.6f}, c={c[2]:.6f}")
            lines.append("")

        out = "\n".join(lines)
        win = tk.Toplevel(self)
        win.title("Exported Coefficients")
        win.configure(bg=BG)
        win.geometry("700x450")
        text = tk.Text(win, bg=CARD, fg=TEXT, font=("Courier New", 10),
                       relief=tk.FLAT, padx=12, pady=12)
        text.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)
        text.insert("end", out)
        text.config(state=tk.DISABLED)

    # ── Live Plotter Tab ───────────────────────
    def _build_live_tab(self):
        frame = ttk.Frame(self.nb)
        self.nb.add(frame, text="  4 · LIVE PLOTTER  ")

        ctrl = tk.Frame(frame, bg=CARD)
        ctrl.pack(fill=tk.X, padx=16, pady=(16,8))

        # Manual motor controls
        self._live_speed_vars = []
        for i in range(NUM_MOTORS):
            col_frame = tk.Frame(ctrl, bg=CARD)
            col_frame.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=8, pady=8)

            tk.Label(col_frame, text=MOTOR_NAMES[i], font=("Courier New", 9, "bold"),
                     fg=COLORS[i], bg=CARD).pack()

            speed_var = tk.IntVar(value=0)
            self._live_speed_vars.append(speed_var)

            lbl = tk.Label(col_frame, textvariable=speed_var,
                           font=("Courier New", 12, "bold"), fg=COLORS[i], bg=CARD)
            lbl.pack()

            scale = ttk.Scale(col_frame, from_=-200, to=200, variable=speed_var,
                              orient=tk.HORIZONTAL, length=180)
            scale.pack(pady=4)

            motor_id_closure = i
            def make_cmd(mid):
                def cmd(val):
                    if self.connected:
                        self.mserial.set_motor(mid, int(float(val)))
                return cmd
            scale.config(command=make_cmd(i))

        # Stop all button
        tk.Frame(ctrl, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, padx=4)
        ttk.Button(ctrl, text="■\nSTOP\nALL", style="Danger.TButton",
                   command=self._live_stop_all).pack(side=tk.LEFT, padx=12, pady=8)

        # Live plot
        self._live_fig = Figure(figsize=(12, 4.5), facecolor=BG)
        self._live_ax = self._live_fig.add_subplot(1, 1, 1)
        self._live_ax.set_facecolor(CARD)
        self._live_ax.set_title("Encoder Counts — Live", color=TEXT,
                                fontsize=10, fontfamily="monospace")
        self._live_ax.tick_params(colors=MUTED)
        for sp in self._live_ax.spines.values():
            sp.set_color(BORDER)
        self._live_ax.set_xlabel("Time (s)", color=MUTED)
        self._live_ax.set_ylabel("Encoder Count", color=MUTED)

        live_canvas = FigureCanvasTkAgg(self._live_fig, frame)
        live_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=16, pady=8)
        self._live_canvas = live_canvas

        self._live_lines = []
        for i in range(NUM_MOTORS):
            line, = self._live_ax.plot([], [], color=COLORS[i], linewidth=1.5,
                                       label=MOTOR_NAMES[i])
            self._live_lines.append(line)
        self._live_ax.legend(fontsize=8, framealpha=0.3, labelcolor=TEXT,
                             facecolor=BG2)

        # Animate
        self._live_anim = animation.FuncAnimation(
            self._live_fig, self._live_animate, interval=80, blit=False, cache_frame_data=False)

        # Start live polling
        self._live_running = True
        self._live_poll_thread = threading.Thread(target=self._live_poll, daemon=True)
        self._live_poll_thread.start()
        self._live_t0 = time.time()

    def _live_stop_all(self):
        for v in self._live_speed_vars:
            v.set(0)
        if self.connected:
            self.mserial.stop_all()

    def _live_poll(self):
        while self._live_running:
            if self.connected:
                encs = self.mserial.get_encoders()
                t = time.time() - self._live_t0
                if encs:
                    for i in range(NUM_MOTORS):
                        self._enc_history[i].append(encs[i])
                        self._enc_times[i].append(t)
            time.sleep(0.05)

    def _live_animate(self, frame):
        for i in range(NUM_MOTORS):
            if self._enc_times[i]:
                ts = list(self._enc_times[i])
                ys = list(self._enc_history[i])
                self._live_lines[i].set_data(ts, ys)
        if any(self._enc_times[i] for i in range(NUM_MOTORS)):
            all_t = [t for i in range(NUM_MOTORS) for t in self._enc_times[i]]
            all_y = [y for i in range(NUM_MOTORS) for y in self._enc_history[i]]
            if all_t:
                self._live_ax.set_xlim(max(0, min(all_t)), max(all_t)+0.1)
                span = max(all_y) - min(all_y) if all_y else 10
                margin = span*0.1 if span > 0 else 10
                self._live_ax.set_ylim(min(all_y)-margin, max(all_y)+margin)
        return self._live_lines

    # ── Connection ────────────────────────────
    def _refresh_ports(self):
        ports = self.mserial.list_ports()
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.set(ports[0])

    def _toggle_connect(self):
        if self.connected:
            self.mserial.disconnect()
            self.connected = False
            self.conn_btn.config(text="Connect")
            self.status_dot.config(fg="#444")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showwarning("No Port", "Select a serial port first.")
                return
            ok = self.mserial.connect(port)
            if ok:
                self.connected = True
                self.conn_btn.config(text="Disconnect")
                self.status_dot.config(fg="#7fff6b")
                self._live_t0 = time.time()
            else:
                messagebox.showerror("Connection Failed",
                                     f"Could not connect to {port}")

    def _on_close(self):
        self._live_running = False
        self._sweep_stop = True
        self.mserial.disconnect()
        self.destroy()


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────
if __name__ == "__main__":
    app = MotorPlotterApp()
    app.mainloop()