"""
Robot Control GUI  (STM32 Nucleo-F446RE, PID position)
pip install customtkinter pyserial matplotlib
Protocole : S,mmG,mmD | X | PG/IG/DG/PD/ID/DD<val>
Télémétrie : cibleG,reelG,cibleD,reelD,yaw  (mm, mm, mm, mm, deg)
"""
import math
import queue
import threading
import time
import collections
import tkinter as tk

import customtkinter as ctk
import serial
import serial.tools.list_ports
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

BAUD = 115200
WINDOW_S = 10.0          # fenêtre des courbes (secondes)
MAX_PTS = 600
TOL_MM = 4               # tolérance "arrivé"

BG = "#14161f"
CARD = "#1d2030"
ACCENT = "#4f8cff"
GREEN = "#2ecc71"
RED = "#e74c3c"
ORANGE = "#f39c12"
PURPLE = "#9b59b6"
TXT = "#e6e8f0"
MUTED = "#8b90a5"


# ------------------------------------------------------------------
#  Thread série (lit maintenant 5 valeurs)
# ------------------------------------------------------------------
class SerialLink:
    def __init__(self, rx_queue):
        self.q = rx_queue
        self.ser = None
        self.lock = threading.Lock()
        self.alive = False
        self.thread = None

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port):
        self.close()
        self.ser = serial.Serial(port, BAUD, timeout=0.05, write_timeout=0.5)
        self.ser.reset_input_buffer()
        self.alive = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.alive = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    def send(self, cmd):
        if not self.connected:
            return False
        try:
            with self.lock:
                self.ser.write((cmd + "\n").encode())
            self.q.put(("tx", cmd))
            return True
        except Exception as e:
            self.q.put(("lost", str(e)))
            return False

    def _run(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                self.q.put(("lost", str(e)))
                self.alive = False
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="ignore").strip()
                p = line.split(",")
                # MODIFICATION : On attend 5 valeurs maintenant
                if len(p) == 5:
                    try:
                        self.q.put(("data", time.time(), *map(int, p)))
                    except ValueError:
                        pass
            if len(buf) > 512:
                buf = b""


# ------------------------------------------------------------------
#  Application
# ------------------------------------------------------------------
class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.title("Robot Control • STM32 PID + MPU")
        self.geometry("1280x900") # Un peu plus haut pour le 3ème graph
        self.minsize(1100, 800)
        self.configure(fg_color=BG)

        self.q = queue.Queue()
        self.link = SerialLink(self.q)
        self.t0 = time.time()
        self.t = collections.deque(maxlen=MAX_PTS)
        self.tG = collections.deque(maxlen=MAX_PTS)
        self.cG = collections.deque(maxlen=MAX_PTS)
        self.tD = collections.deque(maxlen=MAX_PTS)
        self.cD = collections.deque(maxlen=MAX_PTS)
        self.yaw = collections.deque(maxlen=MAX_PTS) # NOUVEAU
        
        self.last = (0, 0, 0, 0, 0) # NOUVEAU (ajout du yaw)
        self.last_rx = 0.0
        self.moving = False
        self.move_t0 = 0.0
        self.ok_since = None
        self.last_port = ""
        self.auto_reconnect = False

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self._build_left()
        self._build_right()

        self.bind("<Escape>", lambda e: self.stop())
        for key, fn in (("<Up>", self.forward), ("<Down>", self.backward),
                        ("<Left>", self.turn_left), ("<Right>", self.turn_right)):
            self.bind(key, lambda e, f=fn: self._key(f))

        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.poll)
        self.after(100, self.redraw)

    # ---------------- UI helpers ----------------
    def card(self, parent, title=None):
        f = ctk.CTkFrame(parent, fg_color=CARD, corner_radius=14)
        if title:
            ctk.CTkLabel(f, text=title, font=ctk.CTkFont(size=13, weight="bold"),
                         text_color=MUTED).pack(anchor="w", padx=14, pady=(10, 4))
        return f

    def entry(self, parent, default, width=70):
        e = ctk.CTkEntry(parent, width=width, justify="center", fg_color="#12141d",
                         border_color="#2a2f45")
        e.insert(0, str(default))
        return e

    # ---------------- Panneau gauche ----------------
    def _build_left(self):
        left = ctk.CTkScrollableFrame(self, fg_color="transparent", width=350)
        left.grid(row=0, column=0, sticky="ns", padx=(14, 7), pady=14)

        # Connexion
        c = self.card(left, "CONNEXION")
        c.pack(fill="x", pady=(0, 10))
        row = ctk.CTkFrame(c, fg_color="transparent")
        row.pack(fill="x", padx=12, pady=(0, 6))
        self.port_cb = ctk.CTkComboBox(row, values=[""], width=170)
        self.port_cb.pack(side="left")
        ctk.CTkButton(row, text="⟳", width=34, command=self.refresh_ports,
                      fg_color="#2a2f45", hover_color="#3a4060").pack(side="left", padx=6)
        self.conn_btn = ctk.CTkButton(row, text="Connecter", width=90, command=self.toggle_conn)
        self.conn_btn.pack(side="left")
        st = ctk.CTkFrame(c, fg_color="transparent")
        st.pack(fill="x", padx=14, pady=(0, 10))
        self.led = ctk.CTkLabel(st, text="●", text_color=RED, font=ctk.CTkFont(size=18))
        self.led.pack(side="left")
        self.conn_lbl = ctk.CTkLabel(st, text="Déconnecté", text_color=MUTED)
        self.conn_lbl.pack(side="left", padx=6)

        # Mouvement
        m = self.card(left, "MOUVEMENT")
        m.pack(fill="x", pady=(0, 10))
        r1 = ctk.CTkFrame(m, fg_color="transparent")
        r1.pack(pady=(0, 4))
        ctk.CTkLabel(r1, text="Distance (cm)", text_color=MUTED).pack(side="left", padx=4)
        self.dist = self.entry(r1, 50)
        self.dist.pack(side="left", padx=4)
        ctk.CTkLabel(r1, text="Angle (°)", text_color=MUTED).pack(side="left", padx=4)
        self.ang = self.entry(r1, 90)
        self.ang.pack(side="left", padx=4)

        pad = ctk.CTkFrame(m, fg_color="transparent")
        pad.pack(pady=6)
        bs = dict(width=92, height=46, font=ctk.CTkFont(size=14, weight="bold"))
        ctk.CTkButton(pad, text="▲ Avant", command=self.forward, **bs).grid(row=0, column=1, padx=4, pady=4)
        ctk.CTkButton(pad, text="◀ Gauche", command=self.turn_left, **bs).grid(row=1, column=0, padx=4, pady=4)
        ctk.CTkButton(pad, text="STOP", command=self.stop, fg_color=RED, hover_color="#c0392b",
                      **bs).grid(row=1, column=1, padx=4, pady=4)
        ctk.CTkButton(pad, text="Droite ▶", command=self.turn_right, **bs).grid(row=1, column=2, padx=4, pady=4)
        ctk.CTkButton(pad, text="▼ Arrière", command=self.backward, **bs).grid(row=2, column=1, padx=4, pady=4)

        r2 = ctk.CTkFrame(m, fg_color="transparent")
        r2.pack(pady=(4, 2))
        ctk.CTkLabel(r2, text="Voie roues (cm)", text_color=MUTED).pack(side="left", padx=4)
        self.track = self.entry(r2, 15)
        self.track.pack(side="left", padx=4)

        r3 = ctk.CTkFrame(m, fg_color="transparent")
        r3.pack(pady=(6, 4))
        ctk.CTkLabel(r3, text="Custom G/D (mm)", text_color=MUTED).pack(side="left", padx=4)
        self.cg = self.entry(r3, 0, 60)
        self.cg.pack(side="left", padx=2)
        self.cd = self.entry(r3, 0, 60)
        self.cd.pack(side="left", padx=2)
        ctk.CTkButton(r3, text="Envoyer", width=70, command=self.custom).pack(side="left", padx=4)

        self.status = ctk.CTkLabel(m, text="Prêt", text_color=MUTED, font=ctk.CTkFont(size=13))
        self.status.pack(pady=(4, 10))

        # PID
        p = self.card(left, "RÉGLAGE PID")
        p.pack(fill="x", pady=(0, 10))
        grid = ctk.CTkFrame(p, fg_color="transparent")
        grid.pack(padx=12)
        for j, h in enumerate(("", "Kp", "Ki", "Kd")):
            ctk.CTkLabel(grid, text=h, text_color=MUTED).grid(row=0, column=j, padx=4)
        self.pid = {}
        for i, (side, name) in enumerate((("G", "Gauche"), ("D", "Droite")), start=1):
            ctk.CTkLabel(grid, text=name, text_color=TXT).grid(row=i, column=0, padx=4, pady=3)
            for j, (k, d) in enumerate((("P", 40), ("I", 0.5), ("D", 1.5)), start=1):
                e = self.entry(grid, d, 62)
                e.grid(row=i, column=j, padx=3, pady=3)
                self.pid[k + side] = e
        self.link_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(p, text="Lier gauche = droite", variable=self.link_var).pack(pady=6)
        ctk.CTkButton(p, text="Appliquer PID", command=self.apply_pid).pack(pady=(0, 12))

        # Lectures
        r = self.card(left, "LECTURES")
        r.pack(fill="x")
        self.val = {}
        g = ctk.CTkFrame(r, fg_color="transparent")
        g.pack(padx=12, pady=(0, 12))
        # Ajout du Yaw dans les lectures
        for i, (key, name) in enumerate((("cG", "Gauche (mm)"), ("eG", "Erreur G"),
                                         ("cD", "Droite (mm)"), ("eD", "Erreur D"),
                                         ("yaw", "Lacet (°)"))):
            ctk.CTkLabel(g, text=name, text_color=MUTED).grid(row=i // 2 * 2, column=i % 2, padx=18)
            lb = ctk.CTkLabel(g, text="0", font=ctk.CTkFont(size=22, weight="bold"), text_color=ACCENT)
            lb.grid(row=i // 2 * 2 + 1, column=i % 2, padx=18, pady=(0, 6))
            self.val[key] = lb

    # ---------------- Panneau droit ----------------
    def _build_right(self):
        right = ctk.CTkFrame(self, fg_color="transparent")
        right.grid(row=0, column=1, sticky="nsew", padx=(7, 14), pady=14)
        right.grid_rowconfigure(0, weight=1)
        right.grid_columnconfigure(0, weight=1)

        pc = self.card(right)
        pc.grid(row=0, column=0, sticky="nsew")
        self.fig = Figure(figsize=(7, 6), dpi=100, facecolor=CARD) # Plus grand
        # 3 graphiques maintenant
        self.axG = self.fig.add_subplot(311)
        self.axD = self.fig.add_subplot(312, sharex=self.axG)
        self.axY = self.fig.add_subplot(313, sharex=self.axG) # Nouveau
        
        self.lines = {}
        for ax, title, key in ((self.axG, "Roue gauche", "G"), (self.axD, "Roue droite", "D")):
            ax.set_facecolor("#12141d")
            ax.set_title(title, color=TXT, fontsize=10, loc="left")
            ax.tick_params(colors=MUTED, labelsize=8)
            ax.grid(color="#2a2f45", alpha=0.6, linewidth=0.6)
            for s in ax.spines.values():
                s.set_color("#2a2f45")
            ax.set_ylabel("mm", color=MUTED, fontsize=8)
            (lt,) = ax.plot([], [], color=ORANGE, linestyle="--", linewidth=1.4, label="Cible")
            (lc,) = ax.plot([], [], color=ACCENT, linewidth=1.8, label="Réel")
            ax.legend(loc="upper left", fontsize=8, facecolor=CARD, edgecolor="#2a2f45",
                      labelcolor=TXT)
            self.lines[key] = (lt, lc)

        # Config du graphique Yaw
        self.axY.set_facecolor("#12141d")
        self.axY.set_title("Lacet (Yaw)", color=TXT, fontsize=10, loc="left")
        self.axY.tick_params(colors=MUTED, labelsize=8)
        self.axY.grid(color="#2a2f45", alpha=0.6, linewidth=0.6)
        for s in self.axY.spines.values():
            s.set_color("#2a2f45")
        self.axY.set_ylabel("degrés", color=MUTED, fontsize=8)
        (self.line_yaw,) = self.axY.plot([], [], color=PURPLE, linewidth=1.8, label="Yaw")
        self.axY.legend(loc="upper left", fontsize=8, facecolor=CARD, edgecolor="#2a2f45", labelcolor=TXT)

        self.axY.set_xlabel("temps (s)", color=MUTED, fontsize=8)
        self.fig.tight_layout(pad=1.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=pc)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

        lg = self.card(right, "JOURNAL")
        lg.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self.log = ctk.CTkTextbox(lg, height=90, fg_color="#12141d", text_color=MUTED,
                                  font=ctk.CTkFont(family="Consolas", size=12))
        self.log.pack(fill="x", padx=12, pady=(0, 12))
        self.log.configure(state="disabled")

    # ---------------- Connexion ----------------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()] or [""]
        self.port_cb.configure(values=ports)
        if self.port_cb.get() not in ports:
            self.port_cb.set(ports[0])

    def toggle_conn(self):
        if self.link.connected:
            self.auto_reconnect = False
            self.link.send("X")
            self.link.close()
            self.set_conn(False)
            self.log_add("Déconnecté")
            return
        port = self.port_cb.get()
        if not port:
            self.log_add("Aucun port sélectionné")
            return
        try:
            self.link.open(port)
        except Exception as e:
            self.log_add(f"Erreur ouverture {port} : {e}")
            return
        self.t0 = time.time()
        for d in (self.t, self.tG, self.cG, self.tD, self.cD, self.yaw):
            d.clear()
        self.set_conn(True, port)
        self.last_port = port
        self.auto_reconnect = True
        self.log_add(f"Connecté à {port}")

    def try_reconnect(self):
        if not self.auto_reconnect or self.link.connected:
            return
        try:
            self.link.open(self.last_port)
        except Exception:
            self.after(1000, self.try_reconnect)
            return
        self.set_conn(True, self.last_port)
        self.log_add(f"Reconnecté à {self.last_port}")
        self.set_status("Reconnecté (cible précédente conservée par le STM32)", ORANGE)

    def set_conn(self, ok, port=""):
        self.led.configure(text_color=GREEN if ok else RED)
        self.conn_lbl.configure(text=f"Connecté ({port})" if ok else "Déconnecté")
        self.conn_btn.configure(text="Déconnecter" if ok else "Connecter",
                                fg_color=RED if ok else ["#3a7ebf", "#1f538d"])

    # ---------------- Commandes ----------------
    def _f(self, e, default=0.0):
        try:
            return float(e.get().replace(",", "."))
        except ValueError:
            return default

    def _key(self, fn):
        if "entry" in str(self.focus_get()).lower():
            return
        fn()

    def move(self, mm_g, mm_d):
        if not self.link.connected:
            self.set_status("Non connecté", ORANGE)
            return
        self.link.send(f"S,{int(round(mm_g))},{int(round(mm_d))}")
        self.moving = True
        self.ok_since = None
        self.move_t0 = time.time()
        self.set_status("En mouvement…", ACCENT)

    def forward(self):
        d = self._f(self.dist) * 10
        self.move(d, d)

    def backward(self):
        d = self._f(self.dist) * 10
        self.move(-d, -d)

    def _arc_mm(self):
        return math.pi * self._f(self.track, 15) * abs(self._f(self.ang)) / 360.0 * 10

    def turn_right(self):
        a = self._arc_mm()
        self.move(a, -a)

    def turn_left(self):
        a = self._arc_mm()
        self.move(-a, a)

    def custom(self):
        self.move(self._f(self.cg), self._f(self.cd))

    def stop(self):
        self.link.send("X")
        self.moving = False
        self.set_status("Arrêté", ORANGE)

    def apply_pid(self):
        if not self.link.connected:
            self.set_status("Non connecté", ORANGE)
            return
        if self.link_var.get():
            for k in "PID":
                self.pid[k + "D"].delete(0, "end")
                self.pid[k + "D"].insert(0, self.pid[k + "G"].get())
        cmds = [f"{k}{s}{self._f(self.pid[k + s]):g}" for s in "GD" for k in "PID"]
        for i, c in enumerate(cmds):
            self.after(i * 40, lambda c=c: self.link.send(c))
        self.set_status("PID appliqué", GREEN)

    def set_status(self, text, color=MUTED):
        self.status.configure(text=text, text_color=color)

    def log_add(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S  ") + msg + "\n")
        n = int(float(self.log.index("end-1c").split(".")[0]))
        if n > 200:
            self.log.delete("1.0", "50.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ---------------- Boucles ----------------
    def poll(self):
        try:
            for _ in range(200):
                ev = self.q.get_nowait()
                if ev[0] == "data":
                    # Modification : On récupère 5 valeurs maintenant
                    _, ts, tg, cg, td, cd, yaw = ev
                    self.t.append(ts - self.t0)
                    self.tG.append(tg)
                    self.cG.append(cg)
                    self.tD.append(td)
                    self.cD.append(cd)
                    self.yaw.append(yaw)
                    self.last = (tg, cg, td, cd, yaw)
                    self.last_rx = ts
                elif ev[0] == "tx":
                    self.log_add("→ " + ev[1])
                elif ev[0] == "lost":
                    self.link.close()
                    self.set_conn(False)
                    self.moving = False
                    self.log_add("Liaison perdue : " + ev[1])
                    self.set_status("Liaison perdue, reconnexion…", RED)
                    if self.auto_reconnect:
                        self.after(1000, self.try_reconnect)
        except queue.Empty:
            pass

        tg, cg, td, cd, yaw = self.last
        self.val["cG"].configure(text=str(cg))
        self.val["cD"].configure(text=str(cd))
        self.val["eG"].configure(text=str(tg - cg))
        self.val["eD"].configure(text=str(td - cd))
        self.val["yaw"].configure(text=str(yaw))

        if self.moving:
            if abs(tg - cg) <= TOL_MM and abs(td - cd) <= TOL_MM:
                self.ok_since = self.ok_since or time.time()
                if time.time() - self.ok_since > 0.25:
                    self.moving = False
                    self.set_status(f"Terminé ✓  ({time.time() - self.move_t0:.2f} s)", GREEN)
            else:
                self.ok_since = None
            if time.time() - self.move_t0 > 20:
                self.moving = False
                self.set_status("Timeout", RED)

        if self.link.connected and self.last_rx and time.time() - self.last_rx > 2:
            self.led.configure(text_color=ORANGE)
        elif self.link.connected:
            self.led.configure(text_color=GREEN)
        self.after(30, self.poll)

    def redraw(self):
        if self.t:
            t = list(self.t)
            tmax = t[-1]
            tmin = max(0.0, tmax - WINDOW_S)
            for key, tgt, cur, ax in (("G", self.tG, self.cG, self.axG),
                                      ("D", self.tD, self.cD, self.axD)):
                lt, lc = self.lines[key]
                lt.set_data(t, list(tgt))
                lc.set_data(t, list(cur))
                vals = list(tgt)[-int(WINDOW_S * 22):] + list(cur)[-int(WINDOW_S * 22):]
                lo, hi = min(vals), max(vals)
                pad = max(20, (hi - lo) * 0.15)
                ax.set_ylim(lo - pad, hi + pad)
            
            # Mise à jour de la courbe Yaw
            self.line_yaw.set_data(t, list(self.yaw))
            y_min, y_max = min(self.yaw), max(self.yaw)
            if y_max - y_min < 10: # Si le robot bouge peu, on garde une échelle lisible
                y_min -= 5; y_max += 5
            self.axY.set_ylim(y_min - 5, y_max + 5)

            self.axG.set_xlim(tmin, tmin + WINDOW_S)
            self.canvas.draw_idle()
        self.after(100, self.redraw)

    def on_close(self):
        try:
            self.link.send("X")
            time.sleep(0.05)
        except Exception:
            pass
        self.link.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()