#!/usr/bin/env python3
"""VioPELS wave viewer + netlist editor (stdlib only: tkinter + csv).

Launch:
  python3 wview.py [run.csv]          # viewer, optional file preloaded
  python3 wview.py --netlist buck.net # open editor with a file

Tab 1 "Waves": load `pe run` CSVs, toggle probes, overlay a second run
(method/param comparison), drag-zoom, wheel-zoom, cursor readout.
Tab 2 "Netlist": edit with template starters, pre-flight hints, Run via
the `pe` binary, one-click Plot of the result.
"""
import os
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import csvdata  # noqa: E402
import templates  # noqa: E402

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd",
          "#ff7f0e", "#17becf", "#8c564b", "#e377c2"]
ML, MR, MT, MB = 64, 12, 8, 18  # canvas margins


class WaveCanvas(tk.Canvas):
    """Stacked probe panels, shared time axis, zoom + cursor."""

    def __init__(self, master, status):
        super().__init__(master, bg="white", highlightthickness=1,
                         highlightbackground="#999")
        self.status = status
        self.runs = []  # [(label, RunTrace)]
        self.shown = {}  # probe -> bool
        self.x0 = self.x1 = None  # zoom window (None = full)
        self._press = None
        self._cursor_t = None
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Double-Button-1>", lambda e: self.reset_zoom())
        self.bind("<Motion>", self._on_move)
        self.bind("<MouseWheel>", self._on_wheel)
        self.bind("<Button-4>", lambda e: self._zoom_at(e.x, 1.25))
        self.bind("<Button-5>", lambda e: self._zoom_at(e.x, 0.8))
        self.bind("<Configure>", lambda e: self.redraw())

    # -- data ---------------------------------------------------------
    def set_runs(self, runs):
        self.runs = runs
        probes = []
        for _, tr in runs:
            for p in tr.probes:
                if p not in probes:
                    probes.append(p)
        for p in probes:
            self.shown.setdefault(p, True)
        for p in list(self.shown):
            if p not in probes:
                del self.shown[p]
        self.reset_zoom(silent=True)
        self.redraw()
        return probes

    def reset_zoom(self, silent=False):
        self.x0 = self.x1 = None
        if not silent:
            self.redraw()

    # -- geometry -----------------------------------------------------
    def _panels(self):
        active = [p for p, on in self.shown.items() if on]
        h = max(self.winfo_height(), 60)
        n = max(len(active), 1)
        ph = (h - MT - MB) / n
        return active, ph

    def _xrange(self):
        if not self.runs:
            return 0.0, 1.0
        t = self.runs[0][1].time
        full = (t[0], t[-1])
        if self.x0 is None:
            return full
        return (max(self.x0, full[0]), min(self.x1, full[1]))

    # -- drawing ------------------------------------------------------
    def redraw(self):
        self.delete("all")
        if not self.runs:
            self._msg("Load a `pe run` CSV to begin (or Run a netlist).")
            return
        active, ph = self._panels()
        w = max(self.winfo_width(), 120)
        x0, x1 = self._xrange()
        if x1 <= x0:
            x1 = x0 + 1e-12

        def X(t):
            return ML + (t - x0) / (x1 - x0) * (w - ML - MR)

        for i, probe in enumerate(active):
            y_top = MT + i * ph
            ys = []
            for _, tr in self.runs:
                if probe in tr.probes:
                    ys += [v for t, v in zip(tr.time, tr.column(probe))
                           if x0 <= t <= x1]
            lo, hi = (min(ys), max(ys)) if ys else (0.0, 1.0)
            if hi - lo < 1e-300:
                lo, hi = lo - 0.5, hi + 0.5
            pad = 0.06 * (hi - lo)
            lo, hi = lo - pad, hi + pad

            def Y(v, _lo=lo, _hi=hi, _top=y_top):
                return _top + ph - MB / 2 - (v - _lo) / (_hi - _lo) * (ph - MB)

            for f in (0.0, 0.25, 0.5, 0.75, 1.0):  # grid + y ticks
                y = Y(lo + f * (hi - lo))
                self.create_line(ML, y, w - MR, y, fill="#e3e3e3")
                self.create_text(ML - 4, y, anchor="e", font=("TkDefaultFont", 8),
                                 text=f"{lo + f * (hi - lo):.3g}")
            for f in range(6):  # x ticks on bottom panel only
                t = x0 + f / 5 * (x1 - x0)
                x = X(t)
                self.create_line(x, MT, x, MT + len(active) * ph - MB / 2,
                                 fill="#e3e3e3")
                if i == len(active) - 1:
                    self.create_text(x, MT + len(active) * ph - 4, anchor="n",
                                     font=("TkDefaultFont", 8), text=f"{t:.3g}")
            for j, (label, tr) in enumerate(self.runs):
                if probe not in tr.probes:
                    continue
                td, yd = tr.downsampled(probe)
                pts = []
                for t, v in zip(td, yd):
                    if x0 <= t <= x1:
                        pts += [X(t), Y(v)]
                if len(pts) >= 4:
                    self.create_line(*pts, fill=COLORS[j % len(COLORS)], width=1)
            self.create_text(ML + 4, y_top + 2, anchor="nw",
                             font=("TkDefaultFont", 8, "bold"), text=probe)
            self._geo = (X, x0, x1)
        if self._cursor_t is not None and self._geo:
            X, _, _ = self._geo
            x = X(self._cursor_t)
            self.create_line(x, MT, x, MT + len(active) * ph - MB / 2,
                             fill="#666", dash=(3, 3))

    def _msg(self, text):
        self.create_text(12, 12, anchor="nw", text=text, fill="#555")

    # -- interaction --------------------------------------------------
    def _on_press(self, e):
        self._press = e.x

    def _on_drag(self, e):
        if self._press is not None:
            self.redraw()
            self.create_rectangle(self._press, MT, e.x,
                                  self.winfo_height() - MB, outline="#1f77b4")

    def _on_release(self, e):
        if self._press is None or not self.runs or not hasattr(self, "_geo"):
            self._press = None
            return
        X, x0, x1 = self._geo
        w = max(self.winfo_width(), 120)
        inv = lambda px: x0 + (px - ML) / (w - ML - MR) * (x1 - x0)
        a, b = sorted((inv(self._press), inv(e.x)))
        if b - a > 1e-15 * max(1.0, abs(b)):
            self.x0, self.x1 = a, b
        self._press = None
        self.redraw()

    def _zoom_at(self, px, factor):
        if not self.runs or not hasattr(self, "_geo"):
            return
        _, x0, x1 = self._geo
        w = max(self.winfo_width(), 120)
        c = x0 + (px - ML) / (w - ML - MR) * (x1 - x0)
        half = (x1 - x0) / 2 / factor
        self.x0, self.x1 = c - half, c + half
        self.redraw()

    def _on_wheel(self, e):
        self._zoom_at(e.x, 1.25 if e.delta > 0 else 0.8)

    def _on_move(self, e):
        if not self.runs or not hasattr(self, "_geo"):
            return
        X, x0, x1 = self._geo
        w = max(self.winfo_width(), 120)
        t = x0 + (e.x - ML) / (w - ML - MR) * (x1 - x0)
        tt = self.runs[0][1].time
        if not (tt[0] <= t <= tt[-1]):
            return
        i = min(range(len(tt)), key=lambda k: abs(tt[k] - t))
        self._cursor_t = tt[i]
        parts = [f"t = {tt[i]:.6g} s"]
        for _, tr in self.runs:
            for p, on in self.shown.items():
                if on and p in tr.probes:
                    parts.append(f"{p} = {tr.column(p)[i]:.6g}")
        self.status.config(text="   ".join(parts))
        self.redraw()


class App(tk.Tk):
    def __init__(self, preload=None):
        super().__init__()
        self.title("VioPELS wave viewer")
        self.geometry("960x640")
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True)
        self._build_waves(nb)
        self._build_netlist(nb)
        if preload:
            self._load_files([preload], None)

    # -- Waves tab ----------------------------------------------------
    def _build_waves(self, nb):
        tab = ttk.Frame(nb)
        nb.add(tab, text="Waves")
        bar = ttk.Frame(tab)
        bar.pack(fill="x", padx=6, pady=4)
        self.f1 = tk.StringVar()
        self.f2 = tk.StringVar()
        ttk.Entry(bar, textvariable=self.f1, width=44).pack(side="left")
        ttk.Button(bar, text="Browse…", command=self._browse1).pack(side="left", padx=2)
        ttk.Label(bar, text="overlay:").pack(side="left", padx=(8, 0))
        ttk.Entry(bar, textvariable=self.f2, width=30).pack(side="left")
        ttk.Button(bar, text="Browse…", command=self._browse2).pack(side="left", padx=2)
        ttk.Button(bar, text="Load", command=self._load).pack(side="left", padx=6)
        ttk.Button(bar, text="Reset zoom", command=lambda: self.cv.reset_zoom()).pack(side="left")

        mid = ttk.Frame(tab)
        mid.pack(fill="both", expand=True)
        self.probebox = ttk.Frame(mid, width=130)
        self.probebox.pack(side="left", fill="y", padx=4)
        self.probebox.pack_propagate(False)
        ttk.Label(self.probebox, text="probes").pack(anchor="w")
        self.status = ttk.Label(tab, text="ready", anchor="w")
        self.status.pack(fill="x", padx=6, pady=2)
        self.cv = WaveCanvas(mid, self.status)
        self.cv.pack(side="left", fill="both", expand=True)
        self.legend = ttk.Label(tab, text="", anchor="w")
        self.legend.pack(fill="x", padx=6)
        self._probe_vars = {}

    def _browse1(self):
        p = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"), ("all", "*")])
        if p:
            self.f1.set(p)

    def _browse2(self):
        p = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"), ("all", "*")])
        if p:
            self.f2.set(p)

    def _load(self):
        files = [self.f1.get()] + ([self.f2.get()] if self.f2.get() else [])
        self._load_files(files[0:1] if len(files) == 1 else files,
                         files[1] if len(files) > 1 else None)

    def _load_files(self, mains, overlay):
        try:
            runs = [("run", csvdata.load_run(mains[0]))]
            if overlay:
                runs.append(("overlay", csvdata.load_run(overlay)))
        except (ValueError, KeyError, OSError) as e:
            messagebox.showerror("load failed", str(e))
            return
        probes = self.cv.set_runs(runs)
        for w in self.probebox.winfo_children()[1:]:
            w.destroy()
        self._probe_vars = {}
        for p in probes:
            v = tk.BooleanVar(value=True)
            self._probe_vars[p] = v
            ttk.Checkbutton(self.probebox, text=p, variable=v,
                            command=self._retoggle).pack(anchor="w")
        n = len(runs[0][1].time)
        self.legend.config(text=f"base: {mains[0]}" +
                           (f"    overlay: {overlay}" if overlay else "") +
                           f"    [{n} points]")
        self.status.config(text=f"loaded {len(runs)} run(s)")

    def _retoggle(self):
        self.cv.shown = {p: v.get() for p, v in self._probe_vars.items()}
        self.cv.redraw()

    # -- Netlist tab --------------------------------------------------
    def _build_netlist(self, nb):
        tab = ttk.Frame(nb)
        nb.add(tab, text="Netlist")
        bar = ttk.Frame(tab)
        bar.pack(fill="x", padx=6, pady=4)
        self.tpl = tk.StringVar(value="buck (PWM)")
        ttk.OptionMenu(bar, self.tpl, "buck (PWM)", *templates.TEMPLATES,
                       command=lambda _: self._apply_tpl()).pack(side="left")
        ttk.Button(bar, text="Open…", command=self._open_net).pack(side="left", padx=2)
        ttk.Button(bar, text="Save…", command=self._save_net).pack(side="left", padx=2)
        ttk.Label(bar, text="pe:").pack(side="left", padx=(8, 0))
        self.pepath = tk.StringVar(value=self._find_pe())
        ttk.Entry(bar, textvariable=self.pepath, width=34).pack(side="left")
        ttk.Label(bar, text="method:").pack(side="left", padx=(8, 0))
        self.method = tk.StringVar(value="trap")
        ttk.OptionMenu(bar, self.method, "trap", "trap", "trbdf2", "auto").pack(side="left")
        ttk.Button(bar, text="Run", command=self._run_net).pack(side="left", padx=6)
        ttk.Button(bar, text="Plot", command=self._plot_last).pack(side="left")
        self.ed = tk.Text(tab, height=20, font=("TkFixedFont", 10), wrap="none")
        self.ed.pack(fill="both", expand=True, padx=6)
        self.ed.insert("1.0", templates.BUCK)
        self.log = tk.Text(tab, height=7, font=("TkFixedFont", 9),
                           wrap="none", state="disabled", bg="#f4f4f4")
        self.log.pack(fill="x", padx=6, pady=4)
        self._last_csv = None

    @staticmethod
    def _find_pe():
        here = os.path.dirname(os.path.abspath(__file__))
        cand = os.path.normpath(os.path.join(here, "..", "..", "build",
                                             "power_engine", "tools", "pe"))
        return cand if os.path.isfile(cand) and os.access(cand, os.X_OK) else "pe"

    def _apply_tpl(self):
        self.ed.delete("1.0", "end")
        self.ed.insert("1.0", templates.TEMPLATES[self.tpl.get()])

    def _open_net(self):
        p = filedialog.askopenfilename(filetypes=[("netlist", "*.net"), ("all", "*")])
        if p:
            self.ed.delete("1.0", "end")
            self.ed.insert("1.0", open(p).read())

    def _save_net(self):
        p = filedialog.asksaveasfilename(defaultextension=".net")
        if p:
            open(p, "w").write(self.ed.get("1.0", "end"))

    def _say(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _run_net(self):
        text = self.ed.get("1.0", "end")
        for h in templates.validate(text):
            self._say("hint: " + h)
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".net", delete=False) as f:
            f.write(text)
            net = f.name
        out = net + ".csv"
        try:
            pr = subprocess.run([self.pepath.get(), "run", "--netlist", net,
                                 "--method", self.method.get(), "--out", out],
                                capture_output=True, text=True, timeout=300)
        except (OSError, subprocess.TimeoutExpired) as e:
            self._say(f"run failed: {e}")
            return
        self._say((pr.stdout or "") + (pr.stderr or "") + f"[exit {pr.returncode}]")
        if pr.returncode != 0 or not os.path.isfile(out):
            return
        self._last_csv = out
        self.f1.set(out)
        self._load_files([out], None)

    def _plot_last(self):
        if self._last_csv:
            self._load_files([self._last_csv], None)


def main(argv):
    preload, netlist = None, None
    if "--netlist" in argv:
        netlist = argv[argv.index("--netlist") + 1]
    elif argv[1:]:
        preload = argv[1]
    app = App(preload=preload)
    if netlist:
        app.ed.delete("1.0", "end")
        app.ed.insert("1.0", open(netlist).read())
    app.mainloop()


if __name__ == "__main__":
    main(sys.argv)
