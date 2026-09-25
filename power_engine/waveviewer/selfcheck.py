#!/usr/bin/env python3
"""Headless waveviewer check (stdlib only, no display needed).

Covers what CI can cover without X: the Tk-free logic (csvdata,
templates) plus end-to-end `pe run` of every bundled template, then a
load of the produced CSVs. Widget code (wview.py) is compile-checked.
Run:  python3 selfcheck.py [--pe <pe binary>]
"""
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import csvdata  # noqa: E402
import templates  # noqa: E402


class LogicTest(unittest.TestCase):
    def test_run_roundtrip(self):
        with tempfile.NamedTemporaryFile("w", suffix=".csv",
                                         delete=False) as f:
            f.write("time,v:3,v:1\n0,0,12\n1e-6,3,12\n2e-6,6,12\n")
            path = f.name
        tr = csvdata.load_run(path)
        self.assertEqual(tr.probes, ["v:3", "v:1"])
        self.assertEqual(tr.column("v:3"), [0.0, 3.0, 6.0])
        t, y = tr.downsampled("v:3", max_pts=2)
        self.assertEqual((t[0], t[-1]), (0.0, 2e-6))
        with self.assertRaises(KeyError):
            tr.column("v:9")

    def test_run_rejects(self):
        for body in ["time\n", "volts,v:1\n0,1\n", "time,v:1\n0,x\n",
                     "time,v:1\n0,1\n1,2,3\n"]:
            with tempfile.NamedTemporaryFile("w", suffix=".csv",
                                             delete=False) as f:
                f.write(body)
                path = f.name
            with self.assertRaises(ValueError):
                csvdata.load_run(path)

    def test_sweep_skips_errors(self):
        with tempfile.NamedTemporaryFile("w", suffix=".csv",
                                         delete=False) as f:
            f.write("R,vout,error\n1,5.5,\n2,,diverged\n5,6.2,\n")
            path = f.name
        cols, rows, skipped = csvdata.load_sweep(path)
        self.assertEqual(cols, ["R", "vout"])
        self.assertEqual(skipped, 1)
        self.assertEqual([r["R"] for r in rows], [1.0, 5.0])

    def test_templates_validate(self):
        self.assertEqual(templates.validate(templates.BUCK), [])
        hints = templates.validate("R1 1 0 {ZZ}\n")
        self.assertTrue(any("ZZ" in h for h in hints),
                        f"expected undeclared-param hint, got {hints}")
        self.assertTrue(any(".tran" in h for h in templates.validate("V1 1 0 5\n.end\n")))


class TemplateSimTest(unittest.TestCase):
    PE = os.environ.get("PE", os.path.join("build", "power_engine",
                                           "tools", "pe"))

    def test_every_template_simulates(self):
        for name, text in templates.TEMPLATES.items():
            with self.subTest(template=name):
                with tempfile.TemporaryDirectory() as d:
                    net = os.path.join(d, "t.net")
                    out = os.path.join(d, "t.csv")
                    with open(net, "w") as f:
                        f.write(text)
                    pr = subprocess.run([self.PE, "run", "--netlist", net,
                                         "--out", out], capture_output=True,
                                        text=True, timeout=300)
                    self.assertEqual(pr.returncode, 0, pr.stderr)
                    tr = csvdata.load_run(out)
                    self.assertGreater(len(tr.time), 10)


class SnapshotTest(unittest.TestCase):
    """Snapshot path (canvas PostScript + gs PNG). Needs X + ghostscript;
    skipped loudly otherwise (e.g. CI runners)."""

    @classmethod
    def setUpClass(cls):
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            root.destroy()
        except Exception as e:
            raise unittest.SkipTest(f"no display: {e}")
        import shutil
        if shutil.which("gs") is None:
            raise unittest.SkipTest("no ghostscript")

    def test_snapshot_renders_zoom(self):
        import wview
        with tempfile.TemporaryDirectory() as d:
            csv_path = os.path.join(d, "run.csv")
            with open(csv_path, "w") as f:
                f.write("time,v:1\n0,0\n1e-6,5\n2e-6,5\n")
            app = wview.App(preload=csv_path)
            try:
                app.update_idletasks()
                app.update()
                app.cv.x0, app.cv.x1 = 0.0, 2e-6
                png = app.snapshot()
                self.assertTrue(png and os.path.isfile(png), "no PNG written")
                self.assertGreater(os.path.getsize(png), 0, "empty PNG")
            finally:
                app.destroy()


if __name__ == "__main__":
    if "--pe" in sys.argv:
        TemplateSimTest.PE = sys.argv[sys.argv.index("--pe") + 1]
        sys.argv = [a for a in sys.argv if a != "--pe"]
    unittest.main(verbosity=1)
