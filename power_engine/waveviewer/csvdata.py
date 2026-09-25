"""CSV data layer for the wave viewer (stdlib only, Tk-free, CI-testable).

Contracts mirrored from the producers:
  `pe run`   -> header: time,<probes sorted>; one row per step.
  `pe sweep` -> header: <params>,<measures>,error; errored rows carry
                a non-empty `error` cell and must never plot.
"""
import csv


class RunTrace:
    """One `pe run` CSV: uniform-access columns of floats."""

    def __init__(self, path, header, cols):
        self.path = path
        self.header = header  # list[str], header[0] == 'time'
        self.cols = cols      # list[list[float]], same order as header

    @property
    def time(self):
        return self.cols[0]

    @property
    def probes(self):
        return self.header[1:]

    def column(self, probe):
        if probe not in self.header[1:]:
            raise KeyError(f"{self.path}: unknown probe {probe!r} (have {self.probes})")
        return self.cols[self.header.index(probe)]

    def downsampled(self, probe, max_pts=4000):
        """Even stride so Canvas stays fast on 100k+ point runs."""
        y = self.column(probe)
        t = self.time
        if len(t) <= max_pts or max_pts < 2:
            return t, y
        stride = -(-len(t) // max_pts)  # ceil div, keeps last point
        idx = list(range(0, len(t), stride))
        if idx[-1] != len(t) - 1:
            idx.append(len(t) - 1)
        return [t[i] for i in idx], [y[i] for i in idx]


def load_run(path):
    """Load a `pe run` CSV. Raises ValueError/KeyError on contract breach."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        raise ValueError(f"{path}: need header + >= 1 data row")
    header = rows[0]
    if not header or header[0] != "time":
        raise ValueError(f"{path}: first column must be 'time', got {header[:1]}")
    for i, row in enumerate(rows[1:], start=2):
        if len(row) != len(header):
            raise ValueError(f"{path}: row {i} has {len(row)} cells, want {len(header)}")
    try:
        cols = [[float(c) for c in row] for row in zip(*rows[1:])]
    except ValueError as e:
        raise ValueError(f"{path}: non-numeric cell: {e}") from e
    if any(len(c) != len(rows) - 1 for c in cols):
        raise ValueError(f"{path}: ragged rows")  # defensive; widths checked above
    return RunTrace(path, header, cols)


def load_sweep(path):
    """Load a `pe sweep` table CSV -> (columns, rows, skipped).

    columns: header names minus the trailing `error` column; rows: list
    of dicts column -> float for successful points only. Errored rows
    are dropped and counted, never plotted.
    """
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path}: no data rows")
    names = list(rows[0])
    if not names or names[-1] != "error":
        raise ValueError(f"{path}: last column must be 'error', got {names[-1:]!r}")
    columns = names[:-1]
    good, skipped = [], 0
    for r in rows:
        if r.get("error"):
            skipped += 1
            continue
        try:
            good.append({k: float(r[k]) for k in columns})
        except ValueError as e:
            raise ValueError(f"{path}: non-numeric cell: {e}") from e
    return columns, good, skipped
