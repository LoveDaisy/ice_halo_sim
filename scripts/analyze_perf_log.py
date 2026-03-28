#!/usr/bin/env python3
"""Lumice performance log analyzer.

Parses GUI/Poller log files, detects three operational phases, and reports
per-phase performance metrics:

  STEADY  — continuous simulation, no config changes (hardware throughput)
  DRAG    — rapid DoRun commits from slider interaction (responsiveness)
  PAUSE   — brief gap during drag session (recovery between gestures)

Usage:
    # Analyze with phase detection
    python scripts/analyze_perf_log.py lumice-mac02.log

    # Compare multiple files
    python scripts/analyze_perf_log.py lumice-mac02.log lumice-win02.log

    # Generate plots (PNG next to log, or -o DIR)
    python scripts/analyze_perf_log.py -p lumice-mac02.log

    # Per-cycle verbose breakdown
    python scripts/analyze_perf_log.py -v lumice-mac02.log

    # Filter time range (seconds from log start)
    python scripts/analyze_perf_log.py --from 2.0 --to 8.0 lumice-mac02.log
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Phase classification: rays-based adaptive thresholds
# ---------------------------------------------------------------------------

# STEADY if max_rays >= this fraction of the global peak.
# Rationale: 20% of peak means the simulation accumulated significantly.
STEADY_FRAC = 0.20

# PAUSE if max_rays > PAUSE_MULT × median(drag-level rays).
# Rationale: 3× the typical drag level means the user paused briefly.
PAUSE_MULT = 3.0


class Phase(Enum):
    STEADY = "STEADY"
    DRAG = "DRAG"
    PAUSE = "PAUSE"

    @property
    def short(self) -> str:
        return {"STEADY": "S", "DRAG": "D", "PAUSE": "P"}[self.value]


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class LogEvent:
    ts: datetime
    level: str
    component: str
    message: str
    ts_sec: float = 0.0


@dataclass
class StagedEvent:
    ts: datetime
    ts_sec: float
    rays: int
    intensity: float
    gen: int


@dataclass
class UploadEvent:
    ts: datetime
    ts_sec: float
    rays: int
    intensity: float
    eff_pixels: int
    factor: float


@dataclass
class SkipEvent:
    ts: datetime
    ts_sec: float
    reason: str  # "poller_busy" | "quality_gate"
    rays: int = 0


@dataclass
class DoRunEvent:
    ts: datetime
    ts_sec: float
    commit_ms: float


@dataclass
class DoStopEvent:
    ts: datetime
    ts_sec: float


@dataclass
class Cycle:
    """Time span from one DoRun to the next (or to end-of-log)."""
    start: DoRunEvent
    uploads: list[UploadEvent] = field(default_factory=list)
    skips_busy: int = 0
    skips_quality: int = 0
    staged_count: int = 0
    max_rays: int = 0
    max_intensity: float = 0.0
    duration_sec: float = 0.0
    phase: Phase = Phase.STEADY
    is_last: bool = False


@dataclass
class DragSession:
    """A contiguous run of DRAG + PAUSE cycles."""
    cycles: list[Cycle] = field(default_factory=list)

    @property
    def start_sec(self) -> float:
        return self.cycles[0].start.ts_sec if self.cycles else 0.0

    @property
    def duration_sec(self) -> float:
        return sum(c.duration_sec for c in self.cycles)

    @property
    def drag_cycles(self) -> list[Cycle]:
        return [c for c in self.cycles if c.phase == Phase.DRAG]

    @property
    def pause_cycles(self) -> list[Cycle]:
        return [c for c in self.cycles if c.phase == Phase.PAUSE]


@dataclass
class ParsedLog:
    filepath: str
    events: list[LogEvent] = field(default_factory=list)
    staged: list[StagedEvent] = field(default_factory=list)
    uploads: list[UploadEvent] = field(default_factory=list)
    skips: list[SkipEvent] = field(default_factory=list)
    doruns: list[DoRunEvent] = field(default_factory=list)
    dostops: list[DoStopEvent] = field(default_factory=list)
    cycles: list[Cycle] = field(default_factory=list)
    drag_sessions: list[DragSession] = field(default_factory=list)

    @property
    def duration_sec(self) -> float:
        if len(self.events) < 2:
            return 0.0
        return self.events[-1].ts_sec - self.events[0].ts_sec

    @property
    def label(self) -> str:
        return Path(self.filepath).stem

    def cycles_by_phase(self, phase: Phase) -> list[Cycle]:
        return [c for c in self.cycles if c.phase == phase]

    def uploads_by_phase(self, phase: Phase) -> list[UploadEvent]:
        result: list[UploadEvent] = []
        for c in self.cycles:
            if c.phase == phase:
                result.extend(c.uploads)
        return result


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

_LOG_RE = re.compile(
    r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}) "
    r"\[([TDVIWEO])\] "
    r"\[(\w+)\] "
    r"(.+)$"
)
_STAGED_RE = re.compile(
    r"staged: rays=(\d+) intensity=([\d.]+(?:e[+-]?\d+)?) gen=(\d+)"
)
_UPLOAD_RE = re.compile(
    r"SyncFromPoller: (?:upload (?:tex_)?|texture \d+x\d+, )"
    r"rays=(\d+), "
    r"intensity=([\d.]+(?:e[+-]?\d+)?), "
    r"(?:eff_pixels=(\d+), )?"
    r"factor=([\d.]+(?:e[+-]?\d+)?)"
)
_SKIP_BUSY_RE = re.compile(r"SyncFromPoller: skipped \(poller busy\)")
_QUALITY_GATE_RE = re.compile(
    r"quality gate: skipped rays=(\d+) \(min=(\d+)\) gen=(\d+)"
)
_DORUN_RE = re.compile(r"DoRun: config committed \(([\d.]+)ms\)")
_DOSTOP_RE = re.compile(r"DoStop")


def parse_log(filepath: str, t_from: float = 0.0, t_to: float = float("inf")) -> ParsedLog:
    result = ParsedLog(filepath=filepath)
    base_ts: Optional[datetime] = None

    with open(filepath) as f:
        for line in f:
            line = line.rstrip()
            m = _LOG_RE.match(line)
            if not m:
                continue
            ts = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f")
            if base_ts is None:
                base_ts = ts
            ts_sec = (ts - base_ts).total_seconds()
            if ts_sec < t_from or ts_sec > t_to:
                continue

            ev = LogEvent(ts=ts, level=m.group(2), component=m.group(3),
                          message=m.group(4), ts_sec=ts_sec)
            result.events.append(ev)
            msg = ev.message

            sm = _STAGED_RE.search(msg)
            if sm:
                result.staged.append(StagedEvent(
                    ts=ts, ts_sec=ts_sec, rays=int(sm.group(1)),
                    intensity=float(sm.group(2)), gen=int(sm.group(3))))
                continue
            um = _UPLOAD_RE.search(msg)
            if um:
                result.uploads.append(UploadEvent(
                    ts=ts, ts_sec=ts_sec, rays=int(um.group(1)),
                    intensity=float(um.group(2)),
                    eff_pixels=int(um.group(3)) if um.group(3) else 0,
                    factor=float(um.group(4))))
                continue
            if _SKIP_BUSY_RE.search(msg):
                result.skips.append(SkipEvent(ts=ts, ts_sec=ts_sec, reason="poller_busy"))
                continue
            qm = _QUALITY_GATE_RE.search(msg)
            if qm:
                result.skips.append(SkipEvent(
                    ts=ts, ts_sec=ts_sec, reason="quality_gate", rays=int(qm.group(1))))
                continue
            dm = _DORUN_RE.search(msg)
            if dm:
                result.doruns.append(DoRunEvent(
                    ts=ts, ts_sec=ts_sec, commit_ms=float(dm.group(1))))
                continue
            if _DOSTOP_RE.search(msg):
                result.dostops.append(DoStopEvent(ts=ts, ts_sec=ts_sec))

    _build_cycles(result)
    _classify_phases(result)
    _build_drag_sessions(result)
    return result


def _build_cycles(log: ParsedLog) -> None:
    if not log.doruns:
        return
    boundaries = [(d.ts_sec, d) for d in log.doruns]
    for i, (start_t, dorun) in enumerate(boundaries):
        end_t = boundaries[i + 1][0] if i + 1 < len(boundaries) else float("inf")
        cycle = Cycle(start=dorun)
        for u in log.uploads:
            if start_t <= u.ts_sec < end_t:
                cycle.uploads.append(u)
                cycle.max_rays = max(cycle.max_rays, u.rays)
                cycle.max_intensity = max(cycle.max_intensity, u.intensity)
        for s in log.skips:
            if start_t <= s.ts_sec < end_t:
                if s.reason == "poller_busy":
                    cycle.skips_busy += 1
                else:
                    cycle.skips_quality += 1
        for st in log.staged:
            if start_t <= st.ts_sec < end_t:
                cycle.staged_count += 1
        if i + 1 < len(boundaries):
            cycle.duration_sec = end_t - start_t
        elif log.events:
            cycle.duration_sec = log.events[-1].ts_sec - start_t
        cycle.is_last = (i == len(boundaries) - 1)
        log.cycles.append(cycle)


def _classify_phases(log: ParsedLog) -> None:
    """Classify cycles by rays pattern (adaptive, platform-independent).

    Uses max_rays per cycle relative to the global peak:
      - rays steadily increasing to high peak  → STEADY
      - rays reset to low values repeatedly     → DRAG
      - occasional higher rays during drag      → PAUSE
    """
    if not log.cycles:
        return

    all_max = [c.max_rays for c in log.cycles]
    peak = max(all_max) if all_max else 0
    if peak == 0:
        for c in log.cycles:
            c.phase = Phase.DRAG
        return

    steady_threshold = peak * STEADY_FRAC

    # Compute drag baseline from non-steady, non-zero cycles
    drag_candidates = [r for r in all_max if 0 < r < steady_threshold]
    if drag_candidates:
        drag_med = _median(drag_candidates)
        pause_threshold = drag_med * PAUSE_MULT
    else:
        # No drag cycles found — everything is steady or zero-upload
        pause_threshold = steady_threshold

    for c in log.cycles:
        if c.max_rays >= steady_threshold:
            c.phase = Phase.STEADY
        elif c.max_rays > pause_threshold:
            c.phase = Phase.PAUSE
        else:
            c.phase = Phase.DRAG


def _build_drag_sessions(log: ParsedLog) -> None:
    current: Optional[DragSession] = None
    for c in log.cycles:
        if c.phase in (Phase.DRAG, Phase.PAUSE):
            if current is None:
                current = DragSession()
            current.cycles.append(c)
        else:
            if current is not None:
                log.drag_sessions.append(current)
                current = None
    if current is not None:
        log.drag_sessions.append(current)


# ---------------------------------------------------------------------------
# Stats helpers
# ---------------------------------------------------------------------------

def _mean(xs: list[float]) -> float:
    return sum(xs) / len(xs) if xs else 0.0

def _median(xs: list[float]) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 == 1 else (s[n // 2 - 1] + s[n // 2]) / 2

def _percentile(xs: list[float], p: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    k = (len(s) - 1) * p / 100
    f = int(k)
    c = f + 1 if f + 1 < len(s) else f
    return s[f] + (k - f) * (s[c] - s[f])

def _stddev(xs: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return (sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) ** 0.5

def _cv(xs: list[float]) -> float:
    m = _mean(xs)
    return _stddev(xs) / m * 100 if m > 0 else 0.0

def _pct(a: int, b: int) -> str:
    return f"{a/b*100:.1f}%" if b else "N/A"

def _drag_session_spans(log: ParsedLog) -> list[tuple[float, float]]:
    """Contiguous DRAG time spans (merging adjacent DRAG cycles, excluding PAUSE/STEADY gaps)."""
    spans: list[tuple[float, float]] = []
    for ds in log.drag_sessions:
        # Each drag session may contain PAUSE cycles; build sub-spans of only DRAG runs
        run_start: Optional[float] = None
        run_end: float = 0.0
        for c in ds.cycles:
            if c.phase == Phase.DRAG:
                if run_start is None:
                    run_start = c.start.ts_sec
                run_end = c.start.ts_sec + c.duration_sec
            else:
                if run_start is not None:
                    spans.append((run_start, run_end))
                    run_start = None
        if run_start is not None:
            spans.append((run_start, run_end))
    return spans


def _within_any_span(t1: float, t2: float, spans: list[tuple[float, float]]) -> bool:
    """True if both timestamps fall within the same span."""
    for lo, hi in spans:
        if lo <= t1 < hi and lo <= t2 < hi:
            return True
    return False


def _fmt_rays(r: float) -> str:
    if r >= 1e6:
        return f"{r/1e6:.2f}M"
    if r >= 1e3:
        return f"{r/1e3:.1f}K"
    return f"{r:.0f}"


# ---------------------------------------------------------------------------
# Summary output
# ---------------------------------------------------------------------------

def _print_phase_bar(log: ParsedLog) -> None:
    """Compact visual: one char per cycle showing phase."""
    if not log.cycles:
        return
    chars = [c.phase.short for c in log.cycles]
    print(f"\n  Phase timeline: [{''.join(chars)}]")
    ns = sum(1 for c in chars if c == "S")
    nd = sum(1 for c in chars if c == "D")
    np_ = sum(1 for c in chars if c == "P")
    print(f"  S={ns} steady  D={nd} drag  P={np_} pause  ({len(chars)} cycles total)")

    # Show adaptive thresholds
    all_max = [c.max_rays for c in log.cycles]
    peak = max(all_max) if all_max else 0
    if peak > 0:
        steady_th = peak * STEADY_FRAC
        drag_cands = [r for r in all_max if 0 < r < steady_th]
        if drag_cands:
            drag_med = _median(drag_cands)
            pause_th = drag_med * PAUSE_MULT
        else:
            drag_med = 0
            pause_th = steady_th
        print(f"  Thresholds: peak={_fmt_rays(peak)}  "
              f"steady>{_fmt_rays(steady_th)}  "
              f"pause>{_fmt_rays(pause_th)}  "
              f"(drag median={_fmt_rays(drag_med)})")


def _print_steady_stats(log: ParsedLog) -> None:
    steady = log.cycles_by_phase(Phase.STEADY)
    if not steady:
        return
    uploads = log.uploads_by_phase(Phase.STEADY)

    total_dur = sum(c.duration_sec for c in steady)
    peak_rays = max((c.max_rays for c in steady), default=0)
    peak_intensity = max((c.max_intensity for c in steady), default=0.0)
    n_uploads = len(uploads)
    n_busy = sum(c.skips_busy for c in steady)

    print(f"\n--- STEADY  ({len(steady)} segments, {total_dur:.1f}s) ---")
    print(f"  Reflects: hardware throughput + thread utilization")
    if total_dur > 0 and peak_rays > 0:
        # Estimate rays/sec from the longest steady cycle (most stable measurement)
        longest = max(steady, key=lambda c: c.duration_sec)
        if longest.duration_sec > 0 and longest.max_rays > 0:
            rays_per_sec = longest.max_rays / longest.duration_sec
            print(f"  Throughput:       {_fmt_rays(rays_per_sec)}/s  "
                  f"(from {longest.duration_sec:.1f}s segment, peak {_fmt_rays(longest.max_rays)} rays)")
    print(f"  Peak rays:        {peak_rays:,}")
    print(f"  Peak intensity:   {peak_intensity:,.1f}")
    print(f"  Uploads:          {n_uploads}")
    print(f"  Busy skips:       {n_busy}  ({_pct(n_busy, n_uploads + n_busy)})")
    if n_uploads > 1:
        intervals = []
        for c in steady:
            for i in range(1, len(c.uploads)):
                intervals.append((c.uploads[i].ts_sec - c.uploads[i - 1].ts_sec) * 1000)
        if intervals:
            print(f"  Upload interval:  mean {_mean(intervals):.0f}ms  "
                  f"median {_median(intervals):.0f}ms  P95 {_percentile(intervals, 95):.0f}ms")


def _print_drag_stats(log: ParsedLog) -> None:
    drag = log.cycles_by_phase(Phase.DRAG)
    if not drag:
        return
    uploads = log.uploads_by_phase(Phase.DRAG)

    total_dur = sum(c.duration_sec for c in drag)
    n_cycles = len(drag)
    n_uploads = len(uploads)
    n_busy = sum(c.skips_busy for c in drag)
    n_quality = sum(c.skips_quality for c in drag)

    upload_rays = [u.rays for u in uploads]
    commit_ms = [c.start.commit_ms for c in drag]

    n_pause = len(log.cycles_by_phase(Phase.PAUSE))
    pause_note = f"  ({n_pause} pause gaps filtered out)" if n_pause else ""
    print(f"\n--- DRAG  ({n_cycles} cycles, {total_dur:.1f}s{pause_note}) ---")
    print(f"  Reflects: interaction responsiveness + timing/contention")

    # Uploads per cycle
    ups_per_cycle = n_uploads / n_cycles if n_cycles else 0
    print(f"  Uploads/cycle:    {ups_per_cycle:.2f}")
    print(f"  Total uploads:    {n_uploads}")
    print(f"  Busy skips:       {n_busy}  ({_pct(n_busy, n_uploads + n_busy)})")
    print(f"  Quality gate:     {n_quality} skipped")

    # Quality gate leak check
    gate_leaks = [u for u in uploads if u.rays < 5000]
    if gate_leaks:
        print(f"  ** Gate leaks:    {len(gate_leaks)} uploads with rays < 5000  "
              f"(min={min(u.rays for u in gate_leaks)})")

    # Commit latency
    if commit_ms:
        print(f"  Commit latency:   mean {_mean(commit_ms):.1f}ms  "
              f"median {_median(commit_ms):.1f}ms  P95 {_percentile(commit_ms, 95):.1f}ms  "
              f"max {max(commit_ms):.1f}ms")

    # First-upload delay: time from DoRun commit to first successful upload in each cycle
    first_upload_ms = []
    for c in drag:
        if c.uploads:
            delay = (c.uploads[0].ts_sec - c.start.ts_sec) * 1000
            first_upload_ms.append(delay)
    if first_upload_ms:
        n_no_upload = n_cycles - len(first_upload_ms)
        miss_note = f"  ({n_no_upload} cycles had no upload)" if n_no_upload else ""
        print(f"  First upload:     mean {_mean(first_upload_ms):.0f}ms  "
              f"median {_median(first_upload_ms):.0f}ms  P95 {_percentile(first_upload_ms, 95):.0f}ms  "
              f"max {max(first_upload_ms):.0f}ms{miss_note}")

    # First-upload decomposition: commit_latency + gate_wait
    if first_upload_ms and commit_ms:
        gate_wait_ms = []
        for c in drag:
            if c.uploads:
                total = (c.uploads[0].ts_sec - c.start.ts_sec) * 1000
                gw = total - c.start.commit_ms
                gate_wait_ms.append(gw)
        if gate_wait_ms:
            cm_of_uploaded = [c.start.commit_ms for c in drag if c.uploads]
            print(f"  1st upload split:  commit {_mean(cm_of_uploaded):.1f}ms"
                  f" + gate_wait {_mean(gate_wait_ms):.0f}ms"
                  f" = total {_mean(first_upload_ms):.0f}ms"
                  f"  (commit {_mean(cm_of_uploaded) / _mean(first_upload_ms) * 100:.0f}%"
                  f" / gate {_mean(gate_wait_ms) / _mean(first_upload_ms) * 100:.0f}%)")

    # Inter-DoRun interval (only between adjacent drag cycles — skip gaps)
    drag_set = set(id(c) for c in drag)
    all_cycles = log.cycles
    inter = []
    for i in range(1, len(all_cycles)):
        if id(all_cycles[i]) in drag_set and id(all_cycles[i - 1]) in drag_set:
            inter.append(all_cycles[i].start.ts_sec - all_cycles[i - 1].start.ts_sec)
    if inter:
        inter_ms = [x * 1000 for x in inter]
        print(f"  DoRun interval:   mean {_mean(inter_ms):.0f}ms  "
              f"median {_median(inter_ms):.0f}ms")

    # Upload rays distribution
    if upload_rays:
        print(f"  Upload rays:      mean {_fmt_rays(_mean(upload_rays))}  "
              f"median {_fmt_rays(_median(upload_rays))}  "
              f"min {_fmt_rays(min(upload_rays))}  "
              f"CV {_cv(upload_rays):.1f}%")

    # Upload interval within drag
    # Include intervals between consecutive DRAG uploads even across cycle boundaries,
    # but exclude gaps that cross a STEADY or PAUSE segment.
    if len(uploads) > 1:
        drag_spans = _drag_session_spans(log)
        ui = [(uploads[i].ts_sec - uploads[i - 1].ts_sec) * 1000
              for i in range(1, len(uploads))
              if _within_any_span(uploads[i - 1].ts_sec, uploads[i].ts_sec, drag_spans)]
        if ui:
            print(f"  Upload interval:  mean {_mean(ui):.0f}ms  "
                  f"median {_median(ui):.0f}ms  P95 {_percentile(ui, 95):.0f}ms")


def _print_drag_sessions(log: ParsedLog) -> None:
    if not log.drag_sessions:
        return
    print(f"\n--- Drag Sessions ({len(log.drag_sessions)}) ---")
    print(f"  {'#':>3}  {'start':>6}  {'dur':>6}  {'cycles':>6}  "
          f"{'D/P':>5}  {'uploads':>7}  {'ups/cyc':>7}  {'busy':>5}  {'qgate':>5}")
    print(f"  {'':->3}  {'':->6}  {'':->6}  {'':->6}  "
          f"{'':->5}  {'':->7}  {'':->7}  {'':->5}  {'':->5}")
    for i, ds in enumerate(log.drag_sessions):
        nd = len(ds.drag_cycles)
        np_ = len(ds.pause_cycles)
        nu = sum(len(c.uploads) for c in ds.cycles)
        nb = sum(c.skips_busy for c in ds.cycles)
        nq = sum(c.skips_quality for c in ds.cycles)
        nc = len(ds.cycles)
        upc = nu / nc if nc else 0
        print(f"  {i:3d}  {ds.start_sec:5.1f}s  {ds.duration_sec:5.1f}s  {nc:6d}  "
              f"{nd:2d}/{np_:<2d}  {nu:7d}  {upc:7.2f}  {nb:5d}  {nq:5d}")


def print_summary(log: ParsedLog) -> None:
    dur = log.duration_sec
    print(f"\n{'='*70}")
    print(f"  {log.label}")
    print(f"  {log.filepath}")
    print(f"{'='*70}")
    print(f"\n  Duration: {dur:.2f}s  |  Events: {len(log.events)}  |  "
          f"DoRuns: {len(log.doruns)}  |  Uploads: {len(log.uploads)}")

    _print_phase_bar(log)
    _print_steady_stats(log)
    _print_drag_stats(log)


def print_cycles(log: ParsedLog) -> None:
    if not log.cycles:
        return
    _print_drag_sessions(log)
    print(f"\n--- Per-Cycle Detail ({len(log.cycles)} cycles) ---")
    print(f"  {'#':>4}  {'ph':>2}  {'t(s)':>6}  {'dur(ms)':>8}  {'commit':>8}  "
          f"{'uploads':>7}  {'busy':>5}  {'qgate':>5}  {'max_rays':>10}")
    print(f"  {'':->4}  {'':->2}  {'':->6}  {'':->8}  {'':->8}  "
          f"{'':->7}  {'':->5}  {'':->5}  {'':->10}")
    for i, c in enumerate(log.cycles):
        print(f"  {i:4d}  {c.phase.short:>2}  {c.start.ts_sec:6.2f}  "
              f"{c.duration_sec*1000:8.1f}  {c.start.commit_ms:7.1f}ms  "
              f"{len(c.uploads):7d}  {c.skips_busy:5d}  "
              f"{c.skips_quality:5d}  {c.max_rays:10,}")


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

def print_comparison(logs: list[ParsedLog]) -> None:
    print(f"\n{'='*100}")
    print(f"  Comparison ({len(logs)} files)")
    print(f"{'='*100}")

    headers = ["Metric"] + [log.label for log in logs]
    rows: list[list[str]] = []

    def add(name: str, vals: list[str]) -> None:
        rows.append([name] + vals)

    add("Duration (s)", [f"{l.duration_sec:.1f}" for l in logs])

    # Phase distribution
    for ph in Phase:
        cycles = [l.cycles_by_phase(ph) for l in logs]
        counts = [len(c) for c in cycles]
        durs = [sum(x.duration_sec for x in c) for c in cycles]
        add(f"{ph.value} cycles", [f"{n} ({d:.1f}s)" for n, d in zip(counts, durs)])

    add("", ["" for _ in logs])  # separator

    # STEADY metrics
    def steady_throughput(l: ParsedLog) -> str:
        steady = l.cycles_by_phase(Phase.STEADY)
        if not steady:
            return "N/A"
        longest = max(steady, key=lambda c: c.duration_sec)
        if longest.duration_sec > 0 and longest.max_rays > 0:
            return f"{_fmt_rays(longest.max_rays / longest.duration_sec)}/s"
        return "N/A"

    add("STEADY throughput", [steady_throughput(l) for l in logs])
    add("STEADY peak rays", [
        f"{max((c.max_rays for c in l.cycles_by_phase(Phase.STEADY)), default=0):,}"
        for l in logs
    ])

    add("", ["" for _ in logs])

    # DRAG metrics
    def drag_uploads_per_cycle(l: ParsedLog) -> str:
        drag = l.cycles_by_phase(Phase.DRAG)
        if not drag:
            return "N/A"
        n = sum(len(c.uploads) for c in drag)
        return f"{n/len(drag):.2f}"

    def drag_upload_ratio(l: ParsedLog) -> str:
        drag = l.cycles_by_phase(Phase.DRAG)
        if not drag:
            return "N/A"
        nu = sum(len(c.uploads) for c in drag)
        nb = sum(c.skips_busy for c in drag)
        return _pct(nu, nu + nb)

    def drag_commit_ms(l: ParsedLog) -> str:
        drag = l.cycles_by_phase(Phase.DRAG)
        if not drag:
            return "N/A"
        ms = [c.start.commit_ms for c in drag]
        return f"{_mean(ms):.1f} / {_percentile(ms, 95):.1f}"

    def drag_upload_cv(l: ParsedLog) -> str:
        uploads = l.uploads_by_phase(Phase.DRAG)
        if len(uploads) < 2:
            return "N/A"
        rays = [u.rays for u in uploads]
        return f"{_cv(rays):.1f}%"

    def drag_upload_rays_mean(l: ParsedLog) -> str:
        uploads = l.uploads_by_phase(Phase.DRAG)
        if not uploads:
            return "N/A"
        return _fmt_rays(_mean([u.rays for u in uploads]))

    def drag_gate_leaks(l: ParsedLog) -> str:
        uploads = l.uploads_by_phase(Phase.DRAG)
        leaks = [u for u in uploads if u.rays < 5000]
        return str(len(leaks)) if leaks else "0"

    def drag_first_upload(l: ParsedLog) -> str:
        drag = l.cycles_by_phase(Phase.DRAG)
        if not drag:
            return "N/A"
        delays = [(c.uploads[0].ts_sec - c.start.ts_sec) * 1000 for c in drag if c.uploads]
        if not delays:
            return "N/A"
        return f"{_mean(delays):.0f} / {_percentile(delays, 95):.0f}"

    def drag_1st_upload_split(l: ParsedLog) -> str:
        drag = l.cycles_by_phase(Phase.DRAG)
        if not drag:
            return "N/A"
        cm_vals = []
        gw_vals = []
        for c in drag:
            if c.uploads:
                total = (c.uploads[0].ts_sec - c.start.ts_sec) * 1000
                cm_vals.append(c.start.commit_ms)
                gw_vals.append(total - c.start.commit_ms)
        if not cm_vals:
            return "N/A"
        return f"{_mean(cm_vals):.0f} + {_mean(gw_vals):.0f}"

    add("DRAG uploads/cycle", [drag_uploads_per_cycle(l) for l in logs])
    add("DRAG upload ratio", [drag_upload_ratio(l) for l in logs])
    add("DRAG commit (mean/P95)", [drag_commit_ms(l) for l in logs])
    add("DRAG 1st upload (mean/P95)", [drag_first_upload(l) for l in logs])
    add("DRAG 1st split (cm+gw)", [drag_1st_upload_split(l) for l in logs])
    add("DRAG rays mean", [drag_upload_rays_mean(l) for l in logs])
    add("DRAG rays CV", [drag_upload_cv(l) for l in logs])
    add("DRAG gate leaks", [drag_gate_leaks(l) for l in logs])

    add("DRAG pause gaps", [str(len(l.cycles_by_phase(Phase.PAUSE))) for l in logs])

    # Print table
    col_widths = [max(len(r[i]) for r in [headers] + rows) for i in range(len(headers))]
    fmt = "  ".join(f"{{:<{w}}}" if i == 0 else f"{{:>{w}}}" for i, w in enumerate(col_widths))
    print()
    print("  " + fmt.format(*headers))
    print("  " + "  ".join("-" * w for w in col_widths))
    for row in rows:
        if all(v == "" for v in row):
            print()
        else:
            print("  " + fmt.format(*row))


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _phase_colors():
    return {Phase.STEADY: "#4CAF50", Phase.DRAG: "#2196F3", Phase.PAUSE: "#FF9800"}


def plot_timelines(log: ParsedLog, output_dir: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter
        from matplotlib.patches import Patch
    except ImportError:
        print("  [!] matplotlib not found. Install: pip install matplotlib")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True,
                             gridspec_kw={"height_ratios": [0.4, 1.2, 1, 1]})
    fig.suptitle(f"Performance Timeline: {log.label}", fontsize=13, fontweight="bold")
    pc = _phase_colors()

    def _rays_formatter(x: float, _: object) -> str:
        if x >= 1e6:
            return f"{x/1e6:.0f}M"
        if x >= 1e3:
            return f"{x/1e3:.0f}K"
        return f"{x:.0f}" if x >= 1 else ""

    # --- Panel 0: Phase bar ---
    ax = axes[0]
    for c in log.cycles:
        ax.barh(0, c.duration_sec, left=c.start.ts_sec, height=0.8,
                color=pc[c.phase], alpha=0.8, edgecolor="white", linewidth=0.3)
    ax.set_yticks([])
    ax.set_ylabel("Phase")
    ax.legend(handles=[Patch(facecolor=pc[p], label=p.value) for p in Phase],
              loc="upper right", fontsize=7, ncol=3)
    ax.set_xlim(0, log.duration_sec * 1.02)

    # --- Panel 1: Upload rays (log scale) ---
    ax = axes[1]
    for phase in Phase:
        ups = log.uploads_by_phase(phase)
        if ups:
            ax.scatter([u.ts_sec for u in ups], [u.rays for u in ups],
                       s=10, alpha=0.6, color=pc[phase], zorder=2, label=phase.value)
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(FuncFormatter(_rays_formatter))
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.set_ylabel("Upload rays")
    ax.grid(True, alpha=0.3, which="major")
    ax.grid(True, alpha=0.1, which="minor")
    ax.legend(fontsize=7, loc="upper right")

    # --- Panel 2: Upload interval ---
    ax = axes[2]
    if len(log.uploads) > 1:
        ts = [log.uploads[i].ts_sec for i in range(1, len(log.uploads))]
        intervals = [(log.uploads[i].ts_sec - log.uploads[i - 1].ts_sec) * 1000
                     for i in range(1, len(log.uploads))]
        colors = []
        for i in range(1, len(log.uploads)):
            u = log.uploads[i]
            ph = Phase.STEADY
            for c in log.cycles:
                if c.start.ts_sec <= u.ts_sec < c.start.ts_sec + c.duration_sec:
                    ph = c.phase
                    break
            colors.append(pc[ph])
        ax.scatter(ts, intervals, s=8, alpha=0.6, c=colors, zorder=2)
    ax.set_ylabel("Upload interval (ms)")
    ax.grid(True, alpha=0.3)

    # --- Panel 3: Event density ---
    ax = axes[3]
    if log.events:
        bin_w = 0.1
        max_t = log.events[-1].ts_sec
        bins = int(max_t / bin_w) + 1
        upload_h = [0] * bins
        busy_h = [0] * bins
        quality_h = [0] * bins
        for u in log.uploads:
            upload_h[min(int(u.ts_sec / bin_w), bins - 1)] += 1
        for s in log.skips:
            idx = min(int(s.ts_sec / bin_w), bins - 1)
            if s.reason == "poller_busy":
                busy_h[idx] += 1
            else:
                quality_h[idx] += 1
        xs = [i * bin_w for i in range(bins)]
        ax.bar(xs, upload_h, width=bin_w * 0.9, color="#2196F3", alpha=0.7, label="upload")
        ax.bar(xs, busy_h, width=bin_w * 0.9, bottom=upload_h,
               color="#FF5722", alpha=0.7, label="busy skip")
        comb = [a + b for a, b in zip(upload_h, busy_h)]
        ax.bar(xs, quality_h, width=bin_w * 0.9, bottom=comb,
               color="#FF9800", alpha=0.7, label="quality skip")
        ax.legend(loc="upper right", fontsize=7)
    ax.set_ylabel("Events / 100ms")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(output_dir, f"{log.label}_perf.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  [+] Plot saved: {out_path}")


def plot_comparison(logs: list[ParsedLog], output_dir: str) -> None:
    if len(logs) < 2:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f"Comparison: {' vs '.join(l.label for l in logs)}", fontsize=13, fontweight="bold")
    colors = plt.cm.tab10.colors

    # --- DRAG upload rays CDF ---
    ax = axes[0][0]
    for i, log in enumerate(logs):
        rays = sorted(u.rays for u in log.uploads_by_phase(Phase.DRAG))
        if rays:
            cdf = [(j + 1) / len(rays) for j in range(len(rays))]
            ax.plot(rays, cdf, label=log.label, color=colors[i % len(colors)])
    ax.set_xlabel("Upload rays")
    ax.set_ylabel("CDF")
    ax.set_title("DRAG: Upload Rays")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- DRAG upload interval CDF ---
    ax = axes[0][1]
    for i, log in enumerate(logs):
        uploads = log.uploads_by_phase(Phase.DRAG)
        if len(uploads) > 1:
            spans = _drag_session_spans(log)
            intervals = sorted(
                (uploads[j].ts_sec - uploads[j - 1].ts_sec) * 1000
                for j in range(1, len(uploads))
                if _within_any_span(uploads[j - 1].ts_sec, uploads[j].ts_sec, spans)
            )
            if intervals:
                cdf = [(j + 1) / len(intervals) for j in range(len(intervals))]
                ax.plot(intervals, cdf, label=log.label, color=colors[i % len(colors)])
    ax.set_xlabel("Upload interval (ms)")
    ax.set_ylabel("CDF")
    ax.set_title("DRAG: Upload Interval")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Commit latency CDF (drag only) ---
    ax = axes[1][0]
    for i, log in enumerate(logs):
        ct = sorted(c.start.commit_ms for c in log.cycles_by_phase(Phase.DRAG))
        if ct:
            cdf = [(j + 1) / len(ct) for j in range(len(ct))]
            ax.plot(ct, cdf, label=log.label, color=colors[i % len(colors)])
    ax.set_xlabel("Commit latency (ms)")
    ax.set_ylabel("CDF")
    ax.set_title("DRAG: Commit Latency")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Phase time breakdown stacked bar ---
    ax = axes[1][1]
    pc = _phase_colors()
    labels = [l.label for l in logs]
    x = range(len(logs))
    bottoms = [0.0] * len(logs)
    for phase in Phase:
        durs = [sum(c.duration_sec for c in l.cycles_by_phase(phase)) for l in logs]
        ax.bar(x, durs, bottom=bottoms, label=phase.value, color=pc[phase], alpha=0.8)
        bottoms = [b + d for b, d in zip(bottoms, durs)]
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("Time (s)")
    ax.set_title("Phase Distribution")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    out_path = os.path.join(output_dir, "comparison_perf.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"\n  [+] Comparison plot saved: {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Lumice performance log analyzer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("files", nargs="+", help="Log file(s) to analyze")
    parser.add_argument("-p", "--plot", action="store_true", help="Generate timeline plots (PNG)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Per-cycle breakdown + drag sessions")
    parser.add_argument("-o", "--output-dir", default=None,
                        help="Output directory for plots (default: next to log file)")
    parser.add_argument("--from", dest="t_from", type=float, default=0.0,
                        help="Start time in seconds from log start")
    parser.add_argument("--to", dest="t_to", type=float, default=float("inf"),
                        help="End time in seconds from log start")

    args = parser.parse_args()
    logs: list[ParsedLog] = []
    for filepath in args.files:
        if not os.path.isfile(filepath):
            print(f"  [!] File not found: {filepath}", file=sys.stderr)
            continue
        log = parse_log(filepath, t_from=args.t_from, t_to=args.t_to)
        logs.append(log)
        print_summary(log)
        if args.verbose:
            print_cycles(log)

    if len(logs) > 1:
        print_comparison(logs)

    if args.plot and logs:
        for log in logs:
            out_dir = args.output_dir or os.path.dirname(os.path.abspath(log.filepath))
            os.makedirs(out_dir, exist_ok=True)
            plot_timelines(log, out_dir)
        if len(logs) > 1:
            out_dir = args.output_dir or os.path.dirname(os.path.abspath(logs[0].filepath))
            plot_comparison(logs, out_dir)


if __name__ == "__main__":
    main()
