"""The `analyze` subcommand, end to end through the static CLI (`Lumice analyze`).

The scene is the 22° halo of ``test/e2e/configs/raypath_analysis_halo_22.json`` (200k rays,
sun at altitude 20°) — the same scene, seed and thresholds ``test_raypath_analysis_capi.py``
pins through ctypes on the shared library, read here off the CSV the CLI prints. This file is
the fast (static-build) half of that coverage: everything below runs under bare ``pytest``.

What is pinned:
  * the option surface — the three ROIs run and print a CSV; the four option conflicts the
    subcommand refuses (a cone without its geometry, geometry under another ROI, a frame id
    under another ROI, an unknown symmetry) exit non-zero with a message naming the option,
    and a document the engine cannot load fails on the load, before `--roi frame` reads it;
  * the physics, through the CSV — the top chain of the whole sky is ``3-5``; a 2° cone on
    the ring (sun altitude + 23°) leads with ``3-5``; the same cone on halo-free sky (85°)
    leads with far less energy (the same 10x floor as the ctypes test);
  * the output contract — stdout is the CSV and nothing else, progress is stderr's, and with
    ``--csv`` stdout is empty;
  * the read-time symmetry — ``none`` and ``PBD`` of one seeded run report one total and the
    finer read never has fewer rows;
  * the infinite budget — a scene whose ray_num is ``"infinite"`` runs until SIGINT, the
    ``--csv`` file is complete at every rewrite, and the interrupted run still exits 0 having
    written its last result.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

from test.e2e.base import LumiceTestCase
from test.e2e.runner import find_lumice_binary, get_project_root

CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"
_CONFIG = CONFIGS_DIR / "raypath_analysis_halo_22.json"

# The same seed as test_raypath_analysis_capi.py: a seeded run is single-worker and
# deterministic, so two invocations of one question are the same run and their numbers compare
# exactly, not statistically.
_SEED = "20260911"
_SUN_ALTITUDE_DEG = 20.0
# Where the 22° halo's energy sits, straight above the sun (the ring's minimum deviation is
# 21.8°; +23° lands on the bright edge); and above every arc a randomly oriented prism makes.
_ON_RING = f"{_SUN_ALTITUDE_DEG + 23.0:.0f},0"
_OFF_RING = "85,0"
_HALO_22 = "3-5"


def _parse_csv(text: str):
    """The CSV as (head dict, rows): `# key: value` lines, then the column header, then rows
    as lists of the four cells. Asserts the shape on the way, since every test below reads
    through this."""
    lines = text.splitlines()
    assert lines and lines[0] == "# Lumice raypath analysis", text[:200]
    head = {}
    i = 1
    while i < len(lines) and lines[i].startswith("# "):
        key, _, value = lines[i][2:].partition(": ")
        head[key] = value
        i += 1
    assert i < len(lines) and lines[i] == "Raypath,Energy,Cumulative %,+/-", lines[i] if i < len(lines) else "(no header)"
    rows = []
    for line in lines[i + 1:]:
        cells = line.split(",")
        assert len(cells) == 4, line
        rows.append(cells)
    return head, rows


def _top_energy(head, rows) -> float:
    """The leading row's absolute energy: its share times the run's total."""
    return float(rows[0][1]) / 100.0 * float(head["total_energy"])


class TestAnalyzeCli(LumiceTestCase):
    def _analyze(self, *args, timeout=180):
        return self.run_lumice(["analyze", "-f", str(_CONFIG), "--seed", _SEED, *args], timeout=timeout)

    # ---- AC1: the three ROIs run; the four conflicts are refused ----

    def test_whole_sky_prints_the_csv_with_the_22_degree_path_on_top(self):
        result = self._analyze()
        self.assertEqual(result.returncode, 0, result.stderr)
        head, rows = _parse_csv(result.stdout)
        self.assertEqual(head["region"], "whole sky")
        self.assertEqual(head["symmetry"], "P|B|D")
        self.assertEqual(head["record_full_hits"], "0")
        self.assertGreater(int(head["total_rays"]), 0)
        self.assertGreaterEqual(len(rows), 2)
        self.assertEqual(rows[0][0], _HALO_22, [r[0] for r in rows[:5]])
        # The undeviated pass through opposite prism faces is #2, at roughly 80% of the halo
        # (the ctypes test measures 1.24-1.27x over three seeds; 1.1x is the same floor).
        self.assertEqual(rows[1][0], "3-6")
        self.assertGreater(float(rows[0][1]), 1.1 * float(rows[1][1]))
        # Energy descending, cumulative monotone, closing at 100 (no "other" bucket here).
        energies = [float(r[1]) for r in rows]
        self.assertEqual(energies, sorted(energies, reverse=True))
        cumulative = [float(r[2]) for r in rows]
        self.assertEqual(cumulative, sorted(cumulative))
        self.assertAlmostEqual(cumulative[-1], 100.0, places=2)

    def test_cone_on_and_off_the_ring(self):
        on_ring = self._analyze("--roi", "cone", "--center", _ON_RING, "--radius", "2")
        self.assertEqual(on_ring.returncode, 0, on_ring.stderr)
        head_on, rows_on = _parse_csv(on_ring.stdout)
        self.assertEqual(head_on["region"], "point")
        self.assertEqual(head_on["cone_centre_altitude_deg"], "43.00")
        self.assertEqual(head_on["cone_centre_azimuth_deg"], "0.00")
        self.assertEqual(head_on["cone_request_radius_deg"], "2.0")
        # The CLI has no radius slider: the display radius is the request radius and every
        # ring is summed — the head says so.
        self.assertEqual(head_on["cone_display_radius_deg"], "2.0")
        self.assertEqual(head_on["cone_rings_summed"], "30 / 30")
        self.assertTrue(rows_on, "the cone on the ring must count rays")
        self.assertEqual(rows_on[0][0], _HALO_22, [r[0] for r in rows_on[:5]])
        self.assertTrue(all(r[0] != "3-6" for r in rows_on), "the sun's image cannot land 23° from the sun")

        off_ring = self._analyze("--roi", "cone", "--center", _OFF_RING, "--radius", "2")
        self.assertEqual(off_ring.returncode, 0, off_ring.stderr)
        head_off, rows_off = _parse_csv(off_ring.stdout)
        off_top = _top_energy(head_off, rows_off) if rows_off else 0.0
        # The ctypes test measures 41x-67x over three seeds at this budget; 10x is its floor,
        # far from the off-ring cone's ~10-ray noise and an order of magnitude away from what a
        # wrong centre convention (--center read as the antipode, say) would produce.
        self.assertGreater(_top_energy(head_on, rows_on), 10.0 * off_top,
                           f"on-ring {rows_on[0]} total {head_on['total_energy']} vs off-ring "
                           f"{rows_off[0] if rows_off else '(nothing)'} total {head_off['total_energy']}")

    def test_frame_roi_reads_the_render_entry(self):
        """`--roi frame` on the fixture's one render[] entry: a non-default lens
        (fisheye_equal_area) and the default visible range, read through the engine's own
        codec. Fewer rays land in a 120° frame looking at elevation 20 than in the whole sky,
        and the frame still sees the halo (the ring at 43° is inside it)."""
        sky = self._analyze()
        frame = self._analyze("--roi", "frame")
        self.assertEqual(frame.returncode, 0, frame.stderr)
        head_sky, _ = _parse_csv(sky.stdout)
        head, rows = _parse_csv(frame.stdout)
        self.assertEqual(head["region"], "in frame")
        self.assertLess(int(head["total_rays"]), int(head_sky["total_rays"]))
        self.assertGreater(int(head["total_rays"]), 0)
        self.assertIn(_HALO_22, [r[0] for r in rows[:3]], [r[0] for r in rows[:5]])
        # By id, the same entry: the same run.
        by_id = self._analyze("--roi", "frame", "--render-id", "1")
        self.assertEqual(by_id.returncode, 0, by_id.stderr)
        head_by_id, rows_by_id = _parse_csv(by_id.stdout)
        self.assertEqual(head_by_id["total_rays"], head["total_rays"])
        self.assertEqual(rows_by_id, rows)

    def test_frame_roi_covers_non_default_visible_and_a_defaulted_fov(self):
        """Two render[] entries the fixture does not have: `visible: full` with a `front`
        clip on a rectangular lens, and a lens with no `fov` at all (the engine's default
        applies — 90°, from the one function the GUI's import path also calls). Both must
        be readable; and a `--render-id` that names neither is refused, not defaulted."""
        doc = json.loads(_CONFIG.read_text(encoding="utf-8"))
        doc["render"] = [
            {"id": 3, "lens": {"type": "rectangular", "fov": 200}, "resolution": [256, 128],
             "view": {"elevation": 0}, "visible": "full", "front": True},
            {"id": 4, "lens": {"type": "linear"}, "resolution": [128, 128], "view": {"elevation": 20}},
        ]
        cfg = Path(self.output_dir) / "frames.json"
        cfg.write_text(json.dumps(doc), encoding="utf-8")
        base = ["analyze", "-f", str(cfg), "--seed", _SEED, "--roi", "frame"]
        for render_id in ("3", "4"):
            result = self.run_lumice(base + ["--render-id", render_id])
            self.assertEqual(result.returncode, 0, f"--render-id {render_id}: {result.stderr}")
            head, rows = _parse_csv(result.stdout)
            self.assertEqual(head["region"], "in frame")
            self.assertGreater(int(head["total_rays"]), 0, render_id)
        # Without --render-id the first entry is taken: the same run as id 3 (the export time
        # is the one head line two runs may disagree on).
        first = self.run_lumice(base)
        self.assertEqual(first.returncode, 0, first.stderr)
        head_first, rows_first = _parse_csv(first.stdout)
        head_three, rows_three = _parse_csv(self.run_lumice(base + ["--render-id", "3"]).stdout)
        head_first.pop("exported_at")
        head_three.pop("exported_at")
        self.assertEqual((head_first, rows_first), (head_three, rows_three))
        missing = self.run_lumice(base + ["--render-id", "9"])
        self.assertNotEqual(missing.returncode, 0)
        self.assertIn("--render-id 9 names no render[] entry", missing.stderr)
        self.assertIn("ids present: 3, 4", missing.stderr)

    def test_config_the_engine_cannot_load_is_refused_before_the_frame_is_read(self):
        """A `--roi frame` run whose document the engine refuses (a render[].id spelled as a
        string, which core's decoder rejects on type) exits 1 on the load error alone: the
        scene is loaded before the frame view is read off it, so a failed load never reaches
        the read. Beside it, the two refusals given before any engine call — a config that is
        not there, and one that is not JSON."""
        doc = json.loads(_CONFIG.read_text(encoding="utf-8"))
        doc["render"][0]["id"] = "1"
        cfg = Path(self.output_dir) / "string_id.json"
        cfg.write_text(json.dumps(doc), encoding="utf-8")
        result = self.run_lumice(["analyze", "-f", str(cfg), "--roi", "frame", "--render-id", "1"])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("failed to load configuration", result.stderr)
        self.assertNotIn("names no render[] entry", result.stderr, "the load must fail before the frame is read")
        self.assertEqual(result.stdout, "")
        missing = self.run_lumice(["analyze", "-f", str(Path(self.output_dir) / "absent.json")])
        self.assertNotEqual(missing.returncode, 0)
        self.assertIn("cannot open config file", missing.stderr)
        broken = Path(self.output_dir) / "broken.json"
        broken.write_text("{", encoding="utf-8")
        garbled = self.run_lumice(["analyze", "-f", str(broken)])
        self.assertNotEqual(garbled.returncode, 0)
        self.assertIn("invalid JSON", garbled.stderr)

    def test_option_conflicts_are_refused_with_the_option_named(self):
        cases = [
            (["--roi", "cone"], "--roi cone requires both --center"),
            (["--roi", "cone", "--center", "43,0"], "missing --radius"),
            (["--roi", "cone", "--radius", "2"], "missing --center"),
            (["--center", "43,0", "--radius", "2"], "--center only applies to --roi cone"),
            (["--roi", "frame", "--radius", "2"], "--radius only applies to --roi cone"),
            (["--render-id", "1"], "--render-id only applies to --roi frame"),
            (["--roi", "cone", "--center", "43,0", "--radius", "2", "--render-id", "1"],
             "--render-id only applies to --roi frame"),
            (["--symmetry", "PX"], "--symmetry must be 'none' or a combination of P, B, D"),
            (["--symmetry", "all"], "--symmetry must be"),
            (["--roi", "dome"], "--roi must be 'sky', 'frame' or 'cone'"),
            (["--roi", "cone", "--center", "4;3", "--radius", "2"], "--center must be '<altitude_deg>,<azimuth_deg>'"),
            (["--roi", "cone", "--center", "95,0", "--radius", "2"], "--center altitude must be between -90 and 90"),
            (["--roi", "cone", "--center", "43,0", "--radius", "0"], "--radius must be a number of degrees in (0, 180]"),
            (["--rays", "0"], "--rays must be a positive integer"),
            (["--rays", "2x"], "--rays must be a positive integer"),
            (["--seed", "0"], "--seed must be a positive integer"),
            (["--workers", "0"], "--workers must be a positive integer"),
            (["-o", "somewhere"], "unknown option: -o"),
        ]
        for extra, message in cases:
            with self.subTest(args=extra):
                result = self.run_lumice(["analyze", "-f", str(_CONFIG), *extra])
                self.assertNotEqual(result.returncode, 0, f"{extra}: expected a refusal")
                self.assertIn(message, result.stderr, f"{extra}: {result.stderr[:300]}")
                # A refusal prints the usage page (stdout, as every subcommand does) and no CSV.
                self.assertNotIn("# Lumice raypath analysis", result.stdout, f"{extra}: a refused run must print no CSV")

    def test_help_page_and_top_level_listing(self):
        page = self.run_lumice(["analyze", "-h"])
        self.assertEqual(page.returncode, 0)
        for option in ("--roi", "--center", "--radius", "--render-id", "--symmetry", "--rays", "--seed", "--csv",
                       "--workers", "--backend"):
            self.assertIn(option, page.stdout, option)
        top = self.run_lumice(["-h"])
        self.assertIn("analyze", top.stdout)
        # benchmark's page must not have grown the shared --workers step (its prose names the
        # option only to say it is absent; the option line itself would start the line).
        bench = self.run_lumice(["benchmark", "-h"])
        self.assertNotIn("  --workers <N>", bench.stdout)
        self.assertNotEqual(self.run_lumice(["benchmark", "-f", str(_CONFIG), "--workers", "2"]).returncode, 0)

    # ---- AC5: stdout is the CSV alone ----

    def test_stdout_is_only_the_csv_and_progress_is_stderr(self):
        result = self._analyze("--rays", "3M")
        self.assertEqual(result.returncode, 0, result.stderr)
        # Every stdout line is a `#` head line, the column header, or a four-cell row —
        # nothing the engine logs (those lines start with a timestamp) gets through.
        for line in result.stdout.splitlines():
            self.assertTrue(line.startswith("# ") or line == "Raypath,Energy,Cumulative %,+/-" or line.count(",") == 3,
                            f"unexpected stdout line: {line!r}")
        self.assertNotIn("# Lumice raypath analysis", result.stderr)
        # 3M rays on one worker takes several seconds, so at least one progress tick lands.
        self.assertIn("[analyze] ", result.stderr)
        self.assertIn(" rays traced, ", result.stderr)
        self.assertIn(" s elapsed", result.stderr)

    def test_csv_option_writes_the_file_and_silences_stdout(self):
        path = Path(self.output_dir) / "out.csv"
        result = self._analyze("--csv", str(path))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "")
        head, rows = _parse_csv(path.read_text(encoding="utf-8"))
        self.assertEqual(rows[0][0], _HALO_22)
        self.assertFalse((Path(self.output_dir) / "out.csv.tmp").exists(), "the temporary is renamed away")
        # And the same run to stdout is the same bytes but for the export time.
        again = self._analyze()
        head2, rows2 = _parse_csv(again.stdout)
        head.pop("exported_at")
        head2.pop("exported_at")
        self.assertEqual((head, rows), (head2, rows2))

    # ---- AC6: the read-time symmetry ----

    def test_symmetry_changes_the_grouping_and_not_the_totals(self):
        finest = self._analyze("--symmetry", "none")
        merged = self._analyze("--symmetry", "PBD")
        lower = self._analyze("--symmetry", "pd")
        self.assertEqual((finest.returncode, merged.returncode, lower.returncode), (0, 0, 0))
        head_f, rows_f = _parse_csv(finest.stdout)
        head_m, rows_m = _parse_csv(merged.stdout)
        head_l, rows_l = _parse_csv(lower.stdout)
        self.assertEqual(head_f["symmetry"], "no symmetry")
        self.assertEqual(head_m["symmetry"], "P|B|D")
        self.assertEqual(head_l["symmetry"], "P|D")
        self.assertEqual(head_f["total_energy"], head_m["total_energy"])
        self.assertEqual(head_f["total_rays"], head_m["total_rays"])
        self.assertEqual(head_l["total_energy"], head_m["total_energy"])
        self.assertGreaterEqual(len(rows_f), len(rows_l))
        self.assertGreaterEqual(len(rows_l), len(rows_m))
        # At the finest the 22° orbit is split twelve ways; under P|B|D it is one row on top.
        self.assertEqual(rows_m[0][0], _HALO_22)
        self.assertGreaterEqual({"3-5", "4-6", "3-7"} & {r[0] for r in rows_f}, {"3-5", "4-6", "3-7"})


# ---- AC4: the infinite budget and SIGINT ----

_FIRST_WRITE_TIMEOUT_SEC = 30.0


@pytest.mark.skipif(sys.platform == "win32",
                    reason="SIGINT delivery differs on Windows (CTRL_C_EVENT needs a fresh console process "
                           "group); the cross-platform signal shape is not this subcommand's to settle")
def test_infinite_budget_runs_until_sigint_and_writes_a_complete_csv(tmp_path):
    """A scene whose ray_num is "infinite" and no --rays: the run does not refuse, the --csv
    file appears at the first save tick and is a complete CSV at every rewrite, SIGINT ends
    the run with exit 0, and the file is written once more after the stop.

    The signal is sent on an observable — the file's first appearance — not a fixed sleep;
    the wait for it is bounded so a run that never writes fails with its stderr rather than
    hanging. The final write is proven by the file changing after the interrupt (a longer run
    has more rays, so the head's total_rays grows), not by timing.
    """
    try:
        binary = find_lumice_binary()
    except FileNotFoundError as e:
        pytest.skip(str(e))
    doc = json.loads(_CONFIG.read_text(encoding="utf-8"))
    doc["scene"]["ray_num"] = "infinite"
    cfg = tmp_path / "infinite.json"
    cfg.write_text(json.dumps(doc), encoding="utf-8")
    out = tmp_path / "infinite.csv"
    proc = subprocess.Popen([str(binary), "analyze", "-f", str(cfg), "--csv", str(out)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + _FIRST_WRITE_TIMEOUT_SEC
        while not out.exists():
            if proc.poll() is not None:
                pytest.fail(f"analyze exited early with {proc.returncode}: {proc.stderr.read()[-2000:]}")
            if time.monotonic() > deadline:
                proc.kill()
                pytest.fail(f"no CSV within {_FIRST_WRITE_TIMEOUT_SEC}s: {proc.stderr.read()[-2000:]}")
            time.sleep(0.05)
        # The file is complete whenever it is read: parse it right away, mid-run.
        first_head, first_rows = _parse_csv(out.read_text(encoding="utf-8"))
        assert first_rows, "a second of tracing records chains"
        # Let at least one more tick pass so the final write has something new to say.
        time.sleep(1.5)
        os.kill(proc.pid, signal.SIGINT)
        stdout, stderr = proc.communicate(timeout=60)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert proc.returncode == 0, stderr[-2000:]
    assert stdout == "", "with --csv nothing goes to stdout"
    assert "[analyze] interrupted" in stderr, stderr[-2000:]
    final_head, final_rows = _parse_csv(out.read_text(encoding="utf-8"))
    assert final_rows
    assert int(final_head["total_rays"]) > int(first_head["total_rays"]), "the stop wrote the run's last state"
    assert not (tmp_path / "infinite.csv.tmp").exists()
