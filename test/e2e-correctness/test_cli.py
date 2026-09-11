"""CLI behavior tests for Lumice."""

import glob
import json
import os
import re
from pathlib import Path

from test.e2e.base import LumiceTestCase
from test.e2e.runner import get_project_root

# TODO: relocate configs when follow-up task completes
CONFIGS_DIR = get_project_root() / "test" / "e2e" / "configs"


def _cheap_halo_22_config(output_dir, ray_num=20000):
    """halo_22.json with ray_num cut from 10,000,000 down to `ray_num`.

    The CLI-surface tests in this file assert on stdout text, the exit code, the
    suffix/count of the produced files, or a JPEG's byte size. Not one of those
    oracles changes its verdict with the ray budget, so paying for 10M rays buys
    them nothing — the shipped fixture is simply the one that happened to be at
    hand. halo_22's crystal, lens and scattering layers are left untouched (the
    scene still has to render something and still has to carry last-layer
    prob=0.0); only the ray count shrinks. Same shape as `_build_config` in
    test/regression-sentinel/test_max_hits_crash.py.

    Not for anything whose oracle is image similarity: a PSNR comparison against
    a committed reference does depend on the ray budget, and shrinking it there
    would trade discriminating power for wall clock.
    """
    doc = json.loads((CONFIGS_DIR / "halo_22.json").read_text())
    doc["scene"]["ray_num"] = ray_num
    out = Path(output_dir) / "halo_22_cheap.json"
    out.write_text(json.dumps(doc))
    return out


class TestCli(LumiceTestCase):
    def test_help_flag(self):
        """Lumice -h should exit 0 and print usage."""
        result = self.run_lumice(["-h"])
        self.assertEqual(result.returncode, 0)
        self.assertIn("Usage:", result.stdout)

    def test_no_args(self):
        """Lumice with no arguments should exit non-zero."""
        result = self.run_lumice([])
        self.assertNotEqual(result.returncode, 0)

    def test_output_directory(self):
        """Lumice -o should write images to the specified directory."""
        if not (CONFIGS_DIR / "halo_22.json").exists():
            self.skipTest("halo_22.json not found")
        cfg = _cheap_halo_22_config(self.output_dir)

        result = self.run_lumice(["-f", str(cfg), "-o", self.output_dir])
        self.assertEqual(result.returncode, 0)

        output_imgs = glob.glob(os.path.join(self.output_dir, "img_*.jpg"))
        self.assertTrue(
            len(output_imgs) > 0,
            f"No images found in {self.output_dir}",
        )


class TestOutputFormat(LumiceTestCase):
    """Tests for --format and --quality CLI options."""

    def _get_config(self):
        if not (CONFIGS_DIR / "halo_22.json").exists():
            self.skipTest("halo_22.json not found")
        return _cheap_halo_22_config(self.output_dir)

    def test_output_format_png(self):
        """--format png should produce .png files."""
        cfg = self._get_config()
        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir, "--format", "png"]
        )
        self.assertEqual(result.returncode, 0)

        output_imgs = glob.glob(os.path.join(self.output_dir, "img_*.png"))
        self.assertTrue(
            len(output_imgs) > 0,
            f"No PNG images found in {self.output_dir}",
        )

    def test_output_format_jpg_explicit(self):
        """--format jpg should produce .jpg files."""
        cfg = self._get_config()
        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir, "--format", "jpg"]
        )
        self.assertEqual(result.returncode, 0)

        output_imgs = glob.glob(os.path.join(self.output_dir, "img_*.jpg"))
        self.assertTrue(
            len(output_imgs) > 0,
            f"No JPEG images found in {self.output_dir}",
        )

    def test_jpeg_quality(self):
        """--quality should affect JPEG file size (quality 1 < quality 95)."""
        cfg = self._get_config()

        # Run with quality 95 (default)
        dir_q95 = os.path.join(self.output_dir, "q95")
        os.makedirs(dir_q95)
        result = self.run_lumice(
            ["-f", str(cfg), "-o", dir_q95, "--quality", "95"]
        )
        self.assertEqual(result.returncode, 0)

        # Run with quality 1
        dir_q1 = os.path.join(self.output_dir, "q1")
        os.makedirs(dir_q1)
        result = self.run_lumice(
            ["-f", str(cfg), "-o", dir_q1, "--quality", "1"]
        )
        self.assertEqual(result.returncode, 0)

        imgs_q95 = sorted(glob.glob(os.path.join(dir_q95, "img_*.jpg")))
        imgs_q1 = sorted(glob.glob(os.path.join(dir_q1, "img_*.jpg")))
        self.assertTrue(len(imgs_q95) > 0, "No quality-95 images")
        self.assertEqual(len(imgs_q95), len(imgs_q1))

        for f95, f1 in zip(imgs_q95, imgs_q1):
            size_q95 = os.path.getsize(f95)
            size_q1 = os.path.getsize(f1)
            self.assertGreater(
                size_q95, size_q1,
                f"Expected quality 95 ({size_q95}B) > quality 1 ({size_q1}B)",
            )

    def test_invalid_format(self):
        """--format with unsupported value should exit non-zero."""
        result = self.run_lumice(
            ["-f", "dummy.json", "--format", "bmp"]
        )
        self.assertNotEqual(result.returncode, 0)

    def test_invalid_quality_range(self):
        """--quality outside [1, 100] should exit non-zero."""
        for val in ["0", "101"]:
            result = self.run_lumice(
                ["-f", "dummy.json", "--quality", val]
            )
            self.assertNotEqual(
                result.returncode, 0,
                f"--quality {val} should be rejected",
            )

    def test_invalid_quality_non_numeric(self):
        """--quality with non-numeric value should exit non-zero."""
        result = self.run_lumice(
            ["-f", "dummy.json", "--quality", "abc"]
        )
        self.assertNotEqual(result.returncode, 0)

    def test_quality_missing_value(self):
        """--quality as last argument (missing value) should exit non-zero."""
        result = self.run_lumice(
            ["-f", "dummy.json", "--quality"]
        )
        self.assertNotEqual(result.returncode, 0)


class TestWorkerCount(LumiceTestCase):
    """--workers N: the CLI entry point for the simulation worker count.

    The observation point is ServerImpl's startup INFO line
    ``ServerImpl: gpu_route=... worker_count=N (preferred_backend=...)``, which
    already existed for the GPU single-engine assertion (296.6) and reports the
    number the server actually constructed rather than the number the CLI parsed.

    On the no-flag path: ``server_config.num_workers`` is set from a variable
    whose no-flag value is 0, and server.cpp's
    ``num_workers > 0 ? num_workers : min(PhysicalCoreCount(), cap)`` guard sends
    that zero to the automatic branch. These tests therefore pin the observable
    consequences (a default in the physically possible range, at or below the
    cap, stable across runs, and different from an explicitly requested value)
    rather than re-deriving PhysicalCoreCount() in Python, which would be a
    second authority free to drift from the first.

    The cap's own value is the one number these tests do restate from C++
    (``EXPECTED_DEFAULT_CAP``). There is no way around that: the whole point of
    the assertion is that the shipping default does not exceed a specific
    number, and a test that read the number back out of the binary it is testing
    would agree with any value the binary happened to hold. Changing the
    constant is therefore meant to break this file — that is the alarm, not a
    maintenance cost to be engineered away.
    """

    # Mirrors kMaxDefaultWorkerCount in src/server/server.cpp. See the class
    # docstring for why this is deliberately a second copy rather than a lookup.
    EXPECTED_DEFAULT_CAP = 10

    WORKER_LINE = re.compile(r"worker_count=(\d+)")

    def _get_config(self):
        if not (CONFIGS_DIR / "halo_22.json").exists():
            self.skipTest("halo_22.json not found")
        return _cheap_halo_22_config(self.output_dir)

    def _run_and_read_worker_count(self, extra_args):
        cfg = self._get_config()
        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir] + list(extra_args)
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        match = self.WORKER_LINE.search(result.stdout)
        self.assertIsNotNone(
            match,
            f"no worker_count= line in stdout:\n{result.stdout}",
        )
        return int(match.group(1))

    def test_explicit_workers_is_what_the_server_constructs(self):
        """--workers N must make the server run exactly N workers."""
        self.assertEqual(self._run_and_read_worker_count(["--workers", "2"]), 2)

    def test_default_is_a_stable_machine_property_and_differs_from_a_request(self):
        """No flag → a stable automatic count; and the flag demonstrably moves it.

        The inequality is the red-state half of the explicit-override contract:
        if --workers were silently ignored, the explicit-2 run above and this one
        would report the same number. Skipped on a machine whose default happens
        to be 2, where the two arms are indistinguishable by construction.
        """
        default_workers = self._run_and_read_worker_count([])
        self.assertGreaterEqual(default_workers, 1)
        # Physical cores never exceed logical CPUs — an independent upper bound
        # that does not restate PhysicalCoreCount()'s own per-platform logic.
        self.assertLessEqual(default_workers, os.cpu_count() or 1)
        # Deterministic: the default is a property of the machine, not of the run.
        self.assertEqual(self._run_and_read_worker_count([]), default_workers)
        if default_workers == 2:
            self.skipTest(
                "machine default is 2; the explicit-2 arm cannot be told apart from it"
            )
        self.assertNotEqual(
            self._run_and_read_worker_count(["--workers", "2"]), default_workers
        )

    def test_default_never_exceeds_the_cap(self):
        """The automatic worker count is capped, whatever the machine has.

        DISCRIMINATING POWER DEPENDS ON THE HOST. On a machine with at most
        EXPECTED_DEFAULT_CAP physical cores the inequality holds no matter what
        the cap does, so this arm alone would pass on a build with the cap
        deleted. The second assertion is the one that does not depend on the
        host: on a machine with MORE cores than the cap, the default must be the
        cap exactly — not merely below it — because that is the only value
        min(cores, cap) can take. os.cpu_count() (logical CPUs) is an upper
        bound on physical cores, so "logical CPUs > cap" is a sound-but-not-
        complete detector of that case: it can fail to fire on a host with SMT
        and few cores, but it never fires wrongly.
        """
        default_workers = self._run_and_read_worker_count([])
        self.assertLessEqual(
            default_workers,
            self.EXPECTED_DEFAULT_CAP,
            "the automatic worker count must not exceed the cap",
        )
        if (os.cpu_count() or 1) > self.EXPECTED_DEFAULT_CAP:
            self.assertEqual(
                default_workers,
                self.EXPECTED_DEFAULT_CAP,
                "on a machine with more cores than the cap the default IS the cap",
            )

    def test_explicit_workers_above_the_cap_is_not_capped(self):
        """An explicit request above the cap is honoured verbatim.

        The red-state half of "the cap bounds the automatic value only": if the
        clamp were applied to the whole expression rather than to the no-flag
        branch, this would come back as EXPECTED_DEFAULT_CAP instead. Asserting
        equality rather than "> cap" is what makes that distinguishable.

        The requested count deliberately exceeds the cap and may exceed the
        host's core count; the assertion reads the number the server was
        constructed with, not a throughput, so oversubscription does not affect
        it.
        """
        requested = self.EXPECTED_DEFAULT_CAP + 1
        self.assertEqual(
            self._run_and_read_worker_count(["--workers", str(requested)]),
            requested,
        )

    def test_non_positive_workers_is_rejected(self):
        """--workers <= 0 must exit non-zero, not fall back to the default."""
        for val in ["0", "-1"]:
            result = self.run_lumice(["-f", "dummy.json", "--workers", val])
            self.assertNotEqual(
                result.returncode, 0, f"--workers {val} should be rejected"
            )

    def test_non_numeric_workers_is_rejected(self):
        """Including a digit prefix with a trailing tail.

        ``std::stoi("3abc")`` does not throw — it returns 3 and stops. Without a
        check that the whole argument was consumed, "3abc" would be silently
        accepted as 3, which is exactly the silent fallback AC1 forbids.
        """
        for val in ["abc", "3abc", "3.5", "", " "]:
            result = self.run_lumice(["-f", "dummy.json", "--workers", val])
            self.assertNotEqual(
                result.returncode, 0, f"--workers {val!r} should be rejected"
            )

    def test_workers_leading_whitespace_or_sign_is_rejected(self):
        """std::stoi skips leading whitespace and accepts a leading '+'/'-'.

        The trailing-garbage check alone (``pos == size``) does not catch this:
        " 3" and "+3" fully consume under std::stoi and would silently become 3
        without an explicit leading-character check. Requiring the first
        character to be a digit closes that gap (code-review round 1, Minor 2).
        """
        for val in [" 3", "+3"]:
            result = self.run_lumice(["-f", "dummy.json", "--workers", val])
            self.assertNotEqual(
                result.returncode, 0, f"--workers {val!r} should be rejected"
            )

    def test_workers_missing_value(self):
        """--workers as the last argument (missing value) should exit non-zero."""
        result = self.run_lumice(["-f", "dummy.json", "--workers"])
        self.assertNotEqual(result.returncode, 0)

    def test_benchmark_mode_says_out_loud_that_it_ignores_workers(self):
        """--workers + --benchmark: ignored, but with a visible notice.

        The benchmark's 1-worker and PhysicalCoreCount()-worker passes are its
        measurement methodology (per-core vs parallel efficiency), not a default
        a preference may override. Validation still runs, so an illegal value is
        still an error in this mode.
        """
        cfg = self._get_config()
        result = self.run_lumice(
            ["-f", str(cfg), "-o", self.output_dir, "--benchmark", "--workers", "4"]
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--workers is ignored in --benchmark mode", result.stderr)

        rejected = self.run_lumice(
            ["-f", str(cfg), "--benchmark", "--workers", "0"]
        )
        self.assertNotEqual(rejected.returncode, 0)


class TestLastLayerProbWarning(LumiceTestCase):
    """task-gui-ms-prob-footguns Step 3: CLI warns on last-layer prob>0.

    Uses real hand-written configs (feedback_test_must_use_issue_scenario) — the
    parity_single_ms_bd_filter fixture has last-layer prob=0.5, the halo_22
    fixture has last-layer prob=0.0. No fabricated test-only configs.
    """

    def test_warns_on_last_layer_prob_positive(self):
        cfg = CONFIGS_DIR / "parity_single_ms_bd_filter.json"
        if not cfg.exists():
            self.skipTest(f"{cfg} not found")
        result = self.run_lumice(["-f", str(cfg), "-o", self.output_dir])
        self.assertEqual(result.returncode, 0)
        # Warning is emitted via spdlog (LOG_WARNING → stdout). Match on the
        # distinctive prefix to avoid coupling to the full message wording.
        combined = result.stdout + result.stderr
        self.assertIn(
            "Last scattering layer has prob=",
            combined,
            f"expected last-layer-prob warning, got:\nSTDOUT:\n{result.stdout}\n"
            f"STDERR:\n{result.stderr}",
        )

    def test_no_warn_when_last_layer_prob_zero(self):
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest(f"{cfg} not found")
        # Only the ray budget is cut; the scattering layers this asserts on are
        # copied through unchanged. See _cheap_halo_22_config.
        cfg = _cheap_halo_22_config(self.output_dir)
        result = self.run_lumice(["-f", str(cfg), "-o", self.output_dir])
        self.assertEqual(result.returncode, 0)
        combined = result.stdout + result.stderr
        self.assertNotIn(
            "Last scattering layer has prob=",
            combined,
            "warning should NOT fire on last-layer prob=0.0 configs",
        )


class TestScatteringProbRequired(LumiceTestCase):
    """`scene.scattering[].prob` is required; the CLI must reject a config omitting it.

    This is the only layer that answers the question the change was actually about.
    The C++ unit tests each prove one parser rejects the document when called
    directly; neither proves the binary a user runs does, because the CLI reaches
    core through the C API, which re-serializes what it parsed. Run the real
    binary or the claim stays unverified.

    Derived from a real repo config rather than a hand-built minimal one, so the
    rejected document differs from a working one in exactly the deleted key.
    """

    def _config_missing_prob(self, layer_index=0):
        """Copy halo_22.json with one layer's `prob` deleted; return the new path."""
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest(f"{cfg} not found")
        with open(cfg) as f:
            doc = json.load(f)
        layers = doc["scene"]["scattering"]
        self.assertGreater(len(layers), layer_index)
        self.assertIn(
            "prob",
            layers[layer_index],
            "fixture no longer writes `prob`; this test would pass vacuously",
        )
        del layers[layer_index]["prob"]

        out = Path(self.output_dir) / "halo_22_missing_prob.json"
        with open(out, "w") as f:
            json.dump(doc, f)
        return out

    def test_cli_rejects_missing_prob(self):
        tmp_cfg = self._config_missing_prob()
        result = self.run_lumice(["-f", str(tmp_cfg), "-o", self.output_dir])
        self.assertNotEqual(
            result.returncode,
            0,
            f"CLI accepted a config with no `prob`:\nSTDOUT:\n{result.stdout}\n"
            f"STDERR:\n{result.stderr}",
        )
        # No images: "reports an error" and "produces nothing" are separate claims.
        self.assertEqual(glob.glob(os.path.join(self.output_dir, "img_*.jpg")), [])

    def test_error_message_is_actionable(self):
        """The message is the migration guidance — assert its three load-bearing parts.

        A config outside this repo cannot be enumerated, so whoever wrote one has
        only this text to go on: which layer, that `prob` is the field, and what
        to write to keep the old behavior. A bare "parse error" is not enough.
        """
        tmp_cfg = self._config_missing_prob()
        result = self.run_lumice(["-f", str(tmp_cfg), "-o", self.output_dir])
        # Warnings/errors go through spdlog, which writes to stdout here; check both.
        combined = result.stdout + result.stderr
        context = f"\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        self.assertIn("scattering[0]", combined, "must name the offending layer" + context)
        self.assertIn("prob", combined, "must name the field" + context)
        self.assertIn('"prob": 0', combined, "must state the migration write-up" + context)


class TestAxisSlotTypeRequired(LumiceTestCase):
    """A crystal's `axis` slot written as an object must name its `type`.

    Same reason the `prob` suite above exists at this layer: the C++ unit tests
    each prove one parser rejects the document when called directly, and neither
    proves the binary a user runs does. The CLI reaches core only through the C
    API, which re-serializes what it parsed — so a check landed in core alone
    would leave core's own test green while the CLI went on accepting the file.
    Run the real binary or the claim stays unverified.

    Derived from a real repo config, so the rejected document differs from a
    working one in exactly the deleted key.
    """

    def _config_axis_slot_without_type(self, slot):
        """Copy halo_22.json with crystal 0's `axis.<slot>.type` deleted."""
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest(f"{cfg} not found")
        with open(cfg) as f:
            doc = json.load(f)
        axis = doc["crystal"][0]["axis"]
        self.assertIn(
            "type",
            axis.get(slot, {}),
            f"fixture no longer writes axis.{slot}.type; this test would pass vacuously",
        )
        del axis[slot]["type"]

        out = Path(self.output_dir) / f"halo_22_axis_{slot}_no_type.json"
        with open(out, "w") as f:
            json.dump(doc, f)
        return out

    def _config_axis_without_zenith(self):
        cfg = CONFIGS_DIR / "halo_22.json"
        if not cfg.exists():
            self.skipTest(f"{cfg} not found")
        with open(cfg) as f:
            doc = json.load(f)
        axis = doc["crystal"][0]["axis"]
        self.assertIn("zenith", axis, "fixture no longer writes axis.zenith")
        del axis["zenith"]

        out = Path(self.output_dir) / "halo_22_axis_no_zenith.json"
        with open(out, "w") as f:
            json.dump(doc, f)
        return out

    def test_cli_rejects_axis_slot_without_type(self):
        """Both slots the fixture writes, separately.

        Before this narrowing the slots produced *different* wrong answers —
        `zenith` kept a fixed angle and discarded `std`, `azimuth` kept a
        360-degree sweep and discarded what `mean` asked for — so one slot is not
        evidence for the other.
        """
        for slot in ("zenith", "azimuth"):
            with self.subTest(slot=slot):
                tmp_cfg = self._config_axis_slot_without_type(slot)
                result = self.run_lumice(["-f", str(tmp_cfg), "-o", self.output_dir])
                self.assertNotEqual(
                    result.returncode,
                    0,
                    f"CLI accepted an axis.{slot} object with no `type`:\n"
                    f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}",
                )
                # "reports an error" and "produces nothing" are separate claims.
                self.assertEqual(glob.glob(os.path.join(self.output_dir, "img_*.jpg")), [])

    def test_cli_rejects_axis_without_zenith(self):
        tmp_cfg = self._config_axis_without_zenith()
        result = self.run_lumice(["-f", str(tmp_cfg), "-o", self.output_dir])
        self.assertNotEqual(
            result.returncode,
            0,
            f"CLI accepted an `axis` with no `zenith`:\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}",
        )
        self.assertEqual(glob.glob(os.path.join(self.output_dir, "img_*.jpg")), [])

    def test_error_message_is_actionable(self):
        """The message is the migration guidance — assert its load-bearing parts.

        A config outside this repo cannot be enumerated, so whoever wrote one has
        only this text to go on: which crystal, which slot, and both legal ways to
        write it. A bare "parse error" is not enough.
        """
        tmp_cfg = self._config_axis_slot_without_type("azimuth")
        result = self.run_lumice(["-f", str(tmp_cfg), "-o", self.output_dir])
        combined = result.stdout + result.stderr
        context = f"\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        self.assertIn("crystal[id=1]", combined, "must name the offending crystal" + context)
        self.assertIn("axis.azimuth", combined, "must name the offending slot" + context)
        self.assertIn('"azimuth": 20', combined, "must show the bare-number form" + context)
        self.assertIn('"type": "gauss"', combined, "must show the object form" + context)

    def test_missing_zenith_message_is_actionable(self):
        """The old text here was nlohmann's raw `out_of_range.403`, which names
        neither the crystal nor a way forward. The document is rejected exactly as
        before; only the message changed, so that is what is asserted."""
        tmp_cfg = self._config_axis_without_zenith()
        result = self.run_lumice(["-f", str(tmp_cfg), "-o", self.output_dir])
        combined = result.stdout + result.stderr
        context = f"\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        self.assertNotIn("out_of_range.403", combined, "still nlohmann's raw message" + context)
        self.assertIn("crystal[id=1]", combined, "must name the offending crystal" + context)
        self.assertIn("zenith", combined, "must name the missing slot" + context)
        self.assertIn("omit `axis`", combined, "must say how to opt out of `axis`" + context)


class TestBenchmarkIsaField(LumiceTestCase):
    """`--benchmark`'s JSON must say which ISA tier the binary was compiled for.

    Why the key exists at all: a Release build's ISA tier is `LUMICE_ISA_LEVEL`
    (`baseline` | `x86-64-v4` | `native`); local `scripts/build.sh` takes the `native`
    default, every CI job passes `baseline`, the Linux release builds `baseline` and
    `x86-64-v4` side by side, and MSVC has no equivalent flag wired up. So a
    throughput number recorded off a local build is not the shipped binary's number,
    and a Windows-vs-other comparison of two local builds is ISA-asymmetric by
    default. The `isa` key makes an already-recorded number answer that on its own.

    Why this test is not "run it and read the output": the cross-check below reads
    `CMakeCache.txt`, which CMake writes at configure time. That is a different
    producer from the `LUMICE_ISA_LEVEL_STR` macro → `#if` → JSON path being
    checked, so the two do not share a failure mode: if the generator expression
    resolving the macro were written wrong (inverted, or missing one of its two
    conjuncts), the cache would still hold what CMake actually consumed and this
    would go red.
    """

    _ISA_VALUES = {"native", "baseline", "x86-64-v4"}

    def _run_benchmark(self):
        """Run `--benchmark` on the shared bench config with a ray budget small
        enough for the fast leg, and return the parsed `[BENCHMARK]` lines."""
        cfg = json.loads((CONFIGS_DIR / "bench_light_single_ms.json").read_text())
        cfg["scene"]["ray_num"] = 200000
        tmp_cfg = Path(self.output_dir) / "bench_isa.json"
        tmp_cfg.write_text(json.dumps(cfg))

        result = self.run_lumice(["--benchmark", "-f", str(tmp_cfg), "-o", self.output_dir])
        context = f"\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        self.assertEqual(result.returncode, 0, "benchmark run failed" + context)

        rows = [
            json.loads(line.split("[BENCHMARK]", 1)[1].strip())
            for line in result.stdout.splitlines()
            if "[BENCHMARK]" in line
        ]
        self.assertTrue(rows, "no [BENCHMARK] line was emitted" + context)
        return rows

    def _find_cmake_cache(self) -> Path | None:
        """Locate the CMakeCache.txt belonging to the binary under test.

        Two layouts exist in this repo and both are covered: `scripts/build.sh`
        installs to `build/cmake_install/<flavor>/` and configures in
        `build/cmake_build/<flavor>/`; CI configures with `cmake -S . -B build` and
        (for the shared-lib leg) points $LUMICE_BIN into the build tree. The
        flavor-derived candidate is tried FIRST on purpose — walking up from an
        installed binary would otherwise reach a `build/CMakeCache.txt` left behind
        by an unrelated plain-cmake configure and cross-check against the wrong one.
        """
        binary = Path(self.lumice_bin).resolve()
        root = get_project_root().resolve()

        candidates = []
        parts = binary.parts
        if "cmake_install" in parts:
            flavor = parts[parts.index("cmake_install") + 1]
            candidates.append(root / "build" / "cmake_build" / flavor / "CMakeCache.txt")
        candidates.extend(parent / "CMakeCache.txt" for parent in binary.parents)
        candidates.append(root / "build" / "CMakeCache.txt")

        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return None

    @staticmethod
    def _read_cache_entry(cache_text: str, key: str) -> str | None:
        """Read one `KEY:TYPE=VALUE` line out of CMakeCache.txt's flat text format."""
        for line in cache_text.splitlines():
            name, sep, value = line.partition("=")
            if sep and name.split(":", 1)[0] == key:
                return value.strip()
        return None

    def test_isa_key_is_present_and_well_formed(self):
        """Schema layer — runs unconditionally, independent of any build layout."""
        for row in self._run_benchmark():
            self.assertIn("isa", row, f"[BENCHMARK] row lacks the isa key: {row}")
            self.assertIn(
                row["isa"],
                self._ISA_VALUES,
                f"isa={row['isa']!r} is outside {sorted(self._ISA_VALUES)}: {row}",
            )

    def test_isa_key_matches_the_configure_time_record(self):
        """Cross-check layer — the `isa` value against CMake's own configure record.

        Stated assumption: the oracle below treats `compiler_id == "MSVC"` as
        equivalent to CMake's `MSVC` boolean, which is what CMakeLists.txt actually
        gates the macro on. Those two decouple under clang-cl (`MSVC` is true, but
        the compiler id is "Clang"), which would make this oracle expect "native"
        against a binary correctly reporting "baseline". No clang-cl build exists in
        this repo's toolchain matrix today; if one is introduced, persist the `MSVC`
        boolean itself as a second cache snapshot rather than inferring it from the
        compiler id.
        """
        cache_path = self._find_cmake_cache()
        if cache_path is None:
            self.skipTest(
                "no CMakeCache.txt found for this binary — cross-check skipped, "
                "schema assertions still cover the key (see the other test here)"
            )
        cache_text = cache_path.read_text()

        isa_level = self._read_cache_entry(cache_text, "LUMICE_ISA_LEVEL")
        build_type = self._read_cache_entry(cache_text, "CMAKE_BUILD_TYPE")
        compiler_id = self._read_cache_entry(cache_text, "LUMICE_COMPILER_ID_SNAPSHOT")
        missing = [
            name
            for name, value in (
                ("LUMICE_ISA_LEVEL", isa_level),
                ("CMAKE_BUILD_TYPE", build_type),
                ("LUMICE_COMPILER_ID_SNAPSHOT", compiler_id),
            )
            if value is None
        ]
        if missing:
            # A build tree configured before LUMICE_COMPILER_ID_SNAPSHOT existed, or a
            # multi-config generator with no single CMAKE_BUILD_TYPE. Say so rather
            # than raising KeyError or inventing an expectation.
            self.skipTest(f"{cache_path} lacks {', '.join(missing)} — cross-check skipped")

        # The tier name is reported only when its -march flag was actually applied:
        # Release config on a non-MSVC compiler. Everything else (Debug, MinSizeRel,
        # MSVC, or the baseline tier itself) compiles without a flag and must say so.
        expected = (
            isa_level
            if (build_type == "Release" and compiler_id != "MSVC")
            else "baseline"
        )
        for row in self._run_benchmark():
            self.assertEqual(
                row["isa"],
                expected,
                f"isa={row['isa']!r} but {cache_path} records "
                f"LUMICE_ISA_LEVEL={isa_level}, CMAKE_BUILD_TYPE={build_type}, "
                f"LUMICE_COMPILER_ID_SNAPSHOT={compiler_id} (expected {expected!r})",
            )
