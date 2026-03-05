"""Error handling tests for Lumice — verify graceful failure on bad input."""

import os
from pathlib import Path

from e2e.base import LumiceTestCase

ERROR_CONFIGS_DIR = Path(__file__).resolve().parent / "configs" / "error"


class TestErrors(LumiceTestCase):
    """Tests that Lumice exits non-zero and does not crash on invalid input."""

    def _assert_fails_gracefully(self, args, context=""):
        """Run Lumice with args, assert non-zero exit and no signal death."""
        result = self.run_lumice(args)
        self.assertGreater(
            result.returncode, 0,
            f"Expected non-zero exit for {context}, got {result.returncode}",
        )

    def test_nonexistent_config(self):
        """Lumice -f nonexistent.json should fail."""
        self._assert_fails_gracefully(
            ["-f", "nonexistent_file_that_does_not_exist.json"],
            "nonexistent config",
        )

    def test_invalid_json(self):
        """Lumice with syntactically invalid JSON should fail."""
        cfg = ERROR_CONFIGS_DIR / "invalid_json.json"
        self._assert_fails_gracefully(["-f", str(cfg)], "invalid JSON")

    def test_missing_scene(self):
        """Config missing 'scene' field should fail."""
        cfg = ERROR_CONFIGS_DIR / "missing_scene.json"
        self._assert_fails_gracefully(["-f", str(cfg)], "missing scene")

    def test_missing_render(self):
        """Config missing 'render' field should fail."""
        cfg = ERROR_CONFIGS_DIR / "missing_render.json"
        self._assert_fails_gracefully(["-f", str(cfg)], "missing render")

    def test_missing_crystal(self):
        """Config missing 'crystal' field should fail."""
        cfg = ERROR_CONFIGS_DIR / "missing_crystal.json"
        self._assert_fails_gracefully(["-f", str(cfg)], "missing crystal")

    def test_nonexistent_output_dir(self):
        """Lumice -o /nonexistent/dir should fail."""
        cfg = next(
            (Path(__file__).resolve().parent / "configs").glob("halo_22.json"),
            None,
        )
        if cfg is None:
            self.skipTest("halo_22.json not found")
        self._assert_fails_gracefully(
            ["-f", str(cfg), "-o", "/nonexistent/path/that/does/not/exist"],
            "nonexistent output dir",
        )

    def test_unknown_option(self):
        """Lumice -z (unknown option) should fail."""
        self._assert_fails_gracefully(["-z"], "unknown option")
