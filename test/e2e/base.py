"""Base test case for Lumice E2E tests."""

import shutil
import tempfile
import unittest

from test.e2e.runner import find_lumice_binary, run_lumice, DEFAULT_TIMEOUT


class LumiceTestCase(unittest.TestCase):
    """Base class for Lumice E2E tests.

    Provides:
      - self.lumice_bin: Path to the Lumice executable (skips test if not found).
      - self.output_dir: A fresh temporary directory for test output.
      - self.run_lumice(args, timeout): Convenience method.
    """

    def setUp(self):
        try:
            self.lumice_bin = find_lumice_binary()
        except FileNotFoundError as e:
            self.skipTest(str(e))

        self.output_dir = tempfile.mkdtemp(prefix="lumice_e2e_")

    def tearDown(self):
        if hasattr(self, "output_dir") and self.output_dir:
            shutil.rmtree(self.output_dir, ignore_errors=True)

    def run_lumice(self, args, timeout=DEFAULT_TIMEOUT):
        """Run Lumice with the given arguments.

        Returns subprocess.CompletedProcess with stdout/stderr as strings.
        """
        return run_lumice(args, timeout=timeout)
