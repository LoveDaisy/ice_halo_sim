"""CLI behavior tests for Lumice."""

from e2e.base import LumiceTestCase


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
