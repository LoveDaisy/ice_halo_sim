# E2E Shared Fixtures

This directory holds shared fixtures for the pytest-based E2E layers; it contains no test files of
its own. The tests that consume these fixtures now live under `test/e2e-correctness/`,
`test/parity-cross-backend/`, `test/performance/`, `test/gui/`, and `test/regression-sentinel/` —
see `doc/testing-architecture.md` §1 for what each layer covers, or the "Testing and Platform
Notes" section of `AGENTS.md` for how to run them.

## Contents

- `runner.py` — binary discovery and subprocess execution against the built `Lumice` CLI.
  Override the binary path with the `LUMICE_BIN` env var.
- `capi_runner.py` — ctypes-based runner that drives Lumice through the C API directly, for tests
  that need scalar fields a subprocess run cannot observe. Requires the shared-lib build
  (`./scripts/build.sh -sj release`); override the library path with `LUMICE_LIB`.
- `base.py` — `LumiceTestCase`, the shared `unittest.TestCase` base for subprocess-driven tests.
- `image_utils.py` — JPEG dimensions and MSE/PSNR computation (requires Pillow).
- `_parity_metrics.py` — shared parity metrics for the Metal/CUDA/CPU cross-backend battery.
- `_projection_battery.py` — shared scaffolding for the per-projection cross-backend parity battery.
- `configs/` — scenario JSON configs consumed by tests across the layers above.

Because pytest collects tests by path, running `pytest test/e2e/` collects nothing — there are no
`test_*.py` files here. Run `pytest -v` (fast subset) or point at one of the layer directories
above instead.
