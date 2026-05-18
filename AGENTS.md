# AGENTS.md

## Project Overview

Lumice is a C++17 ice halo ray-tracing simulator. It reproduces halo patterns by tracing light through ice crystals and supports CLI, GUI, unit tests, E2E tests, and performance-oriented workflows.

Core conventions:

- namespace: `lumice`
- public API boundary: `src/include/lumice.h`
- source layout: `.hpp` headers and `.cpp` implementations
- build system: CMake + Ninja
- dependency management: CPM.cmake

## Common Commands

```bash
# Build
./scripts/build.sh -j release
./scripts/build.sh -tj release
./scripts/build.sh -gtj release
./scripts/build.sh -k release

# Run
./build/cmake_install/Lumice -f examples/config_example.json
./build/cmake_install/Lumice -f config.json -v

# Tests
./scripts/build.sh -tj release
./scripts/build.sh -gtj release
LUMICE_SKIP_GUI_TESTS=1 ./scripts/build.sh -gtj release
pytest test/e2e/ -v                       # fast e2e only (matches CI)
./scripts/build.sh -sj release && \
  pytest test/e2e/ -v -m slow             # slow e2e (needs shared lib; run before PR)

# Format
./scripts/format.sh
```

Release artifacts land in `build/cmake_install/`. Debug builds stay in `build/cmake_build/`.

## Code Structure

- `src/config/`: configuration parsing and simulation config data
- `src/core/`: math, optics, simulator, filters, buffers, ray paths
- `src/gui/`: GUI app, panels, preview, file IO, poller
- `src/server/`: server-side render, consumer, stats, C API bridge
- `src/util/`: logger, threading, queue, arguments, color data
- `src/include/`: public C API header
- `test/`: unit tests, GUI tests, and E2E tests

## Style and Engineering Rules

- Follow the repository `.clang-format` and `.clang-tidy` rules.
- Naming:
  - types: `CamelCase`
  - functions: `CamelCase`
  - variables: `lower_case`
  - private members: `lower_case_`
  - constants: `kCamelCase`
- Prefer `const`, `constexpr`, smart pointers, `nullptr`, and `override`.
- Do not use raw `new` / `delete`.
- Do not use `using namespace`.
- Keep code comments in English.
- Public API boundary: `src/gui/` code must only access core/config functionality
  through the C API (`src/include/lumice.h`). Direct `#include` of `core/` or `config/`
  headers from `src/gui/` is prohibited.

## Testing and Platform Notes

- CLI, core, and unit-test flows should remain cross-platform.
- GUI tests require a display server unless explicitly skipped with `LUMICE_SKIP_GUI_TESTS=1`.
- E2E test split:
  - Default `pytest test/e2e/ -v` runs the fast subset — matches CI behavior (CI uses `-m "not slow"`).
  - `@pytest.mark.slow` tests (e.g. `test_additivity.py`) require the shared-lib build (`./scripts/build.sh -sj release`) and are excluded from CI to keep PR feedback fast. Run them locally with `pytest test/e2e/ -v -m slow` before opening a PR that touches the simulator core, query filter, or C API surface.
- GUI screenshot references live under `test/gui/references/`.
- Windows physical-desktop validation uses `scripts/win_remote_test.sh` together with `scripts/win_test_watcher.ps1`.
- Performance diagnostics and workflows are documented in `doc/performance-testing.md`.

### GUI Test Reference Regeneration (auto_ev tests)

The `auto_ev` reference images are pixel-averaged means of N=10 stochastic renders to suppress
per-run noise. Per-scene PSNR thresholds are `mean − 3σ` (floored to 0.5 dB precision).

**`--keep-export-png` flag** — When passed to `LumiceGUITests`, `CheckAgainstReference` skips
`std::remove` so the per-run export PNGs at `/tmp/lumice_auto_ev_*.png` are preserved for
collection by the driver script.

**Regeneration workflow:**
```bash
# Full regen (Phase A: generate mean-ref + Phase B: calibrate thresholds, ~20 min):
python scripts/regen_gui_test_refs.py

# Phase A only (generate mean-ref images, then manually update thresholds):
python scripts/regen_gui_test_refs.py --phase-a-only

# Phase B only (recalibrate thresholds against existing mean-refs):
python scripts/regen_gui_test_refs.py --phase-b-only

# Quick smoke test (2 runs each phase):
python scripts/regen_gui_test_refs.py --n 2 --n-calib 2
```

After Phase B, copy the `threshold` values from `test/gui/references/_thresholds.json` into
`kScenes[]` in `test/gui/test_gui_auto_ev.cpp`. Use `min(off_threshold, on_threshold)` per scene
since both modes share one `psnr_threshold` field.

## Logging and Troubleshooting

- `VERBOSE` is a project-defined log level between `DEBUG` and `INFO`.
- When debugging intermittent issues, first map the full data flow, then audit silent-return paths and add observability before changing behavior.
- If a known-good branch or prior fix exists, diff it first before starting fresh analysis.
- Do not split one decision across multiple threads or modules when a single owner can make it.

## Collaboration Constraints

- `config.json`, `test.json`, `scratchpad/`, remote test output files, and most generated artifacts are intentionally git-ignored.
- Do not use `git add -f` to force-track ignored files. If a file is ignored and you are unsure, stop and ask first.
- Reference images under `test/e2e/references/*.jpg` and `test/gui/references/*.jpg` are explicitly unignored and may be tracked normally.
- CI runs build and unit tests on branch pushes; E2E tests run on PRs and `main`.

## Useful Paths

- Main docs: `README.md`, `README_zh.md`
- Configuration docs: `doc/configuration.md`
- Performance testing: `doc/performance-testing.md`
- Windows remote testing: `doc/windows-remote-testing.md`
- Example config: `examples/config_example.json`
