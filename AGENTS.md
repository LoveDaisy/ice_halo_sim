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
  - `@pytest.mark.slow` tests require the shared-lib build (`./scripts/build.sh -sj release`) and are excluded from CI to keep PR feedback fast. Run them locally with `pytest test/e2e/ -v -m slow` before opening a PR that touches the simulator core, query filter, or C API surface.
    - `test_capi_sentinel_overflow.py` — sentinel-overflow regression: 3-config × 12 rounds = 36 server lifecycles via `LUMICE_GetRawXyzResults(max_count=1)`; guards against reintroduction of the c_api.cpp off-by-one sentinel write (fix: 5287efe)
    - `test_ms_filter_leak.py` — Design A filter-fail termination regression: confirms filter-fail rays do not propagate across MS layers
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

## Documentation Index (`doc/`)

Valuable design/architecture docs live in `doc/` (tracked). Consult the relevant one
**before** redesigning a subsystem — many decisions are already reasoned out here.
(Most have a `_zh` Chinese sibling.)

- **Overview / guides**: `README.md`, `architecture.md`, `developer-guide.md`, `gui-guide.md`, `configuration.md`
- **Core / rendering architecture**: `accumulator-consumer-architecture.md`, `raypath-rayseg-architecture.md`, `raypath-symmetry.md`, `coordinate-convention.md`, `crystal-orientation-sampling.md`, `ev-pipeline-architecture.md`, `adaptive-brightness.md`, `filter-architecture.md`
- **C API**: `c_api.md`, `capi-lifecycle-architecture.md`
- **GPU / Metal route** (read these before touching the GPU path):
  - `seam-design.md` — **the `TraceBackend` host/device seam redesign blueprint**; §5 = single-engine, three-clock-decoupled GPU simulator (the target architecture); §3.6 "原始之罪" = why GPU must not mirror the CPU pipeline.
  - `gpu-route-history.md` — systematic retrospective of the GPU migration (#250→265): decision evolution, accumulated data assets, leftover-item ledger.
  - `trace-backend-frame-lifecycle.md` — Metal frame lifecycle (as-built, multi-MS transit, parity harness methodology).
- **Perf / testing**: `performance-testing.md`, `windows-remote-testing.md`, `xyz-stats-tool.md`
- Example config: `examples/config_example.json`

## Knowledge Base & Working Discipline

This project carries a deliberate accumulated memory. Its value depends on it being
**retrieved**, not just stored (信息价值 = 内在价值 × 被检索到的概率). The recurring
failure mode is starting each session like a newcomer and re-deriving decisions the
owner already settled. Avoid it:

- **Retrieve before re-deriving.** Before starting work on a continuing/recurring topic
  (GPU/Metal perf, GUI perf, parity, batch/throughput, architecture decisions), FIRST:
  1. `grep` `scratchpad/backlog.md` for the topic — it holds owner concerns, start
     conditions, and dependencies (e.g. concern #2 commit↔batch decoupling).
  2. Read the relevant prior `scratchpad/explore-*/{SUMMARY,insights}.md` and
     `scratchpad/scrum-*/SUMMARY.md` — they hold verified conclusions and rejected
     directions. Check the `doc/` index above.
  3. **Explicitly continue prior threads** ("this refines concern #X") rather than
     starting fresh. If new evidence conflicts with a prior conclusion, connect them.
- **The scratchpad system is the source of truth for in-flight reasoning.**
  `scratchpad/tasks.md` (the task ledger), `scratchpad/backlog.md` (deferred work +
  owner concerns), `scratchpad/explore-*/` & `scratchpad/scrum-*/` (per-effort
  hypotheses/experiments/insights/SUMMARY), `scratchpad/learnings/` (extracted lessons).
  These are git-ignored working memory — do not treat their absence from git as absence
  of knowledge.
- **Promote durable design docs out of `scratchpad/` into `doc/`.** scratchpad is
  git-ignored, so design docs left there are undiscoverable and get lost (this is how
  the `seam-design.md` blueprint sat unbuilt). When an explore/scrum produces a durable
  design or decision record, copy it to `doc/` and add it to the index above.
- **Think at the architecture level before decomposing into tasks.** The task/scrum
  machinery rewards fast decomposition and immediate action; resist acting on the first
  promising small direction before the architecture-level question is reasoned through.
