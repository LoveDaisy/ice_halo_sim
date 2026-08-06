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
./build/cmake_install/static/Lumice -f examples/config_example.json
./build/cmake_install/static/Lumice -f config.json -v

# Tests — scripts/test.sh is the recommended entry point; see the test-scope
# table under "Testing and Platform Notes" for which one fits a given situation.
LUMICE_SKIP_GUI_TESTS=1 ./scripts/build.sh -gtj release   # headless: build gui_test, skip running it
./scripts/test.sh quick                   # after -gtj above: ctest + fast e2e + gui_test correctness pool
./scripts/test.sh full                    # quick + gui_test real-timing pool
./scripts/test.sh pr                      # full + shared-lib freshness + slow e2e (before opening a PR)
pytest -v                                 # fast e2e only, direct — pinned by pyproject.toml addopts (matches CI)
pytest -v -m ''                           # full e2e set — the addopts pin's escape hatch
pytest -v -m slow                         # slow e2e only (needs shared lib: ./scripts/build.sh -sj release)

# Format
./scripts/format.sh
./scripts/format.sh --check          # read-only: exit 1 if anything needs formatting (used by the pre-commit hook; CI runs the format-check job)

# Engineering-policy gate (env-knob centralization/registration, GUI API boundary, using-namespace)
python3 scripts/check_policies.py    # whole-tree: what state is the repo in?
python3 scripts/check_new_refs.py --staged        # diff-scoped: what did you just write?
python3 scripts/check_new_refs.py --range A..B    # same, over a commit range (what CI runs)
./scripts/install-hooks.sh           # install the non-interactive pre-commit hook (one-time)
```

Build trees and install trees are per-flavor: `-s` (shared) and no `-s` (static) never share a
directory, so both stay warm and switching between them does not force a rebuild. `-t`/`-g`/`-b`
choose *what* to build (tests / GUI app / benchmarks) and compose with either flavor — only `-s`
switches the flavor itself, and it defaults to static when omitted, so `-t`/`-g` alone never
produce a shared library. Release artifacts are installed to `build/cmake_install/<flavor>/`; the
CMake build tree is `build/cmake_build/<flavor>/` and compiler output lands in
`build/<BUILD_TYPE>/<flavor>/{bin,lib}/`.

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
- Logging: all diagnostic output goes through the logger — `ILOG_*` (core/server, `src/util/logger.hpp`)
  or `GUI_LOG_*` (GUI, `src/gui/gui_logger.hpp`). A raw `printf` / `fprintf` / `std::cout` / `std::cerr`
  bypasses the unified sinks, the level filter and the format, so it is invisible exactly where a user
  looks: the GUI log panel, the file sink they enabled, and — on Windows, where the GUI calls
  `FreeConsole()` unless a diagnostic flag was passed — anywhere at all. Two files under `src/` are
  exempt, and both are exemptions with a name and an owner rather than a per-line escape hatch:
  `src/main.cpp`, whose stdout **is** the CLI's product output (two of its lines are parsed contracts —
  the `[BENCHMARK]` JSON and `ColorClassSignal:`, so a log prefix would break the e2e suite); and
  `src/util/fatal.hpp`, the single owner of the pre-abort trap, where unbuffered stderr is the whole
  point — a per-message-unflushed file sink can lose the line precisely when it matters. For an
  unrecoverable invariant call `lumice::FatalAbort(...)` instead of hand-rolling print-then-`abort()`.
  Enforced by the `no-bare-print` rule in `scripts/check_policies.py`, which also scans `.cu` / `.metal`
  so a GPU backend cannot reopen the side channel. `test/` is deliberately out of scope: test binaries
  are their own harness with no app logger to bypass, and some of their output is a parsed contract
  (the regen driver reads gui_test's `PSNR=` line). Device-side `printf` in a CUDA kernel stays a
  legitimate debugging tool — the gate does not stop you adding one while working, only from landing it.
- Public API boundary: `src/gui/` code must only access core/config functionality
  through the C API (`src/include/lumice.h`). Direct `#include` of `core/` or `config/`
  headers from `src/gui/` is prohibited.
- Environment variables: before adding any new `std::getenv`, apply the decision gate in
  `doc/env-var-policy.md`. User-facing behavior switches must go through CLI/config/API,
  not env vars (env causes silent per-machine drift). Env is fine for dev/experiment knobs
  (centralized + logged on startup) and test/build infra. All `LUMICE_*` env reads live in
  `src/util/env_knobs.cpp` and are enforced by `scripts/check_policies.py` (CI `policy` job +
  local pre-commit hook, installed via `./scripts/install-hooks.sh`).
- Working-note references: prose in a tracked file must not cite `scratchpad/` artifacts —
  task ids (`task-<name>`, `explore-<name>`, `scrum-<name>`, `chore-<name>`), per-task note
  filenames (`plan.md`, `progress.md`, `SUMMARY.md`), `code-review-0N`, `round-N`, or
  `scratchpad/` paths. That tree is git-ignored and archived per task, so the reference
  dangles for anyone reading later; state the mechanism itself instead. Permanent anchors
  are fine and should stay: PR numbers, commit hashes, `file:line`, code symbol names.
  This applies to any long-lived prose read apart from the context it was written in —
  production comments, test docstrings, CI config comments, and `doc/` alike.
  Enforced by `scripts/check_new_refs.py`, a **second, diff-scoped entry point** next to
  `check_policies.py` — not a check inside it. It reads only the lines a change *adds*
  (CI `new-refs` job on PRs vs merge-base; pre-commit hook on the staged diff), so the
  large body of pre-existing references never blocks a commit. Only natural-language
  regions are read: comments, Python docstrings, and Markdown — never string literals.
  There is no inline exemption, by design: if some prose genuinely needs such a reference,
  the rule is wrong and the rule should change. Files exempt from it are limited by one
  criterion — the file's *subject* is this working-notes system itself, i.e. it must name
  the system to do its job (this file, `CLAUDE.md`, and the root `.gitignore`). Citing the
  system is not grounds for exemption; that is the thing being checked.
  **The checker is the rule, not an approximation of it.** If `check_new_refs.py` passes, the
  change is compliant — full stop. Do not flag a reference in review that the checker accepts,
  and do not appeal to the rule's "spirit" beyond what it matches: the checker deliberately
  misses some citation forms (the trade-offs are argued at each pattern in the script), and
  those misses are accepted, not oversights awaiting discovery. Rationale: this rule guards
  against a harm measured to be *low* — 41% of tracked files carried such references for years
  with no observed cost — so an unbounded review-time judgement about it is worth less than it
  costs. Tightening is legitimate only by changing a pattern in the script (and accepting the
  false positives that buys), never by case-by-case escalation in review.

## Testing and Platform Notes

- CLI, core, and unit-test flows should remain cross-platform.
- GUI tests require a display server unless explicitly skipped with `LUMICE_SKIP_GUI_TESTS=1`.
  `gui_test` is therefore **built on three platforms and run on one**: the `Ubuntu x86_64` leg of
  `.github/workflows/ci.yml` supplies a display with `xvfb-run` + Mesa's llvmpipe software
  rasterizer and runs the `modal_layout` and `defaults_panel_layout` reference groups there; the
  macOS and Windows legs still only compile it. Which groups may run under a software rasterizer
  is a measured fact, not a preference — the per-scene numbers, the reason `lens_proj` is
  excluded despite its pixels being portable, and **the checklist a red in this layer obliges you
  to follow instead of calling it a known flake** are all in `doc/testing-architecture.md` §4.6
  (whose numbers, as that section itself flags, were measured on an arm64 proxy for the amd64
  runner this step actually runs on — read the caveat, not just the table).
  Read that before widening the CI filter, and before dispositioning a visual-regression failure.
  A `src/gui/` test that needs no live frame still should not live there: the
  `gui_unit_test` target (`test/CMakeLists.txt`, inside `if(BUILD_GUI)`) links `lumice_gui_obj`
  with **no window, no GL context and no ImGui test engine**, so its cases really do run in CI on
  every platform that builds the GUI. Its sources sit in `test/unit-correctness/gui/` next to
  `unit_correctness_test`'s and carry the same `unit-correctness` CTest LABEL — the two targets
  are split on a *link* boundary (does the case call into `file_io.cpp` / `user_defaults.cpp`?),
  not a layer boundary, so `ctest -L` and `./scripts/test.sh` need no per-target knowledge.
  Rule of thumb for a new `src/gui/` test: needs a rendered frame or an `ImGuiTestContext` →
  `gui_test`; pure logic → `gui_unit_test` (or `unit_correctness_test` if it is header-only).
  Design reasoning and the target-naming rule: `doc/testing-architecture.md` §2 and §5.
  That boundary is a **gate, not a rule of thumb**, for cases you add:
  `scripts/check_new_gui_tests.py` is a third, diff-scoped entry point alongside
  `check_policies.py` and `check_new_refs.py` (CI `new-refs` job on PRs vs merge-base;
  pre-commit hook on the staged diff). It rejects a `TestFunc = [](ImGuiTestContext* …)`
  lambda under `test/gui/**.cpp` belonging to a case the change **brought into existence**
  — its `category/name` did not exist anywhere under `test/gui/` before, or its signature
  line was rewritten in the file it already lived in — when either:
  (1) the parameter is anonymous, `[](ImGuiTestContext*)`, including the equivalent
  `[](ImGuiTestContext* /*ctx*/)` with the name commented out; or
  (2) the parameter is named and the body calls `IM_UNUSED(<name>)` while containing no
  other mention of `<name>` anywhere — a bare `<name>` passed to a helper counts as a use,
  so `Helper(ctx, …)` is fine and the narrower `<name>->` reading is deliberately not used.
  Both are an explicit statement — by the compiler, or by the author's own hand — that the
  case does not drive the GUI, which is why the rule rests on them and not on inference.
  **What it does not reject**, all accepted deliberately: `TestFunc = SomeNamedFunction`
  (no signature to read); a named parameter never referenced and never marked `IM_UNUSED`
  (the build is `-Wall -Wextra` without `-Werror`, so it only warns — catching it needs an
  inference no one stands behind); pre-existing cases of any shape, including ones a change
  relocates between `test/gui/` files or shifts within one; and renaming an existing
  rejected-shape case, which does read as a new registration.
  There is **no inline exemption, by design**, and the same "checker is the rule" discipline
  applies as for `check_new_refs.py`: if it passes, the change is compliant — do not flag in
  review what the checker accepts. If a new case genuinely needs a frame but happens not to
  touch `ctx`, make it drive `ctx` (`ctx->Yield()` is the frame pump); that states the
  dependency and satisfies the gate. If some case truly cannot be written that way, the rule
  is wrong and the rule changes — in the script, never per case.
- E2E test layout (purpose-primary; see `doc/testing-architecture.md` §6):
  - `test/e2e-correctness/` — full-stack correctness via CLI/PSNR (smoke, CLI behavior, raypath equivalence) + `references/*.jpg`
  - `test/parity-cross-backend/backend/` — backend-equivalence oracles (Metal exit-seam parity, device-gen default path, cpu_backend route, Metal batch invariance) + C++ siblings from 270.3
  - `test/performance/` — throughput gates (Metal throughput)
  - `test/gui/` — GUI acceptance (Metal GUI north-star) alongside the C++ GUI tests (`functional/`, `visual/`, `responsiveness/` subdirs; target `gui_test`)
  - `test/regression-sentinel/` — bug-resurfacing guards (errors, capi sentinel overflow, MS filter leak)
  - Shared fixtures stay under `test/e2e/` (`base.py`, `runner.py`, `capi_runner.py`, `image_utils.py`, `_parity_metrics.py`, `_projection_battery.py`, `configs/`).
- E2E test split:
  - `./scripts/test.sh <quick|full|pr>` is the recommended entry point for a local run — see the
    test-scope table below for which one fits a given situation. It runs ctest, the fast e2e
    subset, `gui_test`, and (for `pr`) the shared-lib slow e2e leg as one call and prints a single
    per-layer summary ending in one `RESULT: PASS`/`FAIL` line, so one invocation is enough to
    read the outcome — see the judgment-discipline note below for why that matters.
  - Bare `pytest -v` runs the fast subset directly. This is pinned by `pyproject.toml`'s
    `addopts` (`-m "not slow"`), so it is structurally true rather than a convention a caller has
    to remember, and it matches CI's fast leg. `pytest -v -m ''` is the escape hatch back to the
    full set; because a command-line `-m` overrides `addopts`'s `-m` value rather than combining
    with it (any other `addopts` flags stay in effect), any invocation that needs the full set
    (or a marker CI never uses) must pass `-m` explicitly.
    See `doc/testing-architecture.md` §5 for the `addopts` rationale and the incident it fixed.
  - `@pytest.mark.slow` tests require the shared-lib build (`./scripts/build.sh -sj release` —
    `-g`/`-t` alone never produce it, see "Build trees..." above) and are excluded from CI's fast
    leg. Run them locally with `pytest -v -m slow` before opening a PR that touches the simulator
    core, query filter, or C API surface.
    - `test/regression-sentinel/test_capi_sentinel_overflow.py` — sentinel-overflow regression: 3-config × 12 rounds = 36 server lifecycles via `LUMICE_AcquireResultFrame` + `LUMICE_FrameGetRawXyz(max_count=1)`; guards against reintroduction of the c_api.cpp off-by-one sentinel write (fix: 5287efe)
    - `test/regression-sentinel/test_ms_filter_leak.py` — Design A filter-fail termination regression: confirms filter-fail rays do not propagate across MS layers
  - **Test-scope table** — which command fits a given situation:

    | Situation | Run |
    |---|---|
    | Verify the change you just made | `./scripts/test.sh quick` |
    | Right after a static rebuild, before moving on | `./scripts/test.sh quick` |
    | Broader regression sweep — unsure how far a change reaches | `./scripts/test.sh full` |
    | Before opening a PR — reproduce the CI e2e-slow shape locally | `./scripts/test.sh pr` |
    | Confirm one specific gate/test actually fails on a deliberately-broken state (not a general sweep) | run that one check directly against the broken state, then again after the fix: `ctest -R <label>`, `pytest <file>::<test> -m ''`, or `gui_test --filter <name>` |

  - **Judgment discipline**: a command's exit code is the only thing that says pass/fail, and two
    common habits obscure it in different ways. A pipe (`cmd | tail`) leaves `$?` reporting
    `tail`'s exit code, not `cmd`'s, unless the shell has `set -o pipefail` — so a failing command
    piped through one still reads as success. A redirect (`cmd > log 2>&1`) does not touch `$?` at
    all, but it does move the PASS/FAIL text out of sight, so a finished run still needs a second
    call (`grep`/`cat` on the log) just to learn the outcome it already produced. Read `$?`
    straight off the foregrounded command, or use `./scripts/test.sh`, which already does this for
    every layer.
- GUI screenshot references live under `test/gui/references/`.
- GUI tests (the C++ `gui_test` binary and the `test/gui/` e2e layer that drives it) are isolated
  from personal defaults by default: `gui_test`'s own `--user-config`/
  `--no-user-config` no-flag default is `kDisabled` (the interactive app defaults to auto-detect
  instead), so a visual-regression reference never depends on whichever `user_defaults.json`
  happens to exist on the machine that captured it. Before adding a new reference image, confirm
  this isolation is actually in effect for the scene under test (an explicit, freshly emptied
  `--user-config` directory installed **before** `ResetTestState()`, not after — installed after,
  the capture is built from whatever personal defaults the running machine has saved).
  `gui_unit_test` gets the same baseline, but from a different place: it shares `test_main.cpp`
  with the non-GUI targets, which cannot call into `lumice_gui_obj`, so the install lives in a
  gtest global environment only that target compiles
  (`test/unit-correctness/gui/gui_unit_test_env.cpp`). Without it the process-wide source stays at
  `kAutoDetect` — the unset value, `src/gui/user_defaults.cpp` — which resolves to the developer's
  real OS config directory, so any case reaching the no-arg `MakeNewDocumentState()` reads
  whatever that machine has saved. A green run is not evidence the isolation holds; point `HOME`
  at a directory containing a `user_defaults.json` and re-run to check.
- Windows physical-desktop validation uses `scripts/win_remote_test.sh` together with `scripts/win_test_watcher.ps1`.
- Performance diagnostics and workflows are documented in `doc/performance-testing.md`.

### GUI Test Reference Regeneration (reference groups)

Reference images for a group whose scenes render a simulated (i.e. stochastic) frame are
pixel-averaged means of N=10 renders, which suppresses per-run noise; per-scene PSNR thresholds
are then `mean − max(4σ, 1.0 dB)`, floored to 0.5 dB precision. Which of the two terms binds is
worth knowing before reading a threshold: every scene shipping today calibrates at 4σ well under
1 dB, so in practice all of them are `mean − 1.0 dB` and the sigma margin is dormant (see
`MIN_MARGIN_DB` in the driver for why the floor exists at all). Both phases sample
**full-suite** runs under `scripts/build.sh`'s correctness-pool invocation, never a single group
in isolation: isolated runs measured 0.34–0.62 dB optimistic, which is how the since-retired
`auto_ev` references once ended up flaking. The driver has no switch for this. `lens_proj` is
currently the only stochastic group; the other three are deterministic and take the 40 dB floor
described further down.

The `lens_proj` references cover the preview fragment shader's projection math
(`src/gui/preview_renderer.cpp`: `linearInverse` / `fisheyeInverse` / `dualFisheyeInverse` /
`rectangularInverse`) via an off-screen FBO export, independent of window size or layout — see the
docking-coupling note below. Six scenes: one per projection branch (`linear`,
`fisheye_equal_area_120`, `fisheye_orthographic_180`, `dual_fisheye_equal_area_full`,
`rectangular`), plus `overlay_ea`, which reuses the equal-area branch at elevation 45° with the
zenith/nadir markers and coordinate grid enabled — it is this repo's only committed pixel coverage
of the `overlayAuxLines()` stage, and the one scene kept when the `auto_ev` group was retired.
Regen trigger: any change to that projection math, to the overlay drawing, or to
`export_fbo_renderer.cpp`'s render path. Command:
`python scripts/regen_gui_test_refs.py --group lens_proj`. Threshold backfill: the
`psnr_threshold` field of each `kScenes[]` row in `test/gui/visual/test_gui_lens_projection.cpp`.

The `modal_layout` references cover the edit-modal's internal control layout
(`src/gui/edit_modals.cpp`) via an on-screen capture of the live "Edit Entry" window rectangle — see
the docking-coupling note below. All four scenes are deterministic (no simulation, no RNG) and
compare pixel-identical, so their thresholds sit at the shared 40 dB deterministic floor rather than
a calibrated mean − 4σ. Regen trigger: any layout change to the edit modal (slider/input widths,
property-table columns, control ordering, auto-resize behavior), or a harness window size / font
atlas / ImGui style change. Command: `python scripts/regen_gui_test_refs.py --group modal_layout`.
Threshold backfill: the `psnr_threshold` field of each `kScenes[]` row in
`test/gui/visual/test_gui_modal_layout.cpp` — normally left at `kDeterministicThresholdDb` unless a
scene stops comparing pixel-identical.

The `defaults_panel_layout` references cover the `Settings` modal
(`src/gui/defaults_panel.cpp`) through the same on-screen sub-region capture as `modal_layout`, and
inherit the same docking coupling. Six deterministic scenes at the 40 dB floor: four over the merged
settings list (`pending_changes` / `other_expanded` / `filtered` / `no_changes` — the two-section
diff/adopt split these names once referred to was merged into one list with an inline-edited "Current
value" column; see `doc/gui-state-governance.md` §8.5) and two over the preset library
(`presets_expanded` / `presets_warning`, one preset unfolded to show the nine typed cells, the live
std input and the warning column beside it). Every scene installs an explicit, freshly emptied
user-config directory **before** `ResetTestState()` — installed after, the capture is built from
whatever personal defaults the running machine has saved, which is worth 20.7 dB on a scene that
looks isolated. Regen trigger: any layout change to the panel's section headers, the settings
table's columns, the preset table's columns, or the pinned action row. Command:
`python scripts/regen_gui_test_refs.py --group defaults_panel_layout`. Threshold backfill: the
`psnr_threshold` field of each `kScenes[]` row in `test/gui/visual/test_gui_defaults_panel.cpp`.

**`--keep-export-png` flag** — When passed to `gui_test`, `CheckAgainstReference` skips
`std::remove` so the per-run export PNGs at `/tmp/<group tmp_prefix><scene>.png` are preserved
for collection by the driver script.

**Reference groups** — the registry is `GROUPS` at the top of
`scripts/regen_gui_test_refs.py`, currently holding `capture_harness`, `lens_proj`,
`modal_layout` and `defaults_panel_layout`. A
group names the `gui_test` category it tags its output with (also the `[<tag>]` its comparisons
print and its key in `_thresholds.json`), its scenes/modes, and the `/tmp` and reference filename
prefixes. Adding a visual-regression suite means adding a `GROUPS` entry — Phase A/B themselves
are group-agnostic.
Two constraints when registering one: the key must be unique across groups (PSNR samples are
attributed by an exact match on the `[<tag>]` prefix in a shared full-suite stderr), and the
test must compare via `lumice::test::CheckAgainstReference` so Phase B can parse its PSNR line.

A scene whose frame is deterministic (no simulation, no RNG) compares pixel-identical, i.e.
`PSNR=inf`, which leaves `mean − 4σ` no finite sample. Phase B records `identical_runs` for such
scenes and reports a fixed 40 dB floor instead of a calibrated statistic — bit-exactness is not
demandable of a committed reference compared on other machines.

**Regeneration workflow:**
```bash
# Full regen of EVERY registered group (Phase A: mean-ref + Phase B: thresholds, ~20 min):
python scripts/regen_gui_test_refs.py

# One group only (recommended — regen what you changed, leave other groups' refs alone):
python scripts/regen_gui_test_refs.py --group lens_proj

# Phase A only (generate mean-ref images, then manually update thresholds):
python scripts/regen_gui_test_refs.py --group lens_proj --phase-a-only

# Phase B only (recalibrate thresholds against existing mean-refs):
python scripts/regen_gui_test_refs.py --group lens_proj --phase-b-only

# Single scene (--scene requires --group; scene names are only unique within a group):
python scripts/regen_gui_test_refs.py --group lens_proj --scene overlay_ea

# Quick smoke test (2 runs each phase):
python scripts/regen_gui_test_refs.py --group lens_proj --n 2 --n-calib 2
```

Phase B merges per group and per scene: groups and scenes not covered by the run keep their
existing entries, including their own `generated_at`. Thresholds live under
`groups.<key>.scenes` in `test/gui/references/_thresholds.json`.

After Phase B, copy the `threshold` values into the group's test source — for `lens_proj` that is
`kScenes[]` in `test/gui/visual/test_gui_lens_projection.cpp` (one threshold per scene); the
script prints the path for whichever group it ran.

**Docking coupling boundary** — relevant to a planned migration of the GUI's fixed-layout panels
(and the edit modal, from `BeginPopupModal` to a dockable window) onto ImGui's docking branch:
- The regen harness itself (readback, comparison, threshold, trigger mode) is layout-agnostic —
  Phase A/B do not know or care whether panels are docked.
- `lens_proj` renders through its own off-screen FBO (`export_fbo_renderer.cpp`'s
  `RenderExportToRgba`, allocated at the caller-supplied size), never reading the on-screen preview
  viewport. It is **fully docking-decoupled**, in both pixel content and output size — the migration
  needs no action on this group.
- `modal_layout` reads the DEFAULT framebuffer through `g_fullframe_capture`'s sub-region protocol,
  using the live on-screen rectangle of the "Edit Entry" window. Once the edit modal becomes
  dockable, its on-screen position/size source changes and these references will need a **re-shoot**:
  re-run `python scripts/regen_gui_test_refs.py --group modal_layout` (Phase A + B). No harness code
  change is required — the comparison mechanism (`CheckAgainstReference` + `g_fullframe_capture`)
  stays the same; only the captured pixels change.

## Logging and Troubleshooting

- `VERBOSE` is a project-defined log level between `DEBUG` and `INFO`.
- When debugging intermittent issues, first map the full data flow, then audit silent-return paths and add observability before changing behavior.
- If a known-good branch or prior fix exists, diff it first before starting fresh analysis.
- Do not split one decision across multiple threads or modules when a single owner can make it.

## Collaboration Constraints

- `config.json`, `test.json`, `scratchpad/`, remote test output files, and most generated artifacts are intentionally git-ignored.
- Do not use `git add -f` to force-track ignored files. If a file is ignored and you are unsure, stop and ask first.
- Reference images under `test/e2e-correctness/references/*.jpg` and `test/gui/references/*.jpg` are explicitly unignored and may be tracked normally.
- CI runs build and unit tests on branch pushes; E2E tests run on PRs and `main`.

## Documentation Index (`doc/`)

Valuable design/architecture docs live in `doc/` (tracked). Consult the relevant one
**before** redesigning a subsystem — many decisions are already reasoned out here.
(Most have a `_zh` Chinese sibling.)

- **Overview / guides**: `README.md`, `architecture.md`, `developer-guide.md`, `gui-guide.md`, `configuration.md`
- **Core / rendering architecture**: `accumulator-consumer-architecture.md`, `raypath-rayseg-architecture.md`, `raypath-symmetry.md`, `coordinate-convention.md`, `crystal-orientation-sampling.md`, `ev-pipeline-architecture.md`, `adaptive-brightness.md`, `filter-architecture.md`
  - `overlay-label-placement.md` — GUI overlay 文字 label 的 **curve-centric 放置设计**（blueprint；explore-288.5 收敛）：现 boundary-centric 5-source 的 4 缺口审计（globe/rectangular 缺经度、dual_fisheye 零 label、边缘成簇）+ curve-centric 统一模型（每曲线 walk→裁剪可见区域→边界/内部两模式）。改 overlay label 放置前先读。
  - `numerical-robustness.md` — geometric numerical-stability conventions (7 rules): avoid absolute-ε anti-pattern, prefer argmax / relative tolerances, double precision for geometry generation, single predicate owner; distilled from the extreme-wedge bug family (PR #132/#133/#135/#137). Read before adding or **modifying** any geometric predicate.
  - `crystal-geometry-representation.md` — **晶体几何生成 + 内存表达的机制层诊断与重构**（**重构已大半 as-built，PR #214**；§1 是 pre-#214 管线=诊断，§4.a/b/c 是落地状态）：缺陷族根因 = 管线把先验已知信息逐步丢弃再用数值方法重建（平面→顶点→面归组→三角化→反推面归属→反推面号），每个重建一个容差、每个容差一个真缺陷（PR #132–#209 对照表）；结构性错配 = 把参数化的 ≤20 面封闭小族当任意凸多面体处理（通用求解器仅 3 个调用点，全是六方晶体工厂）+ 三角网格/多边形面主次倒置。目标表达 = 平面 + 面存在掩码 + 面号常数表 + 闭式轮廓，扁平 POD 三后端同构。⭐**拓扑复用的失效判据难题在该表达下不存在**（判据问题只在拓扑靠数值发现时才有）。含诚实边界（凹锥/求交 ε/与「校验输出不校验输入」判据的张力）+ 动手前必答三问（消费者清点 / **等价性 gt 不能用旧求解器** / 面存在谓词形态）。**重设计几何前先读这篇；只是改谓词读 `numerical-robustness.md`**。
  - `near-pole-area-measure-sampling.md` — **近极朝向采样从"常数包络拒绝"换为"面积测度重要性采样"的设计蓝图**（explore-326 GO + scrum-328.1 收敛）：`θ=90−lat` 换元 + 严格上界 `sinθ≤θ` → 提案 `base_pdf(θ)·θ`（Gaussian=Rayleigh / Laplacian=Gamma(2,b) / uniform=常数）+ 接受 `sinθ/θ`（M=1）= **精确 + 99% 接受**（vs 当前 20-27%）。⭐AC 硬约束：parity 不可 match 旧采样器（proposed 有意更精确，修当前 Laplacian 尾部 clamp 偏差）→ 对解析目标验证。改近极采样 / Rayleigh 路 / `ComputeJacobianEnvelope` / GPU device gen sampler 前先读。
  - `gui-preview-lifecycle-architecture.md` — **后台 worker × 前台实时显示的时钟解耦设计**（多数 as-built；GUI 侧对偶于 `seam-design.md`）：诊断"跑完但 GUI 仍 Simulating"卡死的机制层根因 = 仿真生命周期被复制 + 边沿触发 + 撕裂读；四时钟解耦（显示/快照物化/batch 生产/生命周期心跳）+ epoch 统一世代键 + 电平触发 reconcile + CQS + 6 条不变量 I1–I6。**§10 落地状态表（2026-08-06 核对）**：I1/I2/I3/I5/I6 合规，I4 前半句合规、后半句（三个计数无廉价 C API 路径）明示已知已接受不修；候选第七条不变量（完成蕴含排空，机制已落地为 `LUMICE_GetDrainStatus`）待 owner 裁决是否升格。（激进）后端 push 帧/退化 poll 时钟仍是未启动的 stretch 项。改 GUI 预览生命周期 / poller-server 交接 / 完成判定前先读。
  - `gui-state-governance.md` — **GUI 状态治理设计蓝图**（explore-gui-state-governance 收敛，2026-07-11）：回答 owner 总纲"每个用户操作 → ①内部状态如何转换 ②所有相关显示如何更新"。诊断 = `GuiState` 数据集中但**状态转换散乱、无统一 owner**，且偏离几乎全部聚于 **display 通道 ↔ sim 通道的交界**（活 bug=display-time 操作借 EnsureRunning+PublishValidReset 污染 sim_state 闪 Simulating / commit↔display 字段割裂 / Revert 不重推 / 显示态与结构态挤同 struct）。目标模型三支柱 = field→tier 声明式分类器（拆 ColorClassConfig 结构态/显示态子结构）+ 每通道单一序列化器+重推纪律 + latch 派生态且 display-time 禁碰 re-sim 原语，外加单一 `ResetFrontendState(reason)` 文档重置 owner（= backlog #5）。含 5 条固化不变量 + T1–T6 scrum 拆解。**§8 用户默认值层**：把字段的 tier 档位复用为默认值资格判定的单一权威（四命名空间：单例文档默认 / 预设库 / app 偏好一期排除 / 集合区排除），含 I1–I5 五条不变量与 `ConfigSnapshot` 的边界（覆盖字段集不同，不可互相复用 struct）。改 GUI 状态转换 / 染色 display 通道 / 文档切换重置 / 仿真生命周期显示联动 / 用户个人默认值前先读。
  - `gui-custom-spectrum-and-raypath-color.md` — GUI 功能扩展设计（功能 1 自定义离散光谱已由 task-323 落地并将 `ray_num` 语义统一为总数）。**功能 2 per-raypath 颜色标记（2026-07-05 深化蓝图，未立项）**：一个物理机制三层角色 = 物理门 filter（1:1 不动）+ 色桶 = filter 的 summand（Fork C，不放宽绑定）+ 跨层 rule（带层键 component 的布尔组合，接跨层轨迹染色）。⭐地基窄而稳 = 产 per-ray component 掩码（复用 §5.1）+ 跨层前向累积（加宽 `is_prior_filter_failed_`）+ 交付 consumer；三硬承诺 = 携带原始掩码/带层键/uint64。⭐隔离契约 = binning + re-sim 边界 + rule + UI 全属地基之上、换之不动 seam。改光谱/光路染色相关前先读。**§4.8 合成算法重设计（2026-07-15 定案）**：dominant 噪声敏感根因 = argmax 不连续；定案 = dominant 保持 hard argmax（诊断视图）/ painter 改为亮度即 alpha 的 over 合成（`alpha=f(ey)=min(ey,1)`、纯色相、修「暗点压亮点变黑」黑洞 + 顺带连续）/ painter 设默认 / EV 解耦（alpha 用 self-anchor、display EV 后置乘）/ composite+EV 在线性 RGB。改 `component_compositor.cpp` 前先读 §4.8。
- **C API**: `c_api.md`, `capi-lifecycle-architecture.md`
  - `capi-lifecycle-architecture.md` §9 — **结果生命周期与所有权不变量的定案记录**（scrum-capi-result-lifetime-ownership 收敛，2026-08-05）：
    不变量 = 读者必须持有结果生命周期的一份真实份额，由 `LUMICE_ResultFrame` 句柄强制；
    两次复发对照表（`34400c17` consumer buffer 在锁外被清空 / 本次 `cached_composite_results_`
    被 `std::move` 覆写——ASan 坐实的 UAF）证明**换存储位置治不了**这类缺陷，真正的修法是换所有权模型
    （可变缓存改为不可变发布帧）。⭐**AC6 三版演变**（运行时计数器→C++ RAII 门禁→照抄 API 既有
    acquire/release 家规）是本 scrum 最贵的教训：四轮 plan-review 里 6/8 条独立意见全指向前两版，
    因为评审拿 plan 对 issue 里的 AC 检验、AC 本身错时没人会质疑需求本体。诚实边界：UAF 自然发生率
    从未测定，6 次自然交织复现全部低功效阴性，不构成"不会发生"的证据。改结果生命周期 / `DoSnapshot`
    发布模型 / C API 借用契约前先读。
  - `api-layering-and-product-lines.md` — **C API 层次混杂的诊断 + 产品线扩张方向的设计讨论**（讨论记录，非定稿；2026-08-01）：
    起点"core/gui 是否拆仓"被判为问错层次（想要的解耦已成立：`src/gui/` 零 core include + 只链 C API + 有门禁），
    真发现 = **今天的 C API 把三个高度压在一个平面**（L0 引擎无独立出口 / L1 冰晕域模型 / L2 编辑器支撑——
    后者含四个**返回 JSON 键名**的函数）。⭐顺序约束 = **产品线决定 API 形状，正式发布则冻结 API**，
    故"先发库再想产品线"是把顺序做反；拆仓同属提前冻结（会把 L1|L2 这道最不该固化的缝宣布为最终答案）。
    含跨树改动实测（近 12 个月已合并代码 PR 中 37% 同时动两棵树，皆为功能纵切）、
    唯一未守住的边界（`test/gui/CMakeLists.txt:54` 链 `lumice_obj` 而非 `lumice`⇒「GUI 能否只靠 C API 活下来」目前无证据）、
    以及拆仓触发条件（真实外部消费者 / 不同授权策略 / 第二团队；且届时该拆的是 L0 而非 core|gui）。
    考虑发布动态库、设计新产品线、或再次提起拆仓前先读。
- **GPU / Metal route** (read these before touching the GPU path):
  - **🔒 设计纪律（GPU 后端实现硬约束）**：按 `seam-design.md` 蓝图走，**不要自己重新发明**。几何遍历 / 出射 seam / per-ray 旋转上传 / 单引擎大 dispatch — **复用已验证的实现**：参考当前 Metal（`gpu-single-engine-implementation.md` as-built）+ legacy `PropagateSlab`（`optics.cpp` 的 polygon-slab 遍历）。**蓝图是最终判据**：Metal/legacy 与蓝图冲突处以蓝图为准（如历史 Metal 投影焊进 trace 已被 §4.1/scrum-258 纠正，别照搬旧形态）。教训：CUDA #295 自创 Möller-Trumbore 遍历复现了 task-275~278 已解决的绝对-ε 漏面 bug（energy 0.735）；详见 `scratchpad/backlog.md`「MVP 落地后的架构发现」。
  - `seam-design.md` — **the `TraceBackend` host/device seam redesign blueprint**; §5 = single-engine, three-clock-decoupled GPU simulator (the target architecture); §3.6 "原始之罪" = why GPU must not mirror the CPU pipeline.
  - `gpu-route-history.md` — systematic retrospective of the GPU migration (#250→268): decision evolution, accumulated data assets, leftover-item ledger; §9 = single-engine arc high-point (CLI 9.5×/GUI 2.07×).
  - `trace-backend-frame-lifecycle.md` — Metal frame lifecycle (as-built, multi-MS transit via device `transit_root_kernel`, parity harness methodology, §8 DR-3 per-ray wavelength).
  - `gpu-single-engine-implementation.md` — **§0 as-built 接手须知**（scrum-267+268 完成：CLI 9.5×/GUI 2.07×/dispatch 32768/concern #2 解/DR-3 波长/R1 occupancy 640 benign）+ §5 设计推理 + §8 DR-3 决策链 + §9 关键发现；接手 GPU 路线先读 §0。
- **Perf / testing**: `performance-testing.md`, `windows-remote-testing.md`, `xyz-stats-tool.md`
  - `geometry-randomization-value-and-measurement.md` — **几何随机化"值不值钱"的两轴框架 + 偏置轴度量方法论 + stage-1 发现**（新，explore 收敛 2026-07-23）：⭐**混轴是度量原罪**——方差/效率轴（K=D vs 64，收敛多快）vs 偏置/正确性轴（ON vs OFF，确定性收敛到物理错的过锐图）；**已有 moat 数字（28–40×/15×）是方差轴的**，卖点在没测过的偏置轴。偏置发现：主峰软化 −21% / 暗区填充 +50% / 对日弱峰抑制减半，**gauss≡uniform 同 σ ⇒ 干净多分散非退化背景**，效应集中 faint/away-from-peak（两轴同指向）；plate 幻日环更丰富 + 120° 反常变亮（未解）。含**变量定律 blueprint**（`Var(K)=(σ̄²_w+K·σ²_b)/N`，`K*=√((σ̄²_w/σ²_b)·(C/c))` + 三结构不变量 + 护栏红线）+ 度量方法论（散射角空间 / 天顶-rectangular / plate 幻日环 / uniform disentangle / 积分能量纪律 / C API no-filter 读 xyz_buffer）。设计几何随机化质量/吞吐 gate 前先读。
  - `geometry-randomization-perf.md` — **几何随机化性能的成本模型 + 前沿图**（**顶部有 post-闭式更正段**）：真正指标 = equal-error throughput（变量轴，非 raw）；⚠️「随机比确定性慢 15.7×」是冷 vs 暖 CUDA context-init 假象；⭐**闭式落地（PR #214）后 C（拓扑复用）是错杠杆**（只碰 92ns compute=构造 5%，「B+C 不可分」作废），真杠杆 = 砍 MakeCrystal 对象构造（多晶体 ROI 最大）；⭐**K=64 惩罚场景相关 0.25–0.43、多晶体更差**（非普适 0.37×），K=D 跨场景免费。含「压到极限没有」核对清单 + 测量纪律。优化几何随机化路径前先读，配 `geometry-randomization-value-and-measurement.md` 一起。
  - `gpu-remote-cuda-build-testing.md` — **dev49 + win-builder 现成 recipe**（CUDA build + parity/正确性
    验证的两机操作手册：源码同步、docker/BuildTools 工具链、`LUMICE_HAS_CUDA` un-skip 闸、parity battery
    三文件、PS-over-ssh 坑）。**任何触及 `cuda_trace_backend.*` / 三后端共享头 / SimData / simulator 的
    改动，先读这份**，别对两台机器从头摸索。与 `windows-remote-testing.md`（GUI VSync 物理桌面）场景正交。
  - `testing-architecture.md` — **authoritative test-organization spec**: verification-purpose primary axis × subsystem tag, seven layers (unit-correctness / golden-analytic / parity-cross-backend / e2e-correctness / performance / gui / regression-sentinel), the "how to add a test" decision tree, cross-cutting rules (perf denominator = legacy CPU; parity metric-masks-bugs battery; reference ownership), and the layer×subsystem physical-layout blueprint. Read before adding or reorganizing any test.
- **Engineering policy**: `env-var-policy.md` — **环境变量使用策略**: user-facing behavior switches must NOT live only in env vars (they cause silent per-machine drift / undebuggable bugs); use CLI/config/API instead. A-class runtime knobs (`LUMICE_TRACE_BACKEND` + 6 perf knobs, with file:line) vs B-class test/build infra (leave alone); three disposition rules; and the **decision gate to answer before adding any new `getenv`**. Read before introducing a new env knob.
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
- **Don't re-measure what's already de-risked.** Before running ANY benchmark or
  experiment on a recurring topic, `grep` the relevant prior `experiments.md` for that
  exact measurement — if it's there, harvest the number, do NOT re-run it. Reading a
  SUMMARY is not enough: it gives conclusions, but the failure mode is re-running the
  experiment, so check `experiments.md` at row level. A *design* explore's job is to
  de-risk the unresolved frontier (the blueprint's open §-items), not to re-confirm the
  premise the blueprint already rests on. Self-check: if your "finding" restates a
  sentence already in some prior insights/experiments, you are re-deriving — stop.
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
