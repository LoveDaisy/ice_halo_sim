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
  `src/util/` is deliberately outside that prohibition, and the exemption has a shape:
  what may live there and be included from both sides is a **pure, stateless helper that
  carries no simulation or configuration semantics** — `bit_utils.hpp`, `color_space.hpp`,
  `label_viewport_clamp.hpp`. The gate exists so the GUI cannot reach into the engine's
  data model and pin its layout by including it; a geometry function with no state does
  not offer that reach, and forcing one through the C API would only mean the rule gets
  written twice, once per side, which is the divergence the boundary is there to prevent.
  So: a rule BOTH renderers must obey, expressible without a core or config type, belongs
  in `src/util/`. Anything that needs to see a `RenderConfig`, a `SimData` or a crystal
  does not, and still goes through the C API.
- Environment variables: before adding any new `std::getenv`, apply the decision gate in
  `doc/env-var-policy.md`. User-facing behavior switches must go through CLI/config/API,
  not env vars (env causes silent per-machine drift). Env is fine for dev/experiment knobs
  (centralized + logged on startup) and test/build infra. All `LUMICE_*` env reads live in
  `src/util/env_knobs.cpp` and are enforced by `scripts/check_policies.py` (CI `policy` job +
  local pre-commit hook, installed via `./scripts/install-hooks.sh`).
  The **write** side has its own single owner, on the test side only: a test that has to set or
  clear a variable calls `lumice::test::SetEnvVar` / `UnsetEnvVar` from `test/support/env_var.hpp`,
  which is the one place in the tree holding the `_WIN32` branch (`_putenv_s` vs POSIX
  `setenv`/`unsetenv` — MSVC declares neither of the latter two). This is not enforced by a
  checker, deliberately: a bare `::setenv` is a *compile error* on MSVC, not a silent failure,
  which is the line the other four gates are drawn on. What it is not, is a compile error you
  will see — the two CUDA sources that first hit it sat behind `#if LUMICE_CUDA_ENABLED` and the
  Metal ones behind `#if defined(__APPLE__)`, so the breakage was invisible for three weeks until
  a Windows host built with both CUDA and `BUILD_TEST` on. `src/` never writes env vars at all.
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
  macOS and Windows legs still only compile it. Those two groups are the whole of it — the filter
  is **positive**, so the binary's other 36 categories (every `functional/` case and the entire
  `parity/` tag among them) are evaluated on a developer machine and nowhere else. Which groups
  may run under a software rasterizer is a measured fact, not a preference — the per-scene
  numbers, the reason `lens_proj` is excluded despite its pixels being portable, and **the
  checklist a red in this layer obliges you to follow instead of calling it a known flake** are
  all in `doc/testing-architecture.md` §4.6
  (whose numbers, as that section itself flags, were measured on an arm64 proxy for the amd64
  runner this step actually runs on — read the caveat, not just the table).
  Read that before widening the CI filter, and before dispositioning a visual-regression failure.
  A `src/gui/` test that needs no live frame still should not live in `gui_test` — but "no live
  frame" now splits two ways, not one. `test/unit-correctness/gui/` (targets
  `unit_correctness_test` / `gui_unit_test`, see below) holds a proposition about **one**
  `src/gui/` unit; `test/composition-correctness/gui/` (target `composition_correctness_test`)
  holds a proposition that needs **two or more** units cooperating — a document round trip, a
  cross-channel consistency check, a multi-step lifecycle — but still no rendered frame or
  synthesized input event. Chain length (how many units the proposition needs) and mechanical
  need (whether it needs a frame or an input event) are independent axes; conflating them is
  what used to make GUI test suite entropy — see `doc/testing-architecture.md` §1 for the full
  argument. Rule of thumb for a new `src/gui/` test: needs a rendered frame or an
  `ImGuiTestContext` → `gui_test`; needs ≥2 collaborating units but no mechanics →
  `composition_correctness_test`; a single unit's pure logic → `gui_unit_test` (or
  `unit_correctness_test` if it is header-only).
  The `gui_unit_test` target (`test/CMakeLists.txt`, inside `if(BUILD_GUI)`) links
  `lumice_gui_obj` with **no window, no GL context and no ImGui test engine**, so its cases
  really do run in CI on every platform that builds the GUI. Its sources sit in
  `test/unit-correctness/gui/` next to `unit_correctness_test`'s and carry the same
  `unit-correctness` CTest LABEL — the two targets are split on a *link* boundary (does the case
  call into `file_io.cpp` / `user_defaults.cpp`?), not a layer boundary, so `ctest -L` and
  `./scripts/test.sh` need no per-target knowledge. `composition_correctness_test` links the same
  way (`lumice_gui_obj` + `lumice_obj`, no window) but is **not** split across two targets the
  way `unit-correctness`'s `gui` subsystem is — every case there already needs `lumice_gui_obj`,
  so there is no header-only half to separate out, and it carries its own CTest LABEL
  (`composition-correctness`), not `unit-correctness`.
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
- `test/gui/parity/` is a fourth `gui_test` tag beside `functional/`, `visual/` and
  `responsiveness/`, and the one whose oracle is neither a committed image nor a widget outcome:
  **the same document rendered twice by two production paths and compared to each other**. Two
  members today. `test_gui_cli_export_parity.cpp` renders through the GUI's own per-frame
  `PreviewParams` assembly and — in a child process, the first time this tree starts one of its
  own binaries from a test — through the `Lumice` CLI fed by `BuildExportJsonOrWarn`'s output, so
  "what the GUI shows is what the exported config renders" can go red.
  `test_gui_preview_export_parity.cpp` compares the two arms one step earlier and inside one
  process: the on-screen blit (`RenderPreviewFrameAndBlit`) read back off the default framebuffer,
  against `RenderExportToRgba` fed the same frame's published `params` / `curve_labels`, so "the
  Screenshot writes the pixels the screen is showing" can go red. Three consequences worth
  knowing before adding to it: a member owns no reference asset and must never enter
  `scripts/regen_gui_test_refs.py`'s `GROUPS`; its threshold is placed against **the smallest
  break the scene must catch** (measured by breaking each field on purpose) rather than against
  `visual/`'s cross-machine 1.0 dB floor, which does not apply when both images are made in the
  same run — and where the two paths share a process and one snapshot of inputs, that smallest
  break puts the bar at **byte-exact** rather than at any dB figure at all (skipping the export's
  label layer, the shape of a real past defect, measures 44.75 dB, i.e. above `visual/`'s 40 dB
  deterministic floor: every calibrated threshold in this tree would have stayed green on it);
  and a parity comparison inherits every divergence between the two paths, so its scene
  design is constrained by measured facts (only equal-area projections are comparable across the
  CLI seam, since the CLI bakes the projection's solid-angle Jacobian and the GUI's resampling of
  an equal-area texture does not; and a scene built on the 8-bit `UploadTexture` path is blind to
  exposure, sky and vignetting divergence, because that texture mode's shader branch reads none of
  them) rather than by preference. `doc/testing-architecture.md` §4.10 has the full statement,
  including the CMake shape a second child-process test would need. A fourth thing, easiest of all
  to get wrong because it is invisible from a green pipeline: **no CI job runs this tag today**.
  The one leg that executes `gui_test` names a positive filter of two reference groups that does
  not include it, and the `E2E Slow` legs build with `-DBUILD_GUI=OFF` and have no such binary, so
  `./scripts/test.sh {quick,full,pr}` on a machine with a GL context is the only place either
  fixture is ever evaluated. `doc/testing-architecture.md` §7.5 has the mechanism, the measured cost of
  enabling it, and why the llvmpipe leg is not currently the right home for it.
- `scripts/check_loop_fatal_asserts.py` is a **fourth** diff-scoped entry point, alongside
  `check_policies.py`, `check_new_refs.py`, and `check_new_gui_tests.py` above (same "checker is
  the rule" discipline; CI `new-refs` job on PRs vs merge-base, pre-commit hook on the staged
  diff). Scope is `test/` broadly, not just `test/gui/`. The rule: in a scope that executes
  repeatedly, a non-terminating error report (gtest `ASSERT_*` / this repo's fatal `IM_CHECK*`,
  or the non-fatal `IM_ERRORF`) must not be followed by code that keeps driving the same test
  context — a fatal assert directly inside a `for` loop body silently hides every row after the
  first failure, and a non-fatal report followed by continued driving cascades false reds off an
  already-invalid state. See `doc/testing-architecture.md` §4.9 for the full rule statement and
  the four defect-shape generalizations (syntax → fatality → loop order → parameter binding) that
  motivated it — a checker for only the first of those reads a clean scan as "does not occur
  here" on exactly the inputs it was meant to catch.
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

    What each row **costs** in measured seconds, what each scope catches that the cheaper one is
    structurally unable to report, and the budget rule naming who has to answer for the spend and
    when, are in `doc/testing-architecture.md` §7. They are deliberately not duplicated here: two
    copies of a measurement drift apart, and the copy in this table would be the stale one.

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
docking-coupling note below. Nine scenes. Five are one per projection branch (`linear`,
`fisheye_equal_area_120`, `fisheye_orthographic_180`, `dual_fisheye_equal_area_full`,
`rectangular`). The other four each REUSE an already-covered projection branch in order to cover a
different shader stage on top of it, which is why they are exceptions to the one-scene-per-branch
rule rather than additions to it:
`overlay_ea` (equal-area at elevation 45° with the zenith/nadir markers and coordinate grid on) is
this repo's only committed pixel coverage of the `overlayAuxLines()` stage, and the one scene kept
when the `auto_ev` group was retired; `fisheye_equal_area_120_border` and
`dual_fisheye_equal_area_full_border` cover `overlayLensBorder()`, one per shader branch that can
draw a border (the single-lens radius formula and the dual-fisheye clip circle);
`sky_colour_ea_180` covers the background fill and the gate it sits behind — equal-area at fov 180
on a deliberately non-square 256×192 canvas with `visible: upper` and the camera on the horizon, so
one frame carries all three regions (sky, the half-sky `visible` discards, and outside the image
circle). It is the only scene in the group whose background is not black, and that is the point:
with the black default, "painted where it should not have" and "left black" are the same pixels.
Regen trigger: any change to that projection math, to the overlay drawing, to the background fill
or its gate, or to `export_fbo_renderer.cpp`'s render path. Command:
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

**`--keep-export-png` / `--export-dir` flags** — `--keep-export-png` makes
`CheckAgainstReference` skip its `std::remove`, so the per-run export PNGs survive for the driver
to collect. `--export-dir <path>` says where every scratch file this suite writes goes — capture
PNGs, round-tripped `.lmc`, exported JSON. The driver passes both, and collects
`<group tmp_prefix><scene>.png` from the directory it just named.

Without `--export-dir`, `gui_test` writes to a **per-process** subdirectory of the platform temp
directory (`GuiTestTempPath`, `test/gui/test_gui_shared.hpp`). Two consequences worth knowing:
the suite has no hardcoded `/tmp`, so its scratch paths are valid on Windows; and two `gui_test`
processes cannot collide on a fixed filename, which is what a sharded correctness pool would
otherwise do to itself. The group's `tmp_prefix` is now a **filename** prefix only — the
directory is not part of that contract.

**Reference groups** — the registry is `GROUPS` at the top of
`scripts/regen_gui_test_refs.py`, currently holding `capture_harness`, `lens_proj`,
`modal_layout` and `defaults_panel_layout`. A
group names the `gui_test` category it tags its output with (also the `[<tag>]` its comparisons
print and its key in `_thresholds.json`), its scenes/modes, and the export and reference filename
prefixes. Adding a visual-regression suite means adding a `GROUPS` entry — Phase A/B themselves
are group-agnostic.
Two constraints when registering one: the key must be unique across groups (PSNR samples are
attributed by an exact match on the `[<tag>]` prefix in a shared full-suite stderr), and the
test must compare via `lumice::test::CheckAgainstReference` so Phase B can parse its PSNR line.

Not every `CheckAgainstReference` caller is in `GROUPS`. The `visual` category
(`test/gui/visual/test_preview_pixels.cpp`: `crystal_preview_prism/pyramid/wireframe/shaded`,
`left_panel`) compares at a compile-time constant (`kDeterministicThresholdDb`), not a Phase-B-
calibrated one, and its reference filenames do not follow the `<ref_prefix><scene>` convention
every `ReferenceGroup` above assumes — see the comment above `STAGING_DIR` in
`scripts/regen_gui_test_refs.py` for why registering it buys nothing and how to reshoot it by
hand. A theme/layout change that reaches the left panel or the crystal-preview FBO needs that
manual step in addition to the three `--group` commands above; running only the registered
groups leaves `visual`'s references stale with no automated signal pointing at it.

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
  - `ev-pipeline-architecture.md` §2.4/§2.5/§2.6/§2.8/§7 + `adaptive-brightness.md`/`.zh.md` §2–§3 — **两种曝光锚点（`ev_mode`: relative 默认 / absolute）的机制与契约**：`relative` 锚到**场景的天空辐亮度**（`anchor_l99_sky`，固定全天等面积平面上的 P99，每球面度）——⚠️ 这是 482.3 换掉的东西，**旧文写的「自锚到当前帧自身 P99」已作废**；`absolute` 锚到光源发射能量（`SimData::emitted_energy_` → `LUMICE_RawXyzResult::emitted_energy`），使不同 config 在同一 EV 下直接可比。⭐**双侧共锚的形状（§2.8）**：CLI **除以**自己视图的 `Ω_axis`、GUI **乘以**源纹理的 `Ω_axis`，两条链收敛到同一个逐像素式 `L·m(pos)/L99_sky`（`m` = 相对照度）——**两侧公式长得不一样是设计**，别当遗漏；`Ω_axis` 由 `LUMICE_RawXyzResult::axis_solid_angle`（v4.23）发布，**在此之前 relative 分支是本 API 唯一无法被消费者复现的公式**。⛔ GUI 侧漏乘 `Ω_axis` 值 **15 stop**，且**不表现为黑屏**——`ComputeEvAuto` 的 ±6 clamp 会把它吸收成「看起来只暗几档的正常画面」（实测代价：export_parity 三行掉 5–8 dB，其余测试全绿）。⭐**§2.8 迁移律**：`new_scale/old_scale = p99_view/(Ω_axis(view)·L99_sky)`；**每一个 relative config 都会动**，实测 13 组合 × 2 场景跨 **−2.02…+2.55 stop（24×）**，⚠️ 符号不随视场宽窄单调（`linear 20°` 一场景 −2.02、另一场景 +0.26），量的其实是`L99(取景框内)/L99(全天)`，⛔ 不得外推。⭐**换来的两条有意行为变化**：① 换视角/镜头/朝向不再改亮度；② 改输出或 `sim_resolution` 不再改亮度（旧规则下 `linear 20°` 512²→1024² 实测 0.52–0.57 stop）——即相机语义。⭐**「config 不完整决定亮度」这条旧代价现在只付掉一半**：**视角**不再共同决定亮度（已消除），但 `ray_num` 仍然共同决定（锚点仍是对累积的 MC 统计量，这正是外观随 N 稳定的来源；要完全由 config 决定就用 `absolute`）。② 欠采样场景在任何固定 EV 下都会随累积变暗，这与选哪个分母无关（两种分母的 N 标度斜率相同），`relative` 靠 N 相关的锚增益把它抵消掉，`absolute` 没有抵消项而使其显形——是诚实的 MC 行为，不是「跑越久越坏」。⭐**`absolute` 的迁移律**（§7.2，与上面那条不是一回事）：`new_scale / old_scale ≡ landed_fraction`；实测位移全天球无 filter −0.038…−0.002 stop、窄视场无 filter −0.67…−1.25 stop、filter/高 `ms_prob` −0.97…−10.47 stop。⭐**诚实边界**：`absolute` 的可比性只覆盖同 lens/FOV/分辨率，跨 lens 差一个 `landed_fraction` 比值，故意不做逐像素 `Ω_p` 归一（会重新加权改变画面外观）。⚠️**别把这条读成「`Ω_p` 处处被否决了」**：它否的是**在 core 侧把 `Ω_p` 除掉**；GUI 预览侧的毛病方向相反（重采样等面积纹理 ⇒ 没有随位置变化的 `Ω_p`），已由 §7.5 补上**相对照度**（`Ω_p` 归一到轴上值，`src/gui/preview_jacobian.hpp` + shader `relIllum*`）——两条合起来说的是同一件事：两侧都该是相机语义。§7.5 记着为何归一化取**轴上值**而非 `Ω_texel_source`：双侧共锚之后这已是恒等式（两条链代数上收敛到同一式），用绝对比值会引入一个由画布角分辨率决定的全局增益，实测 0.331×/16×，把已绿的场景打红。⭐**§2.7 测光规则（`visible` 不参与测光）现在是结构性事实而非纪律**：锚点建在一块没有 `visible`/lens/view 字段的固定全天平面上，**已经没有地方可以把掩码应用到测光上**；测试也随之升格为更强的命题`RelativeExposureIsIndependentOfThisFramesOwnPixels`（scale 只由 anchor/Ω_axis/intensity_factor 决定），并把**已退役的按视图 P99 规则复述在测试内部当控制组**（否则不变性是空命题）。§2.7 里 `visible` 那次亮度迁移（单镜头族 + `visible != full` + `relative`，最坏 −1.68 stop）⚠️ 现为**历史记录**：其机制已不可能复发，当前这一步的迁移以 §2.8 为准。⛔ **合成路（`ParticipatingExposureScale`）不在本次范围内**：它锚「参与色类 lane 的 P99」，是另一个物理问题且从来单源（server 算一次、两侧读同一个 `composite_p99_y`）——但由此产生一条新事实：**mono relative 现在与视角无关、composite relative 仍与视角有关**。改曝光归一化 / `ev_mode` / `ExposureScale()` / `emitted_energy` / **测光是否看 `visible`** / **GUI 预览的投影亮度** / **`anchor_l99_sky` 或 `axis_solid_angle` 的消费**前先读。
  - `overlay-label-placement.md` — GUI overlay 文字 label 的 **curve-centric 放置设计**（blueprint；explore-288.5 收敛）：现 boundary-centric 5-source 的 4 缺口审计（globe/rectangular 缺经度、dual_fisheye 零 label、边缘成簇）+ curve-centric 统一模型（每曲线 walk→裁剪可见区域→边界/内部两模式）。改 overlay label 放置前先读。
  - `numerical-robustness.md` — geometric numerical-stability conventions (7 rules): avoid absolute-ε anti-pattern, prefer argmax / relative tolerances, double precision for geometry generation, single predicate owner; distilled from the extreme-wedge bug family (PR #132/#133/#135/#137). Read before adding or **modifying** any geometric predicate.
  - `crystal-geometry-representation.md` — **晶体几何生成 + 内存表达的机制层诊断与重构**（**重构已大半 as-built，PR #214**；§1 是 pre-#214 管线=诊断，§4.a/b/c 是落地状态）：缺陷族根因 = 管线把先验已知信息逐步丢弃再用数值方法重建（平面→顶点→面归组→三角化→反推面归属→反推面号），每个重建一个容差、每个容差一个真缺陷（PR #132–#209 对照表）；结构性错配 = 把参数化的 ≤20 面封闭小族当任意凸多面体处理（通用求解器仅 3 个调用点，全是六方晶体工厂）+ 三角网格/多边形面主次倒置。目标表达 = 平面 + 面存在掩码 + 面号常数表 + 闭式轮廓，扁平 POD 三后端同构。⭐**拓扑复用的失效判据难题在该表达下不存在**（判据问题只在拓扑靠数值发现时才有）。含诚实边界（凹锥/求交 ε/与「校验输出不校验输入」判据的张力）+ 动手前必答三问（消费者清点 / **等价性 gt 不能用旧求解器** / 面存在谓词形态）。**重设计几何前先读这篇；只是改谓词读 `numerical-robustness.md`**。
  - `near-pole-area-measure-sampling.md` — **近极朝向采样从"常数包络拒绝"换为"面积测度重要性采样"的设计蓝图**（explore-326 GO + scrum-328.1 收敛）：`θ=90−lat` 换元 + 严格上界 `sinθ≤θ` → 提案 `base_pdf(θ)·θ`（Gaussian=Rayleigh / Laplacian=Gamma(2,b) / uniform=常数）+ 接受 `sinθ/θ`（M=1）= **精确 + 99% 接受**（vs 当前 20-27%）。⭐AC 硬约束：parity 不可 match 旧采样器（proposed 有意更精确，修当前 Laplacian 尾部 clamp 偏差）→ 对解析目标验证。改近极采样 / Rayleigh 路 / `ComputeJacobianEnvelope` / GPU device gen sampler 前先读。
  - `gui-preview-lifecycle-architecture.md` — **后台 worker × 前台实时显示的时钟解耦设计**（多数 as-built；GUI 侧对偶于 `seam-design.md`）：诊断"跑完但 GUI 仍 Simulating"卡死的机制层根因 = 仿真生命周期被复制 + 边沿触发 + 撕裂读；四时钟解耦（显示/快照物化/batch 生产/生命周期心跳）+ epoch 统一世代键 + 电平触发 reconcile + CQS + **7 条不变量 I1–I7**。**§10 落地状态表（2026-08-06 核对）**：I1/I2/I3/I5/I6 合规，I4 前半句合规、后半句（三个计数无廉价 C API 路径）明示已知已接受不修；**I7「完成蕴含排空」已由 owner 拍板升格（2026-08-06），§5 的 `Completed` 定义已同步指向它**，机制 = `LUMICE_GetDrainStatus`（消费线程发布，⛔不挂 `kIdle`）；⚠️ I7 后半句「销毁路径不得丢弃未消费批次」在 `Stop()` 路径上**尚未落实**（`Queue::Shutdown` 仍丢弃，明示决定不改）。（激进）后端 push 帧/退化 poll 时钟仍是未启动的 stretch 项。改 GUI 预览生命周期 / poller-server 交接 / 完成判定前先读。
  - `gui-state-governance.md` — **GUI 状态治理设计蓝图**（explore-gui-state-governance 收敛，2026-07-11）：回答 owner 总纲"每个用户操作 → ①内部状态如何转换 ②所有相关显示如何更新"。诊断 = `GuiState` 数据集中但**状态转换散乱、无统一 owner**，且偏离几乎全部聚于 **display 通道 ↔ sim 通道的交界**（活 bug=display-time 操作借 EnsureRunning+PublishValidReset 污染 sim_state 闪 Simulating / commit↔display 字段割裂 / Revert 不重推 / 显示态与结构态挤同 struct）。目标模型三支柱 = field→tier 声明式分类器（拆 ColorClassConfig 结构态/显示态子结构）+ 每通道单一序列化器+重推纪律 + latch 派生态且 display-time 禁碰 re-sim 原语，外加单一 `ResetFrontendState(reason)` 文档重置 owner（= backlog #5）。含 5 条固化不变量 + T1–T6 scrum 拆解。**§8 用户默认值层**：把字段的 tier 档位复用为默认值资格判定的单一权威（四命名空间：单例文档默认 / 预设库 / app 偏好一期排除 / 集合区排除），含 I1–I5 五条不变量与 `ConfigSnapshot` 的边界（覆盖字段集不同，不可互相复用 struct）。**§9 两个 Scene 编码投影**：`BuildScene` 的 `kSimCommit`（core 该产什么**纹理**供 shader 重投影）与 `kJsonExport`（该告诉 CLI 什么才能画出屏幕上这张**成品图**）**必须分叉**，含权威分叉面清单（`test_scene_commit_chain.cpp` 的 `kDivergingKeys` 是机械权威）、三处具名例外（`ray_color` / `lens_shift` / `front`，其中 `front` ⛔ 绝不可编码成 `"visible": "front"`——枚举容错会把它静默映射成 `upper`）、以及「补上注解层之后分叉面往哪边走」。⭐ 可迁移判据：**任何「两条路共用一个编码器、只在个别字段上分叉」的结构，都要问：不分叉的那些字段，对两条路是不是同一个意思？** 改 GUI 状态转换 / 染色 display 通道 / 文档切换重置 / 仿真生命周期显示联动 / 用户个人默认值 / **导出 config 的字段填充**前先读。
  - `gui-visual-language.md` — **GUI 视觉语言蓝图**（排版 / 色彩 / 尺寸节奏 / 信息形态；已由可运行原型逐项验证，2026-08-13）：诊断 = 外观**从未被设计过**（`AddFontDefault()` 的 13px 像素字体 + 零定制 `StyleColorsDark()`），且 `src/gui/main.cpp` 与 `test/gui/test_gui_main.cpp` **各自重复**一遍 style/font 初始化 ⇒ 截图与真 app 只是碰巧一致。⭐核心结论 = **精致感 = 秩序 = 重复 + 对齐 + 量化，与留白无关**；密度不是精致的敌人，不一致才是（参照应用信息密度更高却更有条理）。⭐**色相与面积是两个独立旋钮**：实测现状本就是高饱和蓝主题（输入框 216°/59%、标题条 212°/65%、面板底色纯中性），两版「加蓝」实际把蓝减掉 2–3 倍 ⇒ **色彩判断必须采样测量，目视断言曾致一次完整错误往返**。含可直接采用的数值（间距/圆角全表、调色板定案列）、四条被证伪方向（均匀加大间距 / 统一右侧标签列牺牲邻近性 / 去色中性化）、以及与面板重排的**顺序约束**（视觉语言正交可先行；控件形态与信息形态撞层，分两次做等于改两遍）。**§4 定案已落地并留在 `main` 上**（PR #271：Roboto Medium 15 构建期嵌入 + 调色板/节奏 + 语义色，单一 owner `theme.{hpp,cpp}`）；与之并行的形态重排已被内测否决并回退（见 `gui-layout-architecture.md` §8）⇒ 今天 `main` 上是**老 shell + 新视觉语言**，本文凡提「docking 迁移」读作「将来任何一次重排」。⭐**配色已被内测直接接受**（更细粒度反馈，2026-08-26；上一版索引写的「粒度未知、不构成接受的证据」已作废）；⚠️ 但被问到的只有**配色**，§4 其余条目（字体/节奏/语义色）仍只是「没有被反对」。改 GUI 外观 / 面板排版 / 控件形态前先读。
  - `gui-layout-architecture.md` — ⛔**方向已被内测否决，不是待办**（2026-08-26）：本文 §1–§4 的形态曾完整实施并合入 `main`（PR #272/#273），随后小范围内测中几乎全部用户选择回到老 shell ⇒ `main` 回退到 PR #271，形态层留在分支 `feat/new-gui-layout`（尖端 `7a66c523`；内测用的 `v4.4.2-new` 标签与 release 已删）。⭐**方法层教训（已改写）**：不是「缺一道测迁移成本的闸」——本地拿不到用户反馈是结构性事实，发版取证那个环本来就是通的；真正的偏差是**验收问错了量**：两道闸都在问「形态好不好用」，从没量过**常见任务的操作步数**与**一屏可见字段集合**，而这两个数本地就能机械得到，且正是用户点名的。⭐**反馈粒度已查明**（2026-08-26，原「粒度未知＋窄 A/B 取证」已作废）：**配色被接受，被拒的是形态**，理由具体＝不如老 plain 布局一眼看到所有信息、同时调冰晶与太阳高度要多点好几步（机制＝master–detail 一次只看得见一个对象）。⇒ ⭐**新增硬约束：将来任何一次重排，最常一起调的字段组必须保持同屏可见**（§0 骂的「分割轴任意」属实，但那条任意的轴恰好在从未被度量的「同屏可见性」这一维上是好的）。以下为原蓝图内容，仍作设计记录有效：**GUI 布局架构蓝图**（面板组织形态；已由可交互 HTML 原型验证并经 owner 上手定案，2026-08-14）：诊断 = 现状左右侧栏的**分割轴任意**（晶体在左、太阳在右却同属文档；执行与显示挤同一滚动列；最深编辑住阻塞 modal）⇒ 定案 = 按字段生命周期重排为**「文档 | 图像 | 运行」三区**（生命周期分档复用 `gui-state-governance.md` 的 field→tier 分类器——面板组织追随状态架构）：文档列 = 树 + 检视器**双面板同柱**（VS Code 形态：独立滚动/可拖分隔/节头折叠；master–detail 取代 crystal/axis/filter 编辑 modal，docking 同列双窗口原生即此行为）、执行簇进顶栏（含 dirty 芯片 + Rays ∞ 档位）、显示条挂视口下缘（Grade/Overlays/**Components 预留位**按未来住户验收）、空态天空坐标系。⭐**六条被推翻形态勿重提**（检视器放右缘=跨屏实测推翻 / 单列共享高度=压测推翻 / 树内联展开 / 锚定 popover / workspace 模式页 / 节点图）；**§6 落地自由度**明确细节不定死。⭐顺序约束 = 本文就是面板重排的设计输入，禁止「机械搬运再重组」改两遍；纯视觉语言正交先行（已由那一轮实施反向印证）。**§0 诊断与 §5 被推翻清单不随否决失效**：问题本身仍在老 shell 上为真，六条形态仍不得重提。⛔ 不得据本文重启面板重排；确要重排 / 新增面板级功能（光路成分分析 UI 等）前先读，尤其 §8。
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
    那处「唯一未守住的边界」（`gui_test` 曾链 `lumice_obj` 而非 `lumice`）——**2026-08-06 已收口**：`gui_test` 现链 `lumice_gui_obj + lumice`（`test/gui/CMakeLists.txt:73`），即 core 侧只经 C API ⇒ 这是「GUI 能只靠 C API 活下来」的第一份正面证据；余下 `gui_unit_test` / `composition_correctness_test` 链 `lumice_obj` 是 CMake 注释里写明的**具名跨层 oracle 豁免**、
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
  - `machines.md` — **角色 → 主机绑定的单一真源**：哪台机器承担「CUDA 参照机（Linux/Windows）」
    「GUI 物理桌面机」「Metal 参照机」，及其仓库路径、CUDA 版本、构建脚本。⭐**换机器只改这一份**——
    其余 recipe 正文只写角色名，不写主机名（这个分层是上一代验证机整体失效时用代价换来的：
    写死的别名与路径同时作废，把仍然有效的协议部分一起拖成可疑内容）。含三个一手实测的坑：
    **CUDA 13 编译不了本仓（`compute_61` 已被移除，而仓库 arch floor 就是它）**、
    WSL 上 GLFW 选中 Wayland 后段错误（须 `-DGLFW_BUILD_WAYLAND=OFF`）、
    GitHub 慢/Docker Hub 不通（CPM 缓存走局域网 rsync）。⚠️ 另记着一条硬约束：
    两个 CUDA 参照机角色目前由**同一台物理机**承担，一侧跑 bench 时另一侧必须闲置。
  - `gpu-remote-cuda-build-testing.md` — CUDA build + parity/正确性验证的**协议与操作步骤**（headless）。
    **任何触及 `cuda_trace_backend.*` / 三后端共享头 / SimData / simulator 的改动，先读这份**：
    `LUMICE_HAS_CUDA` un-skip 闸（须与 `LUMICE_CUDA_ENABLED` **同设**，二者语义不同）、
    parity battery 三文件、验收口径 10/10、「别信 subprocess 自报」纪律，以及 Linux/Windows 两个
    参照机角色各自的 build / parity / 冒烟命令。主机绑定见 `machines.md`。
    ⚠️ **Windows + CUDA + `BUILD_TEST=ON` 至今没有任何 CI job 走过**——主矩阵 Windows job 开
    `BUILD_TEST` 但没开 CUDA、CUDA job 开 CUDA 但没开 `BUILD_TEST`，**两半都在、交集为空**。
    这个缺口一次攒下三处 MSVC 不兼容、三周无信号；它们已修，但缺口本身还在 ⇒ 动 `test/` 或
    `bench/` 后别拿 mac/Linux 的绿推断 Windows 也绿。
    与 `windows-remote-testing.md`（GUI VSync 物理桌面）场景正交。
  - `testing-architecture.md` — **authoritative test-organization spec**: verification-purpose primary axis × subsystem tag, seven layers (unit-correctness / golden-analytic / parity-cross-backend / e2e-correctness / performance / gui / regression-sentinel), the "how to add a test" decision tree, cross-cutting rules (perf denominator = legacy CPU; parity metric-masks-bugs battery; reference ownership), and the layer×subsystem physical-layout blueprint. **§7 is the test-scope contract**: measured per-scope cost (local `quick`/`full`/`pr` and all 16 CI jobs), what each scope catches that the cheaper one structurally cannot, the budget rule (who declares expected test spend, and when they reconcile it), and why independent re-verification is a fixed-cost multiplier that makes cutting base suite cost worth more than its face value. Read before adding or reorganizing any test — and before claiming any change shortens CI.
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
- **A completeness claim carries the same burden of proof as adding code.** "Covers all
  20 panels with zero omissions", "orthogonal", "closed", "every proposition enumerated"
  read as achievements and pass review unchallenged, while their opposites ("this class is
  not worth covering", "50 cases is enough here") have to be argued for. That asymmetry is
  the default state, not a choice anyone makes, so completeness wins every conflict without
  a single person advocating for it — including conflicts against the task's own declared
  budget. Measured instance, PR #261: the pre-committed target was −30% de-commented test
  lines against a baseline of 21,336 pinned by two independent counters
  (`scripts/count_gui_test_lines.py` and `cloc --by-file`, 52/52 files zero diff). It
  landed at −8.7%, and the coverage backfill demanded by "20 panels, zero omissions"
  accounts for 1,000–1,500 lines, roughly a third of the miss — the work shape was spending
  against the metric the same task had committed to. Escape-defect density over those same
  files had already been measured before the partition was drawn and spans 8× (47.2 vs 6.2
  defects per kloc); the equal-weight-per-panel split discarded that measurement.
  So: when a plan, a scope statement, or a review comment asserts coverage of an enumerated
  surface, it must say why each member is worth covering. Where a per-member value measure
  exists or is cheap to obtain (escape-defect density, call-site count, blast radius, user
  reachability), an equal-weight partition needs a stated reason — "it is the whole set" is
  not one. This puts the burden of proof on the side that adds and the side that persists,
  which is where it belongs, and which completeness normally escapes. It is **not** a
  mandate to cut: whether a leaner suite would have let more defects escape is a
  counterfactual and untestable. What is required is the justification, not the reduction.
