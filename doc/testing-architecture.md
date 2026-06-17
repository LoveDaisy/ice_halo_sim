[中文版](testing-architecture_zh.md)

# Testing Architecture

This document is the **authoritative source** for how Lumice's tests are organized and how
to add a new one. It is a **specification**: when you write a test, it tells you which layer
it belongs to, where it lives, what oracle and threshold discipline it must follow, and which
marker/label it carries. If an implementation detail conflicts with a statement here, fix the
doc first (when the doc is wrong) or fix the test layout (when it has drifted), but never
silently diverge.

**Target audience**: contributors adding or moving tests; reviewers checking that a new test
landed in the right layer with the right oracle; anyone reorganizing the test suite.

> **Status note (read first).** This document defines the **target-state** architecture
> (purpose-primary, layer × subsystem). As of writing, the on-disk test tree is still in its
> legacy flat shape (`unit_test` / `integration_test` gtest targets, `test/e2e/` pytest,
> `LumiceGUITests`). The physical migration to `test/<layer>/<subsystem>/` happens in the
> milestone-cleanup tasks 270.3–270.7. §6 gives both the **current** and **target** columns so
> you can locate a test today and know where it is going. Until migration lands, classify new
> tests by *layer* (this doc) but place them in the current physical location for their harness.

---

## §0 Why purpose is the primary axis

The suite was historically organized by **mechanism / harness**: a flat `unit_test` gtest
binary (~30 files), an `integration_test` binary, a `test/e2e/` pytest tree, and an
imgui-engine `LumiceGUITests` binary. Within each bucket, files were laid out flat.

The categories people actually reason in — "unit", "performance", "correctness", "GUI",
"the scrum-268 acceptance gates" — are **verification purposes**, not mechanisms. Purpose and
mechanism are orthogonal, so a single purpose got shredded across buckets: Metal correctness
lived in ~4 places, performance in ~5, the G1–G4 acceptance gates in ~4. "How do I add a new
test?" had no answer, so new tests landed wherever was convenient — entropy.

The deeper reason purpose must be primary: **different purposes obey different rules.** Each
has its own oracle, its own threshold discipline, its own run cadence, and its own
false-green risk. A mechanism bucket does not encode any of these. An acceptance gate (with
pre-registered hard thresholds and anti-drift discipline) and a casual unit assert are both
"a gtest TEST()", but they are not the same *kind* of test and must not be maintained by the
same rules.

**Decision**: the primary axis is **verification purpose** (seven layers, §1). The secondary
dimension is **subsystem** (a layer-internal tag, §2). Harness/language is an implementation
detail of each layer, never the primary axis.

A layer is a **rule-homogeneous unit**: every test in a layer shares the same oracle family,
threshold discipline, and cadence. This is the test that decides layer membership — see the
`test_gui_perf` case in §1 (performance) and the cross-cutting rule §4.4.

---

## §1 The seven layers

Each layer is defined by six fields: **purpose / oracle / threshold convention / run cadence /
naming convention / physical location**. Cadence values: `CI-fast` (every push, fast leg),
`PR` (pull request), `nightly` (scheduled / local heavy), `local` (developer-run).

### §1.1 `unit-correctness`

- **Purpose**: an isolated component is correct and self-consistent (math, geometry, optics
  kernels, parsing, config snapshotting, RNG, queues, threading primitives).
- **Oracle**: hand-computed expected values, invariants, and round-trip identities asserted in
  the test itself. No cross-backend or full-pipeline dependency.
- **Threshold convention**: exact or tight numeric tolerance (`EXPECT_EQ` / `EXPECT_NEAR` with
  a small epsilon). No statistical thresholds.
- **Cadence**: `CI-fast` — every commit.
- **Naming**: `test_<component>.{cpp}`; gtest `TEST(<Component>, <behavior>)`.
- **Physical location**: target-state `test/unit-correctness/<subsystem>/`; current `unit_test`
  target.

### §1.2 `golden-analytic`

- **Purpose**: the pipeline (or a stage of it) reproduces a **closed-form physical truth** —
  an analytic value derived independently of the simulator (e.g. a projection formula, a
  normal-incidence continuation result with a known analytic answer).
- **Oracle**: a closed-form / analytically derived value — an *absolute* truth, not another
  code path. This is what distinguishes it from `parity-cross-backend` (whose oracle is
  another backend) and from `unit-correctness` (whose scope is a single component).
- **Threshold convention**: absolute tolerance against the analytic value; tolerance justified
  by the physics, not by run-to-run noise.
- **Cadence**: `CI-fast` — every commit (these are deterministic).
- **Naming**: `<Phenomenon>AnalyticTruth` / `...NormalIncidence`-style names that make the
  analytic anchor explicit.
- **Physical location**: target-state `test/golden-analytic/<subsystem>/`; currently embedded
  in `unit_test` (e.g. the analytic anchor inside `test_metal_trace_parity.cpp`) and possibly
  in `test_projection` / `test_optics`.

### §1.3 `parity-cross-backend`

- **Purpose**: a non-legacy backend (Metal; future CUDA) is statistically equivalent to the
  **legacy CPU** reference for the same scene.
- **Oracle**: **legacy CPU is ground truth.** Equivalence is asserted *not by correlation
  alone* — correlation has twice masked real bugs (scrum-267). It must be backed by the full
  metric-masks-bugs battery: cross-seed self-consistency + total energy conservation +
  golden/analytic anchor + human-eye check + revert counter-check (see §4.2).
- **Threshold convention**: statistical (correlation floor + energy-conservation bound +
  cross-seed agreement). Never a bare correlation gate.
- **Cadence**: `PR` and `nightly` (heavy variants).
- **Naming**: `test_<backend>_<aspect>_parity.{cpp,py}`; `...Parity` gtest suites.
- **Physical location**: target-state `test/parity-cross-backend/<subsystem>/`; current
  `unit_test` (`.cpp`/`.mm`) + `test/e2e/` (pytest parity tests).

### §1.4 `e2e-correctness`

- **Purpose**: the **whole CLI pipeline** runs end-to-end and produces the right image / output
  for a real config.
- **Oracle**: a tracked reference image compared by **PSNR**; or CLI exit-code + output-shape
  assertions for non-image scenarios.
- **Threshold convention**: per-scene PSNR floor (tracked in the test); exit-code / file
  non-empty checks for smoke-level coverage.
- **Cadence**: `PR` (fast subset runs `-m "not slow"`; shared-lib variants run `-m slow`).
- **Naming**: `test_<feature>.py` under `test/e2e/`.
- **Physical location**: target-state `test/e2e-correctness/`; current `test/e2e/`.

### §1.5 `performance`

- **Purpose**: a backend's **throughput** is at or above baseline — the GPU/single-engine route
  must beat legacy CPU, not merely run.
- **Oracle**: **the denominator is always legacy CPU** (the path the GUI actually runs).
  `CpuTraceBackend` is a GPU-validation reference only and must never be used as the perf
  baseline (see §4.1). Report `median` + `CoV` across repeats.
- **Threshold convention**: ratio-to-legacy-CPU floor; statistics reported with `median` and
  coefficient of variation; floors tightened once a committed bench harness provides `CoV`.
- **Cadence**: `PR` (cheap sentinels) and `nightly` (full bench sweep). The committed bench
  harness is task 270.6.
- **Naming**: `test_<backend>_throughput.py`; `scripts/bench_*` for the committed harness.
- **Physical location**: target-state `test/performance/`; current `test/e2e/` (throughput
  sentinel) + the CI `Benchmark` step (`--benchmark -f examples/bench_config.json`).
- **Boundary note**: GUI frame-latency / responsiveness tests (`test_gui_perf`) are **not** in
  this layer — their oracle is an absolute frame budget, not a ratio to legacy CPU. They belong
  to `gui` (responsiveness tag). See §4.4.

### §1.6 `gui`

- **Purpose**: the GUI is functionally correct, visually correct, and responsive. Three tags
  within one layer: **functional** (widget behavior, file ops, interaction), **visual**
  (rendered output vs reference), **responsiveness** (frame interval, commit→first-upload
  latency).
- **Oracle**: the imgui test engine drives the app; **visual** asserts against tracked
  reference images (PSNR, per-scene thresholds in `_thresholds.json`); **responsiveness**
  asserts against absolute frame-latency budgets; **functional** asserts widget/state outcomes.
- **Threshold convention**: visual = per-scene PSNR (mean−3σ over N stochastic renders, see
  AGENTS.md auto_ev regen); responsiveness = absolute latency budgets; functional = exact.
- **Cadence**: `PR`. Requires a display server unless skipped with `LUMICE_SKIP_GUI_TESTS=1`.
- **Naming**: `test_gui_<aspect>.cpp`; references under `test/gui/references/`.
- **Physical location**: target-state `test/gui/<tag>/`; current `test/gui/` (+ the
  pytest-driven `test_metal_gui_acceptance.py`, a gui-layer test that happens to run through
  the e2e harness — a textbook "layer ≠ directory" case the purpose axis exists to handle).

### §1.7 `regression-sentinel`

- **Purpose**: a specific historical bug does not come back.
- **Oracle**: the **issue's reproduction scenario** — a sentinel must reproduce the original
  failure, not a synthetic stand-in (this is a hard rule: regression tests use the real issue
  scenario, never a fabricated one).
- **Threshold convention**: the assertion that would have caught the original bug (exact, or
  the specific invariant that was violated).
- **Cadence**: `CI-fast` / `PR` depending on harness — every commit where cheap.
- **Naming**: `test_<bug-symptom>.py/.cpp`, with a comment linking the fixing commit / issue.
- **Physical location**: target-state `test/regression-sentinel/`; current `test/e2e/`
  (`test_capi_sentinel_overflow.py`, `test_ms_filter_leak.py`, `test_errors.py`).

---

## §2 Subsystem dimension (layer-internal tag)

Subsystem is the **secondary** axis: a tag *within* a layer, never a top-level bucket (a pure
subsystem axis would re-mix unit and perf, which is exactly what we are leaving behind).

| tag | boundary |
|-----|----------|
| `core` | math, optics, geometry, simulator, ray paths, filters, buffers |
| `backend` | `TraceBackend` implementations: Metal device engine, `CpuTraceBackend`, the host/device seam |
| `server` | server render loop, consumer, stats, C API bridge |
| `gui` | imgui app, panels, preview, file IO, poller |
| `config` | configuration parsing and simulation-config data |
| `util` | logger, threading, queue, arguments, color data |

How a tag is encoded depends on the layer's physical form (§6): a subdirectory
(`test/<layer>/<subsystem>/`) for layers with a natural subsystem split, or a CTest
`LABELS` / pytest marker for layers that stay flat.

---

## §3 Decision tree — "I want to add a test for X"

Route to a **layer + subsystem tag**. Concrete target names and physical paths are resolved by
the physical blueprint in §6 — this tree decides *membership*, §6 decides *placement*.

```
1. Is X a historical bug I'm preventing from recurring?
   → YES → regression-sentinel. Use the issue's repro scenario verbatim. (§1.7)
   → NO  → continue.

2. Does X need the whole CLI pipeline to run (produces an image / CLI output)?
   → YES → Is the oracle a reference image / output, or a throughput number?
            • image/output correctness → e2e-correctness (§1.4)
            • throughput vs legacy CPU → performance (§1.5)
   → NO  → continue.

3. Is X about the GUI (widget behavior, rendered view, responsiveness)?
   → YES → gui, pick a tag: functional / visual / responsiveness. (§1.6)
            (Responsiveness/frame-latency stays here, NOT performance — §4.4.)
   → NO  → continue.

4. Does X compare a non-legacy backend (Metal/CUDA) against legacy CPU?
   → YES → parity-cross-backend. Oracle = legacy CPU + the full §4.2 battery
            (correlation alone is insufficient). (§1.3)
   → NO  → continue.

5. Does X assert against a closed-form / analytic physical truth?
   → YES → golden-analytic. (§1.2)
   → NO  → unit-correctness. Tag by subsystem (core/backend/server/gui/config/util). (§1.1)
```

Then: pick the subsystem tag (§2), and place per §6. *Concrete target / path / marker: see §6.*

---

## §4 Cross-cutting rules

These hold across all layers. They encode lessons that were learned the hard way.

### §4.1 Performance denominator is legacy CPU

Any performance claim's denominator **must be legacy CPU** — the path the GUI actually runs.
`CpuTraceBackend` exists only to validate the GPU seam on non-Metal machines; it is **not** a
performance baseline and must never stand in as the denominator. Every perf assertion and every
benchmark report states its denominator explicitly and names which line it measures.

### §4.2 Parity: the metric-masks-bugs battery

Correlation between a backend and legacy CPU can be high while the backend is **wrong** —
this has happened twice (under-sampling bugs hidden behind a healthy correlation, scrum-267).
A `parity-cross-backend` test therefore must **not** rest on correlation alone. It must combine:

1. **Cross-seed self-consistency** — the backend agrees with itself across RNG seeds (catches
   under-sampling that correlation smooths over).
2. **Total energy conservation** — emitted energy is accounted for across MS layers.
3. **Golden / analytic anchor** — at least one configuration with a closed-form answer (§1.2).
4. **Human-eye check** — a rendered comparison a human actually looked at.
5. **Revert counter-check** — confirm the test *fails* when the fix is reverted (proves the
   test has teeth).

Correlation is a *smoke signal*, not a verdict. The cross-seed self-consistency + energy
conservation double gate is a deliberate scrum-267.3 reinforcement and must not be removed.
This battery is also what lets a future CUDA backend distinguish "kernel is wrong" from "both
backends agree and are both wrong".

### §4.3 Config and reference ownership

- Each reference image is **owned by exactly one layer**: `e2e-correctness` owns
  `test/e2e/references/*.jpg`; `gui` owns `test/gui/references/*.jpg` + `_thresholds.json`.
- Reference images are explicitly un-ignored in `.gitignore` and tracked normally; configs and
  most generated artifacts are git-ignored. Moving a reference path requires updating the
  un-ignore rule, the test that reads it, and any CI path assumption — together.
- Regeneration of stochastic references follows a documented procedure (GUI `auto_ev`:
  `scripts/regen_gui_test_refs.py`, see AGENTS.md). A reference is never hand-edited.

### §4.4 `performance` vs `gui`-responsiveness boundary

The `performance` layer's defining oracle is **throughput ratio to legacy CPU baseline**
(`median` + `CoV`). A test whose oracle is an **absolute frame-latency / responsiveness budget**
(e.g. `test_gui_perf`: frame interval, commit→first-upload delay, measured through the imgui
engine) does **not** share that oracle, so by the rule-homogeneity principle (§0) it belongs to
`gui` (responsiveness tag), not `performance`. "Has perf in the name" is not the test —
**oracle homogeneity is the test.**

---

## §5 Physical-layout naming conventions

Three naming systems must stay aligned across a migration (270.3–270.5):

- **CMake targets**: target-state introduces purpose-named targets in place of the flat
  `unit_test`. Until then `unit_test` / `integration_test` / `LumiceGUITests` remain.
- **CTest LABELS**: target-state adds a purpose-axis label per layer (e.g.
  `unit-correctness`, `golden-analytic`, `parity`, `performance`, `gui`,
  `regression-sentinel`); the current labels are mechanism-axis (`unit` / `integration` /
  `gui`).
- **pytest markers**: `slow` (requires shared-lib build; excluded from CI fast path) and
  `heavy` (slow + redundant parity variant; deselected per-PR via `not heavy`) are run-cadence
  markers and stay. Layer/subsystem are expressed via directory + marker in the target state.

**Migration anchor checklist (mandatory for every 270.3–270.7 move).** CI hard-codes these;
any rename/move/marker-change that misses one turns CI red:

- [ ] `ctest -R LumiceUnitTest` (and any other `-R`/`-L` selector) in `.github/workflows/ci.yml`
      still resolves after a target rename.
- [ ] pytest path arguments in `ci.yml` still resolve — the E2E-Slow matrix references specific
      files by name (`test_metal_exit_seam_parity.py` parity leg; the `--ignore=...` rest leg).
- [ ] marker selectors `-m "not slow"` / `-m "slow and not heavy"` still select the intended set.
- [ ] reference paths (`test/e2e/references/`, `test/gui/references/`) and their `.gitignore`
      un-ignore rules move together with the tests that read them.
- [ ] `release.yml` is unaffected (it runs no tests) — confirm, don't assume.

---

## §6 Existing tests → seven layers (exhaustiveness map)

This table proves the seven layers cover the entire existing suite with **no orphans**, and is
the migration source-of-truth for 270.3–270.7. The **Migration constraint** column flags
health items that must not be moved/deleted casually.

> Target-state directory rule (resolves the "subdirectory or flat?" ambiguity for 270.3):
> layers with a **natural subsystem split** (`unit-correctness`, `parity-cross-backend`,
> `golden-analytic`) use `test/<layer>/<subsystem>/`; layers whose subsystem boundary is
> fuzzy (`e2e-correctness`, `performance`, `regression-sentinel`) stay **flat** as
> `test/<layer>/`, with subsystem encoded by marker/label. The `gui` layer uses
> `test/gui/<tag>/` (functional/visual/responsiveness).

| Layer | Current C++ (unit/integration) | Current e2e (pytest) | Current gui | Migration constraint |
|-------|-------------------------------|----------------------|-------------|----------------------|
| **unit-correctness** | `test_math`, `test_geo3d`, `test_optics`†, `test_crystal`, `test_rng`, `test_queue`, `test_threading_pool`, `test_color_space`, `test_json`, `test_filter`, `test_filter_spec`, `test_config_snapshot`, `test_render_config`, `test_sim_data`, `test_simulator`, `test_cpu_info`, `test_axis_presets`, `test_slider_mapping`, `test_window_sizing`, `test_raypath_segments`, `test_reduce_raypath_audit`, `test_c_api`, `test_exit_records`, `test_ev_auto`, `test_proj`(integration), `test_integration_main` | — | — | — |
| **golden-analytic** | `test_projection`†, analytic segments inside `test_optics`†, `MultiMsContinuationNormalIncidence` (in `test_metal_trace_parity.cpp`, 2-MS analytic anchor) | — | — | †split out only after per-file confirmation of the analytic-truth boundary vs unit-correctness |
| **parity-cross-backend** | `test_metal_trace_parity`, `test_metal_root_gen`, `test_metal_trace_backend`, `test_metal_filter_match_parity`(.mm), `test_cpu_trace_backend` | `test_metal_exit_seam_parity`, `test_metal_batch_invariance`, `test_device_gen_default_path`, `test_cpu_backend_route` | — | `_parity_metrics.py` is the single source of parity metrics — **DO_NOT_MIGRATE_INDEPENDENTLY** (move with its dependents). Energy-conservation + cross-seed double gate is a 267.3 reinforcement — **DO NOT DELETE**. |
| **e2e-correctness** | — | `test_smoke`, `test_cli`, `test_raypath_equivalence` | — | — |
| **performance** | (no standalone C++ perf target; CI `Benchmark` step runs `--benchmark`) | `test_metal_throughput` | — | The `test_metal_batch_invariance` exit-conservation `xfail` is **legitimate** (worst-case drain not yet landed) — do not "fix" it by deleting. |
| **gui** | — | `test_metal_gui_acceptance` (G4; gui layer, runs via pytest harness) | `test_gui_auto_ev`, `test_gui_visual`, `test_gui_render`, `test_gui_bg`, `test_gui_crystal_renderer`, `test_gui_export`, `test_gui_import_export`, `test_gui_interaction`, `test_gui_face_number_overlay`, `test_gui_overlay_labels`, `test_gui_smoke`, `test_project_world_dir`, **`test_gui_perf` (responsiveness tag)**, `test_gui_main`/`test_screenshot`/`test_gui_shared` (harness) | `test_gui_perf` oracle = absolute frame budget (§4.4), not throughput-vs-legacy. |
| **regression-sentinel** | — | `test_capi_sentinel_overflow`, `test_ms_filter_leak`, `test_errors` | — | `test_capi_sentinel_overflow` / `test_ms_filter_leak` guard real bugs via issue repro — **DO NOT alter the scenario**. `test_ms_filter_leak` is also parity-related; its **primary** purpose is sentinel (multi-purpose → classify by primary purpose). |

**Multi-purpose tie-break rule**: when a test serves more than one purpose, classify it by its
**primary** purpose (the bug/property whose regression it most directly guards), and note the
secondary purpose in a comment. Example: `test_ms_filter_leak` → `regression-sentinel`
(primary), parity-related (secondary).

**Health items — do not over-clean (consolidated "do-not-touch" list)**:
`_parity_metrics.py` (single source), the energy-conservation + cross-seed self-consistency
double gate (267.3 corr-blind reinforcement), `test_capi_sentinel_overflow.py` and
`test_ms_filter_leak.py` (issue-repro sentinels), and the legitimate `xfail` in
`test_metal_batch_invariance.py`.

> **Legacy CPU red line**: legacy CPU is the parity ground truth (§1.3, §4.2) and the perf
> denominator (§1.5, §4.1). It and its tests are **never** a cleanup target in any layer.
