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
> (purpose-primary, layer × subsystem). The physical migration is in progress under
> milestone-cleanup tasks 270.3–270.7: 270.3 split the unit gtest layer into
> `unit_correctness_test` / `golden_analytic_test` / `parity_test`, 270.4 reorganized the
> pytest tree under `test/e2e-correctness/`, `test/parity-cross-backend/`, `test/performance/`,
> `test/gui/`, `test/regression-sentinel/`, and 270.5 (this layer) moved the GUI tests into
> `test/gui/{functional,visual,responsiveness}/` and renamed the target to `gui_test`. §6 gives
> both the **historical** and **target** columns so you can trace where a test came from.

---

## §0 Why purpose is the primary axis

The suite was historically organized by **mechanism / harness**: a flat `unit_test` gtest
binary (~30 files), an `integration_test` binary, a `test/e2e/` pytest tree, and an
imgui-engine `gui_test` (formerly `LumiceGUITests`) binary. Within each bucket, files were
laid out flat.

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

**Decision**: the primary axis is **verification purpose** (eight layers, §1). The secondary
dimension is **subsystem** (a layer-internal tag, §2). Harness/language is an implementation
detail of each layer, never the primary axis.

A layer is a **rule-homogeneous unit**: every test in a layer shares the same oracle family,
threshold discipline, and cadence. This is the test that decides layer membership — see the
`test_gui_perf` case in §1 (performance) and the cross-cutting rule §4.4.

Purpose alone does not fully separate two of the eight layers: `unit-correctness`,
`composition-correctness`, and the functional/interaction slice of `gui` can all be asked to
prove the same kind of proposition about `src/gui/` correct. What actually separates them is
two more axes, orthogonal to each other and to the layer/subsystem split above (§1.7 and §3
give the full judgment procedure):

- **Chain length** — how many `src/` units must cooperate for the proposition to be provable
  false: one (`unit-correctness`), several but short of the whole app
  (`composition-correctness`), or the whole application (`e2e-correctness`).
- **Mechanical need** — what it takes to falsify the proposition: nothing (a plain function
  call), a real rendered frame, or a synthesized input event (click/drag/keypress). Only the
  last two ever require `gui`'s harness.

These two axes were, for a long time, collapsed into one: everything touching `src/gui/` was
routed by mechanical need alone, which is why `composition-correctness` did not exist as a
named layer until propositions that needed zero mechanics but spanned multiple units were found
already living, unlabelled, inside both `unit-correctness` and `gui` (§1.2, §1.7).

---

## §1 The eight layers

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
- **Naming**: `test_<component>.{cpp}`; gtest `TEST(<Component>, <behavior>)`. Python members
  follow pytest's own convention: `test_<component>.py`, `test_<behavior>` functions.
- **Physical location**: target-state `test/unit-correctness/<subsystem>/`; current `unit_test`
  target. The layer is not C++-only: a component written in Python is tested in Python under
  the same path (`test/unit-correctness/scripts/`, run by pytest). Those files are deliberately
  **not** in `pyproject.toml`'s `testpaths` — that list is the e2e surface, and bare `pytest`
  should keep meaning "the end-to-end suite". The `policy` CI job names them explicitly, which
  also gets them this layer's every-commit cadence; the e2e job runs on PR/main only and waits
  on a build.

### §1.2 `composition-correctness`

- **Purpose**: several components, each already correct in isolation, cooperate correctly across
  a call chain that a single-unit test cannot see — a document round trip, a cross-channel
  consistency check, a multi-step lifecycle. This is `unit-correctness`'s oracle discipline
  applied to a proposition that is *about* more than one collaborating unit, not a weaker or
  looser layer.
- **Oracle**: the same family as `unit-correctness` — hand-computed expected values, invariants,
  round-trip identities asserted in the test itself. No statistical threshold, no cross-backend
  dependency, no live rendering.
- **Threshold convention**: exact or tight numeric tolerance, same as `unit-correctness`.
- **Cadence**: `CI-fast` — every commit.
- **Naming**: `test_<chain-topic>_chain.cpp`; gtest `TEST(<ChainTopic>Chain, <behavior>)`, where
  `<chain-topic>` names the collaboration (e.g. document round trip, run lifecycle), never a
  single `src/` file — a chain that spans several units does not compress into one unit's name.
- **Physical location**: `test/composition-correctness/<subsystem>/`; today populated only by
  `test/composition-correctness/gui/`.
- **Membership test (what distinguishes this layer from its two neighbors)**: does falsifying the
  proposition require **two or more collaborating `src/` units** (not one built only to construct
  a fixture for the other), **and** does it need **neither a live frame nor a synthesized input
  event**? Both must hold. Chain length alone does not send a case here — a chain that also needs
  a rendered frame or a click/drag/keypress belongs to `gui` regardless of how many units it
  spans (§1.7); a chain that needs the whole running application belongs to `e2e-correctness`
  (§1.5). See §3 for the full decision procedure and §1.7 for the worked evidence that chain
  length and mechanical need are independent — 21 cases / 621 lines were found spanning 2–4
  `src/gui/` units while requiring no runtime mechanics at all, which is what first made this
  layer's existence undeniable.

### §1.3 `golden-analytic`

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

### §1.4 `parity-cross-backend`

- **Purpose**: a non-legacy backend (Metal; future CUDA) is statistically equivalent to the
  **legacy CPU** reference for the same scene.
- **Oracle**: **legacy CPU is ground truth.** Equivalence is asserted *not by correlation
  alone* — correlation has twice masked real bugs (scrum-267). It must be backed by the full
  metric-masks-bugs battery: cross-seed self-consistency + total energy conservation +
  golden/analytic anchor + human-eye check + revert counter-check (see §4.2).
- **Threshold convention**: statistical (correlation floor + energy-conservation bound +
  cross-seed agreement). Never a bare correlation gate.
- **Cadence**: `PR`. (The `nightly` heavy variants this line used to list were deleted on
  2026-08-11 along with the `heavy` marker — no workflow has a `schedule:` trigger, so nothing
  ever ran them.)
- **Naming**: `test_<backend>_<aspect>_parity.{cpp,py}`; `...Parity` gtest suites.
- **Physical location**: target-state `test/parity-cross-backend/<subsystem>/`; current
  `unit_test` (`.cpp`/`.mm`) + `test/e2e/` (pytest parity tests).
- **Projection subsystem (315.5)**: `test/parity-cross-backend/backend/test_{metal,cuda}_projection_parity.py`
  drive one legacy-oracle comparison per LensType so that **all 11 projections** (now rendered on
  Metal + CUDA via the shared `src/core/shared/projection_shared.h::ProjectExitToPixel`) are held
  cross-backend equivalent — and confirm each type truly runs on the GPU rather than silently
  falling back to legacy CPU. Both consume the shared battery `test/e2e/_projection_battery.py`.

### §1.5 `e2e-correctness`

- **Purpose**: the **whole CLI pipeline** runs end-to-end and produces the right image / output
  for a real config.
- **Oracle**: a tracked reference image compared by **PSNR**; or CLI exit-code + output-shape
  assertions for non-image scenarios.
- **Threshold convention**: per-scene PSNR floor (tracked in the test); exit-code / file
  non-empty checks for smoke-level coverage.
- **Cadence**: `PR` (fast subset runs `-m "not slow"`; shared-lib variants run `-m slow`).
- **Naming**: `test_<feature>.py` under `test/e2e/`.
- **Physical location**: target-state `test/e2e-correctness/`; current `test/e2e/`.

### §1.6 `performance`

- **Purpose**: a backend's **throughput** is at or above baseline — the GPU/single-engine route
  must beat legacy CPU, not merely run.
- **Oracle**: **the denominator is always legacy CPU** (the path the GUI actually runs).
  `CpuTraceBackend` is a GPU-validation reference only and must never be used as the perf
  baseline (see §4.1). Report `median` + `CoV` across repeats.
- **Threshold convention**: ratio-to-legacy-CPU floor; statistics reported with `median` and
  coefficient of variation; floors tightened once a committed bench harness provides `CoV`.
- **Cadence**: `PR` (cheap sentinels — e.g. `test_metal_throughput`) and `nightly` (full bench
  sweep via `scripts/bench_throughput.py`, landed task-270.6). The committed harness is the
  standard tool for any `--benchmark`-based throughput claim in this repo; ad-hoc
  one-off scripts in `scratchpad/bench/` are not authoritative.
- **Naming**: `test_<backend>_throughput.py`; `scripts/bench_throughput.py` is the committed
  harness (matrix = {legacy, cpu_backend, metal} × heavy configs × Metal dispatch sweep,
  median+CoV, ratio denominator locked to legacy CPU).
- **Physical location**: target-state `test/performance/`; current `test/e2e/` (throughput
  sentinel) + the CI `Benchmark` step (`--benchmark -f examples/bench_config.json`).
- **Boundary note**: GUI frame-latency / responsiveness tests (`test_gui_perf`) are **not** in
  this layer — their oracle is an absolute frame budget, not a ratio to legacy CPU. They belong
  to `gui` (responsiveness tag). See §4.4.

### §1.7 `gui`

- **Purpose**: **a positive definition, not a residual one** — a proposition belongs here only
  if falsifying it requires a real rendered frame or a synthesized input event
  (click/drag/keypress/window op) through the imgui test engine. This is narrower than "anything
  touching `src/gui/`": a proposition about `src/gui/` code that needs neither belongs to
  `unit-correctness` (§1.1) or `composition-correctness` (§1.2) instead, by chain length. Two
  tags within one layer: **functional** (widget behavior and interaction — the widget reads or
  writes the right state, or reacts correctly to the input event) and **visual** (rendered output
  vs reference). A third, **responsiveness**, covers interactive delivery of the live loop —
  frame interval, commit→first-upload latency, **and ray-delivery counts such as rays/restart
  and upload_rays in steady-state / slider-drag scenarios**. The ray-delivery metrics *reflect*
  throughput but their oracle is the GUI interactive loop, not a legacy-CPU ratio — so they live
  here, not in `performance` (see §4.4).
  Earlier revisions of this layer also carried a **functional** sub-bucket for file
  I/O propositions ("file ops"); those did not share this layer's oracle (they needed neither a
  frame nor an input event) and have moved to `unit-correctness` or `composition-correctness` by
  chain length, same as any other non-mechanical proposition — see §2's name-overlap caution and
  §3.
- **Oracle**: the imgui test engine drives the app; **visual** asserts against tracked
  reference images (PSNR, per-scene thresholds in `_thresholds.json`); **responsiveness**
  asserts against absolute frame-latency budgets; **functional** asserts widget/state outcomes.
- **Threshold convention**: visual = per-scene PSNR (mean−4σ over N stochastic renders, see
  AGENTS.md lens_proj regen); responsiveness = absolute latency budgets; functional = exact.
- **Cadence**: `PR`, plus the two reference groups CI runs on every push under `xvfb-run` +
  llvmpipe (§4.6). Requires a display server unless skipped with `LUMICE_SKIP_GUI_TESTS=1`.
- **Naming**: `test_<aspect>.cpp` (functional/visual) or `test_gui_<aspect>.cpp` where the
  historical name predates this convention; references under `test/gui/references/`.
- **Physical location**: `test/gui/<tag>/` (`functional/`, `visual/`, `responsiveness/`,
  `parity/`) + the pytest-driven `test_metal_gui_acceptance.py`, a gui-layer test that happens to
  run through the e2e harness — a textbook "layer ≠ directory" case the purpose axis exists to
  handle. `parity/` is a fourth tag rather than more files under `visual/`: its cases compare two
  renders made in the same run, own no committed reference image, and must never enter
  `scripts/regen_gui_test_refs.py`'s `GROUPS` registry — see §4.10.

### §1.8 `regression-sentinel`

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
| `scripts` | repo tooling under `scripts/`: the engineering-policy gates and their diff parsing |

> **Name-overlap caution**: the `gui` *tag* here is only a layer-internal subsystem label —
> e.g. a GUI-component unit test lives at `test/unit-correctness/gui/`. It is **not** the same
> thing as the `gui` *layer* (§1.7, physical `test/gui/`), which spans functional/visual/
> responsiveness and is keyed by purpose, not subsystem. Tag ≠ layer despite the shared word.
>
> That one directory is compiled by **two** CMake targets, and the split is a LINK boundary, not
> a layer or subsystem boundary: `unit_correctness_test` links `lumice_obj` alone and takes the
> header-only cases; `gui_unit_test` additionally links `lumice_gui_obj` and takes the cases that
> call into `file_io.cpp` / `user_defaults.cpp` (e.g. `test_defaults_diff.cpp`). Same directory,
> same `unit-correctness` LABEL — only the link line differs, so `ctest -L unit-correctness`
> selects both. `gui_unit_test` creates no window and no GL context; a case that needs a live
> frame belongs in the `gui` layer (`test/gui/`, target `gui_test`) instead.
>
> The `gui` *tag* is not the only other place `src/gui/` propositions land, either: a case that
> spans ≥2 collaborating units but needs no live frame or input event is `composition-correctness`
> (§1.2), not `gui` — its own `gui`-subsystem directory is `test/composition-correctness/gui/`.
> Unlike `unit-correctness`'s `gui` tag, `composition-correctness`'s is **not** split across two
> CMake targets: `composition_correctness_test` is the only target that compiles it, since every
> case there already needs `lumice_gui_obj` (there is no header-only-and-therefore-`lumice_obj`-
> only member of a *chain* the way a single unit's test can be).

How a tag is encoded depends on the layer's physical form (§6): a subdirectory
(`test/<layer>/<subsystem>/`) for layers with a natural subsystem split, or a CTest
`LABELS` / pytest marker for layers that stay flat.

---

## §3 Decision tree — "I want to add a test for X"

Route to a **layer + subsystem tag**. Concrete target names and physical paths are resolved by
the physical blueprint in §6 — this tree decides *membership*, §6 decides *placement*.

```
1. Is X a historical bug I'm preventing from recurring?
   → YES → regression-sentinel. Use the issue's repro scenario verbatim. (§1.8)
   → NO  → continue.

2. Does X need the whole CLI pipeline to run (produces an image / CLI output)?
   (A GUI-app harness driving the whole pipeline — e.g. test_metal_gui_acceptance via
    pytest — is NOT a CLI pipeline; answer NO here and route through step 3.)
   → YES → Is the oracle a reference image / output, or a throughput number?
            • image/output correctness → e2e-correctness (§1.5)
            • throughput vs legacy CPU → performance (§1.6)
   → NO  → continue.

3. Does falsifying X require a live rendered frame, or a synthesized input event
   (click/drag/keypress/window op) through the imgui test engine?
   (Ask this about the PROPOSITION, not the code it happens to be written against today — a
    chain that could be asserted without ever touching `ctx` still answers NO here even if an
    existing case drives it through the GUI. §1.2's honest-boundary note applies.)
   → YES → gui, pick a tag: functional (widget behavior, interaction) / visual /
            responsiveness. (§1.7) (Responsiveness/frame-latency stays here, NOT
            performance — §4.4.)
   → NO  → continue.

4. Does falsifying X require ≥2 collaborating `src/` units acting together — a round trip,
   a cross-channel consistency check, a multi-step lifecycle? (A unit invoked only to build
   a fixture for the one actually under test does not count — the proposition must be ABOUT
   the collaboration, not merely exercise code that happens to call another unit.)
   → YES → composition-correctness. (§1.2)
   → NO  → continue.

5. Does X compare a non-legacy backend (Metal/CUDA) against legacy CPU?
   → YES → parity-cross-backend. Oracle = legacy CPU + the full §4.2 battery
            (correlation alone is insufficient). (§1.4)
   → NO  → continue.

6. Does X assert against a closed-form / analytic physical truth?
   → YES → golden-analytic. (§1.3)
   → NO  → unit-correctness. Tag by subsystem
            (core/backend/server/gui/config/util/scripts). (§1.1)
```

Steps 3 and 4 are the mechanical-need axis and the chain-length axis from §1's opening note,
asked in the order that matches how the old, purpose-collapsed routing used to fail: mechanical
need first, because that is the question the pre-`composition-correctness` tree already asked
(§1.7's "Purpose" now answers it *positively* instead of by exclusion) — chain length second,
so a proposition that clears step 3's NO does not fall straight into `unit-correctness` by
default the way it used to. Landing in `unit-correctness` now requires clearing step 4's NO too,
not just failing to be about the GUI.

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
3. **Golden / analytic anchor** — at least one configuration with a closed-form answer (§1.3).
4. **Human-eye check** — a rendered comparison a human actually looked at.
5. **Revert counter-check** — confirm the test *fails* when the fix is reverted (proves the
   test has teeth).

Correlation is a *smoke signal*, not a verdict. The cross-seed self-consistency + energy
conservation double gate is a deliberate scrum-267.3 reinforcement and must not be removed.
This battery is also what lets a future CUDA backend distinguish "kernel is wrong" from "both
backends agree and are both wrong".

#### §4.2.1 A differential test is structurally blind to drift its two sides share

The battery above guards against a *metric* that masks a bug. There is a second blind spot, and
it belongs to the **differential shape itself** rather than to the metric: a test that compares
two paths and asserts they agree can only see disagreement. When both paths derive from the same
authority, a change to that authority moves both sides together and the test stays green — no
matter how carefully the comparison is written.

The worked example is `JsonParserParity`
(`test/unit-correctness/server/test_json_parser_parity.cpp`): it runs a config corpus through
core's parser directly, and through the C API (parse → re-encode → parse again with core), then
asserts the two agree. Both sides read core's key table. Rename a key **in that table and in its
consumers together** and the two sides remain perfectly self-consistent, so the differential test
never reddens — while the JSON now emitted is silently incompatible with every config file
already on disk. This is measured, not hypothetical: it is what a mutation did while the
red-state criterion for PR #230 was being established.

What does catch it is a pin whose expected value is a **bare string literal** and which **never
calls the authority under test** — `CrystalSchemaKeyNames.*` in
`test/unit-correctness/config/test_json.cpp` for the `shape` and `axis` objects, and the
sync-group sub-map pins in `test/unit-correctness/config/test_crystal_sync_group.cpp`. Expressing
those expectations through `ShapeScalarSyncKeyName(...)` would make the authority table and its
own test drift together and pass forever.

**Rule.** When a change gives some domain a single source of truth for names or wire format
(the same single-source treatment is a plausible future for render, filter, and light-source
config), a differential parity test **cannot** serve as its red-state criterion. At least one
test must assert the literal wire format against something outside the code under test.

### §4.3 Config and reference ownership

- Each reference image is **owned by exactly one layer**: `e2e-correctness` owns
  `test/e2e/references/*.jpg`; `gui` owns `test/gui/references/*.jpg` + `_thresholds.json`.
- Reference images are explicitly un-ignored in `.gitignore` and tracked normally; configs and
  most generated artifacts are git-ignored. Moving a reference path requires updating the
  un-ignore rule, the test that reads it, and any CI path assumption — together.
- Regeneration of stochastic references follows a documented procedure (GUI `lens_proj`:
  `scripts/regen_gui_test_refs.py`, see AGENTS.md). A reference is never hand-edited.

### §4.4 `performance` vs `gui`-responsiveness boundary

The discriminator is the **oracle**, not whether a metric "reflects speed". The `performance`
layer's defining oracle is **throughput ratio to legacy CPU baseline** (`median` + `CoV`) — it
always has a legacy-CPU denominator. A test whose oracle is the **GUI interactive loop measured
against an absolute budget through the imgui engine** does **not** share that oracle, so by the
rule-homogeneity principle (§0) it belongs to `gui` (responsiveness tag), not `performance`.

This explicitly includes the **throughput-flavored** GUI metrics, not just latency ones:
`test_gui_perf` measures frame interval and commit→first-upload delay (latency) **and**
rays/restart, upload_rays in its steady-state / slider-drag scenarios (ray-delivery counts).
The ray-delivery counts *look like* throughput, but their oracle is "how much the live preview
delivered under real-time constraints (poller cadence, commit interval, texture hold)" — an
absolute interactive budget with **no legacy-CPU denominator** (historically tracked as
GUI-regime absolute deltas like "rays 18K→79K", never a ratio to legacy). Putting them in
`performance` would force them under perf's legacy-CPU-denominated discipline, which they do not
satisfy — a rule-heterogeneous member. So: **gui (responsiveness).**

The throughput that *does* belong to `performance` is the legacy-CPU-denominated kind: the
committed bench harness (270.6) and `test_metal_throughput`. **"Reflects performance" is not the
test; "oracle = ratio to legacy CPU" is.**

### §4.5 Enforcement: the `gui_test` / `gui_unit_test` split is a gate, not a convention

§1.7 and §5 place a `src/gui/` test by whether it needs a live frame. That placement decides
whether the test runs in CI at all — `gui_test` needs a display, which only one runner leg
supplies and only for the reference groups named in §4.6, while `gui_unit_test` links the same
object library with no window, no GL context and no ImGui test engine, so it runs on every
platform that builds the GUI. Left as a convention the
split does not survive: the path of least resistance runs the wrong way, because adding to
`gui_test` is the muscle memory and it always compiles. A convention that loses to the default
path is not a boundary, it is a preference.

So the boundary is enforced on what a change adds, by `scripts/check_new_gui_tests.py` — a
third diff-scoped entry point beside `check_policies.py` (whole-tree) and `check_new_refs.py`
(prose). It runs in the CI `new-refs` job on PRs against the merge-base, and in the pre-commit
hook against the staged diff. It rejects a case the change brought into existence whose
`TestFunc` lambda states that it does not drive the GUI — either by taking an anonymous
`ImGuiTestContext*`, or by marking the parameter `IM_UNUSED` and never mentioning it again.
Both are explicit statements, one the compiler's and one the author's; the rule rests on those
rather than on inferring intent from a body, which is what keeps its false-positive rate at
zero over this repo's history.

The full criterion, what it deliberately does not catch, and the no-inline-exemption rule live
in `AGENTS.md` under "Testing and Platform Notes" — one authority, cited here rather than
restated, so the two cannot drift apart. Two properties are worth knowing at this level:

- **"Newly registered" is an identity question, not a line-number one.** A case is new when its
  `category/name` did not exist under `test/gui/` before. Reading the line diff instead fails in
  both directions and was measured doing so: the rejected signatures are boilerplate, so a
  change that deletes cases and adds one lets git pair the new signature line with a deleted
  identical one and the gate misses it; while anchoring on the `IM_REGISTER_TEST` line — the one
  line that cannot alias, because it carries the name — bills untouched cases that merely
  shifted when a neighbour was deleted. Identity is immune to both.
- **The gate is diff-scoped because the existing body cannot be zeroed out.** 26 cases under
  `test/gui/` match a rejected shape today (8 anonymous, 18 marked `IM_UNUSED`), so a whole-tree
  gate could not start green; a frozen baseline is the alternative, and it is an asset someone
  must keep correct forever. Same trade, for the same reason, as its prose-scanning sibling.
  Note the shape alone does not establish those 26 are misplaced — a case can need a frame
  without touching `ctx`, which is exactly why the gate only speaks about cases being added and
  why its message proposes rather than asserts the move.

### §4.6 A red in the GUI visual-regression layer: what it means and what to do

Detection power for this layer is a product of two factors — P(a break turns it red) ×
P(a red is taken seriously) — and only the first has ever been engineered. The second was
measured directly: across the five merged PRs whose descriptions mention a GUI visual failure
(#90, #119, #129, #168, #180), **every one was released on the same move** — re-run once,
compare against the base commit, declare a known stochastic flake, merge. #180's wording is
representative: "the one miss is the known `overlay_ea` stochastic flake, 3/3 on standalone
re-run". Once that is the default response, **a true positive is disposed of by exactly the
same motion as a false one**, and the first factor stops buying anything. Lowering the noise
floor cannot fix a second factor that is zero.

That disposal habit sits on top of a more basic question: has this layer ever caught anything,
across its whole history? Two independent searches — every commit message in this repository's
history, and the bodies of the 29 merged PRs whose description mentions a GUI visual-regression
result — turned up no record of a product defect discovered because this layer went red. Read
that as **not found**, not as **never happened**: both search surfaces are prose written by
whoever merged the change, and a catch that was never described in text would be invisible to
either. The distinction is not a hedge — it is the same honesty the rest of this section argues
for.

This section is what replaced that default. It has three parts: what actually executes in CI,
what a red obliges you to do, and — stated rather than left implicit — which parts of this
layer are still nobody's gate but yours.

**What executes in CI, and what does not.** The `Ubuntu x86_64` leg of the `build` job installs
Xvfb + Mesa's llvmpipe software rasterizer and runs `gui_test`'s `modal_layout` and
`defaults_panel_layout` reference groups under it. That leg is already a required status check,
so this needs no new branch-protection context: those ten scenes now block a merge the same way
`ctest` does. Coverage was chosen on measurement, not on convenience:

| Reference group | Linux + llvmpipe vs. the macOS-captured reference | In CI? |
|---|---|---|
| `defaults_panel_layout` (6 scenes) | **60.56–61.99 dB** on the real amd64 runner, vs. a 40 dB floor; bit-identical across 5 arm64 runs | yes |
| `modal_layout` (4 scenes) | **46.88–47.96 dB** on the real amd64 runner, vs. a 40 dB floor; bit-identical across 5 arm64 runs | yes |
| `lens_proj` (6 scenes) | 19.49–27.72 dB — every scene **above** its macOS-calibrated threshold, by 0.88–1.49 dB | no, see below |
| `screenshot`/`visual` crystal scenes (3) | 34.78–35.94 dB vs. a 40 dB floor | no — not portable |
| `capture_harness` `fullframe` | 21.92 dB vs. a 40 dB floor | no — not portable |

**The amd64 confirmation, and what it changed.** The two in-CI rows above now carry numbers from
the real `ubuntu-24.04` runner (the CI step's own log, 12/12 scenes passed). The comparison against
the arm64 figures they replaced is worth keeping, because it is the only measurement of how far
that substitution actually travelled:

| Group | arm64 container (proxy) | amd64 runner (real) | Delta |
|---|---|---|---|
| `defaults_panel_layout` | 60.56–61.99 dB | 60.56–61.99 dB | none — identical |
| `modal_layout` | 47.12–48.23 dB | 46.88–47.96 dB | ≈ −0.27 dB |

So the proxy was exact for one group and off by about a quarter of a dB for the other. Neither
outcome threatens the conclusion — the tightest real margin is 46.88 dB against a 40 dB floor,
6.88 dB of headroom — but the second row is why the caveat below was worth writing rather than
assuming portability. The remaining rows are still arm64-only: nothing runs them on amd64.

**A caveat the table cannot carry silently.** Every number below, and every arm64 figure above,
was measured on an **arm64**
Docker container (`ubuntu:24.04` + Xvfb + Mesa llvmpipe, run on an Apple Silicon host) standing in
for the **amd64** `ubuntu-24.04` GitHub Actions runner this step actually runs on — the two differ
in CPU architecture, not just in "a Linux box with a software rasterizer". That substitution was a
deliberate, bounded choice (no x86_64 machine was reachable for this work), not an oversight, and
it answers the architecture-independent questions this section leans on — does a GL 3.3 core
context come up under Xvfb+llvmpipe, does `gui_test` run to completion, is the PSNR the right order
of magnitude — soundly. What it does not give is confirmed amd64 margins: the dB figures above could
shift once run on the real runner. The mitigating fact is that the CI step now actually executing
closes this gap continuously rather than requiring a one-off confirmation — every push that touches
the `Ubuntu x86_64` leg reports real amd64 numbers in the step's own log, kept for 30 days by the
`Upload GUI visual-regression log` step in `ci.yml`, so the first real run after merge is itself
the confirmation this table is missing today. Until a
push has actually happened, treat "10/10 above the deterministic floor" as arm64-proxy evidence, not
amd64-target evidence — the distinction matters exactly because this section exists to stop numbers
from being taken on faith.

The first surprise is how well the references travel. Every reference in this repo was captured
on macOS against a Metal-backed GL stack; compared on Linux against a software rasterizer on a
different CPU architecture, the statistically-thresholded `lens_proj` group still clears
thresholds calibrated on the capture machine. "A distribution-based comparison does not depend
on the random stream, which is where cross-platform portability comes from" had been a design
intention up to that point; it now has evidence.

The second is where portability stops. The crystal-preview scenes go through
`crystal_renderer.cpp`'s shaded and wireframe rasterization, and `fullframe` reads the whole
default framebuffer with that preview inside it. Rasterization and anti-aliasing differ enough
between a GPU driver and llvmpipe to cost 4–18 dB — far past a 40 dB floor that exists to assert
near-bit-exactness. Those scenes are **meaningful only on the class of renderer that captured
them**; excluding them from CI is not a threshold being loosened to dodge a red.

**Why `lens_proj` is not in CI, and what would put it there.** Not for a rendering reason — its
pixels are portable, per the table. It is blocked on
`test/gui/visual/test_gui_lens_projection.cpp:276`, an exact equality on the ray count a
completed run reports, whose own comment asserts the number "is identical on every machine and
every rerun". Under software rendering that premise is false:

| Environment | Runs | Failures | Shortfall |
|---|---|---|---|
| macOS, GPU | 5 | 0 | — |
| Linux + llvmpipe, 12 cores | 5 | 2 | 128 rays |
| Linux + llvmpipe, 4 cores (runner-like) | 5 | 3 | 128 / 896 / 1024 rays |

The shortfall is always a whole number of 128-ray batches. The measurements above pre-date a fix
(tracked in `doc/gui-preview-lifecycle-architecture.md` §9, invariant I3) to the mechanism that
was originally diagnosed here: a poll re-read statistics only when it carried a new snapshot
generation or took the terminal-upload rescue, and the poll that observed
`LUMICE_LIFECYCLE_COMPLETED` self-paused the poller at its own end with no further reconciliation
— so a shortfall landing on that last poll was never corrected. That structure is now a level
trigger (drain-aware self-pause guard + a slow idle heartbeat that keeps reconciling), and a
post-fix white-box probe confirms the poller side of it: sampled mid-run and after completion,
the displayed count matches the backend's live count exactly, and the backend's `Completed` /
drain signals agree with the poller's — the poller is not the one going stale.

**And yet the same probe shows the 128-multiple shortfall still there** — in the backend's own
live count, not just in what the poller displays. That relocates the deficit's root cause
upstream of the display layer entirely: a completed, fully-drained run's cumulative count can
still fall short of the configured budget by a whole number of 128-ray dispatch batches, on the
production/enqueue side, before the poller ever reads it. No display-layer fix — this one
included — can close a gap that already exists in the number being displayed. Whether this
independently-discovered shortfall gets its own defect record, and whether the account of the
original `lens_proj` measurements above should be revised now that the same numeric shape is
known to have an upstream cause, are both open and awaiting a decision from the doc's owner —
not something this section resolves on its own.

**Net effect: the mechanism this section originally named is fixed, but `lens_proj` remains
unblocked from CI only in the sense that its blocker has moved, not closed.** Wiring it into a
required check today would still manufacture a false-red rate on the same two 5,000,000-ray
scenes — which is to say the "when that defect is fixed, adding `lens_proj` to the CI step's
`--filter` is the whole change" claim this section used to make no longer holds as stated; a
second, upstream defect stands in the way. The harness, the LFS fetch and the "it actually ran"
assertion are still already in place, so the mechanical part of unblocking remains a one-line
`--filter` change whenever that second defect is closed.

**Does the gate actually stop you?** Partly, and the honest shape of it matters more than the
reassuring version. The step runs inside `Ubuntu x86_64`, which *is* in
`required_status_checks.contexts`, so a red blocks the merge button. But this repository has
`enforce_admins: false` and `required_approving_review_count: 0`, and its owner is an
administrator — so the owner can merge past any red check here, including this one. That is not
a weakness this step introduces: it is the standing property of **every** required check in this
repository, and the `policy` and `new-refs` jobs are weaker still, being not required at all
despite prose elsewhere describing them as enforced. What the step buys is therefore bounded and
worth naming exactly: it moves this layer from *never executed, no signal at all* to *executed
every push, red is visible, and passing it takes a deliberate act*. That is the whole claim.
Everything below the CI line is the part no machine will catch, which is why it is written down.

**Disposition checklist — what a red obliges you to do.** Applies to every red in this layer, on
CI or on a developer's machine. "Known flake" is not a disposition; the record it leaves behind
is what the audit above found and could not act on.

1. **Does the diff touch a rendering path?** `src/gui/preview_renderer.cpp`,
   `export_fbo_renderer.cpp`, `crystal_renderer.cpp`, `edit_modals.cpp`, `defaults_panel.cpp`,
   any shader, or the reference-capture harness. If yes, the red is a regression until proven
   otherwise — the burden is on the change, not on the test.
2. **Reproduce it on the same base commit, in the same run mode.** Not "re-run and see": run it
   on the base with the same filter and the same pool (`--fixed-dt` or not) as the run that went
   red. A red that survives on the base is a pre-existing condition; a red that does not is yours.
3. **Record the numbers, not the verdict.** The PSNR of the failing scene, its threshold, and how
   many of how many repeats failed. `[<group>] <scene>: PSNR=... (threshold=...)` is printed on
   stderr by every comparison; paste it. A disposition with no number in it is not reviewable.
4. **Name the mechanism before calling it environmental.** A red that is genuinely not a
   rendering regression still has a cause, and it is nearly always locatable —
   `test_gui_lens_projection.cpp:276` above read as "stochastic flake" for as long as nobody
   looked, and turned out to be a terminal-edge race in the poller. If the mechanism cannot be
   named, the red stands.
5. **Regenerated references are a separate claim.** If the red is explained by a reference regen
   in the same PR, say which group was regenerated, with which command, and what its PSNR margins
   were afterwards. A regen inside the PR that a red is being blamed on breaks the very audit
   trail that would let a later reader distinguish leftover regen drift from a new defect — this
   was self-reported at the time in PR #93 and is the reason this rule is here.

**Explicit degradation — the platforms and groups no gate covers.** Stated so that it is a
choice rather than a silence:

- **macOS ARM64 and Windows MSVC x86_64** build `gui_test` and never run it. macOS has no Xvfb
  equivalent, and the Windows software-GL path is unverified — neither was researched here, and
  neither should be assumed easy. On those platforms this layer is a **local gate**: a red is not
  blocked by anything, and the checklist above is the whole of the enforcement.
- **`lens_proj`** is a local gate everywhere until the poller defect above is fixed — including on
  Linux, where the pixels demonstrably work.
- **The crystal-preview and full-frame scenes** are a local gate on the machine class that
  captured them, permanently. There is no plan to make them portable, and a red in them on a
  software rasterizer means nothing at all.

**Why this shape, and not one of the four single answers.** Four routes were on the table, and
the reason none of them is the answer alone is that they turned out not to be alternatives:
each covers a different slice of this layer, and the slices are the ones measurement drew, not
the ones the routes assumed.

- *Put `gui_test` in CI.* Taken, for the slice where it was demonstrated to work — ten scenes,
  bit-identical over repeats on a runner-like configuration. It was recorded as build-only
  because CI runners have no display server, and that premise held right up to the point someone
  tried Xvfb; it had been identified as a possibility years earlier and deferred, never tested.
  It is **not** taken for the other three slices, and in each case for a measured reason rather
  than a cautious one: a poller race for `lens_proj`, renderer non-portability for the crystal
  and full-frame scenes.
- *Make it a gate.* Taken where a gate can exist, which is narrower than it sounds. A gate acts
  on the shape of code; the failure being fixed here is a **disposition** — what a person does
  after seeing red. No checker can read that. Attempts to approximate it (rejecting the phrase
  "known flake" in a PR description, say) gate the wording rather than the act, and are evaded by
  rewording. So the mechanical part is exactly the part that is mechanizable — the check runs and
  is required — and the rest is stated as discipline instead of dressed up as enforcement.
- *Write the discipline down and make a red attributable.* Taken, and it is the load-bearing
  piece for everything CI does not reach: the checklist above, which replaces "re-run once,
  compare with base, merge" with something that leaves a reviewable record. It is a soft
  constraint and will fail the way soft constraints fail. It is still worth writing, because the
  alternative on those platforms is not a stronger rule — it is no rule.
- *Accept the status quo and downgrade the layer.* Taken **only** as an explicit declaration,
  never as the default. What made the old state indefensible was not that some scenes were
  unenforced; it was that the repo paid the calibration and maintenance cost of a signal it did
  not act on, without ever having decided to. The degradation list above is that decision, made
  out loud, per platform and per group.

The one thing rejected outright is the fifth option nobody proposed but everything drifts toward:
loosening a threshold so the layer stops going red. That converts "not believed" into "does not
report", which is the same zero detection power with the evidence removed.

### §4.7 Why `auto_ev` was retired, and `lens_proj` was not

Both were once a set of reference images from stochastic renders, but they varied opposite axes.
`lens_proj`'s scenes hold the simulation fixed — all six share `halo_22.json` — and vary the
display axis, one scene per inverse-projection branch. The retired `auto_ev` group did the
reverse: all ten scenes held the display axis constant (`fisheye_equal_area` at elevation 20) and
varied the simulation axis instead (spectrum, crystal type, zenith distribution, filter, MS layer
count).

That is the wrong axis for a display-regression layer to spend scenes on, and it already had an
owner elsewhere. `auto_ev`'s ten scenes drew on nine distinct configs (`overlay_ea` reused
`halo_22.json`), and seven of those nine (`halo_22`, `color`, `cza`, `filters`,
`multi_scatter`, `parhelion`, `pyramid`) are also `test/e2e-correctness/test_smoke.py` configs,
asserted there at thresholds averaging 8.7 dB stricter — and `test_smoke.py` runs through the CLI
on every PR, where `gui_test`'s `auto_ev` group was build-only. The two are not on identical
footing (the CLI configs also use a higher ray budget than the GUI's), but the axis `auto_ev`
spent nine-tenths of its scenes varying was one this repository already tested harder, on every
push, somewhere else.

The axis `auto_ev` *did* hold exclusively — the display path — turned out not to be exclusive to
it either. `test_gui_lens_projection.cpp` reads the same `snapshot_intensity` / `ev_auto` state
and sets exposure through the identical code path the retired `test_gui_auto_ev.cpp` did, and the
`EvAuto` suite (`test/unit-correctness/gui/test_gui_widget_rules.cpp`, `unit_correctness_test`
target) exercises the auto-EV computation itself with deterministic assertions that still run in
CI on all three platforms. Once the simulation-axis scenes are attributed to `test_smoke.py` and
the auto-EV-pipeline scenes to the `EvAuto` suite, the only territory `auto_ev` held alone was the
zenith/nadir markers and coordinate grid drawn by
`overlayAuxLines()` — one scene's worth. That scene, `overlay_ea`, was kept and migrated into
`lens_proj` rather than retired with the rest.

---

### §4.8 Why the GUI suite is shaped the way it is: a rule that is not data forces instance tests

**Historical diagnosis, kept for the mechanism, not the census.** The numbers immediately below
describe the suite as measured before the layer split and rewrite recorded in §4.8.2 — a whole
layer (`composition-correctness`, §1.2) has been added since, and the two GUI targets counted
here no longer hold the same cases they held at measurement time. The mechanism this section
argues for — a rule that stays control flow instead of becoming data forces its test into
instance shape — is what motivated that rewrite and is unaffected by the count going stale;
§4.8.1 and §4.8.2 record what changed as a result. The same applies to the **file names** below:
`test_gui_interaction.cpp` was the catch-all this rewrite deleted, so a reference to it here
records where those cases lived when the diagnosis was made, not where to find them today
(§6's layout table is the current map).

Of the 721 registered cases across `gui_test` and `gui_unit_test`, **23 assert a proposition
quantified over a set derived from production code** — a field registry, an enum, a capability
table — so that adding a field or a lens type extends their reach without touching them. The other
698 each assert one concrete case. That ratio, roughly 3% against 97%, is the most load-bearing
fact about this suite's shape, and the cause of it is not in the tests.

**The same file carries both patterns fifteen lines apart.** In `src/gui/app_panels.cpp`, the FOV
slider's disabled-state is read out of the field editor registry —
`ConstraintFor("renderer.fov", g_state)`, whose comment states that domain, format and
disabled-when "all come from the field editor registry rather than being written here". A test can
call the same accessor and compare against it, and four cases in `test_gui_interaction.cpp` do
exactly that. The visibility radio buttons a few lines below decide the same kind of question
inline: `if (full_sky) { ImGui::BeginDisabled(); } … ImGui::BeginDisabled(is_globe);`. There is no
`IsVisibilityWidgetEnabled(lens, widget)` to call, so a test cannot compare against the rule at
all — it can only set one lens type and inspect one widget. Nine near-identical cases result, one
per cell of the (lens × widget) grid; two of them are byte-identical once their lens constants and
expected values are normalized away.

That correspondence holds tree-wide: every invariant-shaped case in the GUI targets sits over a
registry- or table-driven subsystem (the field-tier table, the user-defaults eligibility resolver,
the defaults-diff key set, the field editor registry). Where a rule became data, the tests became
invariants; where it stayed control flow, they stayed instances.

**Instance-shaped coverage costs twice.**

*Volume.* Normalizing string, numeric and `k…` constants out of each case body and comparing bodies
by TF-IDF-weighted cosine — calibrated so that hand-verified same-grid pairs score 0.66–1.00 and
hand-verified different-mechanism pairs score 0.01–0.18 — **54% of cases sit in a clique whose
members are pairwise near-copies, and 37% are members beyond the first**. The figure is
threshold-sensitive (roughly 30–50% over a defensible range) and "compressible" is not
"deletable": collapsing a grid requires the invariant form to be expressible, which is the
production-side question above. Every one of the largest cliques is a two-parameter grid written
out one cell per case — boundary value × preset type, lens × trackball gesture, document-entry
path × blanking, projection × lens type.

*Holes that look covered.* `src/gui/file_io.cpp` has 99 `.value(key, default)` fallbacks. Twelve
derive the default from the owning struct (`RenderConfig{}.azimuth` and the like); **87 hardcode a
literal**, of which 44 are non-trivial values (`altitude` 20.0f, `diameter` 0.5f, `max_hits` 8,
`sim_resolution` 1024, `lens_type` "linear"). Commit `00fb12fc` fixed exactly one of those literals
— a `ray_num_millions` fallback that had drifted from the real default — and the remaining
same-shaped sites were not swept. Both test shapes in this area structurally miss them: a full
round-trip always writes the key, so `js.value(key, fallback)` always finds it and the fallback
never evaluates; a hand-written legacy-document test omits some keys and therefore *does* execute
some fallbacks, but asserts only the two or three fields its author had in mind.
**A line-coverage tool marks those fallbacks covered.** Re-injecting the `00fb12fc` defect turns no
case in either GUI target red.

**Working rules that follow.** These are expensive to rediscover; each was paid for at least once.

1. **A cheap signal narrows candidates; it never adjudicates.** Name similarity, lexical
   classifiers, shared-symbol intersection and orphaned JSON keys have each produced confident
   candidate sets here whose members turned out, on reading the code, to be non-redundant. The
   failure is structural, not tunable: redundancy is a semantic property ("no break exists that
   only one of them catches") and every cheap signal measures surface co-occurrence. Two tests
   sharing 61% of their production symbols can have zero assertion overlap — throughput-positive
   and pixels-lit are different questions about the same pipeline.
2. **Before trusting a "nothing found", calibrate the method on a known positive.** A whole-tree
   audit that reports a clean bill is only as good as its demonstrated sensitivity; the retired
   `auto_ev` group (§4.7) is this repository's calibration sample for suite-shape work, recoverable
   from history. A classifier that files it under "clean" is not measuring what it claims to.
3. **Judge shape by reading the body, never the name.** A case name records the author's intent;
   the property under discussion is a property of the code. There is no reliable mapping.
4. **Separate redundancy from triangulation before proposing any deletion.** Two cases that go red
   under the same break are redundant only if neither carries attribution information the other
   lacks — i.e. only if *no* break exists that just one of them catches. Where such a break does
   exist they are triangulation and must both stay; §4.2's metric-masks-bugs battery is the
   standing example, and correlation alone has twice masked a real bug here. Establish which one
   you are looking at with a red probe against the actual code, not by inspection.

**A red probe's baseline must be established under the same conditions, more than once.** A single
green full-suite run is not a baseline: the real-timing cases are load-sensitive, and a concurrent
build elsewhere on the machine is enough to turn one red. Load produces false reds, never false
greens — so "nothing caught it" survives a noisy machine, while "this suite is flaky" does not.

#### §4.8.1 Landed status (2026-08-07)

The diagnosis above named three production-side sites where a rule stayed control flow instead of
becoming data. Outcome, one line each:

| Site | Outcome |
|---|---|
| `src/gui/app_panels.cpp` visibility radio buttons (`renderer.visible`, `renderer.front`) | Landed. Both fields already had a registered predicate (`NotUnderFullSky` / `NotUnderFullSkyOrGlobe`, `src/gui/field_editor_registry.cpp:432-433`) — the same one the FOV slider fifteen lines above already consumed; the radio buttons just weren't wired to it. `app_panels.cpp:697` and `:708` now read `ConstraintFor(...)` the same way the FOV slider does. One `gui_unit_test` invariant plus one `gui_test` call-site matrix (11 lens types × 4 widgets) replaced 9 instance cases. |
| `src/gui/file_io.cpp` hardcoded `.value(key, default)` fallbacks | Landed. Hardcoded-literal share dropped from 88% to 21.2%; one universal "missing key → struct default" invariant now covers 80 fields and reproduces the `00fb12fc` regression class that the prior 721 cases caught zero of. Side finding: on the legacy core-JSON load path, the comparison point turned out to be the *core* struct's default, not the GUI struct's — core's parser seeds every field via `if (contains) get_to`, so a missing key lands on the core seed rather than a GUI-side literal. That reclassified what first looked like 6 genuine GUI/core default divergences down to 2, both left as core-side policy decisions. |
| GLSL/C++ shared projection header | **Not pursued — a NO-GO on rationale, not on feasibility.** See below. |

**The projection header decision.** A real dual compile was run: the same candidate text was accepted
both by clang (`-std=c++17 -Wall -Wextra`, exit 0) and by the driver's own GLSL compiler at `gui_test`
startup, with the scene rendering correctly — the header is buildable. What killed the proposal was the
two property tests it was meant to unlock: `inverse(forward(d)) ≈ d` closes under *any* self-consistent
convention and cannot detect a handedness flip (`test/unit-correctness/gui/test_render_handedness_guard.cpp`'s
header comment already argues this, which is why that suite picked an absolute-screen-side oracle
instead); and "GUI inverse projection agrees with CLI forward projection in handedness" already exists —
the same file's 8 cases cover it across 3 forward implementations plus one inverse read-back. The
repository holds five implementations of the projection math, and a shared header for four of them is
already in place (`src/core/shared/projection_shared.h`, used by CLI/Metal/CUDA); only the fragment-shader
copy was never folded in. `git log -L` on the five GLSL inverse-projection functions (`linearInverse`,
`fisheyeInverse`, `dualFisheyeInverse`, `rectangularInverse`, `globeInverse`) counts 14 edits total, most
recent over three months before this decision, against 54 commits to the same file in the same span —
this logic barely moves, so the cost of keeping two hand-synced copies stays low. **Trigger condition,
not a to-do:** several of those 14 edits were fixing GLSL/C++ divergence itself (commit `458a6785`:
`BuildViewMatrix`'s rotation chain, fisheye `img_radius`, dual-fisheye sign) — the two sides *have*
diverged before, concentrated in the window when the projection math was actively being tuned. If that
math re-enters active development (a docking migration changing viewport semantics, a new lens type),
re-open this decision; the low-churn premise it rests on will no longer hold.

**The suite-scope decision (owner, 2026-08-07).** The original N=20/50/100 budget proposal is replaced
by two independent axes — tier (T1 must-keep / T2 next / T3 fuller coverage) crossed with writability
(ready today / blocked on a production-side change / already satisfied / falsified) — because the three
lists were already cumulative (N=50 was "N=20 plus 30 more", N=100 was "N=50 plus 50 more, no new
invariant shapes"), so pinning a number to a tier boundary ("why 20 and not 25") was the only thing
tiering actually removes. **Scope: write T1 + T2.** Of T1's original 20 candidate invariants: one (the
round-trip identity above) is struck as untestable by construction; 9 were already satisfied, including
the handedness one just found; 2 are blocked on an observability seam that belongs to the
preview-lifecycle line, not this one; and 8 are genuinely unwritten today — each of those 8 must be
re-checked against the current suite before work starts, since the round-trip case shows an "unwritten"
label from this analysis can itself be stale. **Retirement is not a separately scoped task and never
covers a whole tier at once.** It comes attached to whichever invariant lands and is proven to cover an
instance case: landing the visibility predicate retired 8 `visibility_*` cases because each was checked
against a red probe; landing the fallback invariant retired the field-by-field default assertions it
subsumed, the same way. There is no all-at-once "721 → N" cutover — authorization to delete an instance
case comes only from a red-probe or equivalence proof pointing at that specific case, never from a tier
label. Three limits carry forward unconditionally: the compression figure above (37%, 30–50% range,
28% white-box-verified) supports only the hole-coverage axis, not overall coverage, where there is no
evidence and improvement is unlikely; T1 alone gives up six failure classes entirely (overlay-label
placement, concrete widget interaction, filter SOP grammar, color/compositing, malformed-input
degradation, multi-monitor/window-size) and is not a shippable suite by itself.

#### §4.8.2 The composition-correctness split and its line-count outcome (2026-08-10)

Both `composition-correctness` (§1.2) and the narrowed `gui` (§1.7) came out of following the
diagnosis above through to a structural conclusion: `functional`'s three-way residual definition
(widget behavior, file ops, interaction — three unrelated things united only by "not visual, not
responsiveness") was traced to its cause, `gui` was rewritten as a positive definition, and the
lines that were already making cross-unit claims while filed under a single-unit layer's name got
their own layer instead of a longer caveat paragraph.

A separate, harder target ran alongside that split: cut the suite's de-commented line count from a
21,336-line baseline by 30% (≤14,900 lines), stretch goal 40% (≤12,800). The rewrite reached
**19,485 lines / 64 files / 556 cases across the three layers now covering `src/gui/`** — an 8.7%
reduction, short of both figures. This is recorded as **not achieved**, by owner decision, for two
measured reasons, not by redefining the target or loosening what counts toward it:

1. **The coverage expansion this rewrite paid for was a one-time, non-negotiable cost.** Writing
   by panel instead of by legacy file surfaced four `src/gui/` units — together roughly a quarter
   of `src/gui/`'s own line count — that had **zero** unit-level coverage before this rewrite;
   closing that hole is what grew the suite, not redundant instance tests, and it does not recur.
2. **What remains of the gap is boilerplate density, not duplicated assertions.** A measured
   post-rewrite audit of `test/gui/` found roughly two-thirds of its lines are scene setup and
   fixture staging rather than assertions, with a meaningful share of that staging near-identical
   across cases — a shape that further compression would have to address by extracting shared
   fixtures, not by deleting coverage. The owner weighed that extraction against a readability
   cost (over-extracting fixtures can make an individual case harder to read in isolation) and
   scoped it out of this rewrite rather than force it under the line-count target.

The rewrite's other structural outcome — a gate closing a defect shape found repeatedly during
this work — is recorded in §4.9.

### §4.9 Fourth gate: a fatal assert must not sit directly inside a repeatable scope

`scripts/check_loop_fatal_asserts.py` is a **fourth** diff-scoped entry point, alongside
`check_policies.py` (whole-tree), `check_new_refs.py` (prose) and `check_new_gui_tests.py`
(AGENTS.md, "Testing and Platform Notes"). Same discipline as the other three: **the checker is
the rule, not an approximation of it** — 0 hits is the evidence a change is clean, not a
supplement to a hand-written review checklist. It runs in the CI `new-refs` job on PRs against
the merge-base and in the pre-commit hook against the staged diff, scanning added lines under
`test/` (not limited to `test/gui/` — the defect shape is not GUI-specific, only GUI-suite-heavy
in practice).

**The rule it enforces**: in a scope that executes repeatedly, a non-terminating error report
must not be followed by code that keeps driving the same test context. Concretely: a `for` loop
whose body calls a fatal assert (gtest's `ASSERT_*`, or this repo's `IM_CHECK*` from
ImGuiTestEngine) directly aborts the *enclosing function* on the first failing row — silently
hiding every row after it, which is a correctness gap in the test rather than a style
complaint. `IM_ERRORF` (this suite's non-fatal report) has the mirror failure: because it does
not return, a loop that continues past it keeps driving an already-invalid UI state, producing
cascading false reds attributed to the wrong row.

This rule generalized four times over the course of the rewrite that introduced it, each
generalization closing the same rule's next free variable rather than a new, unrelated defect:
syntactic shape (a fatal assert written directly in a loop body) → fatality (`IM_CHECK*` is
fatal, `IM_ERRORF` looked safe but shares the failure through non-termination rather than an
early return) → order (a loop that reports on iteration N and keeps driving on N+1 cascades the
same way a same-iteration report does — the loop wraps back around) → binding (a rewritten
lambda that renames its `ImGuiTestContext*` parameter away from the literal identifier the first
version of the scan matched still drives the context; the scan now binds whatever the enclosing
lambda's actual parameter name is, never a hardcoded identifier). A checker that stops at the
first of these generalizations is worse than no checker: a clean scan result is read as "this
defect shape does not occur here," and a rule with a known-wrong negative case manufactures that
false confidence on exactly the inputs it was supposed to catch.

---

### §4.10 The `parity` tag: comparing two live renders, and starting the CLI from inside `gui_test`

`test/gui/parity/` holds the one shape of GUI test whose oracle is neither a committed image nor a
widget outcome: **the same document rendered twice, by two different production paths, compared to
each other**. Today its single member is `test_gui_cli_export_parity.cpp`, which renders a document
through the GUI's own per-frame `PreviewParams` assembly and, in a child process, through the
`Lumice` CLI fed by `BuildExportJsonOrWarn`'s output — making "what the GUI shows is what the
exported config renders" a proposition that can go red.

Three things about it generalize to any future member, and are the reason it is a tag of its own
rather than more files under `visual/`:

1. **No committed asset, so no regen group.** `visual/` exists to compare against a tracked
   reference `.jpg`, which is what `scripts/regen_gui_test_refs.py`'s `GROUPS` registry
   regenerates. A parity case has nothing to regenerate: both of its images are produced by the
   run that compares them. Registering one there would be meaningless at best. The directory
   boundary states this without anyone having to read a file header.
2. **The threshold floor is a different question.** A `visual/` threshold carries a 1.0 dB floor
   because a committed reference is compared on machines that did not shoot it. A parity case
   compares two images made minutes apart on one machine, from a ray budget fixed by waiting for
   the run to COMPLETE rather than by watching a counter — so machine load cannot leak into the
   noise level, and the measured run-to-run sigma is an order of magnitude smaller (0.026–0.078 dB
   against the visual suites' ~0.05–0.16 dB on a far coarser mean). The floor that matters instead
   is **the smallest break the scene must catch**: place the threshold in the gap between the worst
   honest run and that break, and keep at least 10 sigma on the honest side. Both numbers have to
   be measured, and the second one is measured by breaking each field on purpose, one at a time.
3. **A parity comparison inherits every divergence between the two paths, not only the one under
   test.** The export-parity fixture's scene design is dictated by three that were measured rather
   than assumed — the CLI bakes a projection's solid-angle Jacobian into its pixels while the GUI
   resamples an equal-area texture and does not (so only equal-area scenes are comparable); the
   preview shader keeps inverting past a single fisheye's image circle where core stops; and
   relative-EV self-anchoring uses a different pixel population on each side. A new parity case
   should expect to spend most of its design effort here, and to state what it found.

**Starting the CLI from a test binary.** This fixture is the first place in the tree where a test
starts another of the repo's binaries as a child process. The mechanics, should a second one want
them: `test/gui/CMakeLists.txt` passes `LUMICE_CLI_BIN_PATH="$<TARGET_FILE:Lumice>"` as a compile
definition **and** declares `add_dependencies(gui_test Lumice)` — the generator expression is only
a string, so without the second line the test would happily run a stale binary or none. The call
itself is `std::system` with every path double-quoted (this repo is routinely checked out under a
directory containing a space), and the command line is echoed to stderr on failure so a red is
reproducible by hand. Each scene renders into its own subdirectory of the suite's scratch
directory, because the CLI names its output after the renderer id in the config
(`src/main.cpp` `FormatImagePath`) and two scenes would otherwise collide on one filename.

**Cadence.** Same as the rest of `gui_test`: `scripts/build.sh`'s correctness pool selects by a
NEGATIVE filter, so a new category is included by default and `./scripts/test.sh {quick,full,pr}`
runs it. CI is the exception and the trap — the one leg that runs `gui_test` passes a hand-picked
POSITIVE filter naming two reference groups, so no CI job executes this layer today. Do not read a
green CI as a green here (§4.6 has the wider argument about that leg).

---

## §5 Physical-layout naming conventions

Three naming systems must stay aligned across a migration (270.3–270.5):

- **CMake targets**: target-state introduces purpose-named targets in place of the flat
  `unit_test`. **Naming pattern: `<layer-snake>_test`** (snake_case, matching the existing
  `unit_test` / `integration_test` convention) — e.g. `unit_correctness_test`, `parity_test`,
  `golden_analytic_test`, `composition_correctness_test`; the GUI layer's target is `gui_test`
  (renamed from `LumiceGUITests` in 270.5). Whether targets are split further per subsystem (one
  `unit_correctness_test` vs `unit_correctness_core_test` + …) is **270.3's call**, but the
  *pattern* above is fixed here so the naming does not drift.
  **Exception, stated as a rule rather than as a list of cases**: when a target exists because of
  a *link* boundary rather than a layer/subsystem boundary, name it after the link dependency
  instead of `<layer-snake>_test`. `gui_test` is the original instance (it is the `gui` layer's
  target, but the name says "the one that links the GUI"); `gui_unit_test` is the second — under
  the strict pattern it would be `unit_correctness_gui_test`, which names the layer it *shares*
  with `unit_correctness_test` while hiding the only thing that actually differs, the
  `lumice_gui_obj` dependency. Layer identity lives in the CTest LABEL, which is where selectors
  read it from anyway, so the target name is free to carry the link fact instead.
- **CTest LABELS**: target-state adds a purpose-axis label per layer:
  `unit-correctness`, `composition-correctness`, `golden-analytic`, `parity` (the LABEL is the
  abbreviated form of the `parity-cross-backend` layer — the full name is verbose for CMake;
  this is the **only** abbreviated label, do not similarly truncate the others —
  `unit-correctness` must not become `unit`, which would collide with the legacy mechanism-axis
  label, and `composition-correctness` must not become `composition` for the same reason),
  `performance`, `gui`, `regression-sentinel`. The current labels are mechanism-axis (`unit` /
  `integration` / `gui`). `ctest -L "unit-correctness|composition-correctness|parity|golden-analytic"`
  is the selector `scripts/test.sh`'s `quick`/`full`/`pr` modes use to pick up all four
  non-flat layers with no per-target knowledge.
- **pytest markers**: `slow` (requires shared-lib build; excluded from CI fast path) is the one
  run-cadence marker and stays. A second marker, `heavy`, was registered until 2026-08-11: it
  named a "run locally/nightly" cadence no workflow ever supplied, so its three tests never ran
  anywhere; they and the registration were deleted together. Layer/subsystem are expressed via
  directory + marker in the target state.
- **`addopts = ["-m", "not slow"]` (`pyproject.toml`)**: bare `pytest` is pinned to the fast
  subset so the "bare pytest = e2e fast subset" claim above is structurally true rather than a
  convention callers must remember. Before this was added, bare `pytest` collected the full
  166-test set (not the 81-test fast subset `-m "not slow"` selects) despite `AGENTS.md`
  documenting it as fast-only; a runner-history sample showed callers consistently believed they
  were on the fast path while running the whole suite, which is why the fix is a gate rather
  than a doc correction (a reminder can't catch a mistake the caller doesn't know they're
  making). Command-line `-m` overrides addopts entirely (not an AND-combine), so `-m ''` remains
  the full-suite escape hatch and `-m slow` still selects the CI slow leg unchanged. Any pytest invocation that relies on the old "no `-m` = run everything under
  this path" default needs an explicit `-m` added — see `doc/gpu-remote-cuda-build-testing.md`'s
  CUDA parity recipes for one such fix.

**Migration anchor checklist (mandatory for every 270.3–270.7 move).** CI hard-codes these;
any rename/move/marker-change that misses one turns CI red:

- [ ] `ctest -R LumiceUnitTest` (and any other `-R`/`-L` selector) in `.github/workflows/ci.yml`
      still resolves after a target rename.
- [ ] pytest path arguments in `ci.yml` still resolve — the E2E-Slow matrix references specific
      files by name (`test_metal_exit_seam_parity.py` parity leg; the `--ignore=...` rest leg).
- [ ] marker selectors `-m "not slow"` / `-m slow` still select the intended set.
- [ ] reference paths (`test/e2e/references/`, `test/gui/references/`) and their `.gitignore`
      un-ignore rules move together with the tests that read them.
- [ ] `release.yml` is unaffected (it runs no tests) — confirm, don't assume.

---

## §6 Existing tests → eight layers (exhaustiveness map)

This table proves the eight layers cover the entire existing suite with **no orphans**, and is
the migration source-of-truth for 270.3–270.7. The **Migration constraint** column flags
health items that must not be moved/deleted casually.

> Target-state directory rule (resolves the "subdirectory or flat?" ambiguity for 270.3):
> layers with a **natural subsystem split** (`unit-correctness`, `composition-correctness`,
> `parity-cross-backend`, `golden-analytic`) use `test/<layer>/<subsystem>/`; layers whose
> subsystem boundary is fuzzy (`e2e-correctness`, `performance`, `regression-sentinel`) stay
> **flat** as `test/<layer>/`, with subsystem encoded by marker/label. The `gui` layer uses
> `test/gui/<tag>/` (functional/visual/responsiveness).

| Layer | Target-state path | Current C++ (unit/integration) | Current e2e (pytest) | Current gui | Migration constraint |
|-------|-------------------|-------------------------------|----------------------|-------------|----------------------|
| **unit-correctness** | `test/unit-correctness/<subsystem>/` | `test_math`, `test_geo3d`, `test_optics`†, `test_crystal`, `test_rng`, `test_queue`, `test_threading_pool`, `test_color_space`, `test_json`, `test_filter`, `test_filter_spec`, `test_config_snapshot`, `test_render_config`, `test_sim_data`, `test_simulator`, `test_cpu_info`, `test_raypath_segments`, `test_reduce_raypath_audit`, `test_c_api`, `test_exit_records`, `test_proj`(integration), `test_integration_main`; `gui` subsystem, first target `unit_correctness_test` (`lumice_obj` only, header-only-reachable): `test_axis_presets`, `test_filter_sop_grammar`, `test_gui_widget_rules` (which absorbed the former standalone `test_slider_mapping` and `test_window_sizing` files — their `SliderMapping` / `WindowSizingTest` suites live there now), `test_user_defaults_eligibility`; second target, `gui_unit_test` (see below): `test_defaults_diff`, `test_state_reconcile`, `test_preview_renderer`, `test_export_params`, `test_crystal_renderer`, `test_render_handedness_guard` (render `right=+az` cross-implementation handedness guard — absolute screen-side, pairs with the `test_projection` golden absolute-column pins; it is the one case that needs `lumice_gui_obj` and `lumice_obj` linked together, which is why `gui_unit_test` is its only possible home), `test_axis_absent_alignment`, `test_user_defaults`, `test_render_bg_logic`, `test_sampling_density_stats`, `test_server_poller`, `test_face_number_overlay`, `test_overlay_labels`, `test_composite_preview`, `test_color_window_logic`, plus `gui_unit_test_env` (installs this target's personal-defaults isolation baseline before any case runs) | — | — | `test/unit-correctness/scripts/test_check_new_refs.py` is a pytest member of this layer, run by the `policy` CI job and deliberately outside `testpaths` (§1.1). It is a regression net over a diff parser whose failures are silent — a broken parse reports success — so **do not delete a case for being redundant** without re-running it against the defect it pins. This layer's `gui` subsystem directory is shared by **two** CMake targets split on a link boundary (§2): `unit_correctness_test` (`lumice_obj` only) and `gui_unit_test` (also `lumice_gui_obj`, windowless). Both carry LABEL `unit-correctness`, so no `-L` selector changes when a case moves between them — but a file **does** have to move between the two `add_executable` source lists, and `gui_unit_test` only exists under `if(BUILD_GUI)`. |
| **composition-correctness** | `test/composition-correctness/<subsystem>/` | `gui` subsystem, single target `composition_correctness_test` (§2's name-overlap caution): `test_document_roundtrip_chain`, `test_document_defaults_chain`, `test_document_switch_chain`, `test_legacy_document_chain`, `test_scene_commit_chain`, `test_filter_reconstruct_chain`, `test_raypath_color_document_chain`, `test_run_lifecycle_chain`, `test_run_warning_chain`, `test_user_defaults_chain`, `test_field_editor_chain`, `test_edit_modal_chain`, `test_preview_projection_chain` | — | — | Newest layer (§1.2); today populated only by the `gui` subsystem. A file name is the chain's topic, never a single `src/` unit's name — see §1.2's naming rule. |
| **golden-analytic** | `test/golden-analytic/<subsystem>/` | `test_projection`†, analytic segments inside `test_optics`†, `MultiMsContinuationNormalIncidence` (in `test_metal_trace_parity.cpp`, 2-MS analytic anchor) | — | — | †split out only after per-file confirmation of the analytic-truth boundary vs unit-correctness |
| **parity-cross-backend** | `test/parity-cross-backend/<subsystem>/` | `test_metal_trace_parity`, `test_metal_root_gen`, `test_metal_trace_backend`, `test_metal_filter_match_parity`(.mm), `test_cpu_trace_backend` | `test_metal_exit_seam_parity`, `test_metal_batch_invariance`, `test_device_gen_default_path`, `test_cpu_backend_route`, **projection subsystem** (315.5): `test_metal_projection_parity`, `test_cuda_projection_parity` (shared `_projection_battery.py`) | — | `_parity_metrics.py` is the single source of parity metrics — **DO_NOT_MIGRATE_INDEPENDENTLY** (move with its dependents). Energy-conservation + cross-seed double gate is a 267.3 reinforcement — **DO NOT DELETE**. The `test_metal_batch_invariance` exit-conservation `xfail` is **legitimate** (worst-case drain not yet landed) — do not "fix" it by deleting. `_projection_battery.py` is the shared per-projection battery (oracle = legacy CPU) — move with `test_{metal,cuda}_projection_parity`. |
| **e2e-correctness** | `test/e2e-correctness/` (flat) | — | `test_smoke`, `test_cli`, `test_raypath_equivalence` | — | — |
| **performance** | `test/performance/` (flat) | (no standalone C++ perf target; CI `Benchmark` step runs `--benchmark`) | `test_metal_throughput` | — | — |
| **gui** | `test/gui/<tag>/` (functional/visual/responsiveness) | — | `test_metal_gui_acceptance` (G4; gui layer, runs via pytest harness) | `functional/`: `test_background_overlay`, `test_color_window`, `test_defaults_panel`, `test_edit_modal`, `test_entry_management`, `test_export`, `test_file_ops`, `test_filter_editor`, `test_gui_face_number_overlay`, `test_gui_overlay_labels`, `test_gui_preview_animation`, `test_gui_sim_smoke`, `test_log_panel`, `test_overlay_controls`, `test_preview_texture`, `test_preview_viewport`, `test_run_lifecycle`, `test_scene_controls`, `test_shell_chrome`, `test_status_bar`, `test_view_display_controls`; `visual/`: `test_gui_capture_smoke`, `test_gui_defaults_panel`, `test_gui_lens_projection`, `test_gui_modal_layout`, `test_preview_pixels`; `responsiveness/`: **`test_gui_perf`**; harness (flat under `test/gui/`): `test_gui_main`, `test_screenshot`, `test_gui_shared` | `test_gui_perf` oracle = absolute frame budget (§4.4), not throughput-vs-legacy. `functional/` no longer includes an `interaction`-named catch-all — its former contents are now split by driven window/panel across the files above, or moved out to `unit-correctness`/`composition-correctness` when the case needed no live frame (§1.7). |
| **regression-sentinel** | `test/regression-sentinel/` (flat) | — | `test_capi_sentinel_overflow`, `test_ms_filter_leak`, `test_errors` | — | `test_capi_sentinel_overflow` / `test_ms_filter_leak` guard real bugs via issue repro — **DO NOT alter the scenario**. `test_ms_filter_leak` is also parity-related; its **primary** purpose is sentinel (multi-purpose → classify by primary purpose). |

**Multi-purpose tie-break rule**: when a test serves more than one purpose, classify it by its
**primary** purpose (the bug/property whose regression it most directly guards), and note the
secondary purpose in a comment. Example: `test_ms_filter_leak` → `regression-sentinel`
(primary), parity-related (secondary).

**Health items — do not over-clean (consolidated "do-not-touch" list)**:
`_parity_metrics.py` (single source), the energy-conservation + cross-seed self-consistency
double gate (267.3 corr-blind reinforcement), `test_capi_sentinel_overflow.py` and
`test_ms_filter_leak.py` (issue-repro sentinels), and the legitimate `xfail` in
`test_metal_batch_invariance.py`.

> **Legacy CPU red line**: legacy CPU is the parity ground truth (§1.4, §4.2) and the perf
> denominator (§1.6, §4.1). It and its tests are **never** a cleanup target in any layer.

---

## §7 Test-scope contract: what each scope costs, and what it additionally catches

§1–§6 say where a test belongs. This section says what running them **costs**, what each scope
**buys over the one below it**, and who is required to do that arithmetic before spending it. It
exists because the scope table in `AGENTS.md` answers "which command do I run" without answering
"what does it cost and what does the cheaper one structurally miss" — and a recommendation to run
the cheap scope first is only followed by someone who knows what the cheap scope cannot see.

### §7.0 What this section covers, and what it does not

This is **not** an audit of the suite, and must not be cited as one. It looks at the head of the
cost distribution and states the tail it did not look at:

- **Covered in depth**: the three local scopes end to end, and the 8 CI jobs of 391s or longer —
  4427s of the 5089s of machine time in §7.1's table, i.e. **87%**.
- **Listed but not analyzed**: the 8 remaining CI jobs, together 662s, i.e. **13%**. Four are
  second-scale gates (`policy` 24s, `format-check` 13s, `new-refs` 8s, `benchmark-summary` 7s) and
  three are compile-only jobs (`windows-cuda-compile` 225s, `cuda-compile` 141s, `bench-compile`
  63s) whose duration says nothing about which tests should run, plus `Ubuntu ARM64` (181s), the
  cheapest leg of the build matrix. Both proportions are recomputable from that table.
- **Not enumerated at all**: the individual cases inside a scope. `gui_test` runs 344 cases across
  38 categories (338 in its correctness pool, 6 in the real-timing pool, per the binary's own run
  summary); the fast e2e set collects 105; neither was gone through case by case. One
  measured statement about that distribution is worth carrying, because it bounds what scheduling
  can achieve — and the shape of that bound is itself worth knowing. It used to be a single case:
  `test_smoke.py` ran every showcase config inside one `test_all_configs_run_successfully` item,
  which at 87.3s accounted for 92% of the whole set's 94.8s parallel wall clock. `subTest` made
  that look like 14 cases in the report, but xdist distributes by *collected item*, so all 14
  configs were pinned to one worker no matter how many were free. Splitting it into one item per
  config (`test_config_<stem>`) removed the pin without touching a single assertion — measured
  wall clock fell while total `user` CPU stayed flat, which is the evidence that nothing was
  dropped, only repacked. The general form is worth carrying past this one case: **a loop inside
  a test item is invisible to the scheduler, and a report that shows N sub-cases does not mean
  the runner sees N items.**

### §7.1 What each scope costs

**Local.** Machine: the development Mac that `doc/machines.md` binds to the **Metal reference
machine** role — Apple silicon, macOS ARM64, 12 logical cores (8 performance). Read every local
figure below as a property of that machine and not of the suite; `doc/machines.md` is the single
source for which host holds which role, and a different host moves all of them. Cache state: **warm** static build
tree at the measured commit — `scripts/test.sh` does not build the static flavor, so no compile
time is inside these numbers. Concurrency as noted per layer. Machine otherwise idle (load average
2.2 at the start of the run).

| Scope | Layer | Wall clock | Concurrency |
|---|---|---|---|
| `quick` | `ctest -L "unit-correctness\|composition-correctness\|parity\|golden-analytic"` | 36s | ctest default |
| `quick` | fast e2e (`pytest`, `-m "not slow"` via `pyproject.toml` `addopts`) | 96s | `-n auto` → 12 workers |
| `quick` | `gui_test` correctness pool (`--fixed-dt`, negative filter) | 47s | single process |
| | **`quick` total** | **179s** | |
| `full` | `quick` + `gui_test` real-timing pool (no `--fixed-dt`, positive filter) | +11s | single process |
| | **`full` total** | **190s** | |
| `pr` | `full` + shared-lib build (the flavor was absent: cold configure + 43 TUs + install) | +14s | 12-way build |
| `pr` | slow e2e phase 1 (`pytest --ignore=test/performance -n 3 -m slow`) | 361s | `-n 3` |
| `pr` | slow e2e phase 2 (throughput gates, alone so they do not measure under load) | 14s | serial, single process |
| | **`pr` total** | **627s** | |

Three things the `pr` row does not show on its face. **The shared-library layer is not a constant**:
it cost 14s here because the shared flavor did not exist and had to be built (43 translation units,
no GUI, no tests, CPM dependencies already cached), and on any later run in the same tree it is a
freshness check costing effectively nothing — so the first `pr` of a session and the ones after it
are different prices. **Phase 1 skipped 29 of its 109 collected items** (80 passed): the backend
guards deselect what the machine cannot run, so this figure is what a Metal-capable Mac pays, not
what the set costs everywhere. **The layers sum to 579s against a 627s wall clock**; the ~48s
difference is the script's own preconditions — the static build-tree and `addopts` checks, the
extraction of the two `gui_test` filters from `scripts/build.sh`, and the probe that asks
`test/e2e/capi_runner.py` which library the slow layer would load — none of which is a test.

A standalone `quick` measured 172s on the previous day on the same machine, against 179s inside
this run: treat single-run figures here as accurate to roughly ±5%, not better.

**Compile is not in the table above, and is not negligible.** Measured on the same machine:
a cold configure-plus-build of the static flavor with GUI and tests is **167s**; an incremental
rebuild after touching one leaf `.cpp` is **33s**; after touching `src/core/math.hpp`, included by
63 translation units, **49s**. So the honest first-run cost of `quick` on a cold tree is about
350s, and its steady-state cost during an edit-test loop is about 210s.

**CI.** Runners: GitHub-hosted, per job (`ubuntu-24.04`, `ubuntu-24.04-arm`, `macos-15`,
`windows-2025`; the CUDA compile leg uses `windows-2022`). Cache state:
CPM dependencies come from `actions/cache`, so warmth **depends on that run's cache hit** and is
not guaranteed; nothing else is cached between runs. Concurrency: the jobs below run in parallel as
a matrix, so the run's wall clock is its **longest job**, not the sum; within a job, the fast e2e
leg runs `pytest` serially and the slow e2e legs run `-n 3` (with the throughput gates re-run
serially afterwards, so they do not measure under load).

The figures are the union of a push run and a pull-request run of the same commit (`495c6f6b`),
because neither event alone runs every job: the build matrix, `policy` and `format-check` are
push-only, while `e2e-test`, the `E2E Slow` legs and `new-refs` run on pull requests and `main`.

| CI job | Seconds | In §7.0's covered head? |
|---|---|---|
| Windows MSVC x86_64 | **740** | yes — longest job, sets the run's wall clock |
| shared-gui-test-build | 666 | yes |
| Ubuntu x86_64 | 635 | yes |
| E2E Slow (macOS ARM64 parity) | 597 | yes |
| e2e-test | 512 | yes |
| macOS ARM64 | 464 | yes |
| E2E Slow (Ubuntu x86_64) | 422 | yes |
| E2E Slow (macOS ARM64 rest) | 391 | yes |
| windows-cuda-compile | 225 | no — compile-only |
| Ubuntu ARM64 | 181 | no |
| cuda-compile | 141 | no — compile-only |
| bench-compile | 63 | no — compile-only |
| policy | 24 | no — second-scale gate |
| format-check | 13 | no — second-scale gate |
| new-refs | 8 | no — second-scale gate |
| benchmark-summary | 7 | no — second-scale gate |
| | **5089s total machine time** | head = 4427s (87%) |

Where a job's time goes differs by job, and the split cannot be assumed. Step-level timestamps
from an earlier run of the same shape: `Windows MSVC x86_64` was 78% compile (568s of 726s);
`shared-gui-test-build`, which runs nothing, 92% compile (589s of 641s); `E2E Slow (macOS rest)`
was 8% compile and **89% test execution** (612s of 686s); `e2e-test` 84% test execution (377s of
450s). Those percentages come from a different run than the totals above and are shape, not
precision.

**Two facts about this table that any CI-time proposal has to answer to.**

1. **The critical path is `Windows MSVC x86_64` (740s) and, behind it, `shared-gui-test-build`
   (666s).** A run's wall clock is its longest job and nothing else. Therefore: *any proposal to
   "shorten CI" that does not touch those two jobs buys zero wall clock*, however much machine time
   it saves. Anywhere a claim of the form "this saves N seconds of CI" is made — in a plan, a PR
   description, or a review comment — it must first answer **"does it shorten the longest job?"**.
   The corollary that keeps catching people: rebalancing two shards against each other is worth CI
   time only while one of them **is** the longest job. The two `E2E Slow` macOS legs sit 143s and
   349s under the ceiling, so packing them closer is currently worth nothing (`ci.yml`'s `e2e-slow`
   matrix comment carries the same statement next to the code it constrains).
2. **A floor sits under the `E2E Slow (macOS ARM64 parity)` leg that no repacking removes.**
   `test_capi_sentinel_overflow` is **one indivisible pytest case** (3 configs × 12 rounds inside a
   single function) that pins one xdist worker for the leg's whole duration — ~204s in the grouping
   that shipped — which is what puts that leg near 600s. Splitting files across legs cannot go below
   it; only splitting that case could. Quote this number **with its grouping**: the case's cost is a
   property of the file *and its co-tenants*, and the same case measured 376s when it was starved
   beside the exit-seam parity file. Costs never survive a regrouping; re-derive them after one.

### §7.2 What each scope catches that the one below it cannot

The three local scopes are strictly nested (`quick` ⊂ `full` ⊂ `pr`), so each answer below is about
the increment. The answers are mechanisms, not directory lists — the point of each scope is a class
of defect the cheaper scope is *structurally* unable to report, not merely a set of files it skips.

**`quick` → `full`: wall-clock-dependent behaviour.** `quick` runs `gui_test` with `--fixed-dt`,
which injects a deterministic 1/60s frame delta. Six cases depend on real elapsed time: the two
in the `perf_test` category (main-loop FPS and rays-per-second), an accumulation gated on a
wall-clock deadline rather than a frame count, a background thread that only advances between real
`ctx->Yield()` calls, and two more of the same shape. `scripts/build.sh` names them twice — once as
a negative filter (one category, four case names) for the correctness pool, once as the positive
filter for the real-timing pool — which is the mechanical statement of what `quick` withholds.
Under `--fixed-dt` those cases are not skipped; they are **starved**, and a starved assertion
produces a *wrong answer*, not a missing one. That is why they are a separate pool rather than a
slower part of the same one, and it is the reason a green `quick` is **not evidence** about this
class: the scope did not fail to reach the question, it answered it from a state that cannot be
right. The increment costs 11s — about 6% on top of
`quick`, the cheapest step in this whole table.

**`full` → `pr`: the shared-library route, and the `slow` set.** Two distinct additions.
*(a)* Everything marked `@pytest.mark.slow` is excluded from `quick` and `full` structurally, by
`pyproject.toml`'s `addopts = ["-m", "not slow"]` — the cross-backend parity batteries, the
throughput gates, and the issue-repro sentinels (§6's "do not touch" list) are all in there. *(b)*
More importantly, `pr` is the **only** local scope in which the library is loaded as a library:
those tests drive `liblumice` through `ctypes` (`test/e2e/capi_runner.py`), so the export surface,
the dynamic-symbol resolution, the rpath and the C API's cross-boundary object lifetimes are
exercised here and nowhere else. `quick` and `full` link the same sources into a test binary, which
cannot see a defect in how the code is *packaged*. The scope carries a freshness layer for exactly
this reason: a stale `.dylib` silently tests old code and reports green, so `pr` refuses to trust a
library it cannot prove is newer than its sources.

**`pr` → CI: the platform matrix, and only that.** CI adds MSVC, x86_64 and ARM64 Linux, and a
software rasterizer (Xvfb + Mesa llvmpipe) that renders GUI reference scenes without a GPU. None of
these is reproducible from one developer machine, and the compiler dimension is where they earn
their keep: `AGENTS.md` records three MSVC incompatibilities that a Clang-only local run could not
have seen, and that sat undetected for weeks behind platform guards.

**But the containment does not run the other way, and this is the trap.** CI is *not* a superset of
`pr`. The one CI leg that runs `gui_test` passes a hand-picked **positive** filter naming two
reference groups, so **2 of that binary's 38 categories run in CI**; the local scopes select by a
**negative** filter, so every category runs locally — `quick` withholds the one category and four
cases named above, `full` withholds nothing. For the other 36 categories, including every
functional case and the whole `parity` tag (§7.5), a developer machine is the only place the
assertion is ever evaluated. A green CI is not evidence about them.

### §7.3 The budget rule: who answers, and when

The rule below exists because the alternative is the state this section was written out of: every
individual decision to add a test is locally justified, no one is ever asked what the total costs,
and the total only grows. The burden of proof therefore sits on the side that spends.

1. **Whoever proposes a change states its expected test spend in the written plan, before
   implementation starts**: which scopes it will run, how many times, and the estimated wall clock.
   "I cannot estimate this" is an acceptable answer and should be written down as such. Writing
   nothing is not acceptable.
2. **The same person reconciles it at close-out**, in the description of the PR that lands the
   change: actual against estimate. If the actual exceeds the estimate by more than about 1.5×,
   record the deviation and its cause — **do not retro-fit the estimate to what happened**, which
   destroys the only signal this rule produces.
3. **Whoever reviews the plan checks that declaration 1 is present**; whoever reviews the code
   checks that reconciliation 2 is present. Neither is a judgement about whether the number is big;
   both are a check that the question was asked.

The rule is deliberately about *stating and reconciling*, not about staying under a limit. There is
no budget ceiling to enforce, and inventing one would only produce estimates shaped to fit it.

This section is the first change written under the rule, and it says so rather than presenting the
rule as established practice: earlier changes in this repository carry no such declaration, and none
was back-filled for them.

### §7.4 Independent re-verification is a fixed-cost multiplier

**The rule.** A change's own report about itself does not settle whether the work is done. An
independent pass — someone, or some agent, that did not do the work re-asking whether it meets what
was asked — is a required step, not an optimization to be dropped when the evidence looks tidy.

**It cannot be replaced by an evidence chain.** This is the substitution that keeps getting
proposed: attach the logs, the run ids, the commit hashes, and let provenance stand in for the
second pass. It does not work, because the two answer different questions. A provenance chain
proves *this output came from that run*. Re-verification asks *whether that run asked the right
question*. Classes of defect that have actually been caught this way in this repository, none of
which any amount of provenance could have caught: a self-reported severity that did not match how
reachable the defect was for a user; a self-reported execution path that was the **opposite** of
what the configuration file's filter actually did; and a self-reported direction of net change that
measurement contradicted. In each, the logs were genuine and the run was real. The report about
them was wrong.

**The consequence, and the reason this sits next to the cost tables.** Because re-verification is
required and re-reads the same ground, it behaves as a **fixed-cost multiplier** on the suite:
whatever the base costs, the delivered cost of a change is that base taken more than once. Two
things follow, and they are the practical output of this whole section:

- Every second removed from the base is removed again under the multiplier. **Cutting base suite
  cost is worth more than its face value** — this is the argument for §7.0's observation that one
  87.3s case sets the floor of a 94.8s set.
- Optimizing scheduling, sharding or parallelism **without** cutting the base leaves the multiplied
  portion untouched. That is why §7.1's fact 1 is stated as a gate: rebalancing moves work between
  machines; it does not remove work, so the multiplier keeps charging for it.

### §7.5 Where the `parity` tag runs, and why not in CI

§4.10 introduces `test/gui/parity/`; this is the platform-reach half of it, stated once here and
pointed at from the other two places that describe it (`AGENTS.md`, and the fixture's own header
comment in `test/gui/parity/test_gui_cli_export_parity.cpp`).

**Where it runs today.** Nowhere in CI, and this is a mechanical fact of `.github/workflows/ci.yml`,
not an inference: the single step that executes `gui_test` runs
`xvfb-run -a … gui_test --fixed-dt --filter "modal_layout,defaults_panel_layout" --no-user-config`
inside the `Ubuntu x86_64` leg — a positive filter naming two categories, of which `parity` is not
one — and the three `E2E Slow` legs configure with `-DBUILD_TEST=OFF -DBUILD_GUI=OFF`, so they do
not have the binary at all. Where it does run is a developer machine with a working GL context:
`scripts/build.sh`'s correctness pool selects by a negative filter, so the category is included by
default and `./scripts/test.sh {quick,full,pr}` executes it. Read that as §7.2's last paragraph
applied to one tag: **a green CI is not evidence about this fixture.**

**Why it is not enabled under llvmpipe today.** Not because of cost. The fixture asserts an *exact*
equality on the simulated ray count, through the shared `ExpectedSimRayNum()` helper in
`test/gui/test_gui_shared.hpp` — structurally the same assertion shape as `lens_proj`, which §4.6
excludes from the CI filter because that count comes up short under the software rasterizer for an
upstream reason that is still unfixed. Enabling `parity` there would re-import that known defect as
a red in a required check. For scale, the cost if it were enabled is not the blocker either: the
`parity` filter takes 24.6s on the 12-core machine of §7.1, which extrapolates to roughly 70–90s on
a 4-vCPU runner — **an extrapolation, not a measurement** — and it would land inside the
`Ubuntu x86_64` leg, which sits at 635s — 105s below the `Windows MSVC` ceiling — so by §7.1's
fact 1 it would still cost close to zero wall clock. The blocker is the known red, so the sequence
is: close the upstream ray-count shortfall first, re-measure on a real runner, then widen the
filter. Widening it is a measurement, not an edit.
