[中文版](filter-architecture_zh.md)

# Filter Architecture

This document captures the core design constraints for Lumice's filter subsystem.
It is a **specification**: all implementation must align with the invariants stated here.
If an implementation detail conflicts with a statement in this document, fix the doc first
(when the doc is wrong) or fix the code (when the code drifts), but never silently diverge.

**Target audience**: contributors implementing features that touch filter routing, crystal
configuration, multi-scatter propagation, or the C API, as well as reviewers performing
code-review against Design A.

---

## §1 Binding Model: Filter ↔ Crystal (Single-Key)

A filter is bound to a **crystal**, not to a layer, stage, or scattering entry.
The mapping is one-to-one:

```
crystal_id  ──────────────  filter_id   (single-key pair)
```

### JSON / Configuration

In the JSON configuration, `scattering[].entries[].filter` is a *serialization detail*.
Semantically it is equivalent to "the filter of the crystal referenced by this entry."
A given `crystal_id` is expected to have the same filter across all scattering entries
that reference it — the GUI enforces this invariant (see §5); a JSON loader may assert it.

### GUI Linked Entries

Two entries are **linked** if and only if they share both `crystal_id` and `filter_id`.
The GUI renders a link badge when ≥ 2 entries form such a pair.
A linked group is the **atomic share unit**: any edit to one member must be observable
on all members.

Edit propagation rules — automatic vs needs explicit propagation:

1. **Crystal content edit** (in-place overwrite at the shared pool slot): automatic.
   All entries sharing the `crystal_id` see the update via pool indirection.
2. **Filter content edit** (in-place overwrite at the shared pool slot): automatic.
   Same mechanism via shared `filter_id`.
3. **Filter add** (`entry.filter_id: None → Some N`): needs propagation.
   The new pool slot is bound only to the editing entry by default; linked siblings
   (previously at `(cid, None)` with this entry) must also have their `filter_id`
   flipped to N, otherwise the group decoheres.
4. **Filter remove** (`entry.filter_id: Some → None`): needs propagation.
   Linked siblings must also have `filter_id` cleared.

See `src/gui/gui_state.hpp:294-328` for the authoritative comment block and propagation
owner (`ApplyBuffersToEntry` via the `propagate_filter_id_to_linked` lambda).

### "Same Parameters, Different Filter"

If you want two crystals with identical geometry but different filters, create **two
independent `crystal_id` instances**. The GUI provides link/unlink operations for this:
"Link to…" adopts the target's `(crystal_id, filter_id)` atomically; "Unlink" forks a
private clone.

---

## §2 Filter Role in Simulation (Design A)

Filters **participate in simulation propagation**. They act as gates at every
ray-exit-crystal decision point inside the simulator.

### Decision Rule (per ray, per crystal exit)

```
ray exits crystal
    │
    ├─ filter check passes  ──►  ray is eligible for outgoing / MS propagation
    │
    └─ filter check fails   ──►  ray is DROPPED
                                 (neither emitted as outgoing
                                  nor forwarded to the next MS layer)
```

`FilterSpec::Check(ray)` is called once per outgoing-ray candidate, bound to the filter
of the crystal that just emitted it.  A failing ray does not appear in any downstream
buffer.

### Design Rationale

Placing the gate at the simulator level (rather than at the consumer / render level)
gives two benefits:

1. **Correctness**: filter selectivity applies at every layer in the multi-scatter chain,
   so the physically correct subset of rays propagates through the full crystal sequence.
2. **Performance**: high-selectivity filters prune the ray population early; the pruning
   effect multiplies across layers (see §3).

---

## §3 Multi-Scatter Semantics

In a multi-scatter (MS) configuration, each layer is a `(prob, crystal, filter)` triple.

```
Layer 0:  prob₀ × crystal₀ × filter₀
               │
         filter₀ pass only
               │
Layer 1:  prob₁ × crystal₁ × filter₁
               │
         filter₁ pass only
               │
         ...
```

Filter is evaluated at **each crystal exit** within the simulator.  Layer N's filter
controls which rays are eligible to enter layer N+1.  Rays that fail the filter at any
layer are dropped; they do not propagate further and do not appear in the output buffers.

Higher filter selectivity → faster simulation: the surviving ray fraction compounds
multiplicatively as `∏ᵢ (pass_rateᵢ × probᵢ)` across layers.

---

## §4 Render Phase: No Filter Logic

The render / consumer phase performs **no filter operations**.

All rays received by `RenderConsumer` have already been filtered inside the simulator.
The render pass is pure projection and XYZ accumulation:

```
simulator output  →  RenderConsumer  →  projection  →  XYZ accumulate
                                         (no filter)
```

`RenderConfig` does not hold a filter field.  The previously present `ms_filter_` field
is removed in task-revert (Design A target, removed in task-revert 219.4).  Any attempt
to apply filters at the consumer level is a design violation.

---

## §5 Data Flow Invariants

### Config representation

Filter data is stored in `SceneConfig.ms_.setting_[].filter_` (`ScatteringSetting.filter_`,
per-entry, in `src/config/proj_config.hpp`).

**Underlying assumption**: each `crystal_id` carries the same filter across all scattering
entries that reference it.  The GUI `linked-crystal` invariant enforces this at the GUI
layer.  A JSON loader assertion can enforce it at parse time.

### Filter check entry point

Every outgoing-ray candidate in the simulator calls `FilterSpec::Check(ray)` once.
`FilterSpec::Create(config, crystal, axis_dist)` builds the spec from the filter config
bound to that crystal.

The call site is in `CollectData()` inside `src/core/simulator.cpp` — see §2 for the
decision rule.

### RenderConfig invariant

`RenderConfig` must not hold filter fields.  Filters are a simulation-side concern only.

---

## §6 Historical Context

### task-200 (query-filter-uplift-v2, 2026-05-18) — Routing Decision: Reverted

**Goal**: move filter from simulator-side to consumer-side so that live-editing a filter
would not re-trigger a full simulation.

**Implementation**: `config_manager.cpp:194-225` (removed in task-revert-filter-to-simulator-side / 219.4)
added a post-parse auto-binding that copies `scattering.entries[].filter` into `renderer.ms_filter_`.  This activated the
previously-dormant `FilterRay` path in `render.cpp`.  In the simulator, the filter was
demoted to a pure MS-branch gate (filter-fail rays were still emitted as outgoing).

**Discovered problems** (scrum-prob-zero-leak / #219, 2026-05-22):

1. The intended benefit — "filter edit skips simulation" — was never realized in the GUI.
   Changing a filter still triggered `MarkFilterDirty → full sim re-run`.
2. Three implementation bugs were found in the task-200 code path:
   - Post-parse flattening lost per-entry filter binding for multi-level MS configs.
   - Crystal context was frozen at config-parse time, preventing live crystal updates
     from reaching the consumer-side filter.
   - Chain-walk boundary condition in the config binder caused index overflow for
     certain MS configurations.
3. Consumer-side filtering is less efficient: filter selectivity no longer prunes the ray
   population during simulation, so the multi-layer pruning benefit (§3) is lost.

**Decision**: revert to Design A (task-revert-filter-to-simulator-side, 219.4).

### scrum-210 (filter-architecture-refactor, 2026-05-19) — Retained

scrum-210 introduced `FilterSpec` polymorphism and the canonical-form / orbit-based
approach to raypath matching.  This is a **pure algorithm-layer improvement**, orthogonal
to the routing decision.  task-revert (219.4) migrates the filter *call sites* (consumer
→ simulator) but does not touch the `FilterSpec` algorithm interface.

Retained components: `src/core/filter_spec.{hpp,cpp}`, the per-batch `specs_table`
call in the simulator, and the scrum-210 test suite.

### scrum-prob-zero-leak / #219 (2026-05-22) — Design A baseline

This scrum audited the task-200 bugs, evaluated revert options, and adopted Design A
as the canonical routing model.  Off mode redesign was tracked as a backlog item.

### scrum-221 (adaptive-additivity-redesign, 2026-05-24) — Off mode redesign shipped

Implemented Off mode via F1 anchor lane (see §7).  ABI swap removed `unfiltered_*`
fields and introduced `anchor_p99_y` / `anchor_snapshot_intensity`.  Additivity testing
(partition invariant) added to the E2E test suite.

---

## §7 F1 Anchor Lane (Single-Mode)

> **Historical note**: scrum-221 introduced a GUI Adaptive Brightness ON/OFF toggle that
> selected between Design A (per-frame self-anchor) and the F1 anchor lane.
> `task-remove-adaptive-brightness-on-mode` (this revision) removed the toggle and the
> Design A path after internal testing confirmed F1 produces strictly better UX with
> acceptable performance cost. Only the mode dispatch is gone — the anchor-lane
> implementation is unchanged.

### `LUMICE_RawXyzResult` Anchor Fields

`LUMICE_RawXyzResult` exposes the anchor lane via two fields (see `src/include/lumice.h`):

| Field | Semantics |
|-------|-----------|
| `anchor_p99_y` | P99 of Y over filter-pass + filter-fail emission combined (filter-independent EV anchor). Zero when no filter is configured (anchor lane is empty). |
| `anchor_snapshot_intensity` | Per-pixel landed intensity for the same combined set. Zero when no filter is configured. |

The pre-scrum-221 `unfiltered_xyz_buffer` / `unfiltered_snapshot_intensity` fields
introduced by task-200 stay removed (they degenerated to equality with the post-filter
buffer after task-revert).

### EV Offset Source

The EV offset (loosely ε; formally `ev = log2(target_linear / p99_norm)`) is sourced
from the F1 anchor lane, with a graceful degenerate path when no filter is active:

```
filter present:  ev = log2(target_linear / (anchor_p99_y / anchor_snapshot_intensity))
                 (F1 combined anchor; filter-independent)

no filter:       ev = log2(target_linear / (ComputeP99Y(xyz_buffer) / snapshot_intensity))
                 (anchor_p99_y == 0 → fallback; equals filtered self-anchor since
                  xyz_buffer == full emission when no filter is configured)
```

The GUI (`SyncFromPoller()` in `src/gui/app.cpp`) performs this selection on each frame.

### Anchor Lane

When a filter spec is active, the simulator runs a secondary collection pass alongside
the normal filter-pass path:

```
crystal exit (outgoing candidate)
    │
    ├─ filter passes  →  xyz_buffer  +  anchor lane
    │
    └─ filter fails   →  anchor lane only
                         (not rendered; contributes to EV anchor only)
```

`CollectData()` in `src/core/simulator.cpp` implements the branch table:

| filter | prob | outcome |
|--------|------|---------|
| pass | pass | `IsContinue()` — next MS scatter |
| pass | fail | `IsOutgoing()` — emit (contributes to `xyz_buffer` and anchor) |
| fail | pass | `IsContinue()` — anchor-lane bypass (propagates to next MS) |
| fail | fail | `IsFilterDropped()` — anchor lane only |

`IsContinue()` and `IsFilterDropped()` are mutually exclusive.

`RenderConsumer::Consume()` in `src/server/render.cpp` accumulates anchor emission into
the anchor buffer, gated by `data.anchor_d_.empty()` so the path is naturally inert when
no filter spec is configured (no `IsFilterDropped` writes → empty anchor → no allocation).
`PrepareSnapshot()` exposes `anchor_p99_y` and `anchor_snapshot_intensity` to the C API.

### Performance

Anchor overhead scales with the filter-fail ray volume:

- `ms_prob ≈ 0`: filter-fail rays are rare; overhead is negligible (~0%).
- `ms_prob ≈ 0.5`: roughly half of rays fail the filter; overhead is approximately
  +97% (~2× simulation time).

With no filter the anchor lane stays empty and the cost is zero.

See `doc/adaptive-brightness.md` for the additivity invariant and full EV semantics.

---

## §8 Invariant Automation (Future Work — Placeholder)

The following automated checks are not yet implemented.  They are listed here as
anchor points for future contributors:

- **Simulator output assertion**: a unit test that verifies no ray in the simulator's
  outgoing buffer fails the configured filter.  Guards against reintroduction of the
  task-200 consumer-side filter routing.
- **Linked-crystal invariant assertion**: a JSON-loader assertion that every `crystal_id`
  appearing in multiple `scattering.entries` references the same `filter` config.

---

## Cross-References

| Concern | Location |
|---------|----------|
| Binding model, GUI linked-group invariants | `src/gui/gui_state.hpp:294-328` |
| Filter ↔ crystal per-entry config | `src/config/proj_config.hpp` — `ScatteringSetting` |
| Simulator-side filter check (Design A gate) | `src/core/simulator.cpp` — `CollectData()` |
| FilterSpec algorithm interface | `src/core/filter_spec.hpp` |
| C API anchor fields (Off mode) | `src/include/lumice.h` — `LUMICE_RawXyzResult.anchor_p99_y` / `anchor_snapshot_intensity` |
| Filter JSON schema | `doc/configuration.md` |
| Raypath semantics, P/B/D filter toggles | `doc/raypath-symmetry.md` |
| Adaptive Brightness Off mode, additivity | `doc/adaptive-brightness.md` |
