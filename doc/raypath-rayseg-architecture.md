# RaySeg Architecture: Field Invariants, Chain Rules, and State Machine

This document specifies the run-time constraints of the `RaySeg` struct — field semantics,
initialization rules, state-machine transitions, and filter interface contracts. It
complements `doc/raypath-symmetry.md` (PBD symbolic semantics) and
`doc/filter-architecture.md` (filter routing design); see §5 for exact boundary.

**Target audience**: contributors working on the simulator core, filter subsystem,
multi-scatter propagation, or consumer pipeline.

---

## §1 RaySeg Field Semantics & Invariants

`RaySeg` is defined in `src/core/raypath.hpp:50-140`. Each field is listed below with its
semantic role, valid range, and sentinel values.

### Geometric Fields

| Field | Type | Semantics | Coordinate Space | Source |
|-------|------|-----------|-----------------|--------|
| `d_[3]` | `float[3]` | Ray direction (unit vector). | Crystal-local when `IsNormal()`; world-space after `CollectData` rotation for outgoing/TIR. | `src/core/raypath.hpp:53` |
| `p_[3]` | `float[3]` | Ray **end** point (not start point). | Same coordinate-space rule as `d_`. | `src/core/raypath.hpp:54` |
| `w_` | `float` | Ray weight (intensity fraction). Non-negative for live rays; `-1.0f` is the TIR sentinel. | — | `src/core/raypath.hpp:55` |
| `from_face_` | `IdType` | Polygon-face index the segment originated from (parent's `to_face_`). `kInvalidId` for the first segment of a chain. | — | `src/core/raypath.hpp:57-61` |
| `to_face_` | `IdType` | Polygon-face index this segment hit. `kInvalidId` means "no hit" — either outgoing candidate or TIR-stopped. | — | `src/core/raypath.hpp:62-66` |

### Chain & Identity Fields

| Field | Type | Semantics | Source |
|-------|------|-----------|--------|
| `prev_ray_idx_` | `size_t` | Index of parent segment in `all_data`. `kInfSize` for root rays (first MS layer). | `src/core/raypath.hpp:68` |
| `root_ray_idx_` | `size_t` | Index of the chain root in `all_data`. All segments in one ray tree share this value. | `src/core/raypath.hpp:69` |
| `crystal_idx_` | `IdType` | Index into `SimData::crystals_[]`. Range: `[0, kMaxCrystalNum)` or `kInvalidId` (init sentinel). | `src/core/raypath.hpp:71` |
| `crystal_config_id_` | `IdType` | `Crystal::config_id_` — ties the segment to its configuration-level crystal identity. | `src/core/raypath.hpp:72` |
| `crystal_rot_` | `Rotation` | Crystal-to-world rotation matrix. Sampled per root ray; propagated to all children. | `src/core/raypath.hpp:73` |
| `rp_` | `RaypathRecorder` | Raypath in the **current** crystal, as a face-number sequence. | `src/core/raypath.hpp:91` |

### State Flags

| Field | Type | Semantics | Source |
|-------|------|-----------|--------|
| `is_continue_` | `bool` | Set when this outgoing candidate continues to the next MS layer. Only meaningful when `to_face_ == kInvalidId && w_ >= 0`. Default `false`. | `src/core/raypath.hpp:78` |
| `is_filter_dropped_` | `bool` | Set when the outgoing candidate fails the per-crystal filter check. Mutually exclusive with `is_continue_`. | `src/core/raypath.hpp:84` |
| `is_prior_filter_failed_` | `bool` | Cross-layer persistent flag: set when any upstream MS layer's filter rejects this ray. Propagated via `TraceRayBasicInfo`. Reset only in `InitRayFirstMs`. | `src/core/raypath.hpp:90` |

### N4 Construction-Time Invariants

Checked by `RaySeg::IsValidComplete()` at `RayBuffer::EmplaceBack(RaySeg)` entry
(debug-only via `assert`). Source: `src/core/raypath.hpp:106-139`.

| ID | Invariant | Predicate |
|----|-----------|-----------|
| N4-1 | `to_face_` in valid polygon range | Omitted — requires external crystal context. |
| N4-2 | `w_` is non-negative or the TIR sentinel `-1.0f` | `IsValidW(w)` |
| N4-3 | `is_continue_` implies `to_face_ == kInvalidId` | `IsValidContinueFace(is_continue_, to_face_)` |
| N4-4 | `crystal_idx_` in `[0, kMaxCrystalNum)` or `kInvalidId` | `IsValidCrystalIdx(crystal_idx_)` |
| N4-5 | `d_` and `p_` components are finite (no NaN/Inf) | `IsValidVec3Finite(d_)`, `IsValidVec3Finite(p_)` |

---

## §2 Ray Chain Construction & Traversal

### all_data Layout

`all_data` (`RayBuffer`) is the flat append-only store for all segments across all crystals
and MS layers within one `SimulateOneWavelength` call. Segments are appended in
initialization order (source: `src/core/simulator.cpp:182,215,699`):

```
all_data = [ init_segs_crystal_0 | init_segs_crystal_1 | ... |
             hit1_children_c0    | hit1_children_c1    | ... |
             hit2_children_c0    | ...                        ]
```

### prev_ray_idx_ / root_ray_idx_ Semantics

- **Root rays** (first MS layer): `prev_ray_idx_ = kInfSize`. `root_ray_idx_` is the
  segment's own index in `all_data` (source: `src/core/simulator.cpp:151`).
- **Continuation rays** (subsequent MS layers): `prev_ray_idx_` inherits from the
  `is_continue_` parent in the prior layer (the value is NOT reset in `InitRayOtherMs`
  because it was already valid from the prior layer's `CollectData`).
  `root_ray_idx_` is reassigned to the new `all_data` offset
  (source: `src/core/simulator.cpp:207`).
- **Hit-loop children**: `TraceRayBasicInfo` copies `root_ray_idx_` from parent to both
  child segments (reflect + refract). `prev_ray_idx_` is NOT set by `TraceRayBasicInfo` —
  it remains whatever value the buffer slot held. In practice, the reflect/refract children
  are new slots written by `Propagate`/`HitSurface` and then field-copied in the manual
  loop (source: `src/core/simulator.cpp:380-396`).

### crystal_idx_ Chain-Aware Propagation

`crystal_idx_` is set once per crystal in `InitRay_other_info`
(source: `src/core/simulator.cpp:149`) and propagated to both child segments in
`TraceRayBasicInfo` (source: `src/core/simulator.cpp:388-389`). It identifies which
`Crystal` instance (in `SimData::crystals_[]`) the segment belongs to.

### from_face_ / to_face_ Split

The `from_face_` / `to_face_` pair implements a linked-list of polygon faces along the ray:

1. **Entry segment** (`InitRay_p_fid`): `from_face_ = kInvalidId`, `to_face_` = sampled
   polygon face (source: `src/core/simulator.cpp:76-77`).
2. **Hit-loop children** (`TraceRayBasicInfo`): each child's `from_face_` = parent's
   `to_face_` (the face just hit); `to_face_` = result of `Propagate` (next face hit, or
   `kInvalidId` for outgoing/TIR) (source: `src/core/simulator.cpp:371-377,383-385`).

The source-face guard passed to `Propagate` uses the parent's `to_face_` via
`BufferWrapper` with `step=2` (shared by reflect+refract pair), NOT `from_face_`
directly.

### Raypath Recorder (rp_)

- Initialized in `InitRay_other_info`: cleared, then first face-number appended via
  `crystal.GetFn(to_face_)` (source: `src/core/simulator.cpp:152-153`).
- Extended in `FillRayOtherInfo`: each child with `to_face_ != kInvalidId` gets the
  face-number appended (source: `src/core/simulator.cpp:403-410`).
- Copied from parent to both children in `TraceRayBasicInfo`
  (source: `src/core/simulator.cpp:381-382`).
- Maximum length: `kMaxHits` (64). Overflow silently drops
  (source: `src/core/raypath.cpp:6-8`).

---

## §3 Segment State Machine

### State Derivation

Segment kind is derived from two primary fields (`to_face_`, `w_`) and two 1-bit flags
(`is_continue_`, `is_filter_dropped_`). The five states are **mutually exclusive and
exhaustive** (source: `src/core/raypath.hpp:93-104`):

```
                        ┌─────────────────────────────────────┐
                        │          w_ < 0?                    │
                        │   YES ──► TIR (total reflection)    │
                        │   NO  ──┐                           │
                        │         │                           │
                        │    to_face_ != kInvalidId?          │
                        │   YES ──► NORMAL (in-crystal)       │
                        │   NO  ──┐                           │
                        │         │  (outgoing candidate)     │
                        │    is_continue_?                    │
                        │   YES ──► CONTINUE (next MS)        │
                        │   NO  ──┐                           │
                        │    is_filter_dropped_?              │
                        │   YES ──► FILTER-DROPPED (anchor)   │
                        │   NO  ──► OUTGOING (emit)           │
                        └─────────────────────────────────────┘
```

| State | Predicate | Meaning |
|-------|-----------|---------|
| TIR | `w_ < 0` | Total internal reflection; ray is absorbed. |
| Normal | `to_face_ != kInvalidId && w_ >= 0` | Ray hit another face; continues tracing inside the crystal. |
| Outgoing | `to_face_ == kInvalidId && w_ >= 0 && !is_continue_ && !is_filter_dropped_` | Ray exits crystal, passes filter (or no filter), does not continue to next MS. Emitted to output. |
| Continue | `is_continue_` (implies `to_face_ == kInvalidId && w_ >= 0`) | Ray exits crystal and continues to next MS layer. |
| Filter-Dropped | `is_filter_dropped_` (implies `to_face_ == kInvalidId && w_ >= 0`) | Ray exits crystal but fails filter; routed to anchor lane. |

### Transition Rules in CollectData

`CollectData` (source: `src/core/simulator.cpp:413-474`) is the sole state-assignment
function. It processes `buffer_data[1]` (the output of `TraceRayBasicInfo` + `FillRayOtherInfo`)
and writes the state flags.

**Pre-loop reset** (pool-reuse guard): both `is_continue_` and `is_filter_dropped_` are
explicitly reset to `false` for every segment at the top of the loop
(source: `src/core/simulator.cpp:434-435`).

**Branch table** for outgoing candidates (`to_face_ == kInvalidId && w_ >= 0`):

| Filter | Prior Failed | Prob | is_continue_ | is_filter_dropped_ | is_prior_filter_failed_ | Destination |
|--------|-------------|------|--------------|-------------------|------------------------|-------------|
| pass | no | pass | true | false | unchanged | next MS |
| pass | no | fail | false | false | unchanged | outgoing buffer |
| pass | yes | pass | true | false | unchanged | next MS (anchor bypass) |
| pass | yes | fail | false | true | unchanged | anchor lane |
| fail | — | pass | true | false | **set true** | next MS |
| fail | — | fail | false | true | **set true** | anchor lane |

Source: `src/core/simulator.cpp:430-458`. See also `doc/filter-architecture.md §7` for the
anchor-lane design rationale.

**Coordinate rotation**: outgoing candidates have `d_` and `p_` rotated from crystal-local
to world-space (`crystal_rot_.Apply`) **before** the filter check
(source: `src/core/simulator.cpp:442-443`). TIR segments are rotated **after** the state
branch (source: `src/core/simulator.cpp:463-466`). Normal segments remain in crystal-local
coordinates.

**Routing after state assignment**:
- `IsNormal()` → re-enters `buffer_data[0]` for the next hit iteration
  (source: `src/core/simulator.cpp:468-469`).
- `IsContinue()` → appended to `init_data[1]` for the next MS layer
  (source: `src/core/simulator.cpp:471-472`).
- `IsOutgoing()` / `IsFilterDropped()` → collected in the main loop into `outgoing_*` /
  `anchor_*` vectors respectively (source: `src/core/simulator.cpp:700-714`).

### Reset Points Summary

| Field | Reset Location | Trigger | Source |
|-------|---------------|---------|--------|
| `is_continue_` | `CollectData` loop top | Every segment, every hit iteration | `src/core/simulator.cpp:434` |
| `is_continue_` | `InitRayOtherMs` step 1.4 | Pool-reuse guard for MS layer entry | `src/core/simulator.cpp:211` |
| `is_filter_dropped_` | `CollectData` loop top | Every segment, every hit iteration | `src/core/simulator.cpp:435` |
| `is_prior_filter_failed_` | `InitRayFirstMs` step 1.4 | Once per root ray (first MS only) | `src/core/simulator.cpp:179` |
| `rp_` | `InitRay_other_info` | Each crystal initialization | `src/core/simulator.cpp:152` |

**Key invariant**: `is_prior_filter_failed_` is the **only** cross-MS-layer persistent bool.
It is set in `CollectData` when a filter fails (source: `src/core/simulator.cpp:447`) and
propagated to child segments in `TraceRayBasicInfo` (source: `src/core/simulator.cpp:386-387`).
It is reset only in `InitRayFirstMs` (source: `src/core/simulator.cpp:179`), never in
`InitRayOtherMs` — this is by design, so that a filter failure in layer N causes all
downstream layers to route the ray to the anchor lane.

---

## §4 Filter Interface Contract

### FilterSpec::Check Preconditions

`FilterSpec::Check(const RaySeg& ray)` is called inside `CollectData` on outgoing
candidates only (source: `src/core/simulator.cpp:445`). At the point of call:

1. **Coordinate space**: `d_` and `p_` have already been rotated to world-space via
   `crystal_rot_.Apply()` (source: `src/core/simulator.cpp:442-443`).
2. **Raypath**: `rp_` is fully populated for the current crystal (all face-numbers appended
   by `FillRayOtherInfo`).
3. **Segment kind**: `to_face_ == kInvalidId && w_ >= 0` (outgoing candidate).

### Fields Consumed by Filter Implementations

| FilterSpec subclass | RaySeg fields consumed | Source |
|--------------------|----------------------|--------|
| `NoneSpec` | None (always returns true) | `src/core/filter_spec.cpp:144` |
| `RaypathSpec` | `rp_` (via `RaypathOrbit::Contains`) | `src/core/filter_spec.cpp:152` |
| `EntryExitSpec` | `rp_[0]` and `rp_[size_-1]` (entry/exit face numbers) | `src/core/filter_spec.cpp:163-169` |
| `DirectionSpec` | `d_[3]` (world-space direction, dot product against target) | `src/core/filter_spec.cpp:185` |
| `CrystalSpec` | `crystal_config_id_` | `src/core/filter_spec.cpp:195` |
| `ComplexSpec` | Delegates to sub-filters (OR-of-AND composition) | `src/core/filter_spec.cpp:206-219` |

### FilterSpec::Create

`FilterSpec::Create(config, crystal, axis_dist)` builds the spec from the filter
configuration bound to a crystal (source: `src/core/filter_spec.cpp:277-284`). It
pre-computes:
- `d_applicable`: whether D symmetry applies (azimuth uniform + roll mean at 30° multiple).
- `sigma_a`: the mirror parameter for D symmetry reduction.
- `action_`: filter-in or filter-out (set after construction).

These are **construction-time** computations. The `Match()` call on the hot path is
stateless — no per-ray mutation.

---

## §5 Boundary with raypath-symmetry.md

| Concern | This document | `doc/raypath-symmetry.md` |
|---------|--------------|--------------------------|
| RaySeg struct fields & invariants | §1 — full field table, sentinel values, N4 checks | Not covered |
| Ray chain construction (prev/root indices) | §2 — all_data layout, index propagation | Not covered |
| Segment state machine (TIR/Normal/Outgoing/Continue/FilterDropped) | §3 — derivation rules, CollectData branch table, reset points | Not covered |
| Filter interface contract | §4 — preconditions, consumed fields per FilterSpec | Not covered |
| P, B, D symmetry toggles | Not covered (see cross-ref below) | §3 — full toggle semantics, enabling conditions |
| Face-number encoding (fn) | §2 (briefly, as raypath recorder input) | §2a — face numbering convention table |
| Canonical form / orbit reduction | Not covered | §3-§4 — P/B/D reduction formulas |
| GUI filter behavior | Not covered | §6 — D checkbox, informational indicator |

### Cross-References

- Raypath face-number encoding and PBD symmetry: `doc/raypath-symmetry.md`
- Filter routing design (Design A), anchor-lane semantics: `doc/filter-architecture.md §2/§7`
- Filter JSON configuration schema: `doc/configuration.md`
- Crystal orientation sampling: `doc/crystal-orientation-sampling.md`

---

## Appendix: Key Constants

| Constant | Value | Defined in | Usage |
|----------|-------|-----------|-------|
| `kInvalidId` | `0xffff` | `src/core/def.hpp:11` | "No face" / "no crystal" sentinel for `IdType` fields |
| `kInfSize` | `SIZE_MAX` | `src/core/def.hpp:12` | "No parent" sentinel for `prev_ray_idx_` |
| `kMaxHits` | `64` | `src/core/def.hpp:24` | Maximum face hits per crystal; caps `RaypathRecorder` |
| `kMaxCrystalNum` | `16` | `src/core/def.hpp:26` | Upper bound for `crystal_idx_` (N4-4) |
| `kSmallBatchRayNum` | `32` | `src/core/simulator.hpp:64` | Inner-loop batch size for ray tracing |
