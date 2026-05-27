# EV Pipeline Architecture

This document describes the internal invariants of Lumice's EV (Exposure Value)
pipeline — the data flow from raw XYZ accumulation through to the final
`intensity_scale` that controls pixel brightness in every consumption path.

**Target audience**: contributors who modify the rendering pipeline, the C API
surface, or the GUI EV controls.

**Scope**: internal invariants and data-flow contracts. This document does **not**
cover:

- User-visible Adaptive Brightness behavior, the P99.5 algorithm, or additivity
  — see [`doc/adaptive-brightness.md`](adaptive-brightness.md).
- The F1 anchor-lane design rationale, `LUMICE_RawXyzResult` field semantics, or
  filter branch-table routing — see
  [`doc/filter-architecture.md §7`](filter-architecture.md).

---

## §1 Terminology

| Term | Definition |
|------|------------|
| **internal buffer** | Per-`Consume()` accumulation target (`internal_xyz_`, `anchor_internal_xyz_`). Written under the consumer mutex; never read by the GUI. |
| **snapshot buffer** | Point-in-time copy of the internal buffer (`snapshot_xyz_`, `anchor_snapshot_xyz_`). Produced by `PrepareSnapshot()`; read by `GetRawXyzResult()` and `PostSnapshot()`. |
| **anchor lane** | The filter-independent accumulation path. Collects filter-fail emission in `anchor_internal_xyz_` and combines it with filter-pass emission in `PrepareSnapshot()` to produce filter-independent statistics. |
| **`kNormScale`** | Display brightness baseline constant (0.08). Maps per-pixel radiance to a [0, 1] range at EV = 0 so that the average illuminated pixel is ~5% brightness and bright halo features (~20× average) approach full white. Independent of resolution and FOV. Defined in `render.cpp:30`. |

---

## §2 Data Flow

### §2.1 Per-Consume Accumulation

`RenderConsumer::Consume()` (`render.cpp:341`) processes one batch of simulator
output. Two parallel accumulations happen in every call:

1. **Filtered lane** (always active):
   - Projects `data.outgoing_d_` / `data.outgoing_w_` (already filter-gated by
     the simulator — Design A, see `filter-architecture.md §2`) through the lens
     projection.
   - Compacts valid in-viewport pixels and accumulates via
     `SpectrumToXyz()` → `internal_xyz_`.
   - `total_intensity_ += landed_weight` (sum of all landed ray weights).

2. **Anchor lane** (active only when `data.anchor_d_` is non-empty):
   - Projects `data.anchor_d_` / `data.anchor_w_` (filter-fail emission) through
     the same lens projection.
   - Accumulates via `SpectrumToXyz()` → `anchor_internal_xyz_`.
   - `anchor_total_intensity_ += anchor_landed`.

**Linearity invariant**: both accumulations are linear in XYZ space — each ray's
contribution is `CMF(wavelength) × weight`, summed per pixel. This ensures exact
additivity for complementary filter partitions (see `adaptive-brightness.md §4`).

**Overlap dual-write**: for dual fisheye lens types, pass 2 re-projects overlap-zone
rays to the opposite hemisphere. This pass does **not** update `total_intensity_`
(preserves normalization). The anchor lane intentionally skips overlap dual-write:
the overlap ring is a small geometric artifact that contributes negligibly to the
P99.5 percentile (`render.cpp:498-501`).

### §2.2 PrepareSnapshot

`PrepareSnapshot()` (`render.cpp:560`) creates a point-in-time snapshot of all
accumulation state. Called by `DoSnapshot()` under the consumer mutex.

1. `snapshot_xyz_` ← memcpy of `internal_xyz_`.
2. `snapshot_intensity_` ← `total_intensity_`.
3. **Anchor P99.5 computation** (when `anchor_internal_xyz_` is non-null):
   - `anchor_snapshot_xyz_` ← memcpy of `anchor_internal_xyz_`.
   - `anchor_snapshot_intensity_` = `snapshot_intensity_` + `anchor_total_intensity_`
     (filter-pass + filter-fail combined landed intensity).
   - Computes `anchor_p995_y_`: the P99.5 value of the Y channel across all pixels
     where `snapshot_xyz_[i*3+1] + anchor_snapshot_xyz_[i*3+1] > 0`. Uses
     `std::nth_element` at index `⌊count × 0.995⌋`, clamped to `count - 1`.

**Caching invariant**: `anchor_p995_y_` is computed once per snapshot and cached until
the next `PrepareSnapshot()` call. It is never incrementally updated between snapshots.

**Zero-skip semantics**: pixels with zero combined Y are excluded from the P99.5
population. This matches the GUI-side `ComputeP995Y()` in `gui_ev_auto.hpp:12` and
ensures the percentile reflects only illuminated pixels.

### §2.3 kNormScale and Per-Pixel Intensity

`GetRawXyzResult()` (`render.cpp:673`) converts the raw accumulated
`snapshot_intensity_` to a per-pixel intensity:

```
per_pixel_intensity = snapshot_intensity_ / (kNormScale × total_pixels)
anchor_per_pixel    = anchor_snapshot_intensity_ / (kNormScale × total_pixels)
```

The `kNormScale` factor (0.08) sets the display brightness baseline: at EV = 0, the
average illuminated pixel is ~5% brightness. In the GUI's `intensity_scale` computation,
`kNormScale` algebraically cancels because `ComputeEvAuto()` receives
`anchor_snapshot_intensity` (which already incorporates `kNormScale`) as an input — the
resulting `ev_auto` absorbs the scale so that `2^ev_auto / norm_intensity` is independent
of the specific `kNormScale` value. **C API consumers** receive `per_pixel_intensity`
values whose absolute magnitude is determined by `kNormScale`; this constant is not an
arbitrary scale factor and must not be changed without re-calibrating the brightness
baseline.

### §2.4 C API Surface

`LUMICE_RawXyzResult` (`lumice.h:66-89`) exposes the following EV-relevant fields:

| Field | Source |
|-------|--------|
| `xyz_buffer` | `snapshot_xyz_` (filtered XYZ data) |
| `snapshot_intensity` | Per-pixel filtered intensity |
| `intensity_factor` | CLI/config EV factor `2^EV` |
| `effective_pixels` | Non-zero pixel count |
| `anchor_p995_y` | Server-side F1 anchor P99.5 of combined Y |
| `anchor_snapshot_intensity` | Per-pixel combined (filter-pass + filter-fail) intensity |

**Lifetime**: the `xyz_buffer` pointer is valid until the next
`LUMICE_GetRawXyzResults()` or `LUMICE_CommitConfig()` call. The sentinel value is
`xyz_buffer == NULL`.

### §2.5 GUI SyncFromPoller

`SyncFromPoller()` (`app.cpp:756`) transfers data from the poller thread to GUI state.
The EV-relevant logic (`app.cpp:805-811`):

```
filter present:   p995_raw_y ← anchor_p995_y
                  ev_auto    ← ComputeEvAuto(anchor_p995_y, anchor_snapshot_intensity, target_white)

no filter:        p995_raw_y ← p995_y          (from filtered snapshot; ≡ total emission)
                  ev_auto    ← ComputeEvAuto(p995_y, snapshot_intensity, target_white)
```

**Source-coherence invariant**: the EV numerator (`p995_raw_y`) and the normalization
denominator (`norm_intensity`) must always come from the same data source — either
both from the anchor lane, or both from the filtered snapshot. This is enforced by
the dual condition `anchor_p995_y > 0 && anchor_snapshot_intensity > 0`.

---

## §3 Anchor Lane Lifecycle

### §3.1 Lazy Allocation

Anchor buffers (`anchor_internal_xyz_`, `anchor_snapshot_xyz_`) are **not** allocated
in the `RenderConsumer` constructor (`render.cpp:321-338`). They are lazily allocated
on the first `Consume()` call where `data.anchor_d_` is non-empty
(`render.cpp:506-513`).

**Zero-cost invariant**: when no filter is configured, the simulator never produces
filter-fail rays (`SimData::anchor_d_` stays empty), so anchor buffers are never
allocated and the anchor lane has zero memory and zero CPU cost.

### §3.2 Per-Consume Accumulation

When `data.anchor_d_` is non-empty:

1. Lazy-allocate `anchor_internal_xyz_` and `anchor_snapshot_xyz_` (if first time).
2. Project `anchor_d_` through the same lens projection as the filtered lane.
3. Compact valid in-viewport pixels.
4. `SpectrumToXyz()` → `anchor_internal_xyz_`.
5. `anchor_total_intensity_ += anchor_landed`.

The anchor lane accumulates only filter-fail emission. Filter-pass emission is already
in `internal_xyz_`. The combined buffer is reconstructed pixel-by-pixel in
`PrepareSnapshot()` (§2.2).

### §3.3 PrepareSnapshot Cached State

`PrepareSnapshot()` computes three cached values from the anchor lane:

| Cached field | Formula | Reset condition |
|---|---|---|
| `anchor_snapshot_xyz_[i]` | memcpy of `anchor_internal_xyz_[i]` | `Reset()` zeros `anchor_internal_xyz_` |
| `anchor_snapshot_intensity_` | `snapshot_intensity_ + anchor_total_intensity_` | `Reset()` zeros both components |
| `anchor_p995_y_` | P99.5 of `(snapshot_xyz_[i*3+1] + anchor_snapshot_xyz_[i*3+1])` over positive pixels | `Reset()` sets to 0 |

These values are read-only between snapshots. No incremental update path exists.

### §3.4 Reset Conditions

Two events trigger anchor data reset:

1. **`Reset()`** (`render.cpp:691`): zeros `anchor_internal_xyz_` (if allocated),
   `anchor_total_intensity_`, `anchor_snapshot_intensity_`, and `anchor_p995_y_`.
   Does **not** deallocate the buffer (preserves allocation for reuse).

2. **Consumer destruction** (`consumers_.clear()` in the full rebuild path): the
   `RenderConsumer` destructor releases all buffers via `unique_ptr` RAII.

`Reset()` is called by `ResetWith()` (`render.cpp:712`) — the consumer-reuse path
triggered when `CommitConfig()` calls `Stop()` → `ResetWith()` → `Reset()`.

**Rebuild path** (`CommitConfig()` → `Stop()` → `consumers_.clear()` → `~RenderConsumer()`):
anchor buffers are released via `unique_ptr` RAII. `Reset()` is **not** called; there is
no zero-and-retain step, only deallocation.

---

## §4 Three Consumption Paths

Three code paths consume the EV pipeline output and compute `intensity_scale`. All
three use the same formula:

```
ev_total        = exposure_offset + ev_auto
intensity_factor = 2^ev_total
norm_intensity   = anchor_snapshot_intensity  (when anchor lane active)
                 | snapshot_intensity          (degenerate / no-filter path)
intensity_scale  = intensity_factor / norm_intensity   (0 if norm_intensity ≤ 0)
```

### §4.1 Display Path

`app_panels.cpp:816-824` — sets the GPU shader uniform each frame:

```cpp
float ev_total = rc.exposure_offset + g_state.ev_auto;
pp.exposure.intensity_factor = std::pow(2.0f, ev_total);
float norm_intensity = (g_state.anchor_snapshot_intensity > 0.0f && g_state.p995_raw_y > 0.0f)
                           ? g_state.anchor_snapshot_intensity
                           : g_state.snapshot_intensity;
pp.exposure.intensity_scale = norm_intensity > 0 ? pp.exposure.intensity_factor / norm_intensity : 0.0f;
```

### §4.2 Export Path

`BuildExportParams()` (`app.cpp:297-306`) — constructs `PreviewParams` for PNG export.
Explicitly mirrors the display path formula. The inline comment documents the
intentional byte-identity with the shader uniform.

### §4.3 Screenshot / .lmc Thumbnail

`RefreshCpuTextureForSave()` (`app.cpp:241-256`) — CPU-side XYZ→sRGB for `.lmc`
thumbnail. Uses cached `g_state.anchor_snapshot_intensity` rather than a fresh
`LUMICE_GetRawXyzResults()` call. This introduces a potential ≤1-frame drift from the
display path, which is acceptable for thumbnails (not user-visible in real time).

### §4.4 Consistency Invariant

All three paths share the same formula by construction. The dual condition
(`anchor_snapshot_intensity > 0 && p995_raw_y > 0`) ensures the EV numerator
(`ev_auto`, derived from `p995_raw_y`) and the normalization denominator
(`norm_intensity`) always come from the same data source.

**Drift scope**: the screenshot path reads cached `g_state` values that may lag the
most recent `SyncFromPoller()` by at most one frame. Since `.lmc` thumbnail is saved
on user action (not streamed), this drift is imperceptible.

---

## §5 `unfiltered_intensity` Semantics

### §5.1 Filter Present

When a filter spec is configured:

- The simulator produces both `outgoing_d_/w_` (filter-pass) and `anchor_d_/w_`
  (filter-fail) in each `SimData` batch.
- The anchor lane accumulates filter-fail emission in `anchor_internal_xyz_`.
- `PrepareSnapshot()` combines filter-pass (`snapshot_xyz_`) and filter-fail
  (`anchor_snapshot_xyz_`) pixel-by-pixel to compute `anchor_p995_y_`.
- `anchor_snapshot_intensity_` = filter-pass intensity + filter-fail intensity,
  representing the total landed intensity across all rays regardless of filter.

### §5.2 No-Filter Degenerate

When no filter is configured:

- The simulator produces no filter-fail rays (`anchor_d_` stays empty).
- `anchor_internal_xyz_` is never allocated (§3.1 lazy allocation guard).
- `PrepareSnapshot()` skips the anchor computation (`anchor_internal_xyz_` is null).
- `anchor_p995_y_` and `anchor_snapshot_intensity_` remain at their `Reset()` values
  (both 0).

### §5.3 Equivalence Invariant

In the no-filter scenario, the GUI detects `anchor_p995_y == 0` and falls back to the
filtered snapshot path (`SyncFromPoller`, `app.cpp:808-810`). This is numerically
equivalent to the anchor path because:

- Without a filter, every ray passes the (vacuous) filter. There are zero filter-fail
  rays.
- Therefore `filtered_emission ≡ total_emission`.
- `p995_y(filtered) ≡ p995_y(total)` and
  `snapshot_intensity ≡ anchor_snapshot_intensity` (if anchor were active).

The fallback is a missing-data guard, not a semantic mode switch.

---

## §6 CommitConfig Impact on Anchor Data

`CommitConfig()` (`server.cpp:263-308`) always calls `Stop()` before modifying
consumers. `Stop()` transitions the server to `kStopped` and sets
`has_ever_consumed_ = false` (`server.cpp:524`).

### §6.1 Reuse Path

When `NeedsRebuild()` returns false (layout fields unchanged):

```
CommitConfig → Stop() → ResetWith(new_config) → Reset()
```

`Reset()` zeros all anchor accumulators but **does not deallocate** anchor buffers.
The `unique_ptr`s retain their allocations for reuse by subsequent `Consume()` calls.

### §6.2 Rebuild Path

When `NeedsRebuild()` returns true (layout fields changed):

```
CommitConfig → Stop() → consumers_.clear() → new RenderConsumer(...)
```

`consumers_.clear()` triggers `~RenderConsumer()`, which releases all buffers
(including anchor buffers) via `unique_ptr` RAII. New consumers start with null
anchor buffers (lazy allocation resumes on first filter-fail data).

### §6.3 Filter Switch Semantics

Filter configuration is **not** in the `NeedsRebuild()` field set — the filter is
applied simulator-side (Design A), not in the consumer. However, filter switches in
the GUI trigger `CommitConfig()`, which always calls `Stop()` → `Reset()` (or
rebuild). Therefore:

**Invariant**: anchor data is always zeroed on filter switch. The anchor lane
re-accumulates from scratch after each filter change. For the same scene geometry,
the anchor P99.5 converges to the same value regardless of which filter is active,
because the anchor combines filter-pass and filter-fail emission (total emission is
filter-independent).

This is by design: "anchor doesn't reset" refers to the semantic property that the
anchor value is filter-independent, not that the buffer persists across filter
switches.

### §6.4 NeedsRebuild Trigger Fields

`NeedsRebuild()` (`render_config.cpp:164`) compares layout-affecting fields only:

| Field | Triggers rebuild |
|-------|:---:|
| `resolution_` | ✓ |
| `lens_` | ✓ |
| `lens_shift_` | ✓ |
| `view_` | ✓ |
| `visible_` | ✓ |
| `overlap_` | ✓ |
| `background_`, `ray_color_`, `opacity_`, `intensity_factor_`, `norm_mode_`, grids | ✗ (appearance only) |

A `static_assert(sizeof(RenderConfig) == 144)` guards against silent field additions
(`render_config.cpp:166`).

---

## §7 Source Reference Table

| Invariant / Concept | Source Location |
|---|---|
| `kNormScale` definition (0.08) | `render.cpp:30` |
| `Consume()` — filtered + anchor parallel accumulation | `render.cpp:341-547` |
| Anchor lazy allocation guard | `render.cpp:506-513` |
| Anchor overlap skip rationale | `render.cpp:498-501` |
| `PrepareSnapshot()` — anchor P99.5 computation | `render.cpp:560-593` |
| P99.5 nth_element index formula | `render.cpp:585-591` |
| `GetRawXyzResult()` — per-pixel intensity formula | `render.cpp:673-688` |
| `Reset()` — anchor zeroing semantics | `render.cpp:691-710` |
| `ResetWith()` — config update + reset | `render.cpp:712-717` |
| Constructor — lazy allocation comment | `render.cpp:334-337` |
| `CommitConfig()` — Stop→Reset/Rebuild→Start | `server.cpp:263-308` |
| `has_ever_consumed_` reset in Stop | `server.cpp:524` |
| `NeedsRebuild()` — layout field comparison | `render_config.cpp:164-173` |
| `sizeof(RenderConfig)` static_assert | `render_config.cpp:166` |
| `ComputeP995Y()` / `ComputeEvAuto()` | `gui_ev_auto.hpp:12-46` |
| `SyncFromPoller()` — anchor source selection + ev_auto | `app.cpp:756-813` |
| `BuildExportParams()` — export EV consistency | `app.cpp:297-306` |
| `RefreshCpuTextureForSave()` — .lmc thumbnail EV | `app.cpp:241-256` |
| Display path `ev_total` / `intensity_scale` | `app_panels.cpp:816-824` |
| `LUMICE_RawXyzResult` anchor fields | `lumice.h:66-89` |
