# Accumulator / Consumer Architecture

This document describes the internal invariants of Lumice's accumulator/consumer
subsystem — the bridge between the simulator's raw ray output and the rendering
pipeline (GUI preview and CLI image export).

**Target audience**: contributors who modify consumer lifecycle management,
snapshot protocol, thread synchronization, or the `NeedsRebuild` / `ResetWith`
reuse path.

**Scope**: internal invariants and data-flow contracts. This document does **not**
cover:

- EV auto algorithm, P99.5 computation, or adaptive brightness
  — see [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md).
- Filter subsystem internals or Design A gate semantics
  — see [`doc/filter-architecture.md`](filter-architecture.md).
- C API lifecycle (server creation, config commit, result retrieval)
  — covered by a separate audit.

---

## §1 Overview

The accumulator/consumer subsystem sits between the simulator thread pool and the
result-consumption paths (GUI `ServerPoller`, CLI `GetRenderResults`). Its
responsibilities:

1. **Accumulate** incoming ray data (`SimData`) into per-pixel XYZ buffers.
2. **Snapshot** accumulated state for consumption without blocking the simulation.
3. **Transform** XYZ data to sRGB for CLI image export (`PostSnapshot`).
4. **Expose** raw XYZ data for GPU-side rendering in the GUI path.

Key source files:

| File | Role |
|------|------|
| `src/server/consumer.hpp` | `IConsume` interface |
| `src/server/render.hpp` / `render.cpp` | `RenderConsumer` — projection, accumulation, snapshot |
| `src/server/stats.hpp` / `stats.cpp` | `StatsConsumer` — ray/crystal counters |
| `src/server/server.cpp` | `ServerImpl` — consumer orchestration, locking, snapshot protocol |
| `src/config/render_config.hpp` / `.cpp` | `RenderConfig`, `NeedsRebuild()` |
| `src/gui/server_poller.hpp` / `.cpp` | GUI consumption side |

---

## §2 IConsume Interface

`IConsume` (`consumer.hpp:18`) defines the consumer contract:

| Method | Called under | Purpose |
|--------|-------------|---------|
| `Consume(SimData)` | `consumer_mutex_` | Accumulate one batch of ray data |
| `PrepareSnapshot()` | `consumer_mutex_` | `memcpy` internal → snapshot buffers |
| `PostSnapshot()` | `snapshot_mutex_` | XYZ→RGB conversion (CLI path) |
| `GetResult()` | `snapshot_mutex_` | Return typed result (`RenderResult` / `StatsResult`) |
| `Reset()` | `consumer_mutex_` | Zero accumulators, preserve buffer allocations |

Two concrete implementations:

- **`RenderConsumer`**: projects rays through the lens model, accumulates into
  `internal_xyz_`, snapshots to `snapshot_xyz_`, converts to sRGB in
  `PostSnapshot()`.
- **`StatsConsumer`**: counts `total_rays_`, `sim_rays_`, `crystals_`.

Both are held via `shared_ptr<IConsume>` in `ServerImpl::consumers_`.

---

## §3 Consumer State Machine

### §3.1 Lifecycle Paths

The consumer lifecycle has four entry points:

```
CommitConfig (full rebuild, NeedsRebuild=true or first commit):
  Stop() → consumers_.clear() → new RenderConsumer(config) → new StatsConsumer → Start()

CommitConfig (reuse path, NeedsRebuild=false):
  Stop() → ResetWith(new_config) for RenderConsumers, Reset() for StatsConsumer → Start()

Stop():
  snapshot_dirty_ = false
  has_ever_consumed_ = false          ← prevents stale snapshot reads
  (consumers_ not cleared — CommitConfig decides rebuild vs. reuse)

~ServerImpl():
  Stop() → state_ = kTerminating → join threads → consumers_ destroyed with ServerImpl
```

### §3.2 State Transition Diagram

Consumer state is implicit, encoded in flags rather than an explicit enum:

```
                    CommitConfig
                   (full rebuild)        ConsumeData (under consumer_mutex_)
  ┌──────────┐  ──────────────────►  ┌──────┐  ──────────────────────────►  ┌──────────────┐
  │ Created  │                       │ Idle │                               │ Accumulating │
  └──────────┘                       └──┬───┘  ◄────────────────────────── └──────────────┘
                                        │         Stop() + CommitConfig
                                        │         (rebuild or reuse)
                                        │
                              GetRawXyzResults /
                                DoSnapshot
                                        │
                                        ▼
                                  ┌──────────────┐
                                  │ Snapshotted  │
                                  │ (snapshot_xyz_│
                                  │   valid)     │
                                  └──────────────┘
```

State flags:

| Flag | Set by | Cleared by | Meaning |
|------|--------|------------|---------|
| `has_ever_consumed_` | `ConsumeData` (`server.cpp:613`) | `Stop()` (`server.cpp:524`) | True after first batch consumed; gates `has_valid_data_` in results |
| `snapshot_dirty_` | `ConsumeData` (`server.cpp:612`) | `PrepareSnapshot` in `DoSnapshot`/`GetRawXyzResults` (`server.cpp:350,398`) | Indicates new data since last snapshot |
| `snapshot_generation_` | `GetRawXyzResults` (`server.cpp:399`) | Never reset (monotonic) | Poller detects new snapshots via generation comparison |

---

## §4 Thread Safety Model

### §4.1 Two-Lock Design

| Lock | Type | Guards | Writer thread | Reader thread |
|------|------|--------|---------------|---------------|
| `consumer_mutex_` | `TicketMutex` (FIFO) | `consumers_` list, all consumer mutable state (`internal_xyz_`, `total_intensity_`) | `ConsumeData` thread (high frequency) | Poller → `GetRawXyzResults` (low frequency); `CommitConfig` caller (rebuild/reset) |
| `snapshot_mutex_` | `std::mutex` | `cached_render_results_`, `cached_stats_result_` | `DoSnapshot` / `GetRawXyzResults` (after snapshot) | `Get*Results` callers |

**Why TicketMutex**: on Windows, `std::mutex` uses SRWLOCK which provides no
fairness guarantee. A high-frequency locker (`ConsumeData`, called per simulator
batch) can starve a low-frequency waiter (`ServerPoller`, polling every
~100ms). `TicketMutex` (`server.cpp:35`) assigns monotonic tickets and serves
in FIFO order, guaranteeing bounded wait.

### §4.2 Two-Phase Snapshot Protocol

`DoSnapshot()` (`server.cpp:336`) and `GetRawXyzResults()` (`server.cpp:386`)
use a two-phase protocol to minimize `consumer_mutex_` hold time:

```
Phase 1  (consumer_mutex_ held):
  PrepareSnapshot() for each consumer    ← memcpy internal → snapshot buffers
  snapshot_consumers = consumers_        ← shared_ptr copy keeps objects alive
  snapshot_dirty_ = false

Phase 1.5 (NO lock held):
  CountEffectivePixels()                 ← O(W×H) scan of snapshot_xyz_

Phase 2  (snapshot_mutex_ held):
  PostSnapshot() for each consumer       ← XYZ→RGB (CLI path only)
  Cache results
```

**Critical invariant**: `snapshot_xyz_` is stable between Phase 1 and the next
`PrepareSnapshot()` call. `CountEffectivePixels()` (`render.cpp:596`) relies on
this — it reads `snapshot_xyz_` outside `consumer_mutex_` but before any
subsequent `PrepareSnapshot()` can overwrite it. This is safe because:

1. `ConsumeData` only writes to `internal_xyz_` (never `snapshot_xyz_`).
2. `PrepareSnapshot` is the sole writer of `snapshot_xyz_`, and it runs under
   `consumer_mutex_` which the snapshot caller already released.
3. The snapshot caller holds `snapshot_consumers` via `shared_ptr`, preventing
   object destruction.

### §4.3 Consumer Lifetime Safety

`consumers_` is a `vector<shared_ptr<IConsume>>`. During `DoSnapshot` /
`GetRawXyzResults`, the snapshot code copies the `shared_ptr` vector under
`consumer_mutex_`, then operates on the copies outside the lock. Even if
`CommitConfig` calls `consumers_.clear()` concurrently (under `consumer_mutex_`),
the snapshot's `shared_ptr` copies keep the consumer objects alive until the
snapshot operation completes.

---

## §5 NeedsRebuild / ResetWith

### §5.1 NeedsRebuild Semantics

`NeedsRebuild()` (`render_config.cpp:164`) compares layout-affecting fields
between old and new `RenderConfig`:

| Triggers rebuild (`true`) | Does NOT trigger rebuild (`false`) |
|---------------------------|------------------------------------|
| `resolution_` | `background_` |
| `lens_` (type + fov) | `ray_color_` |
| `lens_shift_` | `opacity_` |
| `view_` | `intensity_factor_` |
| `visible_` | `norm_mode_` |
| `overlap_` | `central_grid_` / `elevation_grid_` / `celestial_outline_` |

**Filter changes do not trigger NeedsRebuild.** Filter specs belong to
`SceneConfig`, not `RenderConfig`. When a filter changes, consumers walk the
reuse path: `Stop()` → `ResetWith()` → `Start()`. Accumulators are zeroed but
buffer allocations are preserved.

### §5.2 sizeof Sentinel

```cpp
static_assert(sizeof(RenderConfig) == 144,
              "Update NeedsRebuild when RenderConfig fields change");
```

This sentinel (`render_config.cpp:166`) forces a compile error when any field is
added to or removed from `RenderConfig`. The developer must then classify the
new field as layout-affecting (add to `NeedsRebuild` comparisons) or
appearance-only (no change needed). This prevents silent drift where a new
layout field is added but `NeedsRebuild` is not updated.

### §5.3 ResetWith Path

`ResetWith(new_config)` (`render.cpp:715`):
```cpp
void RenderConsumer::ResetWith(const RenderConfig& new_config) {
  config_ = new_config;
  Reset();
}
```

Because `NeedsRebuild` returned `false`, all layout fields are identical.
Assigning the full config is safe — `rot_` and all buffer sizes remain valid.
`Reset()` zeros the accumulation state without reallocating buffers.

### §5.4 CommitConfig Reuse Logic

`CommitConfig` (`server.cpp:273`) determines reuse eligibility:

1. `consumers_` must be non-empty (first commit always rebuilds).
2. Renderer count must match (same number of render configs).
3. Renderer keys must match (same map ordering).
4. `NeedsRebuild()` must return `false` for every renderer pair.

If all conditions pass, the reuse path calls `ResetWith()` on each
`RenderConsumer` and `Reset()` on `StatsConsumer`, avoiding the cost of
destroying and reconstructing consumers (~0.5ms vs. ~30ms for full rebuild).

---

## §6 Data Integrity Invariants

| Invariant | Location | Notes |
|-----------|----------|-------|
| `internal_xyz_` size = `W × H × 3` floats | `render.cpp:324` (constructor) | Allocated once, never resized |
| `snapshot_xyz_` size = `W × H × 3` floats | `render.cpp:325` (constructor) | Same allocation as internal |
| `snapshot_work_` size = `W × H × 3` floats | `render.cpp:326` (constructor) | Preserves `snapshot_xyz_` during `PostSnapshot` |
| `snapshot_image_buffer_` size = `W × H × 3` bytes | `render.cpp:327` (constructor) | sRGB output for CLI |
| `d_buf_`/`w_buf_`/`xy_buf_`/`overlap_w_buf_` capacity ≥ max(rays, outgoing) | `render.cpp:347–354` | Grow-only; never shrink |
| `snapshot_xyz_` read-only between `PrepareSnapshot()` calls | `render.cpp:674` (`GetRawXyzResult` returns raw pointer) | Enables lock-free read in Phase 1.5 and result delivery |
| `snapshot_image_buffer_` valid until next `GetRenderResults`/`CommitConfig` | `server.hpp` (`RenderResult` doc) | Pointer-based result; caller must consume before next snapshot |
| StatsConsumer snapshot fields zeroed on `Reset()` | `stats.cpp:23–30` | Both accumulation and snapshot counters cleared |

---

## §7 Consumer ↔ Filter Interaction

Design A (`doc/filter-architecture.md §2`): the filter gate runs on the
simulator side. Consumers receive pre-filtered data — `SimData::outgoing_d_` /
`outgoing_w_` contain only filter-pass rays. Consumers do not hold or consult
`FilterSpec`.

### §7.1 Filter Change Response Chain

When a filter spec changes:

1. `CommitConfig` is called with new config.
2. `Stop()` drains all threads.
3. `NeedsRebuild` returns `false` (filter is not in `RenderConfig`).
4. `ResetWith` resets accumulators (zeros `internal_xyz_`).
5. `Start()` resumes simulation with new scene config (new filter).
6. Filter-fail rays terminate in the simulator (`w_ = -1.0f` TIR sentinel) and
   never reach the consumer — `outgoing_*` only ever carries filter-pass
   emission (`doc/filter-architecture.md §7`). The accumulator therefore always
   describes the currently visible (filtered) image; there is no separate
   filter-independent lane to fill.

### §7.2 EV Reference: The Visible Framebuffer, Not a Filter-Independent Lane

**Note (scrum-237, 2026-05-28)**: an earlier design accumulated filter-fail
emission into a separate "anchor lane" (`anchor_internal_xyz_` /
`anchor_snapshot_xyz_`) to provide a filter-independent EV reference. That
lane was removed — filter-fail rays now terminate immediately in
`CollectData` and never reach the consumer at all (`doc/filter-architecture.md
§7`).

Under the current design (Design A) there is no separate filter-independent
statistic — the EV always anchors on the visible (filtered) framebuffer:
`snapshot_xyz_`, its P99 anchor, and `snapshot_intensity` all describe exactly
what filter-pass emission produced. Switching filters changes the EV anchor
because the visible image changed; this is intentional.

See `doc/ev-pipeline-architecture.md §5` for the full before/after rationale
and `§2` for the EV data flow.

---

## §8 ServerPoller (GUI Consumption Side)

`ServerPoller` (`server_poller.hpp:43`) runs a persistent background thread that
polls the server via the C API and stages results for the main (GUI) thread.

### §8.1 Polling Contract

1. **Background thread** calls `LUMICE_GetRawXyzResults()` (which internally
   calls `ServerImpl::GetRawXyzResults()` — the two-phase snapshot protocol).
2. **Generation tracking**: `last_generation_` detects genuinely new snapshots.
   If `snapshot_generation` hasn't changed, the poll is a no-op for texture data.
3. **Quality gate**: skips texture overwrite when `sim_ray_num < min_rays`
   (calibrated threshold or floor), preventing visible flicker from sparse
   snapshots. Timeout fallback forces upload after `kQualityGateTimeoutMs`.
4. **Main thread** calls `TrySyncData()` (try-lock, never blocks) to swap
   staged data.

### §8.2 Lifecycle

```
Constructor → worker thread spawned (starts in kPaused)

Start(server):
  Stop() → clear valid flag → reset generation/quality gate → kRunning → notify

Stop():
  kPaused → wait for worker to confirm paused (active_ == false)

EnsureRunning(server):
  if kRunning: no-op
  if kPaused: resume polling (reset generation/quality gate → kRunning)

~ServerPoller():
  kTerminating → join worker thread
```

### §8.3 Data Flow: Poller → Main Thread

```
  Poller thread                              Main thread
  ────────────                               ───────────
  PollOnce():
    LUMICE_GetRawXyzResults() ──────┐
    LUMICE_GetCachedStats()         │
    quality gate check              │
    lock(data_mutex_)               │
    staged_ = new data              │
    unlock                          │
                                    │     SyncFromPoller():
                                    │       TrySyncData(out)  ← try_lock(data_mutex_)
                                    │       swap(out, staged_)
                                    └─────► upload xyz_data to GPU texture
```

The `try_lock` in `TrySyncData` ensures the main thread never blocks on the
poller — if the poller is mid-write, the main thread simply skips this frame.
