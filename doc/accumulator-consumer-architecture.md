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
- C API result lifetime and ownership
  — see [`doc/capi-lifecycle-architecture.md`](capi-lifecycle-architecture.md) §9.
- Per-raypath color classes and the compositor's own algorithm
  — see [`doc/gui-custom-spectrum-and-raypath-color.md`](gui-custom-spectrum-and-raypath-color.md) §4.

**Reading the code anchors.** Every reference below names a **symbol first** —
a class, method, field or constant — with a line number in parentheses only where
it locates something inside a long body (a specific assignment, a `static_assert`).
The symbol is the authoritative half: `grep -n <Symbol> <file>` resolves it even
after the line has moved, and **if the two disagree, trust the symbol**. Line
numbers are auxiliary because they drift silently — 9 of this document's 14
pre-2026-08-11 bare-line anchors had come to point at a `}`, a blank line or an
`#include`, which is why they are no longer written bare. **Every section below was
re-verified against the code on 2026-08-11**; anything that could not be verified
is marked as such in place rather than phrased as an assertion.

---

## §1 Overview

The accumulator/consumer subsystem sits between the simulator thread pool and the
result-consumption paths (GUI `ServerPoller`, CLI `LUMICE_AcquireResultFrame`). Its
responsibilities:

1. **Accumulate** incoming ray data (`SimData`) into per-pixel XYZ buffers
   (`RenderConsumer::Consume` / `RenderConsumer::ConsumeDeviceFused`).
2. **Snapshot** accumulated state for consumption without blocking the simulation
   (`ServerImpl::DoSnapshot`).
3. **Transform** XYZ data to sRGB for CLI image export (`RenderConsumer::PostSnapshot`).
4. **Expose** raw XYZ data for GPU-side rendering in the GUI path
   (`RenderConsumer::GetRawXyzResult`).
5. **Composite** per-color-class lanes into one sRGB image when raypath coloring is
   configured (`CompositeColorClassesLinear`, driven from `DoSnapshot` Phase 2).

Every one of those products leaves the subsystem inside a single immutable
`ResultFrame` — see §4.2.

Key source files:

| File | Role |
|------|------|
| `src/server/consumer.hpp` | `IConsume` interface |
| `src/server/server.hpp` | `ResultFrame` and the view structs it publishes (`RenderResult`, `RawXyzResult`, `CompositeResult`, `StatsResult`) |
| `src/server/render.hpp` / `render.cpp` | `RenderConsumer` — projection, accumulation, snapshot; `FrameBufferPool` |
| `src/server/stats.hpp` / `stats.cpp` | `StatsConsumer` — ray/crystal/orientation counters |
| `src/server/server.cpp` | `ServerImpl` — consumer orchestration, locking, snapshot protocol |
| `src/config/render_config.hpp` / `.cpp` | `RenderConfig`, `NeedsRebuild()` |
| `src/gui/server_poller.hpp` / `.cpp` | GUI consumption side |

---

## §2 IConsume Interface

`IConsume` (`consumer.hpp`) defines the consumer contract:

| Method | Called under | Purpose |
|--------|-------------|---------|
| `Consume(SimData)` | `consumer_mutex_` | Accumulate one batch of ray data |
| `PrepareSnapshot()` | `consumer_mutex_` | Freeze accumulated state into a **freshly borrowed** snapshot buffer |
| `PostSnapshot()` | `snapshot_mutex_` | XYZ→sRGB into a freshly borrowed image buffer (CLI path) |
| `GetResult()` | `snapshot_mutex_` | Return typed result (`RenderResult` / `StatsResult`) |
| `Reset()` | `consumer_mutex_` | Zero accumulators, preserve buffer allocations |

All five are called from a snapshot pass that additionally holds
`do_snapshot_mutex_`, except `Consume` (driven by `ServerImpl::ConsumeData`) and
`Reset` (driven by `ServerImpl::CommitConfig`). See §4.1.

One `RenderConsumer` method is deliberately **not** on the interface and is called
under **no lock at all**: `CountEffectivePixels()`, in Phase 1.5 (§4.2).

Two concrete implementations:

- **`RenderConsumer`**: projects rays through the lens model, accumulates into
  `internal_xyz_` (with a Neumaier compensation term in `comp_xyz_`), freezes into
  `snapshot_xyz_` in `PrepareSnapshot()`, converts to sRGB in `PostSnapshot()`.
- **`StatsConsumer`**: counts `total_rays_` and `sim_rays_`, plus a crystal count
  and an orientation count that are each split across **two** accumulators —
  a stochastic half that sums across batches and a deterministic half that is
  overwritten by every batch (`StatsConsumer::Consume` explains why they cannot be
  one counter). The reported figure is the sum of each pair, formed in
  `PrepareSnapshot()`.

Both are held via `shared_ptr<IConsume>` (`ConsumerPtrS`) in
`ServerImpl::consumers_`.

---

## §3 Consumer State Machine

### §3.1 Lifecycle Paths

The consumer lifecycle has four entry points:

```
CommitConfig (full rebuild — first commit, renderer set changed,
              per-renderer NeedsRebuild, or color-class table reshaped):
  Stop() → consumers_.clear()
         → new RenderConsumer(render_config, active_class_table_) per renderer
         → new StatsConsumer
         → Start()

CommitConfig (reuse path — every reuse condition in §5.4 held):
  Stop() → ResetWith(new_config) for RenderConsumers, Reset() for StatsConsumer → Start()

Stop():
  snapshot_dirty_ = false
  has_ever_consumed_ = false          ← prevents stale snapshot reads
  (consumers_ not cleared — CommitConfig decides rebuild vs. reuse)
  (published_frame_ NOT touched — see below)

~ServerImpl():
  Stop() → state_ = kTerminating → join threads → consumers_ destroyed with ServerImpl
```

**`Stop()` does not retract the published frame.** Nothing in `ServerImpl::Stop`
publishes, and only a completed snapshot pass replaces `published_frame_`, so
between a restart and the new run's first produced batch a reader still sees the
*previous* run's pixels and stats. `has_ever_consumed_` — cleared right there — is
what distinguishes them, which is why `RawXyzResult::has_valid_data_` is load-bearing
on the GUI side rather than belt-and-braces (see `ServerPoller::PollOnce`, and §8.1).

A destroyed consumer does **not** invalidate a frame it produced: the frame owns the
buffers, not the consumer (§4.3).

### §3.2 State Transition Diagram

Consumer state is implicit, encoded in flags rather than an explicit enum. (The
`ServerImpl::ServerState` enum is the *thread* state machine — `kStopped` /
`kRunning` / `kTerminating` — not this one.)

```
                    CommitConfig
                   (full rebuild)        ConsumeData (under consumer_mutex_)
  ┌──────────┐  ──────────────────►  ┌──────┐  ──────────────────────────►  ┌──────────────┐
  │ Created  │                       │ Idle │                               │ Accumulating │
  └──────────┘                       └──┬───┘  ◄────────────────────────── └──────────────┘
                                        │         Stop() + CommitConfig
                                        │         (rebuild or reuse)
                                        │
                              AcquireResultFrame →
                                   DoSnapshot
                                (only when snapshot_dirty_)
                                        │
                                        ▼
                              ┌──────────────────────┐
                              │  Published           │
                              │  (a NEW immutable    │
                              │   ResultFrame in     │
                              │   published_frame_)  │
                              └──────────────────────┘
```

The terminal box is a *publication*, not a buffer state: each pass builds a fresh
`ResultFrame` and stores it into `published_frame_`. Earlier frames are not
overwritten — they live until their last holder drops them (§4.3).

State flags:

| Flag | Set by | Cleared by | Meaning |
|------|--------|------------|---------|
| `has_ever_consumed_` | `ServerImpl::ConsumeData` — both the normal batch path (`server.cpp:1348`) and the 0-exit / fully-filtered batch path (`server.cpp:1374`) | `ServerImpl::Stop` (`server.cpp:976`) | True after first batch consumed; becomes `ResultFrame::has_valid_data_` and each `RawXyzResult::has_valid_data_` |
| `snapshot_dirty_` | `ServerImpl::ConsumeData` (`server.cpp:1347`, `server.cpp:1373`); **also** the two display-time entry points `ServerImpl::SetRaypathColors` (`server.cpp:1597`) and `ServerImpl::SetCompositeExposure` (`server.cpp:1614`) | `ServerImpl::DoSnapshot` (`server.cpp:747`) and `ServerImpl::Stop` (`server.cpp:974`) | "Anything changed since the last snapshot." The two display-time writers are what make "change a class colour / composite EV and see new pixels immediately" work with no new ray data |
| `snapshot_generation_` | `ServerImpl::DoSnapshot` (`server.cpp:753`, inside Phase 1) — the **sole** writer | Never reset (monotonic; `Stop()` leaves it alone) | Poller detects new snapshots via generation comparison. Tying the bump to the dirty-consume event rather than to any one accessor is deliberate — see the design note above `DoSnapshot` |

---

## §4 Thread Safety Model

### §4.1 Three-Lock Design

| Lock | Type | Guards | Writer thread | Reader thread |
|------|------|--------|---------------|---------------|
| `do_snapshot_mutex_` | `std::mutex` | A whole snapshot pass (Phase 1 → Phase 2 → publish) | — | Every caller of `ServerImpl::DoSnapshot` |
| `consumer_mutex_` | `TicketMutex` (FIFO) | `consumers_` list, all consumer mutable state (`internal_xyz_`, `total_intensity_`, the color-class lanes), plus `snapshot_dirty_` / `has_ever_consumed_` / `snapshot_generation_` / `active_class_table_` / `active_composite_mode_` / `display_ev_total_` | `ConsumeData` thread (high frequency) | `AcquireResultFrame` (low frequency); `CommitConfig` caller (rebuild/reset); `GetLiveSimRayCount`; `GetColorClassSignals` |
| `snapshot_mutex_` | `std::mutex` | Phase 2's assembly of the new `ResultFrame` (`PostSnapshot` + `GetResult` + storage anchors + composites) | `DoSnapshot` Phase 2 | — |

**Lock ordering (invariant)**: `do_snapshot_mutex_` is the **outermost** of the
three. It is always taken *before* `consumer_mutex_` / `snapshot_mutex_` and never
while either is held. `ServerImpl::DoSnapshot` is its only acquisition site, and
there the two inner scopes do not nest: Phase 1's `consumer_mutex_` scope closes
before Phase 2's `snapshot_mutex_` scope opens.

**Why a third lock exists**: Phase 1 and Phase 2 take *different* locks, so two
`DoSnapshot` calls could interleave — one thread's `PrepareSnapshot()` running
while another was still reading the same consumer's snapshot lanes in Phase 2.
That was already a torn-frame source; once Phase 1 began *re-pointing* the
consumer's buffer members (see `FrameBufferPool`, §4.3) it would be a
pointer-level race. The declaration comment on `do_snapshot_mutex_` is the
authority here.

This serialization is also what makes `linear_rgb_scratch_` — the compositor's
reused Phase 2 scratch buffer — safe to share across consumers: only one snapshot
pass can be writing it at a time, and `CompositeColorClassesLinear` `assign()`s the
full size on entry so no iteration reads what a previous one left behind.

**Why TicketMutex**: on Windows, `std::mutex` uses SRWLOCK which provides no
fairness guarantee. A high-frequency locker (`ConsumeData`, called per simulator
batch) can starve a low-frequency waiter (`ServerPoller`, polling every
`kPollIntervalMs` = 20ms). `TicketMutex` (`server.cpp`, defined just above
`ServerImpl`) assigns monotonic tickets and serves in FIFO order, guaranteeing
bounded wait.

### §4.2 Two-Phase Snapshot Protocol

`ServerImpl::DoSnapshot()` is the single owner of the protocol and of the
`snapshot_dirty_` flag; `ServerImpl::AcquireResultFrame()` is its only caller.
It returns early (having done nothing) when `snapshot_dirty_` is clear.

```
                     ── do_snapshot_mutex_ held for all of the below ──

Phase 1  (consumer_mutex_ held):
  if (!snapshot_dirty_) return false        ← the dirty check is inside the lock
  PrepareSnapshot() for each consumer       ← freeze accumulators into a fresh buffer
  snapshot_consumers = consumers_           ← shared_ptr copy keeps objects alive
  snap_class_table / snap_composite_mode / snap_display_ev_total = the live values
  valid_data = has_ever_consumed_
  snapshot_dirty_ = false
  generation = ++snapshot_generation_

Phase 1.5 (NO lock held):
  CountEffectivePixels()                    ← O(W×H) scan of snapshot_xyz_

Phase 2  (snapshot_mutex_ held):
  frame = new ResultFrame                   ← assembled, never a cache overwrite
  PostSnapshot() for each consumer          ← XYZ→sRGB (CLI path)
  GetResult()  → frame->render_results_ / frame->stats_result_
  SnapshotImageStorage() → frame->render_storage_   ┐ lifetime anchors, parallel
  GetRawXyzResult() → frame->xyz_results_           │ to the view structs
  SnapshotXyzStorage()  → frame->xyz_storage_       ┘
  CompositeColorClassesLinear() for each consumer with ColoredMask() != 0
                                            → frame->composite_results_

Publish (no lock):
  StorePublished(frame)                     ← atomic_store into published_frame_
```

**Phase 2 publishes; it does not cache.** The former
`cached_render_results_` / `cached_stats_result_` / `cached_composite_results_`
trio is gone: each `DoSnapshot` used to `std::move` over them, freeing pixels a
reader was still pointing at — a confirmed heap-use-after-free. A frame is
immutable and reference-counted instead, so publishing a new one cannot disturb an
old reader. See [`doc/capi-lifecycle-architecture.md`](capi-lifecycle-architecture.md)
§9 for the ownership invariant and both recurrences of the defect it closes.

`published_frame_` is **never null**: the `ServerImpl` constructor publishes an
all-zero frame, which is what makes the public header's "all-zero struct if no
snapshot has been taken yet" promise structural rather than a per-call null branch.

`AcquireResultFrame()` re-stamps two lifecycle fields — `has_valid_data_` and
`epoch_` — from the *live* server state onto a shallow copy of the frame, because
they answer "what is true of this server now", not "what was true when these pixels
were made" (`ServerImpl::Stop` clears `has_ever_consumed_` and `CommitConfig` bumps
`committed_epoch_` without producing a new snapshot). A shallow copy is cheap by
construction: every pixel payload sits behind a `shared_ptr`.

**Critical invariant**: the buffer Phase 1.5 scans is not disturbed before Phase 2
reads it. The reason changed with `FrameBufferPool` and is worth stating precisely,
because the old reason ("nobody writes `snapshot_xyz_` outside `PrepareSnapshot`")
no longer describes the mechanism:

1. `Consume` only writes to `internal_xyz_` / `comp_xyz_` (never `snapshot_xyz_`).
2. `PrepareSnapshot` does not *write into* `snapshot_xyz_` — it **re-points** it at
   a buffer freshly borrowed from `xyz_pool_`. The previous buffer stays alive under
   whichever frame co-owns it.
3. `do_snapshot_mutex_` guarantees no second Phase 1 can run while this pass is
   between its phases, so `snapshot_xyz_` cannot be re-pointed under Phase 1.5's feet.
4. The snapshot caller holds `snapshot_consumers` via `shared_ptr`, preventing
   object destruction.

### §4.3 Consumer and Buffer Lifetime Safety

`consumers_` is a `vector<shared_ptr<IConsume>>`. During `DoSnapshot`, the snapshot
code copies the `shared_ptr` vector under `consumer_mutex_`, then operates on the
copies outside the lock. Even if `CommitConfig` calls `consumers_.clear()`
concurrently (under `consumer_mutex_`), the snapshot's `shared_ptr` copies keep the
consumer objects alive until the snapshot operation completes.

Keeping the *consumer* alive is no longer sufficient on its own, because what a
reader points at is a **buffer**, not a consumer. Two mechanisms close that:

- **`ResultFrame` anchors the storage.** `render_storage_` and `xyz_storage_` are
  `shared_ptr` arrays held parallel to the view structs, so
  `render_results_[i].img_buffer_` is exactly `render_storage_[i].get()` and
  `xyz_results_[i].xyz_buffer_` is exactly `xyz_storage_[i].get()`. A view pointer
  is valid for precisely as long as its frame is held.
- **`FrameBufferPool` outlives its consumer.** The deleter of every buffer the pool
  hands out keeps a `shared_ptr` to the pool itself, so a frame may legitimately
  outlive the `RenderConsumer` that produced it — which is exactly what a
  `CommitConfig` rebuild does while the poller still holds a frame. A deleter
  reaching into a destroyed pool would be the same class of lifetime defect the
  frame model exists to remove.

The pool exists to keep this from costing one `W×H×3` allocation per snapshot per
consumer: a released buffer is handed straight back out, since the number of
simultaneously live frames is small (published + in-flight + whatever a reader
holds).

---

## §5 NeedsRebuild / ResetWith

### §5.1 NeedsRebuild Semantics

`NeedsRebuild(const RenderConfig&, const RenderConfig&)` (`render_config.cpp`)
compares layout-affecting fields between old and new `RenderConfig`:

| Triggers rebuild (`true`) | Does NOT trigger rebuild (`false`) |
|---------------------------|------------------------------------|
| `resolution_` | `background_` |
| `lens_` (type + fov) | `ray_color_` |
| `lens_shift_` | `opacity_` |
| `view_` | `intensity_factor_` |
| `visible_` | `central_grid_` / `elevation_grid_` / `celestial_outline_` |
| `overlap_` | |

Those six are the whole of the comparison — the function's `return` expression has
no other term. `id_` appears in neither column and is excluded on purpose: map-key
matching (§5.4 condition 4) already guarantees id agreement on the reuse path.

**Filter changes do not trigger NeedsRebuild.** Filter specs belong to
`SceneConfig`, not `RenderConfig`. When a filter changes, consumers walk the
reuse path: `Stop()` → `ResetWith()` → `Start()`. Accumulators are zeroed but
buffer allocations are preserved.

**A color-class table reshape does force a full rebuild**, through a *different*
predicate: `CommitConfig` calls the `ColorClassTable` overload of `NeedsRebuild`
(`config/color_class_table.hpp`) and treats a true result as scene-level. It is
structural rather than a plain `referenced_mask_` compare, so a reshape at the same
mask (e.g. "one 2-bit `any` class" ↔ "two 1-bit classes") still rebuilds — a reused
`RenderConsumer` would otherwise keep a stale per-class lane layout. This check is
orthogonal to the per-renderer one above.

### §5.2 sizeof Sentinel

```cpp
static_assert(sizeof(RenderConfig) == 136,
              "Update NeedsRebuild when RenderConfig fields change");
```

This sentinel (`render_config.cpp:168`, first statement of `NeedsRebuild`) forces a
compile error when any field is added to or removed from `RenderConfig`. The
developer must then classify the new field as layout-affecting (add to
`NeedsRebuild` comparisons) or appearance-only (no change needed). This prevents
silent drift where a new layout field is added but `NeedsRebuild` is not updated.

### §5.3 ResetWith Path

`RenderConsumer::ResetWith(new_config)` (`render.cpp`):
```cpp
void RenderConsumer::ResetWith(const RenderConfig& new_config) {
  config_ = new_config;
  Reset();
}
```

Because `NeedsRebuild` returned `false`, all layout fields are identical.
Assigning the full config is safe — `rot_` (derived from `config_.view_` in the
constructor, and `view_` is a layout field) and all buffer sizes remain valid.
`Reset()` zeros the accumulation state without reallocating buffers.

### §5.4 CommitConfig Reuse Logic

`ServerImpl::CommitConfig` determines reuse eligibility. **All five** conditions
must hold:

1. `consumers_` must be non-empty (first commit always rebuilds).
2. The color-class table must not have been reshaped (§5.1).
3. Renderer count must match (same number of render configs).
4. Renderer keys must match (same map ordering).
5. `NeedsRebuild()` must return `false` for every renderer pair.

If all conditions pass, the reuse path calls `ResetWith()` on each
`RenderConsumer` and `Reset()` on `StatsConsumer`, avoiding the cost of
destroying and reconstructing consumers.

> **Not re-verified (2026-08-11)**: an earlier revision of this section quantified
> that saving as "~0.5ms vs. ~30ms for full rebuild". Those are historical
> measurements with no corresponding constant, assertion or comment in the code, and
> this revision did not re-measure them. Treat them as an order-of-magnitude claim of
> unknown currency, not as a current figure. `CommitConfig` does log the real split
> (`Stop` + rebuild + `Start`, in ms) on every commit at INFO level, which is the
> cheapest way to get a number that is actually true of your machine and config.

---

## §6 Data Integrity Invariants

| Invariant | Location | Notes |
|-----------|----------|-------|
| `internal_xyz_` size = `W × H × 3` floats | `RenderConsumer::RenderConsumer` | Allocated once (`unique_ptr`), never resized — the consumer's resolution is fixed for its lifetime, since `ResetWith` is gated on `NeedsRebuild` |
| `comp_xyz_` size = `W × H × 3` floats | `RenderConsumer::RenderConsumer` | Neumaier compensation term for the device-fused path. **The true accumulated sum is `internal_xyz_ + comp_xyz_`**, folded together only in `PrepareSnapshot` — `internal_xyz_` alone is not the full accumulation state. All-zero on the legacy projection path |
| `snapshot_xyz_` = a `shared_ptr<float[]>` of `W × H × 3`, **re-borrowed per snapshot** | `RenderConsumer::PrepareSnapshot` (`xyz_pool_->Acquire`) | **Not** a fixed allocation: each snapshot takes a fresh buffer from `xyz_pool_`; the previous one lives as long as the frame co-owning it, then returns to the pool. Constructor borrows the first one so the getters never see null |
| `snapshot_image_buffer_` = a `shared_ptr<uint8_t[]>` of `W × H × 3` bytes, **re-borrowed per snapshot** | `RenderConsumer::PostSnapshot` (`image_pool_->Acquire`) | Same treatment as above; borrowed *before* the `total_pix <= 0 / snapshot_intensity_ <= 0` early-out so both exits hand the frame a buffer this snapshot owns |
| A `FrameBufferPool` serves exactly one element count | `FrameBufferPool::Acquire` | A request for a different count drops the free list rather than growing a size-indexed structure. Free list is capped at `kMaxFreeBuffers` (8); beyond it a returned buffer is simply freed |
| `d_buf_`/`w_buf_`/`xy_buf_`/`overlap_w_buf_`/`wl_buf_`/`overlap_wl_buf_` capacity ≥ max(rays, outgoing) | `RenderConsumer::Consume` (`render.cpp:220-228`, the `needed > buf_capacity_` block) | Grow-only; never shrink. `comp_buf_`/`overlap_comp_buf_` grow in the same block but **only** when the class table is non-empty (zero-config = zero extra allocation) |
| Per-class lanes: `lane_y_` and `snapshot_lane_y_`, each `class_count` arrays of `W × H` floats | `RenderConsumer::RenderConsumer`; shadowed in `PrepareSnapshot` | Allocated only when the class table is non-empty. `Reset()` zeros `lane_y_` but deliberately **not** `snapshot_lane_y_` — `PrepareSnapshot` memcpys over it, mirroring `snapshot_xyz_` |
| The buffer behind a published view is valid for as long as its frame is held | `ResultFrame` doc comment (`server.hpp`) | Not "until the next call": a later snapshot publishes a *new* frame and cannot disturb an outstanding one. §4.3 |
| StatsConsumer accumulation **and** snapshot counters zeroed on `Reset()` | `StatsConsumer::Reset` | All ten fields — both halves of each split counter and all four snapshot fields |

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

**Note (2026-05-28)**: an earlier design accumulated filter-fail
emission into a separate "anchor lane" (`anchor_internal_xyz_` /
`anchor_snapshot_xyz_`) to provide a filter-independent EV reference. That
lane was removed — filter-fail rays now terminate immediately in
`CollectData` and never reach the consumer at all (`doc/filter-architecture.md
§7`). Neither field exists anywhere under `src/` today.

Under the current design (Design A) there is no separate filter-independent
statistic — the EV always anchors on the visible (filtered) framebuffer:
`snapshot_xyz_`, its P99 anchor, and `snapshot_intensity` all describe exactly
what filter-pass emission produced. Switching filters changes the EV anchor
because the visible image changed; this is intentional.

See `doc/ev-pipeline-architecture.md §5` for the full before/after rationale
and `§2` for the EV data flow.

---

## §8 ServerPoller (GUI Consumption Side)

`ServerPoller` (`server_poller.hpp`) runs a persistent background thread that
polls the server via the C API and publishes results for the main (GUI) thread.

### §8.1 Polling Contract

Each `ServerPoller::PollOnce()` reads **two decoupled clocks** and publishes one
coherent immutable bundle:

1. **Lifecycle heartbeat**, read unconditionally on every poll:
   `LUMICE_GetSimLifecycle()` gives the lifecycle level and the committed epoch.
   It is deliberately decoupled from snapshot materialization so the terminal
   completion edge is never lost on a poll that carries no new pixels.
2. **One result frame per poll**: `LUMICE_AcquireResultFrame()`, then
   `LUMICE_FrameGetRawXyz` / `LUMICE_FrameGetComposite` / `LUMICE_FrameGetStats`
   off that same frame. Everything read out of one frame belongs to the same
   snapshot generation *by construction* — which is what retired the earlier
   three-call sequence and its cross-generation drift guard.
3. **Generation tracking**: `last_generation_` detects genuinely new snapshots.
   If `snapshot_generation` hasn't changed, the poll materializes no texture.
4. **Quality gate**: skips texture materialization when `sim_ray_num < min_rays`
   (`calibrated_min_rays_` if calibration ran, else `gui::kMinRaysFloor`),
   preventing visible flicker from sparse snapshots. It has **three** exits, and
   they are not interchangeable:
   - *Timeout fallback* — force through after `gui::kQualityGateTimeoutMs`, for a
     running sim that will never reach the threshold.
   - *Terminal-frame rescue* — force through once per resume when the run has
     COMPLETED and nothing has been put on screen yet, for a run that already ended
     below the threshold. (Without it, a finite low-ray run self-pauses in the same
     call and the timeout is never reached — a permanently blank preview.)
   - *Content-freshness gate*, which pushes the other way: when
     `has_valid_data` is false and no previously materialized payload recorded the
     current epoch, the frame is **suppressed** even though the gate would otherwise
     pass, because the pixel buffer still holds the previous run's image (§3.1) while
     the epoch field already reads the bumped one. Applied after both bypasses — a
     rescue is not a licence to publish another generation's pixels.
5. **Publication**: the candidate payload is built outside `publish_mutex_`; only
   the pointer-level read-modify-write runs inside it, ending in an `atomic_store`.
6. **Main thread** calls `ServerPoller::LoadSnapshot()` — a single `atomic_load`,
   lock-free and non-destructive.

Materializing a texture payload copies **no pixels**: the payload takes a share of
the result frame and points straight at the frame's own buffers (§4.3 is what makes
that safe).

### §8.2 Lifecycle

The worker state machine has **four** states, not three:

| State | Cadence | Entered by |
|---|---|---|
| `kPaused` | untimed wait, zero polling | `Stop()` |
| `kRunning` | poll every `gui::kPollIntervalMs` | `Start()`, `WakeForRestart()`, `WakeForRefresh()` |
| `kIdleHeartbeat` | poll every `gui::kIdleHeartbeatIntervalMs` | `PollOnce()`'s own self-pause, from `kRunning` only |
| `kTerminating` | — | destructor |

```
Constructor → worker thread spawned (starts in kPaused)

Start(server):
  Stop() → PublishValidReset() → ResetPerResumeState() → kRunning → notify

Stop():
  kRunning OR kIdleHeartbeat → kPaused → wait for the worker to confirm active_ == false

WakeForRestart(server):     ┐ both go through TransitionToRunning(server, publish_reset)
WakeForRefresh(server):     ┘ if kRunning/kTerminating: no-op; if kPaused: resume

PollOnce() tail:
  ShouldSelfPause(lifecycle, drain) → kRunning becomes kIdleHeartbeat

~ServerPoller():
  kTerminating → join worker thread
```

Three things about this are load-bearing and easy to get wrong:

- **`kIdleHeartbeat` is not a rest state `Stop()` may early-return on.** The worker
  still wakes on the heartbeat timer and dereferences `server_`, and every caller of
  `Stop()` destroys or stops the server immediately afterwards. `Stop()` therefore
  drives `kIdleHeartbeat → kPaused` and waits on `active_`, which covers **any**
  in-flight `PollOnce()` regardless of which loop issued it.
- **Self-pause goes to the heartbeat, not to a full stop.** The run being over is a
  single observation; the slow heartbeat is what makes it a belief that can
  self-correct rather than a one-shot verdict.
- **`WakeForRestart` and `WakeForRefresh` differ by exactly one bool** — whether a
  `valid=false` snapshot is published across the wake edge. Display-time edits
  (colour / visible / solo / z-order / mode / composite EV) must use
  `WakeForRefresh`: publishing `valid=false` there would let the main thread
  transiently observe an invalid snapshot and pull a completed sim back into
  "simulating" (`doc/gui-state-governance.md` §4).

The self-pause predicate `ShouldSelfPause(lc, drain)` is a free function with no
member access, precisely so it is exercisable as a truth table. All three of its
terms are needed: COMPLETED is a *producer-side* verdict and can be true while the
consumer still has queued batches (pausing there freezes a partial ray count on the
status bar), and a newer epoch may have been committed between the two reads that
bracket the poll body.

### §8.3 Data Flow: Poller → Main Thread

```
  Poller thread                                    Main thread
  ────────────                                     ───────────
  PollOnce():
    LUMICE_GetSimLifecycle()  ────────┐   (clock ④ — read every poll)
    LUMICE_AcquireResultFrame()       │
      ├─ LUMICE_FrameGetRawXyz        │   (all three off ONE frame ⇒
      ├─ LUMICE_FrameGetComposite     │    same generation by construction)
      └─ LUMICE_FrameGetStats         │
    generation / quality / freshness  │
    build TexturePayload (no copy —   │
      takes a share of the frame)     │
    lock(publish_mutex_)              │
      prev = LoadPublished()          │
      next = fresh | carried-forward  │
      StorePublished(next)            │   ← atomic_store
    unlock                            │
    ShouldSelfPause? → kIdleHeartbeat │
                                      │   SyncFromPoller():
                                      │     LoadSnapshot()  ← atomic_load, lock-free
                                      └───► dedup on texture_serial, then upload
                                            xyz_buffer (or rgb_buffer) to GL texture
```

The handoff is a **versioned immutable publication**, not a shared mutable buffer:
the main thread takes no lock and therefore never blocks on the poller, and — unlike
the try-lock channel this replaced — never skips a frame either. It always observes
some complete publication, never a half-updated field combination.

When a poll produces no new texture (sparse data, gate rejection, or no new
generation), the previous payload pointer **and its serial** are carried forward
into the new bundle. That is the anti-flicker path: the consumer dedups on
`texture_serial`, so a carried-forward payload is simply not re-uploaded, and the
last good frame stays on screen. Stats carry forward the same way but keep their own
epoch stamp rather than being re-stamped with the bundle epoch — that asymmetry is
what lets the consumer tell "no new stats this poll" apart from "no new stats
because the run just restarted, and these belong to the old one".
