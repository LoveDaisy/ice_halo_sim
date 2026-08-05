# C API Lifecycle & Design Constraints

This document describes the internal invariants and design constraints of
Lumice's C API layer — the boundary between GUI / CLI / external consumers and
the core simulation engine.

**Target audience**: contributors who modify server lifecycle management,
result retrieval paths, sentinel handling, or the commit path.

**Scope**: internal contracts, state machine, thread safety model, and
sentinel-overflow protection. This document does **not** cover:

- How to *use* the API (function signatures, parameters, examples)
  — see [`doc/c_api.md`](c_api.md).
- Accumulator/consumer internals (snapshot protocol, `NeedsRebuild` / `ResetWith`)
  — see [`doc/accumulator-consumer-architecture.md`](accumulator-consumer-architecture.md).
- EV pipeline or filter architecture
  — see [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) and
  [`doc/filter-architecture.md`](filter-architecture.md).

---

## §1 Overview

The C API (`src/include/lumice.h`, implemented in `src/server/c_api.cpp`)
wraps the internal C++ `Server` class and exposes an opaque-handle interface
suitable for FFI consumption. It enforces:

1. A **lifecycle state machine** governing legal calling sequences.
2. **Per-function contracts** (preconditions, postconditions, error returns).
3. A **thread safety model** (per-function annotations).
4. A **sentinel pattern** with overflow protection for result arrays.
5. A **single configuration path** through the `LUMICE_Scene` opaque handle.
6. **SimData side-effect rules** triggered by a commit.
7. A **result lifetime & ownership invariant** — a reader must hold a real share of a result's
   lifetime, enforced by the `LUMICE_ResultFrame` handle (§9).

Key source files:

| File | Role |
|------|------|
| `src/include/lumice.h` | Public C API header — the stable ABI surface |
| `src/server/c_api.cpp` | C API implementation; wraps `Server` methods |
| `src/server/server.hpp` / `server.cpp` | `ServerImpl` — state machine, threading, consumer management |
| `src/server/consumer.hpp` | `IConsume` interface |
| `src/server/render.hpp` / `render.cpp` | `RenderConsumer` — accumulation, snapshot, XYZ→RGB |
| `src/server/stats.hpp` / `stats.cpp` | `StatsConsumer` — ray/crystal counters |

---

## §2 API Calling State Machine

### §2.1 State Transitions

The server's externally observable state is a projection of the internal
`ServerState` enum (`server.cpp`): `kStopped` / `kRunning` / `kTerminating`.

```
LUMICE_CreateServer / LUMICE_CreateServerEx
    |
    v
[Idle — kStopped, no config committed, threads in cv.wait]
    |
    |  LUMICE_CommitScene ──> internally: Stop() → rebuild consumers → Start()
    v
[Running — kRunning, simulation active]
    |                                              ^
    |  simulation completes (finite rays)          |
    |  or LUMICE_StopServer                        |
    v                                              |
[Idle — kStopped, has_ever_consumed_=false]        |
    |                                              |
    +---- LUMICE_CommitScene (re-launch) ----------+
    |
    |  LUMICE_DestroyServer ──> Terminate()
    v
[Destroyed — impl_ reset, handle invalid]
```

Key observations:

- There is no explicit `Run()` API. The server transitions to Running
  *inside* the commit, which calls `Stop() → rebuild → Start()`.
- `LUMICE_SERVER_NOT_READY` is defined in the `LUMICE_ServerState` enum but
  is never returned by `LUMICE_QueryServerState`. The implementation maps
  to either `LUMICE_SERVER_IDLE` or `LUMICE_SERVER_RUNNING`. This enum
  value is dead code retained for potential future use.
- `LUMICE_DestroyServer(NULL)` is safe (early return on null check).

### §2.2 State Table

| Current State | API Call | Next State | Notes |
|---------------|----------|------------|-------|
| Idle | `CommitScene` | Running | Stop (no-op if already stopped) → rebuild → Start |
| Idle | `StopServer` | Idle | No-op (already stopped) |
| Idle | `DestroyServer` | Destroyed | `Terminate()` → `~ServerImpl` joins all threads |
| Running | `CommitScene` | Running | Stop (drains workers) → rebuild → Start |
| Running | `StopServer` | Idle | Drains queues, waits for `active_workers_==0` |
| Running | `DestroyServer` | Destroyed | `Terminate()` calls Stop first |
| Running | `AcquireResultFrame` | Running | Triggers snapshot if `snapshot_dirty_`; always returns a valid frame |
| Running | `QueryServerState` | Running | Read-only status check |
| Destroyed | Any | UB | Handle is invalid after `DestroyServer` |

---

## §3 Per-Function Contracts

### §3.1 Server Lifecycle APIs

#### `LUMICE_CreateServer()` / `LUMICE_CreateServerEx(config)`

- **Precondition**: none (but see thread safety — §4).
- **Postcondition**: returns a valid `LUMICE_Server*` with all persistent
  threads spawned and waiting in `cv.wait` (state = `kStopped`).
- **Error**: returns `NULL` only on allocation failure.
- **Side effects**: registers the global logger sink (`spdlog`). The
  `LUMICE_ServerConfig` struct allows setting `num_workers` and `sim_seed`;
  `NULL` config or zero fields use defaults.
  (`c_api.cpp:75–93`)

#### `LUMICE_DestroyServer(server)`

- **Precondition**: `server` is a valid handle or `NULL`.
- **Postcondition**: all threads joined; handle memory freed. Passing
  `NULL` is a no-op.
- **Side effects**: calls `Terminate()` which calls `Stop()` then destroys
  `ServerImpl`.
  (`c_api.cpp:96–102`)

### §3.2 Configuration APIs

#### `LUMICE_CommitScene(server, scene, out_reused)`

- **Precondition**: `server != NULL`, `scene != NULL`.
- **Postcondition**: on success, server is Running with the new config active.
  On failure, the running server is left untouched; only the status is set to
  `kError`.
- **Error returns**: `LUMICE_ERR_NULL_ARG`, `LUMICE_ERR_INVALID_JSON`,
  `LUMICE_ERR_INVALID_CONFIG`, `LUMICE_ERR_MISSING_FIELD`,
  `LUMICE_ERR_INVALID_VALUE`, `LUMICE_ERR_SERVER`. If `out_reused` is non-null
  it is set to `1` when consumers were reused, `0` when rebuilt — and left
  untouched on any error.
- **Internal sequence**: `CommitJsonToServer(server, scene->root, …)` →
  `Server::CommitConfig(json, &reused)` → `Stop()` → consumer rebuild/reuse →
  update `active_scene_` → `Start()`.
- **Ownership**: the scene is read as `const`, the server keeps no reference to
  it, and the caller still owns it. The same handle may be committed repeatedly,
  and to more than one server.
- **No bounds-check prologue.** The removed struct entry point needed one
  because a caller could hand-fill a wide C struct with arbitrary counts and
  pointers. A `LUMICE_Scene` cannot reach that state: every `Add*`/`Set*`
  validated its own input at call time, so re-validating here would be dead
  code. What a commit can still surface is cross-field / semantic rejection from
  the core (e.g. `max_hits` out of range).

#### `LUMICE_SceneFromJson(json_str, out_scene)` / `LUMICE_SceneFromJsonFile(filename, out_scene)`

- **Precondition**: pointers non-null.
- **Postcondition**: on success `*out_scene` owns a fresh handle the caller must
  `LUMICE_SceneDestroy`. On **any** failure `*out_scene` is `NULL` and no handle
  leaks, so a caller never destroys a handle that was not produced.
- **Side effects**: none on any server — this is pure parsing. It is the reason
  authoring and committing are separate entry points at all: a config can be
  validated or edited without disturbing a running simulation.
- **Implementation note (known technical debt).** These parse into an internal
  `ConfigScratch` (`src/server/c_api_internal.hpp` — the former public
  `LUMICE_Config`, demoted in v4.12) via `JsonToConfig`, then re-encode that
  struct through `ConfigToJson` into the new handle's root. That double hop —
  text → `ConfigScratch` → JSON root — is deliberate, not an oversight:
  `JsonToConfig`/`ConfigToJson` are the single validated JSON↔config
  implementation in the codebase, carrying every field mapping, bounds check and
  strictness rule, and the handle path inherits its correctness directly from
  them. Growing a second, drifting JSON reader to save one re-encode on a
  non-hot path was judged the worse trade. If it is ever revisited, the
  replacement must be validated against the same differential corpus
  (`test/unit-correctness/server/test_json_parser_parity.cpp`), not against the
  encoder it replaces.

### §3.3 Result Retrieval APIs — the `LUMICE_ResultFrame` handle

Since v4.15 (`2836f399`, see §9), every result read goes through an opaque, immutable,
reference-counted **frame** rather than a server-taking getter. This section covers the
mechanics; §9 covers *why* the model exists — that history is load-bearing context for anyone
tempted to reach back for a per-server cache.

#### `LUMICE_AcquireResultFrame(server, out_frame)`

- **Precondition**: `server != NULL`, `out_frame != NULL`.
- **Postcondition**: materializes a pending snapshot (calls `DoSnapshot()`, same trigger the
  removed getters shared), then writes a handle to `*out_frame`. The caller owns that handle
  and MUST eventually pass it to `LUMICE_ReleaseResultFrame`. `*out_frame` is **never** `NULL`
  on success, even before the first snapshot — such a frame simply reads as "no results" from
  every `FrameGet*` call, making the `lumice.h` "all-zero struct if no snapshot" promise
  structural rather than a per-call null branch (`published_frame_` is published once, non-null,
  in the constructor — see `server.hpp:210–222`).
- **Internal**: `ServerImpl::AcquireResultFrame()` (`server.cpp:843–863`) calls `DoSnapshot()`
  (`server.cpp:703`, itself serialized by `do_snapshot_mutex_`), then loads the currently
  published frame (`LoadPublished()`, `std::atomic_load`) and re-stamps `has_valid_data_` /
  `epoch_` from the live `has_ever_consumed_` / `committed_epoch_` onto a shallow copy if either
  has moved since the frame was built — a cheap shallow copy, because every pixel payload behind
  it sits behind its own `shared_ptr` (see the `ResultFrame` struct doc, `server.hpp:189–208`).
  Two frames acquired back to back, or from two different threads, are fully independent: neither
  affects the other's contents or lifetime.

#### `LUMICE_ReleaseResultFrame(frame)`

- **Precondition**: `frame` is a valid handle or `NULL` (NULL-safe no-op, same contract as
  `LUMICE_DestroyServer`).
- **Postcondition**: the frame handle is destroyed. Each handle must be released **exactly
  once** — releasing the same handle twice is undefined behavior, and there is deliberately no
  double-free sentinel (see §9's AC6 v3 discussion for why that omission is the correct
  contract, not a gap). Forgetting to release leaks only that one frame — it cannot corrupt
  memory or affect any other reader, and the leak is exactly what ASan/LSan/valgrind already
  report.
- **Internal**: `delete frame` (`c_api.cpp:2594`); the `LUMICE_ResultFrame_` wrapper's
  `shared_ptr<const ns::ResultFrame>` member drops one reference. The underlying `ResultFrame`
  and its pixel storage are freed only when the last holder — which may be a different reader's
  frame, or the server's own `published_frame_` if it has since re-published — drops its
  reference.

#### `LUMICE_FrameGetRender(frame, out, max_count)` / `LUMICE_FrameGetComposite(frame, out, max_count)`

- **Precondition**: `frame != NULL`, `out != NULL`.
- **Postcondition**: `out[0..count-1]` filled with sRGB uint8 results — mono/full-spectrum for
  `FrameGetRender`, one per colored renderer for `FrameGetComposite`. Sentinel at `out[count]`
  if `count < max_count` (§5). `img_buffer` stays valid for as long as `frame` is held.
- **Internal**: reads `frame->frame_->render_results_` / `composite_results_`, both already
  materialized when the frame was built (`c_api.cpp:2661–2686` / `2632–2659`). No lock is taken
  here — the frame is immutable once published, so reading it needs no synchronization.

#### `LUMICE_FrameGetRawXyz(frame, out, max_count)`

- **Precondition**: same as above.
- **Postcondition**: `out[0..count-1]` filled with raw XYZ float data plus metadata
  (`snapshot_intensity`, `intensity_factor`, `has_valid_data`, `snapshot_generation`,
  `effective_pixels`, `epoch`). `xyz_buffer` stays valid for as long as `frame` is held.
- **Internal**: reads `frame->frame_->xyz_results_` (`c_api.cpp:2599–2628`), lock-free for the
  same reason as above.

#### `LUMICE_FrameGetStats(frame, out)`

- **Precondition**: `frame != NULL`, `out != NULL`.
- **Postcondition**: `out` filled with the stats this frame's snapshot carries; writes an
  all-zero struct if the frame carries none (e.g. acquired before the first snapshot). Unlike
  the other three, this is a **plain value copy** — no pointer into the frame survives past the
  call, so nothing dangles after `LUMICE_ReleaseResultFrame`. There is no separate
  "cached, may be stale" stats read anymore: a frame's stats ARE the stats of the snapshot it is.
- **Internal**: `frame->frame_->stats_result_` is an `std::optional<StatsResult>`
  (`c_api.cpp:2690–2707`).

### §3.4 State & Control APIs

#### `LUMICE_QueryServerState(server, out)`

- **Precondition**: `server != NULL`, `out != NULL`.
- **Postcondition**: `*out` set to `LUMICE_SERVER_IDLE` or
  `LUMICE_SERVER_RUNNING`.
- **Internal**: maps `GetSimLifecycle()` (`kRunning` → `LUMICE_SERVER_RUNNING`, else
  `LUMICE_SERVER_IDLE`), which itself calls `GetStatus()` — reads `status_` under
  `status_mutex_`; if Running, also polls simulator idle state and `scene_gen_active_` flag.
  (`c_api.cpp:2726–2738`, `server.cpp:978–1004` `GetStatus`, `server.cpp:1036–1049` `GetSimLifecycle`)

#### `LUMICE_StopServer(server)`

- **Precondition**: `server` is valid or `NULL` (null is safe).
- **Postcondition**: server is Idle. Worker threads are drained (not
  terminated — they return to `cv.wait`).
  (`c_api.cpp:2783–2789`, `server.cpp:922–976` `ServerImpl::Stop`)

### §3.5 Stateless Utility APIs

These functions have no shared state and are always thread-safe:

| Function | Precondition | Notes |
|----------|-------------|-------|
| `LUMICE_GetCrystalMesh(crystal, sample_seed, out)` | `crystal != NULL`, `out != NULL` | Samples one shape from `crystal`'s distributions via the core sampler; `server` param removed |
| `LUMICE_ValidateRaypathText(text, kind, out_state, out_msg, size)` | all pointers non-null | Pure validation |
| `LUMICE_IsLegalFace(kind, face)` | none | Pure function |
| `LUMICE_MaxFov(type)` | none | Pure function |
| `LUMICE_XyzToSrgbUint8(xyz_in, out, count, scale)` | `xyz_in != NULL`, `out != NULL` | Batch conversion |

### §3.6 Zero-output completion contract (all-black simulation)

**Contract**: a simulation that runs to completion but produces *legitimately
zero* renderable output (every ray filtered or absorbed) MUST still report
`has_valid_data = 1` once it reaches `LUMICE_SERVER_IDLE`. An all-black image is
a valid answer, not an "incomplete" state. A buffered poller's completion
predicate is `has_valid_data && state == IDLE` (see `test/e2e/capi_runner.py`),
so if `has_valid_data` never flips on an all-black run, the poller waits for
valid data forever and times out (observed: 600 s hang).

**Mechanism**. `has_valid_data` maps from the server flag `has_ever_consumed_`
(`server.cpp` `ServerImpl::AcquireResultFrame`: `valid_data = has_ever_consumed_`). That flag
is set when a batch is consumed in `ConsumeData`. Each produced batch carries
`root_ray_count_ > 0`, which distinguishes it from the queue-shutdown sentinel
(`rays_` empty AND `root_ray_count_ == 0`); the sentinel breaks the loop and is
never treated as data.

**The trap (and the fix)**. `ConsumeData` gates consumption on
`has_renderable = !outgoing_d_.empty() || !rays_.Empty()`. On a *zero-exit
batch* both are empty, so the batch is correctly **not** accumulated (a black
batch must not bias the image). The original code left it at that — which on the
**exit-seam path (Metal + CUDA)** meant `has_ever_consumed_` was never set, and
an all-black simulation hung the poller. The **legacy CPU path never hit this**
because its `rays_` is always non-empty (it carries every ray segment, not just
exits), so `has_renderable` stayed true regardless of filtering. The exit-seam
path was the first consumer to surface the gap; the impossible-raypath-filter
parity test (`ms_filter_leak_impossible.json`, all rays filter-fail) is the
reproducer.

The fix (`server.cpp` `ConsumeData`, the zero-exit `else` branch): on a
completed zero-exit batch, still **set `has_ever_consumed_ = true`** (the run
produced valid data — zero intensity) and **set `snapshot_dirty_ = true`** (so
`PrepareSnapshot` emits a clean zero frame; without it `did_snapshot` stays
false and no snapshot is ever prepared). It still does **not** call
`c->Consume()` — there is nothing to accumulate.

> **Lesson**: "valid data" is a *completion* predicate, not a *non-emptiness*
> predicate. Do not infer completion from the presence of renderable output —
> they are different questions, and a backend that emits only exits (exit-seam)
> rather than full ray buffers (legacy) makes the difference observable.

---

## §4 Thread Safety Model

| Function | Thread-safe | Protection | Notes |
|----------|-------------|------------|-------|
| `LUMICE_CreateServer*` | No | — | Global logger sink registration is not atomic |
| `LUMICE_DestroyServer` | No | — | Destructor joins threads; concurrent calls UB |
| `LUMICE_CommitScene` | No | — | Modifies `config_manager_`, calls Stop/Start |
| `LUMICE_SceneCreate` / `Clone` / `Destroy` / `Add*` / `Set*` | Per-handle | — | A handle has no internal locking, so one handle has one owner thread. Distinct handles are fully independent — building two scenes on two threads is safe. |
| `LUMICE_SceneToJson` / `SceneFromJson` / `SceneFromJsonFile` | Per-handle | — | Never touch a server. `FromJson*` only writes the fresh handle it allocates. |
| `LUMICE_StopServer` | No | — | Direct state/queue mutation |
| `LUMICE_SetLogCallback` | No | static bool (non-atomic) | First-call sink registration has a race window |
| `LUMICE_SetLogLevel` | Conditional | spdlog internal lock | Thread-safe for concurrent calls; ordering not guaranteed |
| `LUMICE_QueryServerState` | Yes | `status_mutex_` | Read-only poll; safe from any thread |
| `LUMICE_AcquireResultFrame` | Yes | `do_snapshot_mutex_` (serializes the two-phase snapshot pass) + `consumer_mutex_` (reads `has_ever_consumed_`) + atomic `published_frame_` load | Materializes a pending snapshot if any, then hands the caller an independent, reference-counted share of the published frame — see §9 |
| `LUMICE_ReleaseResultFrame` | Yes | `shared_ptr` control-block refcount (atomic) | NULL-safe no-op; drops one reference. The underlying pixels are freed only when the LAST holder — of any frame sharing that publication — releases |
| `LUMICE_FrameGetRender` / `FrameGetComposite` / `FrameGetRawXyz` / `FrameGetStats` | Yes | none needed | The frame is immutable once published, so reading it — even the SAME frame from multiple threads — needs no lock |
| `LUMICE_SetRaypathColors` | Yes* | `consumer_mutex_` (TicketMutex) | Display-time only: updates color/visible/solo/z-order/mode on the active class table, sets `snapshot_dirty_`. Never touches Stop/Start/`scene_generation_`/`committed_epoch_`/`consumers_`. *Safe vs `AcquireResultFrame`/`FrameGet*`, but NOT vs a concurrent `LUMICE_CommitScene` (same single-owner rule; the commit writes `active_class_table_` partly outside `consumer_mutex_`, a pre-existing race). |
| `LUMICE_GetCrystalMesh` | Yes | — | No shared state (no `server` param; uses a local RNG per call) |
| `LUMICE_ValidateRaypathText` | Yes | — | Pure function |
| `LUMICE_IsLegalFace` | Yes | — | Pure function |
| `LUMICE_MaxFov` | Yes | — | Pure function |
| `LUMICE_XyzToSrgbUint8` | Yes | — | Pure function |


**Mutex types**:

- **`TicketMutex`** (`server.cpp:41–63`): FIFO spinlock using
  `atomic<uint32_t>` ticket/serving counters. Prevents starvation that
  occurs with Windows SRWLOCK under high-frequency locking
  (see [`doc/accumulator-consumer-architecture.md` §4.1](accumulator-consumer-architecture.md)).
- **`std::mutex`**: used for `snapshot_mutex_`, `do_snapshot_mutex_`, `status_mutex_`,
  `start_mutex_`, `prod_mutex_`, `scene_mutex_`.

**Practical rule**: a single "owner thread" should perform all non-thread-safe
operations (`Create`, `CommitScene`, `Stop`, `Destroy`). Any number of other
threads may safely call `QueryServerState` and acquire/read/release result
frames concurrently — with that owner and with each other, no external mutex
needed on the read side.

---

## §5 Sentinel Pattern & Overflow Protection

### §5.1 Convention

Result retrieval APIs use a sentinel-terminated array pattern. The sentinel
is a zero-initialized struct written at `out[count]` **only** when
`count < max_count` (i.e., there is a spare slot).

| Function | Sentinel field | Sentinel value |
|----------|---------------|----------------|
| `LUMICE_FrameGetRender` | `img_buffer` | `NULL` |
| `LUMICE_FrameGetComposite` | `img_buffer` | `NULL` |
| `LUMICE_FrameGetRawXyz` | `xyz_buffer` | `NULL` |

`LUMICE_FrameGetStats` is NOT part of this table: it has no `max_count` and returns exactly one
value (an all-zero `LUMICE_StatsResult` when the frame carries no stats), so there is no array
to terminate.

When the array is full (`count == max_count`), **no sentinel is written**.
This is the key safety property — writing a sentinel when the array is full
would be an out-of-bounds write.

### §5.2 Bug History & Fix (5287efe)

**Bug**: prior to commit `5287efe`, the sentinel guard used `<=` instead
of `<`:

```cpp
// BEFORE (buggy)
if (count <= max_count) {          // wrote sentinel even when count == max_count
    memset(&out[count], 0, ...);   // out[max_count] is OOB when count == max_count
}
```

When `max_count=1` and `results.size()=1`, `count=1 == max_count`, and
the write to `out[1]` was out-of-bounds.

**Fix** (originally applied to the removed server-taking getters; the same `count < max_count`
guard carried forward into their v4.15 replacements at `c_api.cpp:2625`, `c_api.cpp:2654`,
`c_api.cpp:2683`):

```cpp
// AFTER (fixed)
if (count < max_count) {
    std::memset(&out[count], 0, sizeof(...));
}
```

**Regression test**: `test/regression-sentinel/test_capi_sentinel_overflow.py` exercises
3 configs × 12 rounds = 36 server lifecycles using `max_count=1` to guard
against reintroduction.

### §5.3 Caller Contract

There are two valid calling conventions:

**Sentinel iteration** (requires N+1 array):

```c
LUMICE_RawXyzResult arr[N + 1]{};   // value-initialize to all zeros
LUMICE_FrameGetRawXyz(frame, arr, N);
for (int i = 0; arr[i].xyz_buffer != NULL; i++) {
    // use arr[i] — valid until LUMICE_ReleaseResultFrame(frame)
}
```

The caller must value-initialize the array (`{}` or `memset`) so that
natural zero values act as sentinels even when the API fills all N slots
(in which case no API-written sentinel exists at `arr[N]`).

**Direct index access** (exact-size array):

```c
LUMICE_RawXyzResult arr[N];
LUMICE_FrameGetRawXyz(frame, arr, N);
for (int i = 0; i < N; i++) {
    // use arr[i] directly — check individual fields for validity
}
```

No sentinel is needed when the caller uses direct indexing.

---

## §6 The Single Commit Path

Up to v4.11 there were two commit surfaces — a JSON-string one
(`LUMICE_CommitConfig` / `FromFile`) and a wide-C-struct one
(`LUMICE_CommitConfigStruct`) — that met at the same internal
`Server::CommitConfig(json, out_reused)`. v4.12 removed both. Every
configuration, however it was authored, now reaches the core through
`LUMICE_CommitScene`.

### §6.1 Two authoring routes, one commit

```
  JSON text / file ──> LUMICE_SceneFromJson{,File} ──┐
                        (JsonToConfig -> ConfigToJson)│
                                                      ├─> scene->root ──> LUMICE_CommitScene
  SceneCreate + Add*/Set* ────────────────────────────┘        │             (CommitJsonToServer)
                        (per-section encode helpers)            │                   │
                                                                v                   v
                                                        LUMICE_SceneToJson   Server::CommitConfig(json, &reused)
```

The two authoring routes converge *before* the commit, on one JSON document
(`scene->root`), because they share the same per-section encode helpers. That
shared encoder is the mechanism — not a coincidence — behind the property
`scene->root == ConfigToJson(equivalent config)`, asserted byte-for-byte in
`test/unit-correctness/server/test_c_api_scene.cpp`. Because the routes converge
on a document rather than on a call, reuse judgement and re-simulation
triggering cannot diverge between them: `CommitJsonToServer` sees no evidence of
which route produced its input.

### §6.2 Why `CommitJsonToServer` stays a separate function

`LUMICE_CommitScene` is its only caller today. It remains factored out because
the core commit call, the error log, the error-code mapping and the
`out_reused` write-back are one unit: a future second producer of an
already-encoded document must reuse it verbatim rather than re-derive it. This
is the same reasoning that kept the two v4.11 entry points behaviorally
identical.

### §6.3 What the struct path bought, and where it went

The struct path existed to skip JSON string serialization in the GUI's 50 ms
commit cycle, and it was the only entry point exposing `out_reused`. Both
properties survive it: the Scene handle is built through typed `Add*`/`Set*`
calls with no string round-trip, and `out_reused` is now available to every
caller.

Per-raypath color classes ride the same single path. There is **no** separate
struct→core translation for color: the encoder emits `raypath_color` in the
exact wire shape core's `RaypathColorConfig::from_json` reads. A scene with zero
color classes omits the key entirely, keeping the mono JSON shape byte-for-byte
unchanged.

### §6.4 Display-time color setter (`LUMICE_SetRaypathColors`)

Changing **member structure** (a class's `match[]` refs / `combine`) is a
re-simulation event and must go through `LUMICE_CommitScene` (dirty → rebuild lanes →
re-accumulate). Changing only **appearance** (per-class RGB, `visible`, `solo`,
z-order, composite mode) does NOT need re-simulation: `LUMICE_SetRaypathColors`
mutates the active class table in place under `consumer_mutex_`, sets
`snapshot_dirty_`, and the next `LUMICE_AcquireResultFrame` / `LUMICE_FrameGetComposite`
re-composites the SAME accumulated per-class Y-lanes. It never advances the epoch or
clears the accumulator (AC2).
`class_count` must equal the committed `raypath_color_count` (mismatch =
`LUMICE_ERR_INVALID_CONFIG` — the caller changed structure and must re-commit);
`z_order`, when non-NULL, must be a permutation of `[0, class_count)` (AC3). The
z-order is decoupled from the Y-lane physical index: the compositor sorts by
z-order but always indexes lanes by the original class position, so reordering
draw priority never re-binds a lane to another class's color (see
`doc/gui-custom-spectrum-and-raypath-color.md` §4.0 as-built note).

---

## §7 SimData Side Effects of a Commit

### §7.1 Internal Reset Sequence

When a commit succeeds, the following sequence occurs
(`server.cpp:518–702`):

1. **Parse**: temporary `ConfigManager` parses JSON. On failure, the running
   server is untouched (only `status_` set to `kError`).
2. **`Stop()`** (`server.cpp:922–976`):
   - Sets `state_` to `kStopped`.
   - Shuts down `scene_queue_` and `data_queue_` (unblocks workers).
   - Calls `Simulator::Stop()` on each worker.
   - Waits for `active_workers_ == 0`.
   - Under `consumer_mutex_`: `snapshot_dirty_ = false`,
     `has_ever_consumed_ = false`.
   - Under `status_mutex_`: `status_ = kIdle`.
3. **Consumer decision** (under `consumer_mutex_`):
   - **Reuse path** (renderer key set unchanged, `NeedsRebuild`=false):
     calls `rc->ResetWith(new_config)` — resets accumulators, preserves
     buffer allocations.
   - **Rebuild path**: `consumers_.clear()` → construct new
     `RenderConsumer` + `StatsConsumer`.
4. **Update scene**: `active_scene_ = new SceneConfig`,
   `scene_generation_++`.
5. **`Start()`**: queues started, `state_` set to `kRunning`, threads
   woken via `start_cv_`.

### §7.2 Buffer Lifetime Rules

| Field | State after a commit |
|-------|------------------------|
| `has_ever_consumed_` | `false` — new simulation data required |
| `snapshot_dirty_` | `false` |
| `snapshot_generation_` | **Unchanged** — only `PrepareSnapshot` increments it; poller tracks its own `last_generation_` |
| A frame ALREADY acquired before the commit | **Still fully valid.** Its `img_buffer` / `xyz_buffer` pointers keep working exactly as before — see the rule below |
| `has_valid_data` (C API field) | `0` on the NEXT acquired frame — maps from `has_ever_consumed_=false` |
| A frame's `StatsResult` | Frozen to whatever snapshot that frame was built from — never rewritten in place, so it cannot go stale under a held frame |

**Rule (v4.15, see §9 for why this replaced the old one)**: `img_buffer` / `xyz_buffer` validity
is governed by **frame lifetime, not commit lifecycle**. A `LUMICE_ResultFrame` is an immutable,
independently reference-counted publication — `LUMICE_CommitScene` never reaches into a frame a
caller is holding, on either the reuse or the rebuild consumer path, because a commit produces
new consumer state but never mutates a previously published frame's pixel storage in place. A
pointer obtained from `LUMICE_FrameGet*` stays valid until `LUMICE_ReleaseResultFrame` is called
on **that specific frame**, full stop — independent of how many commits, or how many newer
frames, happen in the meantime.

This is a stricter, simpler guarantee than the pre-v4.15 rule it replaces ("valid until the next
getter call, and invalid across a rebuild"), not a relaxation of it: everything that used to be
guaranteed valid still is, and pointers that used to silently go stale (or dangle, on the
composite path) now stay valid until the reader is done with them. A caller that wants FRESH
data after a commit still needs to acquire a new frame — `LUMICE_CommitScene` does not implicitly
refresh frames a caller is already holding — but that is a request for new data, not a lifetime
hazard on old data. The GUI poller acquires a fresh frame on every poll cycle regardless.

---

## §9 Result Lifetime & Ownership

### §9.1 The Invariant

**A reader must hold a real share of a result's lifetime for as long as it reads that
result.** Not a pointer with no lifetime attached, not a promise that the server will be
"careful" — an actual share, enforced by a type the reader holds (`LUMICE_ResultFrame*`) and
gives back explicitly (`LUMICE_ReleaseResultFrame`). Everything else in this section is either
evidence for why the invariant is necessary or a record of a direction that looked like it
would satisfy the invariant and did not.

Before v4.15, the API's result-retrieval surface was six server-taking getters
(`LUMICE_GetRenderResults`, `LUMICE_GetCompositeResults`, `LUMICE_GetRawXyzResults`,
`LUMICE_GetRawXyzAndCompositeResults`, `LUMICE_GetStatsResults`, `LUMICE_GetCachedStats`). Each
handed back a pointer into memory the *server* owned, valid — per the old contract — "until the
next getter call on the same server". That contract gives the reader nothing: it describes when
the memory happens to still be there, not a lifetime the reader controls. Two recurrences under
that contract, roughly four and a half months apart, are why this document treats it as settled
rather than case-by-case judgment — and the second recurrence happened INSIDE the structure the
first recurrence's fix introduced (§9.2).

### §9.2 Two Recurrences, Same Root Cause — the Second One Grew Inside the First One's Fix

| | `34400c17` (2026-03-18) | `2836f399` (2026-08-04, this scrum) |
|---|---|---|
| **What broke** | `DoSnapshot()` read/wrote consumer state outside `consumer_mutex_` while a concurrent `CommitConfig` restart path could clear `consumers_` from another thread | `DoSnapshot()`'s Phase 2 `std::move`'d a freshly built `composite_results_` over `cached_composite_results_` — the SAME storage location a reader's earlier `img_buffer` still pointed into |
| **Storage shape** | Consumer-owned buffers, read directly with no lifetime handoff to the caller | `cached_render_results_` / `cached_stats_result_` (introduced BY this fix) plus, later, the sibling `cached_composite_results_` (added by the per-raypath-color feature) — all overwritten in place on every snapshot |
| **Fix applied** | Stop reading consumer-owned memory directly: cache results under `snapshot_mutex_`, populated by `DoSnapshot()` under `consumer_mutex_`, read lock-free thereafter ("cache + swap") | Ownership model: stop overwriting a cache in place; `DoSnapshot()` PUBLISHES a new immutable, reference-counted frame instead |
| **Confirmed by** | Code audit of the `consumers_` race | ASan — ground truth, not inference (see the `2836f399` commit message: "被 std::move 覆写的那次正是已由 ASan 坐实的 use-after-free") |

**Why moving the storage does not fix this class of bug — the sharpest evidence is that `34400c17`
already tried it.** `34400c17`'s fix WAS a storage relocation: move results out of consumer-owned
buffers into a dedicated `cached_*_results_` cache, populated under a lock. That genuinely closed
the race it targeted. But the cache it introduced kept the one property that makes this class of
bug possible — a reader holds a raw pointer into memory the *server* still considers itself free
to overwrite, with no lifetime share of its own. Four and a half months later, the per-raypath-
color feature added `cached_composite_results_` as a same-shaped sibling to that cache — same
"populate under a lock, overwrite in place, `std::move` a fresh result over the old one on every
snapshot" pattern — and reproduced the identical defect, this time confirmed as a live UAF by
ASan rather than reasoned about from a race description. The recurrence is structural, not a
missed lock: relocating the cache one more time (a different member, a different mutex, a
different consumer) would only relocate the *next* recurrence, exactly as it did the first time.
The fix had to change *who owns the memory*, not *where the memory lives*. That is what
`LUMICE_ResultFrame` does: `DoSnapshot()` now PUBLISHES a new immutable frame
(`server.cpp:756–830`) rather than overwriting a cache, and the previous frame survives for
exactly as long as its last holder keeps it (`server.hpp:189–208`).

### §9.3 Rejected Directions

Recorded so a future contributor who reaches for one of these first sees why it was already
tried and reverted, rather than re-deriving the same dead end:

1. **Delete the `std::move`, rewrite the cache in place instead.** Looks like it removes the
   free-then-use-after — it does not. A reader's pointer would now observe *torn* data (a
   partial overwrite mid-read) instead of freed data. Torn data on the composite path is not
   hypothetical here: it is the exact failure mode the mono (`snapshot_image_buffer_`) and raw-xyz
   (`snapshot_xyz_`) paths already have, and rewrite-in-place is how they have it. This direction
   trades a crash (loud, ASan-visible) for silent pixel corruption (quiet, easy to ship). Rejected.
2. **"A stable buffer means the worst case is a stale read."** Disproven by measurement, not
   argument: the composite path's `cached_composite_results_` — the exact buffer this scrum's
   `2836f399` replaces — was confirmed by ASan to be a genuine **use-after-free**, not a stale
   read. The premise (stable allocation ⇒ bounded staleness) does not hold once the allocation
   itself gets freed and replaced by `std::move`, which is exactly what a snapshot does. Before
   this scrum's fix, the two consumer-owned buffers (mono `snapshot_image_buffer_`, `xyz`
   `snapshot_xyz_`) genuinely WERE allocate-once-and-rewrite, so for THOSE two the failure mode
   really was bounded to torn/stale reads rather than UAF — but that boundedness was a fact about
   those two specific structures at the time, not a general property of "stable buffer", and it
   stopped applying the moment a caller held a pointer across the frame boundary the old contract
   never gave them, which is exactly the shape the composite path took. The landed fix does not
   rely on that distinction surviving: `snapshot_image_buffer_` / `snapshot_xyz_` are now ALSO
   `FrameBufferPool`-backed and re-pointed to a fresh buffer on every snapshot rather than
   rewritten in place (`render.hpp:116–123`), so mono and xyz get the exact same reference-counted
   protection as composite — the fix generalized to all three result kinds, not just the one that
   had already been caught by ASan.
3. **"Copy-out on every read is a structural non-improvement."** Circular: the poller's existing
   defensive copy (the thing this scrum's `poller-drop-defensive-copies` task removed) existed
   *because* of the exact lifetime defect this invariant now closes at the source. Citing that
   copy's existence as evidence the defect doesn't need fixing assumes the conclusion. Rejected.

### §9.3a Honest boundary — what the ASan evidence does and does not establish

The use-after-free above was reproduced **deterministically**, but by a *constructed* sequence:
a single thread deliberately holding a result pointer across two further getter calls, then
reading it. That is what makes it a reliable regression gate
(`test/unit-correctness/server/test_result_frame_lifetime.cpp`) — it fires every run, not
one in N.

What was **not** established is how often the equivalent interleaving arises on its own. Six
attempts to reproduce it from the natural two-thread production shape (the GUI poller worker
reading a frame while the main thread's save path drives a new snapshot) all came back clean —
but every one of those runs was **low-powered**: 85–475 reader windows per run, against a
measured snapshot-publish rate of roughly 7–9.5 per second, which puts the expected hit count
near zero even if the race is real. Those six clean runs are therefore **not** evidence that the
race does not occur in practice; they are evidence that ASan's overhead makes this particular
race expensive to sample.

Two things follow, and both are load-bearing:

- **Do not cite the clean runs as a safety argument.** "We could not reproduce it naturally"
  and "it does not happen" are different claims, and only the first one is supported.
- **The fix does not depend on the number.** A use-after-free that reproduces on demand is a
  correctness gate, not a cost/benefit tradeoff to be weighed against an incidence estimate.
  The frequency question was deliberately left open rather than answered badly.

If a future change ever needs the actual rate, the cheap route is *not* ASan — its slowdown is
what starved the sample count. Compare the published-snapshot generation counter immediately
before and after a reader's window: a generation that advanced inside the window is exactly one
free that occurred while a pointer was held, and it can be counted at full speed.

### §9.4 AC6's Three Versions — the Most Expensive Lesson of This Scrum

The acceptance criterion governing HOW the invariant would be enforced went through three
designs before landing. The first two each survived a full plan-review round before failing;
the record is kept because the *reason* each failed generalizes past this one AC.

| Version | Approach | Why it was wrong |
|---|---|---|
| v1 | A runtime "live frame count" counter | Replaces a contract with runtime detection, and the counter itself needs a lifetime story — it reintroduced the exact class of defect (a `shared_ptr<ServerImpl*>` deleter capturing a raw pointer) that this scrum exists to remove |
| v2 | C++ RAII wrapper + a `check_policies.py` gate requiring its use | **There is no RAII in C.** This promoted a C++ convenience for THIS project's own C++ consumers into the acceptance bar for the **C API surface**, and the gate it proposed cannot see, let alone enforce, anything about an actual external C caller — zero guarantee on the API surface it was meant to protect |
| **v3 (landed)** | Copy the API's own existing house rule (`lumice.h` `LUMICE_SceneDestroy`): NULL-safe no-op, release exactly once, double-release is UB with **no sentinel** | acquire/release pairing IS the idiomatic C contract already used elsewhere in this API — not a defect to engineer away with C++ machinery |

**Two mechanism lessons, not just a design footnote:**
- A plan-review evaluates a plan **against the issue's stated acceptance criteria**. When an AC
  itself is wrong, every reviewer inherits that premise as a given rather than a candidate
  defect — this is why 6 of 8 independent review comments across four rounds all traced back to
  AC6's v1/v2 shape, and why none of those four rounds could fix it: nobody was positioned to
  question the requirement itself, only its implementation.
- The correct framing for "how much enforcement does a leak deserve" turned out to be: **the
  failure mode is a bounded leak, not memory corruption, and leaks are already detectable by
  tooling that exists independent of this project** (ASan / LSan / valgrind). Reaching for
  project-specific runtime machinery to detect what the platform already detects was solving an
  already-solved problem at the wrong layer.

### §9.5 Honest Boundary

The natural occurrence rate of the `cached_composite_results_` UAF in production was **never
measured**. The scrum's own natural-interleaving repro attempts (6 runs) were all statistically
low-powered (window estimates of 85–475, insufficient to bound a rare race) and came back
negative — that is a **null result under low power**, not evidence the race doesn't happen in
practice. Per this project's correctness-over-probability rule, a confirmed UAF is a
front-of-the-line defect regardless of how rarely it fires; "it probably doesn't happen much" is
not a rebuttal and this document does not make that claim.

### §9.6 Follow-on Items (Backlogged, Non-Blocking)

Surfaced during code review of the frame-ownership implementation; tracked for future work
rather than blocking this scrum:

1. **Frame pointer identity is not stable.** `LUMICE_AcquireResultFrame()` called while the
   server is alive but has not produced a new snapshot since the last call returns a NEW frame
   object wrapping the SAME pixel generation — i.e. two acquired frames can be
   value-equal (same generation, same pixels) while being different objects. This needs to be
   stated explicitly in `ResultFrame`'s type documentation so nobody substitutes an address
   comparison for the actual generation check (`snapshot_generation_`).
2. **Production-side RAII wrapping is not yet uniform.** Three call sites hand-roll their own
   acquire/release wrapper independently (`main.cpp` uses a named local class; `app.cpp` and
   `server_poller.cpp` each inline an anonymous one). The test-side helper
   `test/support/scoped_result_frame.hpp` has already converged on one shape; production has not
   yet been asked to adopt it.

---

## §10 Cross-Reference Index

| Source location | Document section |
|----------------|-----------------|
| `src/include/lumice.h:971–1032` (result frame API block comment + declarations) | §3.3 Result Frame APIs, §5 Sentinel Pattern |
| `src/server/c_api.cpp:2625` (`FrameGetRawXyz` sentinel) | §5.2 Overflow fix |
| `src/server/c_api.cpp:2654` (`FrameGetComposite` sentinel) | §5.2 Overflow fix |
| `src/server/c_api.cpp:2683` (`FrameGetRender` sentinel) | §5.2 Overflow fix |
| `src/server/server.hpp:189–222` (`ResultFrame` struct doc) | §9 Result Lifetime & Ownership |
| `src/server/server.cpp:843–863` (`ServerImpl::AcquireResultFrame`) | §3.3, §9 |
| `src/server/c_api.cpp` (`CommitJsonToServer` — the shared commit tail) | §6.2 Why it stays factored out |
| `src/server/c_api.cpp` (`JsonToScene` — the JSON→handle double hop) | §3.2 Known technical debt |
| `src/server/server.cpp:518` (`ServerImpl::CommitConfig` entry) | §7 SimData side effects |
| `src/server/server.cpp:964` (`has_ever_consumed_ = false` in `Stop()`) | §7.1 Reset sequence |
| `src/server/server.cpp` `ConsumeData` zero-exit `else` branch (`has_ever_consumed_`/`snapshot_dirty_` on all-black batch) | §3.6 Zero-output completion |
| `src/server/server.cpp:41–63` (`TicketMutex`) | §4 Thread safety (FIFO mutex) |
| `test/regression-sentinel/test_capi_sentinel_overflow.py` | §5.2 Regression test |
| `test/parity-cross-backend/backend/test_cuda_filter_parity.py::test_cuda_impossible_filter_produces_zero_intensity` | §3.6 Zero-output reproducer |
