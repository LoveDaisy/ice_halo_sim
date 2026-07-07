# C API Lifecycle & Design Constraints

This document describes the internal invariants and design constraints of
Lumice's C API layer — the boundary between GUI / CLI / external consumers and
the core simulation engine.

**Target audience**: contributors who modify server lifecycle management,
result retrieval paths, sentinel handling, or the `CommitConfig` dual-path
logic.

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
5. **Dual configuration paths** (`CommitConfig` JSON vs. `CommitConfigStruct`).
6. **SimData side-effect rules** triggered by `CommitConfig`.

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
    |  LUMICE_CommitConfig* ──> internally: Stop() → rebuild consumers → Start()
    v
[Running — kRunning, simulation active]
    |                                              ^
    |  simulation completes (finite rays)          |
    |  or LUMICE_StopServer                        |
    v                                              |
[Idle — kStopped, has_ever_consumed_=false]        |
    |                                              |
    +---- LUMICE_CommitConfig* (re-launch) --------+
    |
    |  LUMICE_DestroyServer ──> Terminate()
    v
[Destroyed — impl_ reset, handle invalid]
```

Key observations:

- There is no explicit `Run()` API. The server transitions to Running
  *inside* `CommitConfig`, which calls `Stop() → rebuild → Start()`.
- `LUMICE_SERVER_NOT_READY` is defined in the `LUMICE_ServerState` enum but
  is never returned by `LUMICE_QueryServerState`. The implementation maps
  to either `LUMICE_SERVER_IDLE` or `LUMICE_SERVER_RUNNING`. This enum
  value is dead code retained for potential future use.
- `LUMICE_DestroyServer(NULL)` is safe (early return on null check).

### §2.2 State Table

| Current State | API Call | Next State | Notes |
|---------------|----------|------------|-------|
| Idle | `CommitConfig*` | Running | Stop (no-op if already stopped) → rebuild → Start |
| Idle | `StopServer` | Idle | No-op (already stopped) |
| Idle | `DestroyServer` | Destroyed | `Terminate()` → `~ServerImpl` joins all threads |
| Running | `CommitConfig*` | Running | Stop (drains workers) → rebuild → Start |
| Running | `StopServer` | Idle | Drains queues, waits for `active_workers_==0` |
| Running | `DestroyServer` | Destroyed | `Terminate()` calls Stop first |
| Running | `Get*Results` | Running | Triggers snapshot if `snapshot_dirty_` |
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
  (`c_api.cpp:62–75`)

#### `LUMICE_DestroyServer(server)`

- **Precondition**: `server` is a valid handle or `NULL`.
- **Postcondition**: all threads joined; handle memory freed. Passing
  `NULL` is a no-op.
- **Side effects**: calls `Terminate()` which calls `Stop()` then destroys
  `ServerImpl`.
  (`c_api.cpp:78–84`)

### §3.2 Configuration APIs

#### `LUMICE_CommitConfig(server, config_str)` / `LUMICE_CommitConfigFromFile(server, filename)`

- **Precondition**: `server != NULL`, input non-null.
- **Postcondition**: on success, server is Running with new config active.
  On failure, server may be in `kError` status (parse failures leave the
  running server untouched; only the status is set to `kError`).
- **Error returns**: `LUMICE_ERR_NULL_ARG`, `LUMICE_ERR_INVALID_JSON`,
  `LUMICE_ERR_INVALID_CONFIG`, `LUMICE_ERR_MISSING_FIELD`,
  `LUMICE_ERR_INVALID_VALUE`.
- **Internal sequence**: parse JSON → `Stop()` → consumer rebuild/reuse →
  update `active_scene_` → `Start()`.
  (`c_api.cpp:125–156`, `server.cpp:226–335`)

#### `LUMICE_CommitConfigStruct(server, config, out_reused)`

- **Precondition**: `server != NULL`, `config != NULL`. Array counts must
  not exceed `LUMICE_MAX_CONFIG_*` limits.
- **Postcondition**: same as `CommitConfig` (JSON path). If `out_reused`
  is non-null, set to `1` when consumers were reused, `0` when rebuilt.
- **Internal path**: `ConfigToJson(*config)` → `Server::CommitConfig(json, &reused)`.
  The struct path always produces `dual_fisheye_equal_area` lens at 180° FOV
  (see §6.2).
  (`c_api.cpp:313–336`)

#### `LUMICE_ParseConfigString(json_str, out)` / `LUMICE_ParseConfigFile(filename, out)`

- **Precondition**: pointers non-null.
- **Postcondition**: `out` populated with parsed config. The `spectrum`
  field points to static storage — the caller must not free it.
- **Side effects**: none (pure parsing, no server state change).
- **Supported subset**: only `type="raypath"` filters; only string-form
  spectrum (`"D65"`, `"D50"`, `"A"`, `"E"`); renderer lens/view/visible/
  background fields are ignored.
  (`c_api.cpp:670–704`)

### §3.3 Result Retrieval APIs

All result APIs follow the sentinel pattern (see §5).

#### `LUMICE_GetRenderResults(server, out, max_count)`

- **Precondition**: `server != NULL`, `out != NULL`.
- **Postcondition**: `out[0..count-1]` filled with sRGB uint8 results.
  Sentinel at `out[count]` if `count < max_count`.
- **Internal**: calls `DoSnapshot()` which acquires `consumer_mutex_` →
  `PrepareSnapshot()` → `snapshot_mutex_` → `PostSnapshot()` → cache.
  (`c_api.cpp:708–732`)

#### `LUMICE_GetRawXyzResults(server, out, max_count)`

- **Precondition**: same as above.
- **Postcondition**: `out[0..count-1]` filled with raw XYZ float data
  plus metadata (`snapshot_intensity`, `intensity_factor`, `has_valid_data`,
  `snapshot_generation`, `effective_pixels`, anchor fields).
- **Internal**: independent snapshot path — `PrepareSnapshot()` under
  `consumer_mutex_`, `CountEffectivePixels()` outside lock, results under
  `snapshot_mutex_`. Increments `snapshot_generation_` on actual snapshot.
  (`c_api.cpp:735–766`)

#### `LUMICE_GetStatsResults(server, out, max_count)`

- **Precondition**: same as above.
- **Postcondition**: `out[0]` filled if stats available. Sentinel at
  `out[count]` if `count < max_count`.
- **Internal**: triggers `DoSnapshot()` — includes `PostSnapshot()`
  XYZ→RGB conversion (unlike `GetCachedStats`).
  (`c_api.cpp:769–791`)

#### `LUMICE_GetCachedStats(server, out)`

- **Precondition**: `server != NULL`, `out != NULL`.
- **Postcondition**: `out` filled with most recent cached stats. Returns
  all-zero struct if no snapshot has been taken.
- **Internal**: only acquires `snapshot_mutex_` — does **not** trigger
  `DoSnapshot()`. Stats come from the last `GetRawXyzResults` or
  `GetStatsResults` call.
  (`c_api.cpp:794–809`)

### §3.4 State & Control APIs

#### `LUMICE_QueryServerState(server, out)`

- **Precondition**: `server != NULL`, `out != NULL`.
- **Postcondition**: `*out` set to `LUMICE_SERVER_IDLE` or
  `LUMICE_SERVER_RUNNING`.
- **Internal**: reads `status_` under `status_mutex_`; if Running, also
  polls simulator idle state and `scene_gen_active_` flag.
  (`c_api.cpp:813–825`, `server.cpp:541–574`)

#### `LUMICE_StopServer(server)`

- **Precondition**: `server` is valid or `NULL` (null is safe).
- **Postcondition**: server is Idle. Worker threads are drained (not
  terminated — they return to `cv.wait`).
  (`c_api.cpp:828–834`, `server.cpp:486–539`)

### §3.5 Stateless Utility APIs

These functions have no shared state and are always thread-safe:

| Function | Precondition | Notes |
|----------|-------------|-------|
| `LUMICE_GetCrystalMesh(server, json, out)` | `json != NULL`, `out != NULL` | `server` is unused (reserved) |
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
(`server.cpp` `GetRawXyzResults`: `valid_data = has_ever_consumed_`). That flag
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
| `LUMICE_CommitConfig*` | No | — | Modifies `config_manager_`, calls Stop/Start |
| `LUMICE_StopServer` | No | — | Direct state/queue mutation |
| `LUMICE_SetLogCallback` | No | static bool (non-atomic) | First-call sink registration has a race window |
| `LUMICE_SetLogLevel` | Conditional | spdlog internal lock | Thread-safe for concurrent calls; ordering not guaranteed |
| `LUMICE_QueryServerState` | Yes | `status_mutex_` | Read-only poll; safe from any thread |
| `LUMICE_GetRenderResults` | Yes | `consumer_mutex_` (TicketMutex) + `snapshot_mutex_` | Two-phase snapshot protocol |
| `LUMICE_GetRawXyzResults` | Yes | same as above | GUI polling thread uses this path |
| `LUMICE_GetStatsResults` | Yes | same as above | Triggers DoSnapshot |
| `LUMICE_GetCachedStats` | Yes | `snapshot_mutex_` | Read-only cache; no DoSnapshot |
| `LUMICE_SetRaypathColors` | Yes* | `consumer_mutex_` (TicketMutex) | Display-time only: updates color/visible/solo/z-order/mode on the active class table, sets `snapshot_dirty_`. Never touches Stop/Start/`scene_generation_`/`committed_epoch_`/`consumers_`. *Safe vs `Get*Results`, but NOT vs concurrent `CommitConfig*` (same single-owner rule; `CommitConfig` writes `active_class_table_` partly outside `consumer_mutex_`, a pre-existing race — task-342.2 progress.md risk 3). |
| `LUMICE_GetCrystalMesh` | Yes | — | No shared state (`server` param unused) |
| `LUMICE_ValidateRaypathText` | Yes | — | Pure function |
| `LUMICE_IsLegalFace` | Yes | — | Pure function |
| `LUMICE_MaxFov` | Yes | — | Pure function |
| `LUMICE_XyzToSrgbUint8` | Yes | — | Pure function |
| `LUMICE_ParseConfigString` | Yes | — | Pure function |
| `LUMICE_ParseConfigFile` | Yes | — | Pure function |

**Mutex types**:

- **`TicketMutex`** (`server.cpp:37–53`): FIFO spinlock using
  `atomic<uint32_t>` ticket/serving counters. Prevents starvation that
  occurs with Windows SRWLOCK under high-frequency locking
  (see [`doc/accumulator-consumer-architecture.md` §4.1](accumulator-consumer-architecture.md)).
- **`std::mutex`**: used for `snapshot_mutex_`, `status_mutex_`,
  `start_mutex_`, `prod_mutex_`, `scene_mutex_`.

**Practical rule**: a single "owner thread" should perform all non-thread-safe
operations (`Create`, `CommitConfig`, `Stop`, `Destroy`). Polling threads
may safely call `QueryServerState` and `Get*Results` concurrently.

---

## §5 Sentinel Pattern & Overflow Protection

### §5.1 Convention

Result retrieval APIs use a sentinel-terminated array pattern. The sentinel
is a zero-initialized struct written at `out[count]` **only** when
`count < max_count` (i.e., there is a spare slot).

| Function | Sentinel field | Sentinel value |
|----------|---------------|----------------|
| `LUMICE_GetRenderResults` | `img_buffer` | `NULL` |
| `LUMICE_GetRawXyzResults` | `xyz_buffer` | `NULL` |
| `LUMICE_GetStatsResults` | `sim_ray_num` | `0` |

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

**Fix** (applied at `c_api.cpp:727`, `c_api.cpp:761`, `c_api.cpp:786`):

```cpp
// AFTER (fixed)
if (count < max_count) {
    std::memset(&out[count], 0, sizeof(...));
}
```

**Regression test**: `test/e2e/test_capi_sentinel_overflow.py` exercises
3 configs × 12 rounds = 36 server lifecycles using `max_count=1` to guard
against reintroduction.

### §5.3 Caller Contract

There are two valid calling conventions:

**Sentinel iteration** (requires N+1 array):

```c
LUMICE_RawXyzResult arr[N + 1]{};   // value-initialize to all zeros
LUMICE_GetRawXyzResults(server, arr, N);
for (int i = 0; arr[i].xyz_buffer != NULL; i++) {
    // use arr[i]
}
```

The caller must value-initialize the array (`{}` or `memset`) so that
natural zero values act as sentinels even when the API fills all N slots
(in which case no API-written sentinel exists at `arr[N]`).

**Direct index access** (exact-size array):

```c
LUMICE_RawXyzResult arr[N];
LUMICE_GetRawXyzResults(server, arr, N);
for (int i = 0; i < N; i++) {
    // use arr[i] directly — check individual fields for validity
}
```

No sentinel is needed when the caller uses direct indexing.

---

## §6 CommitConfig Dual Paths

Both paths ultimately call the same internal `Server::CommitConfig(json, out_reused)`.

### §6.1 JSON String Path

`LUMICE_CommitConfig(server, config_str)`:
1. Null-checks arguments.
2. Calls `Server::CommitConfig(string)` which parses JSON then delegates
   to `ServerImpl::CommitConfig(json)`.

This path accepts the **full** configuration format: all filter types
(distribution, raypath, etc.), array-form spectrum, any lens type, any
view parameters.

(`c_api.cpp:125–139`)

### §6.2 C Struct Path

`LUMICE_CommitConfigStruct(server, config, out_reused)`:
1. Null-checks and bounds-checks array counts.
2. Calls `ConfigToJson(*config)` to convert the C struct to a JSON object.
3. Calls `Server::CommitConfig(json, &reused)`.

Constraints imposed by the C struct representation:

| Dimension | Struct path limitation |
|-----------|----------------------|
| Filter types | `type="raypath"` only |
| Spectrum | String enumerations only (`"D65"`, `"D50"`, `"A"`, `"E"`) |
| Lens type | Always `dual_fisheye_equal_area`, FOV 180° |
| View parameters | Fixed at elevation=0, azimuth=0, roll=0 |
| Background | Fixed at black (0,0,0) |
| Visible | Fixed at `"full"` |
| `out_reused` | Exposes consumer reuse decision (JSON path does not) |

(`c_api.cpp:188–336`)

### §6.3 Semantic Equivalence

For the subset of configurations expressible by `LUMICE_Config`, both paths
produce **identical** `CommitConfig(json)` calls. The struct path exists to
eliminate JSON string serialization overhead in the GUI's 50ms commit cycle
(see task-52.2). The reuse flag (`out_reused`) lets the GUI detect whether
buffer pointers remain valid.

`raypath_color[]` (Design 2 per-raypath color classes, task-342.2) rides this
same struct→JSON→`CommitConfig` path — there is **no** separate struct→core
translation for color. `ConfigToJson` emits `raypath_color` in the exact wire
shape core's `RaypathColorConfig::from_json` reads, so the JSON-string commit
and the struct commit produce byte-identical composites for the same config
(verified by `RaypathColorApi.JsonAndStructCommitPixelEquivalent`, AC1). A
zero `raypath_color_count` omits the key entirely, keeping the mono JSON shape
byte-for-byte unchanged (AC4).

### §6.4 Display-time color setter (`LUMICE_SetRaypathColors`)

Changing **member structure** (a class's `match[]` refs / `combine`) is a
re-simulation event and must go through `CommitConfig*` (dirty → rebuild lanes →
re-accumulate). Changing only **appearance** (per-class RGB, `visible`, `solo`,
z-order, composite mode) does NOT need re-simulation: `LUMICE_SetRaypathColors`
mutates the active class table in place under `consumer_mutex_`, sets
`snapshot_dirty_`, and the next `Get*Results` re-composites the SAME accumulated
per-class Y-lanes. It never advances the epoch or clears the accumulator (AC2).
`class_count` must equal the committed `raypath_color_count` (mismatch =
`LUMICE_ERR_INVALID_CONFIG` — the caller changed structure and must re-commit);
`z_order`, when non-NULL, must be a permutation of `[0, class_count)` (AC3). The
z-order is decoupled from the Y-lane physical index: the compositor sorts by
z-order but always indexes lanes by the original class position, so reordering
draw priority never re-binds a lane to another class's color (see
`doc/gui-custom-spectrum-and-raypath-color.md` §4.0 as-built note).

---

## §7 SimData Side Effects of CommitConfig

### §7.1 Internal Reset Sequence

When `CommitConfig` succeeds, the following sequence occurs
(`server.cpp:226–335`):

1. **Parse**: temporary `ConfigManager` parses JSON. On failure, the running
   server is untouched (only `status_` set to `kError`).
2. **`Stop()`** (`server.cpp:486–539`):
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

| Field | State after CommitConfig |
|-------|------------------------|
| `has_ever_consumed_` | `false` — new simulation data required |
| `snapshot_dirty_` | `false` |
| `snapshot_generation_` | **Unchanged** — only `PrepareSnapshot` increments it; poller tracks its own `last_generation_` |
| `img_buffer` / `xyz_buffer` pointers | **Invalid on rebuild**; valid on reuse (same allocation). Callers should re-query after `CommitConfig` |
| `has_valid_data` (C API field) | `0` — maps from `has_ever_consumed_=false` |
| Cached `StatsResult` | Stale from previous run; `GetCachedStats` returns it until overwritten by next snapshot |

**Rule**: after any `CommitConfig` call, callers must not use previously
obtained `img_buffer` / `xyz_buffer` pointers. The GUI re-fetches via
`GetRawXyzResults` on the next poll cycle.

---

## §8 Cross-Reference Index

| Source location | Document section |
|----------------|-----------------|
| `src/include/lumice.h:264–269` (result API block comment) | §5 Sentinel Pattern |
| `src/server/c_api.cpp:726–729` (`GetRenderResults` sentinel) | §5.2 Overflow fix |
| `src/server/c_api.cpp:760–763` (`GetRawXyzResults` sentinel) | §5.2 Overflow fix |
| `src/server/c_api.cpp:785–787` (`GetStatsResults` sentinel) | §5.2 Overflow fix |
| `src/server/c_api.cpp:313` (`CommitConfigStruct` entry) | §6.2 C Struct path |
| `src/server/server.cpp:226` (`CommitConfig` entry) | §7 SimData side effects |
| `src/server/server.cpp:526–527` (`has_ever_consumed_ = false`) | §7.1 Reset sequence |
| `src/server/server.cpp` `ConsumeData` zero-exit `else` branch (`has_ever_consumed_`/`snapshot_dirty_` on all-black batch) | §3.6 Zero-output completion |
| `src/server/server.cpp:37–53` (`TicketMutex`) | §4 Thread safety (FIFO mutex) |
| `test/e2e/test_capi_sentinel_overflow.py` | §5.2 Regression test |
| `test/parity-cross-backend/backend/test_cuda_filter_parity.py::test_cuda_impossible_filter_produces_zero_intensity` | §3.6 Zero-output reproducer |
