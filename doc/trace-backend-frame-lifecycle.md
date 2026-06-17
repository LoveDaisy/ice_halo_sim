# TraceBackend Frame Lifecycle (as-built)

This document describes the **as-built** ray-frame lifecycle across the
`TraceBackend` seam — how ray directions move between the world frame and each
crystal's local frame as rays cross the seam and pass between multi-scattering
(MS) layers. It reflects the implementation landed by scrum
`metal-frame-correctness` (sub-tasks 253.1 single-MS frame fix + 253.2 multi-MS
frame transit). The design rationale lives in
`scratchpad/scrum-metal-frame-correctness/DESIGN-trace-backend-frame-lifecycle.md`;
**this file documents what the code actually does** — where the two diverge,
the code wins.

## 1. Why frames matter

The ray tracer batches rays per crystal: many rays sharing one crystal
orientation are traced together in that crystal's **local frame** (the geometry
is fixed; the rays are rotated into it). But a halo's angular structure (e.g. the
22° ring) is a **world-frame** property — each ray carries a random crystal
orientation, so the same local-frame exit direction maps to different world
directions per ray. Projecting exit rays without first returning them to world
space scatters the ring into a featureless band.

This is exactly the bug 253.1 fixed: the Metal kernel projected exit directions
in crystal-local space and never applied the per-ray rotation, so the production
render showed a horizontal green band instead of the 22° ring (CPU, which does
apply the rotation, rendered the ring correctly).

## 2. Invariant 6 — ray-frame lifecycle (the contract)

Codified in `src/core/trace_backend.hpp:60` (Design invariant 6):

- Rays **crossing the seam are always world-space**: `TraceLayer`'s input rays
  (host ingest or device continuation) and any ray that **leaves** a backend
  (projection, continuation export, readback) are world-space.
- The **crystal-local frame is a backend-internal detail**. A backend may
  transform world→local on ingest and **must** transform local→world before a
  ray leaves it.
- **Per-ray crystal orientation never crosses the seam**: each backend samples
  and owns its own per-ray rotations internally. How they are supplied (host-
  sampled upload vs device-side RNG) is a backend performance choice; the
  invariant is supply-agnostic.

This is a **documentation contract, not an automated gate** (comments can
drift). Its closure is procedural: a future CUDA backend's plan-review must list
"exit rays returned to world space before leaving the kernel" as a mandatory
check.

The CPU backend already modelled this before the contract was written — it is
the reference implementation (§3).

## 3. CPU reference (the frame chain to mirror)

`src/core/simulator.cpp`:

- **Ingest (world→local)**: `InitRay_rot` samples a per-ray `crystal_rot_`
  (`simulator.cpp:177` first MS, `:203` later MS); `ApplyInverse(r.d_)` rotates
  the world-space direction into the crystal-local frame for tracing.
- **Exit projection (local→world)**: `CollectData` applies
  `r.crystal_rot_.Apply(r.d_)` (`simulator.cpp:437` / `:441`) to return the exit
  direction to world space before projecting.
- **Inter-layer transit (`InitRayOtherMs`, `simulator.cpp:199-210`)**: only the
  **direction** carries across a layer boundary, via world space — the previous
  layer leaves the ray world-space, then the next layer resamples a **new**
  `crystal_rot_` (`InitRay_rot`, `:203`), `ApplyInverse`-es into the new local
  frame, and **resamples the entry point + hit face on the new crystal**
  (`InitRay_p_fid`, `:210`). Position `p` and `to_face` are NOT carried over —
  they are re-derived on the next crystal.

## 4. Metal backend (as-built)

`src/core/metal_trace_backend.mm`. The transform lands in two places (DESIGN D2):
ingest on the host, exit/continuation return **in-kernel** (DESIGN D3: any ray
leaving the kernel is world-space).

### 4.1 Supply form — host-sampled per-ray rotation upload

Metal v1 samples per-ray orientations on the host and uploads each ray's 3×3
rotation matrix (9 floats, row-major, identical layout to `Rotation::mat_`; see
`Rotation::GetMat()` in `geo3d.hpp`):

- `GenerateFirstLayerRootsForCi` (`metal_trace_backend.mm:554`) runs the same
  `InitRayFirstMs` path as the CPU, then uploads each ray's `crystal_rot_.GetMat()`
  into `root_rot_buf` (`:590`).
- `root_rot_buf` is bound to the kernel at **buffer index 17**
  (`trace_layer_kernel` signature, `:85`).

The contract is supply-agnostic: a CUDA backend may instead generate rotations
device-side (GPU-RNG). On discrete GPUs the per-layer host-sample-and-upload
becomes a real PCIe round-trip, so device-gen may be preferred there — the seam
does not need to change.

### 4.2 Single-MS exit projection (local→world, in-kernel)

The kernel loads the per-ray matrix once at entry (`m[9]`) and, in the exit-
projection block (`metal_trace_backend.mm:213-217`), applies `world = m · v` to
the exit direction **before** the equirect projection — the exact analogue of
CPU `CollectData`'s `crystal_rot_.Apply`. The multiply is **row-major
`mat·v`** (mirroring `Rotation::Apply`'s `Dot3(mat_ + k*3, v)`), **not** the
transpose (`ApplyInverse` is the transpose) — a transpose here re-introduces a
mis-oriented ring.

### 4.3 Multi-MS inter-layer transit

> **⚠️ Superseded by scrum-267 (device-resident continuation engine, 2026-06-14).**
> The host-side `CopyContSliceToRootBuf` path described in earlier revisions of
> this section has been **deleted** (scrum-267 task-device-resident-continuation).
> Inter-layer transit is now entirely device-side via `transit_root_kernel`.
> The frame-lifecycle invariant (§2) is unchanged — the mechanism moved to device.

Two halves, mirroring the CPU chain (§3):

1. **In-kernel world-return** (continuation emit gate, `metal_trace_backend.mm`
   ms_mode==1 branch): when the trace kernel decides a ray must continue to the
   next MS layer, it applies `world = m · v` to the local-frame exit direction
   before writing `cont_d_out[tid]`. The continuation buffer therefore holds
   **world-space** directions, satisfying invariant 6 at the layer boundary.
   The ray's lifetime wavelength index `wl_idx` is also written to
   `cont_wl_idx_out[tid]` (DR-3, scrum-268.8 — see §8).
   (`out_p` / `out_tf` slots in the old host-transit interface are now retired
   dead writes, documented in the kernel source.)

2. **Device resample-to-new-local** (`transit_root_kernel`, `:957`): a device
   kernel (scrum-267) replaces the former host `CopyContSliceToRootBuf`. It:
   - Samples a **fresh per-ray crystal orientation** on-device (same PCG +
     `sample_lat_lon_roll` as `gen_root_kernel`) keyed by a derived `transit_seed`
     + monotone `transit_ray_count_` counter (unique per layer/ci/batch/tid across
     the full Run() PCG range).
   - Reads the **world-space** continuation direction from `cont_d_in[tid]` and
     rotates it into the new crystal-local frame via `apply_inverse_mat9`.
   - Triangle area-weighted picks the **new entry point + face** on the destination
     crystal (`:1002-1040`), writing `root_p` / `root_tf`.
   - Pass-through copies `cont_wl_idx_in → root_wl_idx_out` so the wavelength
     tag survives the layer hop intact (DR-3).

   After `transit_root_kernel`, `root_d` / `root_p` / `root_tf` / `root_rot` /
   `root_wl_idx_out` are fully populated with zero host-side per-ray work; the
   host only dispatches the kernel and reads back the continuation counter.

This is single-MS and multi-MS derived from **one rule** (leave-kernel ⇒ world),
not two patches.

### 4.4 Performance note — async rotation pool (moot since scrum-267)

DESIGN D6 proposed an async producer/consumer pool to overlap host orientation
sampling with device tracing for multi-MS. **It was never implemented**, and is
now **moot**: scrum-267 moved orientation sampling entirely to `transit_root_kernel`
on-device, so there is no host-side sampling to overlap. §4.3 now describes the
current behaviour (device-resident); D6 is a superseded design note.

## 5. Parity harness methodology (how this is guarded)

`test/test_metal_trace_parity.cpp`:

- **Independent oracle, not a kernel mirror.** Structural correctness is checked
  against the real `CpuTraceBackend` (which projects in world space), NOT the
  in-test `OracleTraceLayer` kernel mirror. The mirror shares the kernel's frame
  handling, so it is **blind to frame bugs** — the original 251.4 harness used a
  kernel-mirror oracle and missed the band-vs-ring bug entirely.
- **Spatially-sensitive metric, not an energy sum.** Comparison is a per-pixel
  Y-channel Pearson correlation, not `ChannelSum` (a whole-image energy total).
  A correlation distinguishes a world-space ring from a crystal-local band;
  an energy sum does not (rotation conserves total landed energy).
  - `MetalVsCpuSingleLayerSpatialStructure` (`test:1012`): single-MS, measured
    correlation ≈ 0.9999 (fixed) vs ≈ 0.028 (deliberately local-frame control).
  - `MetalVsCpuMultiLayerSpatialStructure` (`test:1120`): two MS layers, measured
    correlation ≈ 0.997.
- **Negative control.** The single-MS test also feeds a crystal-local image
  (the pre-fix behaviour) and asserts its correlation collapses — proving the
  metric is not spatially blind (i.e. would catch a frame regression).

The retired kernel-mirror multi-MS assertions (`TwoLayerExitStatsAndXyz`,
`MultiPopTwoLayerExitStatsAndXyz`) keep only their still-valid layer-0 exit-stats
checks; their layer-1 comparison was removed because the kernel-mirror re-entry
(local-frame, no world-transit) is exactly the pre-253.2 bug.

## 6. Brightness ratio (resolved, no fix needed)

A ~1.35× Metal/CPU brightness ratio observed pre-fix was a **broken-frame
artifact** (measured on the band-state render, where ring-vs-band pixel
brightness is not comparable). After the frame fix the energy ratio
`ΣY_metal / ΣY_cpu` self-heals to ≈ 1.0 (measured 1.00011 on the full-random
ring scene). The `ΣY/kCmfY` total-landed-weight formula
(`simulator.cpp:902-914`) is correct; no fix was required. See
`scratchpad/scrum-metal-frame-correctness/explore-y-channel-brightness/SUMMARY.md`.

## 7. Public-API impact

None. The frame lifecycle is entirely internal to `core`. `src/include/lumice.h`,
`src/gui/`, and `src/server/` are unchanged and unaware of which backend or frame
convention is in use (seam invariant 5).

## 8. Per-ray wavelength (DR-3, scrum-268.8)

**Decision**: wavelength is a **per-ray lifetime property** (not per-batch or per-
threadgroup). Rationale: per-threadgroup was the original plan (DR-1(b)), but was
rejected because multi-MS continuation compresses ray populations — a photon's
`tg_id` becomes unstable across layer hops, so per-tg wavelength assignment would
let a photon change refraction index mid-path.

**Mechanism** (as-built, scrum-268.8):
- A `WlPool` of `M` entries (`{n, spd_weight, cmf}`) is pre-sampled on the host
  once per session (CPU-side Sellmeier evaluation, zero device Sellmeier).
- `gen_root_kernel` assigns each root ray a `wl_idx` from the pool
  (`wl_idx = global_idx % M`).
- The trace kernel reads the pool entry via `wl_idx` for per-ray refraction index.
- On continuation emit the `wl_idx` is written to `cont_wl_idx_out[tid]`.
- `transit_root_kernel` copies `cont_wl_idx_in → root_wl_idx_out` (§4.3, step 2)
  so the wavelength tag is preserved across all MS layers.
- The consumer accumulates XYZ using the per-ray CMF weight from the pool entry.

`M` is resolved from `LUMICE_WL_POOL_SIZE` (env var, read once at `BeginSession`;
`WlPoolSize()` exposes it for tests). A static analysis bug that would have
produced a flat spectrum (weight=0) on the `cpu_backend` was caught by the
anti-silent-regression hard gate deployed in §2's `IsOutgoing` path.
