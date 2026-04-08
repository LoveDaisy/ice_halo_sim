[中文版](beam-tracing_zh.md)

# Beam Tracing

This document describes an experimental beam tracing (BT) approach explored as an alternative
to the default Monte Carlo (MC) ray tracing. The implementation lives on the `exp/beam_tracing`
branch and is **not merged into main** — this document explains the approach, results,
and why the cost-benefit trade-off does not favor adoption.

## Motivation

Standard MC ray tracing samples one stochastic path per ray. For each crystal orientation,
many rays are needed to estimate the outgoing light distribution. Beam tracing replaces this
per-ray stochastic sampling with exact geometric computation: given a crystal orientation and
light direction, it analytically computes *all* outgoing beams with their exact directions and
area-weighted Fresnel intensities — eliminating entry-point sampling variance entirely.

The theoretical appeal is variance reduction: by the law of total variance,

```
Var(f) = Var_R[E_s(f|R)] + E_R[Var_s(f|R)]
```

where `R` is orientation and `s` is entry-point sampling. BT eliminates the second term
(within-orientation variance) exactly.

## Algorithm

### Overview

For a given crystal orientation and light direction, `BeamTrace()` performs a BFS through
the crystal interior:

1. **Entry face detection**: Identify faces where `dot(light_dir, face_normal) < 0`.
   For each entry face, compute the projected area and create an initial beam (the refracted
   portion entering the crystal).

2. **BFS propagation**: Each beam in the queue is processed:
   - `PartitionBeam` determines which internal faces the beam's cross-section will hit
   - For each hit face: compute Fresnel coefficients, produce a refracted outgoing beam
     (exits crystal) and a reflected continuation beam (stays in queue)
   - Continuation beams below a minimum area threshold are pruned

3. **Termination**: BFS stops when the queue is empty or `max_hits` depth is reached.

### PartitionBeam: Face-Plane Projection + Sutherland-Hodgman Clipping

The key geometric operation is partitioning a beam's cross-section among candidate next-faces:

1. **Candidate selection**: Find internal faces where `dot(beam_dir, face_normal) > eps`
   (facing the beam direction), excluding the current face.

2. **Projection**: Extract each candidate face's 3D polygon vertices and project them
   onto the beam's 2D cross-section plane (`u = dot(v, basis_u)`, `v = dot(v, basis_v)`).

3. **Intersection**: Use Sutherland-Hodgman clipping (`IntersectConvexPolygons`) to compute
   the intersection of the beam's cross-section polygon with each projected candidate face.

4. **Result**: Each non-empty intersection becomes a partition — a sub-beam that hits that
   specific face, carrying a proportional share of the parent beam's weight.

Crystal convexity guarantees that partitions do not overlap.

### API

```cpp
struct BeamTraceResult {
  std::vector<float> outgoing_d;                   // exit directions, 3 floats each
  std::vector<float> outgoing_w;                   // area-weighted Fresnel weights
  std::vector<std::vector<int>> outgoing_raypath;  // face sequence per beam
  float total_entry_area = 0.0f;                   // total projected entry area
};

BeamTraceResult BeamTrace(const Crystal& crystal, const Rotation& rot,
                          const float* light_dir, float refractive_index,
                          size_t max_hits);
```

## Correctness Verification

### Fixed-Orientation Per-Beam Matching

For planar crystal faces with a fixed orientation and point light source, each unique
raypath (face sequence) produces exactly one exit direction. Both MC and BT should produce
identical sets of discrete exit directions with matching weights.

The `FixedOrientationMcVsBt` test validates this:
- 500K MC rays vs 1 BT orientation at fixed azimuth=45°, zenith=30°, roll=15°
- Cluster both MC and BT outputs by direction (cosine threshold 0.99999)
- Compare normalized weights per matched cluster

**Result after bug fixes**: Pearson correlation **0.999998**, all significant clusters
within 2.1% relative difference.

### Bugs Found and Fixed

1. **Re-projection coordinate mismatch** (primary): When reflecting a beam, the cross-section
   polygon was reconstructed as `p = u*bu + v*bv`, dropping the along-beam component. This
   created a coordinate mismatch with absolute face projections in subsequent `PartitionBeam`
   calls. Fix: compute actual 3D hit points via face-plane intersection before re-projecting.
   Pearson improved from 0.948 to 0.999998.

2. **Off-by-one bounce depth**: BT processed `max_hits + 1` total face interactions vs MC's
   `max_hits`. Fix: adjust depth threshold from `>= max_hits` to `>= max_hits - 1`.

3. **max_hits=0 underflow**: `size_t` subtraction underflow when `max_hits` is 0.
   Fix: early return guard.

## Performance Analysis

### Per-Unit Cost: 219x Slower Than MC

| Metric | MC | BT |
|--------|----|----|
| Config | 10M rays | 1M orientations |
| Wall time | 2.0s | 46.6s |
| CPU cost per unit | ~2 μs/ray | ~437 μs/orientation |

The 219x ratio decomposes into two factors:

- **Step count amplification (~32x)**: MC traces ~4 effective bounces per ray. BT exhaustively
  explores the full BFS tree: ~3.5 entry faces × branching factor ~2 × depth 6, pruned to
  ~130 BFS nodes per orientation.

- **Per-step cost (~10x)**: Each BFS node requires `ExtractPolygonFaceVertices` (scan 20
  triangles + dedup + sort, ~400 ops × ~3.5 candidates) and `IntersectConvexPolygons`
  (Sutherland-Hodgman O(V_a × V_b), ~250 ops × ~3.5 candidates), vs MC's simple ray-plane
  slab intersection (~200 ops).

### Variance: BT is Not Better

Run-to-run consistency measured by pairwise PSNR across 5 runs:

| Method | Config | Mean Pairwise PSNR | Time |
|--------|--------|--------------------|------|
| MC | 10M rays | **28.9 dB** | ~3s |
| BT | 1M orientations | **28.2 dB** | ~50s |

**BT is slightly *less* consistent than MC**, despite being 16x slower.

### Why: Variance Decomposition

The key insight is that for a given crystal orientation with parallel light, outgoing beams
form a **finite set of discrete delta distributions** on the sphere (one per raypath). A
discrete distribution converges extremely fast — MC needs only tens of entry-point samples
to accurately estimate each delta's weight. The within-orientation variance that BT
eliminates is already negligible.

Meanwhile, orientation sampling lives on the **SO(3) manifold** (3-dimensional, continuous),
where convergence follows O(1/√N). This is the dominant noise source, and BT does nothing
to help — it samples orientations the same way MC does, just with 10x fewer of them (because
each orientation costs 219x more to compute).

In short: BT spends ~219x compute to exactly solve a sub-problem that MC solves adequately
with ~30 samples, while the actual bottleneck (orientation sampling) remains unchanged.

## Integration Status

The `exp/beam_tracing` branch includes full integration with the simulation pipeline:

- **CLI output**: BT data flows through `RenderConsumer` to produce images
- **Render-level filters**: Supported via a 2-node `RaySeg` chain workaround (entry + outgoing),
  enabling `FilterRay` chain walk. Limited to 1 render-level filter.
- **Configuration**: `"use_beam_tracing": true` in scene config JSON
- **E2E test**: `test/e2e/configs/halo_22_bt.json` with PSNR threshold

## Branch Reference

All beam tracing code lives on `exp/beam_tracing` (9 commits, not merged to main):

```
0ab370f feat(beam-tracer): add beam tracing core geometry module
f62d3e8 feat(beam-tracer): integrate beam tracing into Simulator
4551953 fix(beam-tracer): fix BT sampling bias + MC vs BT distribution test
cc887fb refactor(beam-tracer): rewrite PartitionBeam with face-plane projection
1439c1f refactor(core): deduplicate InitRay_rot + HitSurface, move to geo3d
913dc50 feat(config): add use_beam_tracing JSON serialization + BT e2e config
d0fb711 fix(server): BT CLI output by replacing rays_.Empty() shutdown sentinel
440ba26 fix(beam-tracer): fix re-projection coordinate mismatch + per-beam test
3482b59 feat(beam-tracer): render-level filter via 2-node RaySeg chain
```

## Potential Optimizations (Not Implemented)

If BT is revisited in the future, the main optimization opportunities are:

1. **Cache `ExtractPolygonFaceVertices`** (~40% of BFS cost): For a fixed crystal, the 8
   polygon face vertex sets are constant across all BFS nodes and orientations. Currently
   re-extracted ~455 times per orientation. Estimated 1.5-1.7x speedup.

2. **Candidate face precomputation**: For each source face, candidate next-faces can be
   precomputed from crystal geometry instead of scanning all faces per BFS node.

3. **Hybrid approach**: Use BT only for the first 1-2 bounces (where it adds value by
   precisely partitioning the entry beam), then switch to MC for deeper bounces where
   Fresnel attenuation makes exact computation less valuable.
