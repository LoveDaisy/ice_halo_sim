[中文版](inverse-rendering_zh.md)

# Inverse Rendering via Precomputed Standard Events

This document describes a proposed alternative rendering approach that replaces the current
Monte Carlo (MC) pipeline with a deterministic, per-pixel computation based on precomputed
standard events. The idea is inspired by the rotational covariance theory in Gislén et al.
(2004) but adapted for parallel-light (solar) halo simulation.

**Status**: Theoretical design — not yet implemented.

## Motivation

The current MC pipeline works in a "forward" direction: sample crystal orientation → ray-trace
→ record which pixel the outgoing ray hits → accumulate. This is inherently noisy; convergence
requires large sample counts, and the computation is "wasteful" in the sense that rays are
distributed across all pixels regardless of whether they are needed.

The proposed approach inverts this: for each pixel, determine which crystal orientations
contribute to it, and compute the contribution analytically. This eliminates MC noise entirely
and separates the expensive ray-tracing precomputation (done once per crystal shape) from the
cheap rendering step (done per parameter set).

### Relationship to Beam Tracing

The beam tracing experiment (`exp/beam_tracing` branch, see [beam-tracing.md](beam-tracing.md))
attempted to eliminate **entry-point sampling** variance. Its key finding was that this variance
is negligible — the true bottleneck is **orientation sampling** on SO(3).

The inverse rendering approach addresses this bottleneck directly: instead of sampling
orientations and hoping they converge, it computes the exact set of contributing orientations
for each pixel.

## Theoretical Foundation

### Rotational Covariance

The crystal scattering distribution satisfies rotational covariance (Gislén eq. 2):

```
p(b̂ | â, U) = p(U⁻¹b̂ | U⁻¹â, 1)
```

where `â` is the incident direction, `b̂` is the scattered direction, `U` is the crystal
orientation (an SO(3) rotation), and `1` is the standard (identity) orientation.

This means: any scattering event with crystal orientation `U` can be obtained by rotating
a **standard event** (crystal at identity orientation) by `U`.

### Standard Events

A standard event is a pair `(â₀, b̂₀)` where:
- `â₀` is a random incident direction on the crystal in standard orientation
- `b̂₀` is the resulting scattered direction, found by ray-tracing
- The scattering angle is `ω = arccos(â₀ · b̂₀)`
- An associated weight `w` captures the Fresnel coefficient and projected area

For a given crystal shape, the standard events can be precomputed densely and stored.

### Per-Pixel Inversion

For a pixel in the output image with observation direction `b̂`, given sun direction `â`:
1. The scattering angle is `ω = arccos(â · b̂)`
2. Any standard event `(â₀, b̂₀)` with the same scattering angle could contribute
3. The rotation `U` mapping `(â₀, b̂₀) → (â, b̂)` is uniquely determined (Gislén eq. 19):

```
U = (â·â₀ᵀ + b̂·b̂₀ᵀ - cosω(â·b̂₀ᵀ + b̂·â₀ᵀ) + (â×b̂)(â₀×b̂₀)ᵀ) / sin²ω
```

4. This `U` is the crystal orientation required for this standard event to produce the
   observed scattering. Evaluate `Q(U)` — the crystal orientation distribution — to get
   the weight.

The pixel value is the weighted sum over all matching standard events:

```
pixel_value = Σᵢ wᵢ × Q(Uᵢ)
```

## Proposed Algorithm

### Phase 1: Precomputation (once per crystal shape)

Densely sample incident directions `â₀` on S² (e.g., ~100K–500K directions). For each `â₀`,
ray-trace through the crystal in standard orientation, recording all raypaths:

```
PrecomputedEvent {
    â₀: Vec3           // incident direction in standard frame
    b̂₀: Vec3           // scattered direction in standard frame
    ω: float           // scattering angle = arccos(â₀ · b̂₀)
    weight: float      // Fresnel × projected area
    raypath_id: int    // which raypath (for filter support)
}
```

Sort all events by `ω` into a single array. This is the **precomputed event store**.

### Phase 2: Rendering (per pixel, deterministic)

For each pixel:

1. **Determine ω range**: The pixel subtends a finite solid angle, corresponding to a range
   `[ω_min, ω_max]`. This can be found by sub-pixel sampling (evaluate `arccos(â · b̂)` at
   pixel corners) or analytically via the projection Jacobian.

2. **Range query**: Binary search the sorted event store for all events with
   `ω ∈ [ω_min, ω_max]`. This is O(log N) for the search plus O(K) for K matching events.

3. **Weight accumulation**: For each matching event `(â₀ᵢ, b̂₀ᵢ, wᵢ)`:
   - Compute rotation `Uᵢ` mapping `(â₀ᵢ, b̂₀ᵢ) → (â, b̂)` via eq. 19
   - Evaluate `Q(Uᵢ)` — the crystal orientation probability
   - Accumulate: `pixel_value += wᵢ × Q(Uᵢ)`

4. **Normalize**: Divide by the number of precomputed incident directions (or appropriate
   solid angle measure) to get physically correct intensity.

### Data Structure

```
// Sorted by omega — the only data structure needed
std::vector<PrecomputedEvent> events;  // sorted by ω

// Per-pixel query
auto [lo, hi] = equal_range(events, omega_min, omega_max);
for (auto it = lo; it != hi; ++it) {
    Mat3 U = compute_rotation(it->a0, it->b0, sun_dir, pixel_dir);
    float q = orientation_distribution(U);
    pixel_value += it->weight * q;
}
```

## Key Properties

### Self-Regulating Intensity

The algorithm naturally produces correct intensity variations:

- **Bright caustics** (e.g., 22° halo edge): Near minimum deviation, `dω/dâ₀ → 0`, so many
  standard events cluster at similar `ω` values. The range query returns many matches →
  bright pixel. This is physically correct — caustic brightness arises from the same
  concentration effect.

- **Dark regions**: Few standard events scatter at these angles → few matches → dim pixel.
  This matches MC behavior (few rays hit these pixels) but without noise.

- **Oriented crystals**: `Q(U)` acts as a filter — only orientations consistent with the
  crystal distribution contribute. Parhelia, tangent arcs, and other orientation-dependent
  features emerge naturally.

### Precomputation Reuse

The event store depends only on crystal geometry and refractive index. Changing any of
the following requires only re-running Phase 2 (cheap), not Phase 1:

- Sun elevation / azimuth
- Crystal orientation distribution `Q(U)` (tilt angle, roll distribution)
- Image resolution, projection type, field of view
- Wavelength (if separate stores are precomputed per wavelength)

This is particularly valuable for interactive GUI parameter adjustment.

### No Interpolation Required

Unlike approaches that require contour extraction or interpolation on S², the range-query
approach works directly with discrete precomputed events. The finite pixel size naturally
defines the query range, and the discrete sum approximates the continuous integral.

If a range query returns no matches, it means:
1. The precomputation is too sparse (increase sampling density), **or**
2. The pixel genuinely has very low intensity (the differential cross-section is small at
   this scattering angle)

In practice, case 2 dominates — pixels with no matches would also be dark under MC.

## Cost Analysis

### Precomputation

| Parameter | Typical Value |
|-----------|---------------|
| Incident directions | 100K–500K |
| Raypaths per direction | ~10 |
| Total events | ~1M–5M |
| Ray-trace cost | ~200 ops/ray |
| Total | ~2×10⁸–10⁹ ops (comparable to one MC run of 1–5M rays) |

This is a one-time cost per crystal shape.

### Rendering

| Parameter | Typical Value |
|-----------|---------------|
| Image pixels | 10⁶ (1000×1000) |
| Matching events per pixel | ~100–1000 (varies with ω) |
| Per-match cost | ~100 ops (rotation + Q eval) |
| Total per render | ~10⁸–10⁹ ops |

Compare with MC:
- 10M rays × ~200 ops = 2×10⁹ ops → noisy image
- 100M rays × ~200 ops = 2×10¹⁰ ops → clean image
- Inverse rendering: ~10⁹ ops → noise-free image

### Parameter Sweep

For exploring N parameter combinations (e.g., different tilt angles or sun elevations):
- MC: N × (full MC run) = N × 2×10⁹
- Inverse: 1 × precomputation + N × rendering = 10⁹ + N × 10⁹

At N > 2, inverse rendering is already more efficient.

## Extension: Divergent-Light Halos

The same precomputed event store can be used for divergent-light (nearby source) halo
simulation, such as street-lamp halos. The key difference is how the scattering angle
varies per pixel.

### Parallel Light vs. Divergent Light

| | Parallel light (sun) | Divergent light (street lamp) |
|---|---|---|
| Incident direction | Fixed `â` for all crystals | Varies with crystal position |
| Per-pixel ω | Nearly constant (one range query) | Varies along the view ray |
| Spatial integration | Not needed (all crystals equivalent) | Must integrate along the view ray |

### Ray Marching Approach

For divergent-light halos, each pixel corresponds to a view ray through 3D space. A crystal
at position **x** along this ray sees the light source at a different angle than a crystal at
position **y**. The scattering angle `ω` therefore varies continuously along the view ray.

The rendering procedure becomes:

1. For each pixel, cast a view ray from the observer into the crystal cloud.
2. Define a near-far range `[t_near, t_far]` bounding the crystal cloud.
3. March along the ray in steps. At each step position `t`:
   - Compute the local incident direction `â(t)` from the light source to position `t`
   - Compute the local observation direction `b̂(t)` from position `t` to the observer
   - Compute the local scattering angle `ω(t) = arccos(â(t) · b̂(t))`
   - Perform a range query on the precomputed event store for `ω(t) ± δω`
   - For each match, compute `U`, evaluate `Q(U)`, weight by `1/|x(t)|²` (inverse-square
     falloff from source) and accumulate
4. Sum contributions from all steps → pixel value.

Adjacent steps along the ray have similar `ω` values, so their query ranges overlap
significantly. This can be exploited: sort matches once for the full `ω` range of the ray,
then slide a window along the sorted results as `t` advances.

### Practical Simplifications

- **Near-far clipping**: Only march within the crystal cloud extent. Crystals very far from
  the observer contribute negligibly (inverse-square falloff from both source and observer).
- **Adaptive step size**: Use coarser steps where `ω(t)` varies slowly, finer steps near
  the observer (where `ω` changes rapidly with position).
- **ω range batching**: Instead of per-step queries, partition the full ray's `ω` range into
  a few sub-ranges. Query each sub-range once, then distribute matches to the appropriate
  ray steps.

### Comparison with Gislén's Cigar Method

Gislén's original divergent-light algorithm uses Minnaert's cigar geometry — for each
standard event, it generates points on a cigar surface and evaluates `Q(U)` there. This
is elegant mathematically but requires understanding and implementing the cigar
parameterization, the bipolar coordinate system, and the associated event-reuse logic.

The ray-marching approach reuses the same precomputed event store and the same range-query
logic as the parallel-light case. The only addition is the spatial loop along the view ray.
This is arguably simpler to implement and easier to reason about, at the cost of somewhat
higher computation (multiple range queries per pixel instead of one).

### Cost Estimate

For a divergent-light render with S steps along each ray:

| Parameter | Typical Value |
|-----------|---------------|
| Image pixels | 10⁶ |
| Ray march steps | ~50–200 |
| Matches per step | ~100–500 |
| Per-match cost | ~100 ops |
| Total per render | ~10¹⁰–10¹¹ ops |

This is 10–100× more expensive than the parallel-light case, but still deterministic and
noise-free. For comparison, Gislén's original MC-based divergent-light simulation also
requires substantially more computation than parallel-light MC, due to the additional
spatial dimension.

## Considerations and Limitations

### Wavelength Dependence

Refractive index varies with wavelength, affecting `b̂₀` and `ω`. Options:
- Separate precomputed stores per wavelength (or per RGB channel): cost ×3
- Interpolate between a few reference wavelengths

### Multi-Scattering

The precomputed events represent single scattering only. Multi-scattering would require
chaining events or a hybrid approach (inverse rendering for first scatter, MC for
subsequent scatters).

### Numerical Stability

The rotation formula (eq. 19) degenerates when `ω → 0` (forward scattering) or `ω → π`
(back-scattering), where `sin²ω → 0`. These regions need special handling or clamping.

### Precomputation Density

The sampling density on S² must be sufficient to resolve rapid variations in the scattering
function, particularly near total internal reflection boundaries and raypath transitions.
Adaptive refinement in these regions may be beneficial.

## Suggested Validation Plan

1. **Single raypath prototype**: Implement for one raypath only (e.g., the 22° halo prism
   path, faces 3→1). Compare with MC output at the same sun elevation.

2. **Random orientation test**: With `Q(U) = const`, verify that the radial intensity profile
   matches MC (should reproduce the 22° minimum deviation peak).

3. **Oriented crystal test**: Use a plate crystal distribution, verify parhelia appear at
   correct positions with correct relative brightness.

4. **Performance benchmark**: Measure precomputation time and per-render time. Compare with
   MC at equivalent image quality (PSNR).

## References

- L. Gislén, J. O. Mattsson, B. Söderberg, "An improved algorithm for simulations of
  divergent-light halos", LU TP 04-27, 2004.
- L. Gislén, J. O. Mattsson, "Observations and Simulations of Some Divergent-Light Halos",
  Appl. Opt. 42, 4269–4279, 2003.
- L. Gislén, "Procedure for Simulating Divergent-Light Halos", Appl. Opt. 42, 6559–6563, 2003.
