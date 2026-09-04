# Crystal Orientation Sampling

This document describes how Lumice samples crystal c-axis orientations
for Monte Carlo ice halo simulations.

## 1. Physical Model

Ice crystals in the atmosphere orient under the combined influence of
aerodynamic torque and stochastic perturbation. The orientation of a
single hexagonal crystal is fully described by three angles:

- **Zenith angle (θ)**: the angle between the crystal's c-axis and the
  vertical. This is the primary determinant of halo geometry.
- **Azimuth angle (φ)**: the rotation of the c-axis projection about
  the vertical. For freely falling crystals this is uniformly distributed.
- **Roll angle (ψ)**: rotation of the crystal body about its own c-axis.

This decomposition is physically motivated: gravity defines a preferred
direction (the vertical), and aerodynamic drag acts primarily along this
axis. The resulting probability distribution on the orientation sphere
is a **zonal band** — concentrated around a preferred colatitude with
uniform azimuth. This is fundamentally different from a rotationally
symmetric cap; the symmetry axis is the gravity vector, not the crystal's
c-axis. This distinction is why von Mises–Fisher (vMF) distributions are
inappropriate here — vMF produces a cap, not a band.


## 2. Ice Crystal Motion Modes and Distribution Selection

The zenith angle distribution depends on the Reynolds number (Re), which
is determined by crystal size, shape, and fall speed. Three distinct regimes
exist, each requiring a different statistical model.

### 2.1 Stable Mode (Low Re)

At low Reynolds numbers, aerodynamic torque restores the crystal toward
its equilibrium orientation (c-axis vertical for plates, horizontal for
columns). Brownian motion and atmospheric turbulence introduce small-angle
fluctuations. The resulting tilt angle follows a **Gaussian distribution**:

```
p(θ) ∝ exp(-(θ - μ)² / 2σ²)
```

with σ typically 0.4°–3°. Polarization lidar retrievals find σ ≈ 1°–2°
for well-oriented plates (Noel & Sassen 2005), and satellite observations
confirm ~80% of oriented crystals deviate less than 3° from horizontal
(Noel & Chepfer 2004).

This is the regime that produces all identifiable halo phenomena: sundogs,
light pillars, circumzenith arcs, and parhelic circles. A key insight is
one of **physical self-consistency**: crystals producing recognizable halos
*must* have high-quality orientation (σ ≤ a few degrees), which corresponds
precisely to the stable mode where Gaussian is exact. Sharp halos require
narrow tilt distributions, and narrow tilt distributions arise only when
the restoring torque dominates. This is not coincidence but a consequence
of the same physics governing both the motion and the optics.

### 2.2 Unstable Mode (High Re)

Above a critical Reynolds number (Re > ~237 for hexagonal plates), the
wake becomes unsteady and the crystal enters oscillatory or chaotic motion.

**Zigzag oscillation.** The crystal rocks back and forth with a tilt angle:

```
θ(t) = |A · sin(ωt) + B|
```

where A is the oscillation amplitude and B is a mean tilt offset. The
time-averaged distribution is a **folded arcsine** — U-shaped, with peaks
near the turning points. Stout et al. (2024) experimentally confirmed that
Gaussian models fail qualitatively for these unstable modes.

**Spiral motion.** The crystal precesses around the vertical with a
relatively stable tilt angle. This can be approximated by a Gaussian with
an offset mean (μ ≠ 0).

Both modes produce diffuse background light rather than sharp halos. Column
crystals may exhibit characteristic canting angles; Zhong et al. (2023)
report θ_e ≈ 38° for column canting modes.

### 2.3 Size-Aggregated Tilt

Real ice clouds contain crystals spanning a range of sizes. For each size,
σ is determined by Re; larger crystals have larger σ. When tilt is
integrated over the full size distribution, the result follows a **Laplace
(double-exponential) distribution**:

```
p(θ) ∝ exp(-|θ - μ| / b)
```

The derivation: if X|σ ~ N(0, σ²) and σ² ~ Exp(λ), then X ~ Laplace(0, b)
with b = 1/√(2λ). Small crystals contribute the sharp Gaussian core;
large crystals contribute the heavier exponential tails.

Deep-space glint observations confirm this prediction. Kostinski, Marshak
& Várnai (2025) found that angular decay follows exp(−δ/s), exactly the
Laplacian signature, in DSCOVR spacecraft data.

The Laplacian distribution simplifies configuration: instead of multiple
crystal entries with different Gaussian σ values, a single entry with a
Laplace scale parameter suffices.


## 3. Supported Distribution Types

Four distribution types are supported, plus a deterministic mode. All types
share the same `mean`/`std` parameter fields in JSON; the meaning of `std`
varies by type.

| Type | Definition | `mean` | `std` | Use Case |
|------|-----------|--------|-------|----------|
| `gauss` | p(θ) ∝ exp(−(θ−μ)²/2σ²) | center μ (deg) | std deviation σ (deg) | Stable-mode orientation |
| `uniform` | p(θ) = const on [μ−w/2, μ+w/2] | center μ (deg) | full width w (deg) | Random orientation; roll |
| `zigzag` | θ = \|A·sin(2πU) + B\|, folded arcsine | tilt offset B (deg) | amplitude A (deg) | Large-crystal oscillation |
| `laplacian` | p(θ) ∝ exp(−\|θ−μ\|/b) | center μ (deg) | scale b (deg) | Size-aggregated tilt |

A **scalar value** in JSON (e.g., `"zenith": 90`) is treated as deterministic:
every crystal receives exactly that angle with no randomization.

**Parameter notes:**
- `gauss` σ = 2° means 68% of crystals fall within ±2° of the mean.
- `uniform` `std` is the full range width, not the standard deviation.
- `zigzag` distribution has support on [0, |A|+|B|] after folding.
- `laplacian` standard deviation is √2·b ≈ 1.41b.


## 4. Sampling Implementation

### 4.1 Jacobian Correction

Sampling an angle θ on the sphere requires accounting for the spherical
area element sin(θ). The target density is:

```
p_sphere(θ) ∝ p_distribution(θ) · sin(θ)
```

Without this, sampling overconcentrates at the poles where sin(θ) → 0.

### 4.2 Sampling Paths

`lat_path::SelectLatPath` (`src/core/shared/lat_path_selection.hpp`) is the
single source for path selection, shared by the CPU sampler
(`math.cpp::SampleSphericalPointsSph`) and both GPU backends. It resolves one
of four paths:

**Path 1: Full-sphere (`kFullSphere`).** Triggered when the axis distribution
is a full-sphere uniform (`AxisDistribution::IsFullSphereUniform()`: azimuth
and latitude are both a full 360° `uniform`, with roll rotationally
symmetric). Latitude is sampled directly with the area measure —
θ = asin(u), u ~ U(-1,1) — giving the uniform-on-sphere distribution with no
rejection. Exact, O(1).

**Path 2: Deterministic (`kNoRandom`).** Triggered when the latitude
distribution type is `kNoRandom` (a scalar `"zenith"` value in JSON). No
sampling — every crystal gets the same angle.

**Path 3: Legacy Gaussian (`kGaussLegacy`).** Triggered when the latitude
distribution type is `kGaussianLegacy` (`"gauss_legacy"` in JSON). This
intentionally skips the sin(θ) area-measure weight described in §4.1 — kept
only to reproduce simulation results predating the Jacobian fix (§5.1), not
a recommended type for new configs.

**Path 4: Unified LUT (`kLutInverseCdf`).** Every other non-degenerate type —
`kGaussian`, `kLaplacian`, `kUniform`, `kZigzag` — routes here. One
precomputed inverse-CDF lookup table replaces what used to be three separate
per-distribution samplers (Rayleigh / generic-rejection / deterministic —
retired in 330.3, see §5.2). Described in §4.3.

These four paths are the complete taxonomy: there is no longer a near-pole
vs. off-pole split, and no rejection loop anywhere in this list (see
`doc/near-pole-area-measure-sampling.md` for the removal record).

### 4.3 The Unified Inverse-CDF LUT (`kLutInverseCdf`)

Build (host-side, once per axis distribution — `lat_lut.cpp::BuildLatLut`,
memoized by `GetSharedLatLut` across CPU worker threads and both GPU
backends, never rebuilt per ray):

- A deterministic quadrature evaluates the exact per-family proposal
  density, folded through the same `normalize_latitude` wrap the sampler
  itself uses, and weighted by sin(θ) — the spherical area Jacobian from
  §4.1 — producing `LatLut::kNodes = 257` uniformly-spaced colatitude nodes
  with a strictly-increasing CDF.
- A per-bin `flip_prob` records the probability that a sample in that bin
  crosses the pole and needs its azimuth/roll flipped by π, so the near-pole
  proposal stays symmetric.

Sample (one uniform draw, no rejection loop):

1. Draw ξ ~ U(0,1).
2. `invert_lat_lut` resolves ξ against the CDF with a fixed 8-step binary
   search plus linear interpolation (256 intervals = 2⁸, so the search cost
   is constant).
3. Look up `flip_prob` for the resolved bin and flip azimuth/roll by π with
   that probability.

This path is rejection-free: the `rejection_m` field on `LatPathDecision`
(and the frozen device wire struct that mirrors it) is a constant `1.0` on
every one of the four paths. Accuracy is governed by node count rather than
by an envelope constant — the lookup is exact up to node-resolution error,
not an approximation with a rejection tail.

### 4.4 Complexity

All four paths are O(1) per sample; none loops. What differs is what
"exact" means for each:

| Path | Cost | Exactness |
|------|------|-----------|
| `kFullSphere` | one `asin` | exact (closed form) |
| `kNoRandom` | none | exact (no distribution) |
| `kGaussLegacy` | one Gaussian draw | intentionally NOT area-corrected — reproduces pre-fix legacy behavior |
| `kLutInverseCdf` | one uniform draw + fixed 8-step binary search | exact up to `LatLut` node resolution (257 nodes) |

There is no acceptance-rate concept left to report: the per-distribution
rejection loop this table used to describe (§5.2), and the near-pole
acceptance-rate collapse that motivated replacing it, are both gone.


## 5. Historical Context

### 5.1 Missing Jacobian Correction (pre-fix)

The original implementation sampled zenith directly from the distribution
without the sin(θ) Jacobian correction. The error magnitude depends on
the mean zenith:

| Mean Zenith | Distortion | Affected Types |
|-------------|-----------|----------------|
| 90° | 1.0× | Column halos |
| 45° | 1.4× | — |
| 10° | 5.8× | Plate halos near vertical |
| 0° | ∞ | Perfectly oriented plates |

The fix was implemented in 2026-04, alongside an evaluation that considered
and rejected vMF, Matrix Fisher, and Bingham distributions as alternatives.

### 5.2 Retired Per-Distribution Envelope/Rejection Architecture (pre-330.3)

Between the Jacobian fix (§5.1) and 330.3, the non-degenerate distributions
were served by three separate per-distribution paths instead of the single
LUT in §4.3 (full record: `doc/near-pole-area-measure-sampling.md`):

**Path: Rayleigh (colatitude < 0.5°).** Near a pole, the Jacobian and
Gaussian combine to give a Rayleigh distribution in the tangent plane.
The sampler drew 2D Gaussian samples, producing correct spherical density
without rejection. Exact, O(1) per sample.

**Path: Generic rejection.** For Gaussian (non-Rayleigh), zigzag, and
Laplacian:

1. Draw proposal θ from the base distribution.
2. Accept with probability cos(φ)/M, where φ is the latitude after
   `NormalizeLatitude` wrapping, and M is an envelope constant from
   `ComputeJacobianEnvelope()`:

| Type | Envelope M | Coverage |
|------|-----------|----------|
| Gaussian | cos(max(\|μ\| − 3σ, 0)°) | 99.7% |
| Zigzag | cos(max(\|μ\| − A, 0)°) | full amplitude |
| Laplacian | cos(max(\|μ\| − 5b, 0)°) | 99.3% |

Per-type proposal generation:

- **Gaussian:** Normal variate × σ + μ. O(1) per proposal.
- **Zigzag:** θ = |A · sin(2πU) + B|, U ~ Uniform(0,1). O(1).
- **Laplacian:** Inverse CDF: θ = μ − b · sign(U−0.5) · ln(1−2|U−0.5|). O(1).
- **Uniform:** Draw from [μ − w/2, μ + w/2]. O(1).

**Path: Deterministic.** No sampling needed (this is today's `kNoRandom`).

Measured acceptance rates for the rejection path:

- Near equator (zenith ≈ 90°): ~100% (cos φ ≈ 1)
- Mid-latitude (zenith ≈ 45°): ~70–80%
- Near pole (zenith ≈ 5°): ~30–40%
- Zigzag/Laplacian: similar rates, governed by the same cos(φ)/M formula

The near-pole acceptance-rate collapse was the reason this architecture was
replaced by the unified LUT in §4.3 — see `doc/near-pole-area-measure-sampling.md`
for the full design rationale and removal record.


## 6. References

- Noel, V. & Sassen, K. (2005). Study of planar ice crystal orientations
  in ice clouds from scanning polarization lidar observations. *J. Appl.
  Meteor. Climatol.*, 44(5), 653–664.
- Noel, V. & Chepfer, H. (2004). Study of ice crystal orientation in cirrus
  clouds based on satellite polarized radiance measurements. *J. Atmos.
  Sci.*, 61(16), 2073–2081.
- Stout, J. E. et al. (2024). Laboratory observations of the orientation
  dynamics of settling ice crystals. *Atmos. Chem. Phys.*, 24, 11133–11155.
- Zhong, X. et al. (2023). Column crystal canting and its implications for
  ice cloud radiative properties. *J. Atmos. Sci.*, 80(6), 1539–1556.
- Kostinski, A. B., Marshak, A. & Várnai, T. (2025). What can deep-space
  glints tell us about ice crystal orientations? *Front. Remote Sens.*,
  6, 1548902.
- Borovoi, A. G. & Kustova, N. V. (2009). Light pillars from plate and
  column ice crystals. *Geophys. Res. Lett.*, 36, L04804.
- Shoemake, K. (1992). Uniform random rotations. *Graphics Gems III*,
  124–132.
- Wood, A. T. A. (1994). Simulation of the von Mises Fisher distribution.
  *Commun. Statist. — Simul. Comput.*, 23(1), 157–164.
