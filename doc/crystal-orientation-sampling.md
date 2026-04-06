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

Three paths are selected automatically:

**Path 1: Rayleigh (colatitude < 0.5°).** Near a pole, the Jacobian and
Gaussian combine to give a Rayleigh distribution in the tangent plane.
The sampler draws 2D Gaussian samples, producing correct spherical density
without rejection. Exact, O(1) per sample.

**Path 2: Generic rejection.** For Gaussian (non-Rayleigh), zigzag, and
Laplacian:

1. Draw proposal θ from the base distribution.
2. Accept with probability cos(φ)/M, where φ is the latitude after
   `NormalizeLatitude` wrapping, and M is an envelope constant.

Envelope constants from `ComputeJacobianEnvelope()`:

| Type | Envelope M | Coverage |
|------|-----------|----------|
| Gaussian | cos(max(\|μ\| − 3σ, 0)°) | 99.7% |
| Zigzag | cos(max(\|μ\| − A, 0)°) | full amplitude |
| Laplacian | cos(max(\|μ\| − 5b, 0)°) | 99.3% |

**Path 3: Deterministic.** No sampling needed.

### 4.3 Per-Type Proposal Generation

- **Gaussian:** Normal variate × σ + μ. O(1) per proposal.
- **Zigzag:** θ = |A · sin(2πU) + B|, U ~ Uniform(0,1). O(1).
- **Laplacian:** Inverse CDF: θ = μ − b · sign(U−0.5) · ln(1−2|U−0.5|). O(1).
- **Uniform:** Draw from [μ − w/2, μ + w/2]. O(1).

All proposals pass through the Jacobian rejection step (Path 2).

### 4.4 Acceptance Rates

- Near equator (zenith ≈ 90°): ~100% (cos φ ≈ 1)
- Mid-latitude (zenith ≈ 45°): ~70–80%
- Near pole (zenith ≈ 5°): ~30–40%
- Zigzag/Laplacian: similar rates, governed by the same cos(φ)/M formula


## 5. Historical Context

The original implementation sampled zenith directly from the distribution
without the sin(θ) Jacobian correction. The error magnitude depends on
the mean zenith:

| Mean Zenith | Distortion | Affected Types |
|-------------|-----------|----------------|
| 90° | 1.0× | Column halos |
| 45° | 1.4× | — |
| 10° | 5.8× | Plate halos near vertical |
| 0° | ∞ | Perfectly oriented plates |

The fix was implemented during explore-zenith-sampling (2026-04), which
also evaluated and rejected vMF, Matrix Fisher, and Bingham distributions.


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
