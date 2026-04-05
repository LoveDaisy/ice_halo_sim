# Crystal Orientation Sampling

This document describes how Lumice samples crystal c-axis orientations
for Monte Carlo ice halo simulations.

## Physical Model

Ice crystals in the atmosphere have preferred orientations determined by
aerodynamic forces and gravity. The orientation of a crystal is decomposed
into two independent components:

1. **C-axis direction** (zenith + azimuth): the direction the crystal's
   principal symmetry axis points. Gravity constrains the zenith angle;
   azimuth is uniform (no preferred horizontal direction).
2. **Roll** (rotation around c-axis): independent of axis direction.

This decomposition is physically correct, not an approximation. The c-axis
direction distribution forms a **zonal band** on the sphere (concentrated
in latitude, uniform in azimuth), NOT a rotationally symmetric cap.

## Configuration

In JSON configuration, crystal orientation is specified as:

```json
"axis": {
  "zenith": { "type": "gauss", "mean": 90, "std": 0.5 },
  "roll": { "type": "uniform", "mean": 0, "std": 360 }
}
```

- `zenith.mean`: angle from vertical in degrees (0 = c-axis vertical, 90 = horizontal)
- `zenith.std`: spread in degrees (Gaussian standard deviation)
- Internal conversion: `latitude_mean = 90 - zenith_mean`

## Sampling Method

### Target Distribution

For Gaussian zenith, the target distribution of the colatitude `theta` is:

```
p(theta) proportional to G(theta - theta0, sigma) * sin(theta)
```

where:
- `theta` is colatitude (= zenith angle in radians)
- `theta0` is the target colatitude
- `sigma` is the standard deviation in radians
- `sin(theta)` is the spherical area element Jacobian

The `sin(theta)` factor is essential: without it, sampling in spherical
coordinates produces incorrect density at the poles (`sin(theta) -> 0`
as `theta -> 0`), over-concentrating samples near the pole.

### Why Not von Mises-Fisher (vMF)?

vMF produces a rotationally symmetric **cap** around the target direction.
Ice crystals need a **zonal band** (concentrated in zenith, uniform in
azimuth). At the equator (`theta0 = pi/2`), vMF would constrain azimuth
to a small range, which is physically incorrect. vMF only matches our
requirements at the poles, where a band degenerates to a cap.

### Implementation: Hybrid Rayleigh + Rejection

The function `SampleSphericalPointsSph(AxisDistribution)` in `math.cpp`
uses two paths:

**1. Rayleigh path** (when `colatitude_center < 0.5 deg`):

For the c-axis nearly at the pole, use 2D Gaussian in the tangent plane:

```
dx, dy ~ N(0, sigma)
colatitude = sqrt(dx^2 + dy^2)
latitude = sign(mean) * (pi/2 - colatitude)
```

This produces the Rayleigh distribution `p(theta) ~ theta * exp(-theta^2 / 2*sigma^2)`,
which equals `sin(theta) * G(theta, sigma)` for small `theta` (since `sin(theta) ~ theta`).
Exact at `theta0 = 0`, no rejection needed, O(1) per sample.

**2. Optimized rejection** (all other cases):

```
M = cos(max(|latitude_mean| - 3*sigma, 0))
do:
    phi = sample from Gaussian(latitude_mean, sigma)
    clamp phi to [-pi/2, pi/2]
while uniform() >= cos(phi) / M
```

Acceptance rate:
- Near equator (`zenith ~ 90 deg`): ~100% (cos(phi) ~ 1)
- Mid-latitude (`zenith ~ 45 deg`): ~70-80%
- Near pole (`zenith ~ 5 deg`): ~30-40%

The envelope `M = cos(max(|mean| - 3*sigma, 0))` is derived from the
explore-verified colatitude formula `M = sin(min(theta0 + 3*sigma, pi/2))`
via the identity `sin(pi/2 - x) = cos(x)`.

### Non-Gaussian Types

- `kNoRandom`: returns the fixed mean value directly (no sampling)
- `kUniform`: uses original logic without Jacobian correction (known
  limitation for `kUniform` latitude + non-`kUniform` azimuth combinations,
  which are extremely rare in practice)
- Fully uniform (`kUniform` latitude AND azimuth): takes a separate code
  path using correct uniform-on-sphere sampling (`asin(u)` method)

## Historical Context

The original implementation sampled latitude directly from `Gaussian(mean, std)`
without the `cos(phi)` Jacobian. The impact varied by configuration:

| Zenith | Distortion factor | Angular std bias |
|--------|-------------------|------------------|
| 90 deg | 1.0x              | < 0.1%           |
| 45 deg | 1.4x              | -0.5%            |
| 10 deg | 5.8x              | -8.8%            |
| 0 deg  | infinity           | -7.9%, density 8x |

The fix was identified through explore-zenith-sampling (2026-04), which
also evaluated and rejected vMF, Matrix Fisher, and Bingham distributions
as alternatives. The zonal band model and Jacobian correction approach
were confirmed as mathematically rigorous and computationally efficient.

## References

- Explore summary: `scratchpad/scrum-sim-sampling-research/explore-zenith-sampling/SUMMARY.md`
- Shoemake (1992): Uniform random rotations (SO(3) sampling background)
- Wood (1994): Simulation of the von Mises-Fisher distribution (vMF sampling, rejected for this use case)
