[中文版](raypath-symmetry.zh.md)

# Raypath Symmetry: P, B, D Filter Toggles

This document explains the geometric reasoning behind the `symmetry` field in Lumice filters,
covering the two sources of raypath equivalence, the precise semantics of the P, B, and D
toggles, D's enabling condition, and the corresponding GUI behavior.

**Target audience**: advanced users who want to understand filter behavior at the crystal
geometry level, beta testers, and future contributors.

---

## 1. Background

Lumice traces light rays through ice crystal faces and accumulates the ray path as a sequence
of face numbers (e.g., `[3, 5]` for a prism-to-prism path). Two geometrically distinct ray
paths are **equivalent** if they produce the same observable halo pattern.

Exploiting raypath equivalence reduces the number of unique paths the simulator must track:
instead of storing every symmetric variant separately, `ReduceRaypath` maps all members of an
equivalence class to a single canonical representative. `ExpandRaypath` does the inverse —
regenerating all symmetric variants from the canonical form for output purposes.

The `symmetry` field on a filter is a bitmask of which equivalence relations to apply.
Setting it incorrectly (enabling symmetry that the actual orientation distribution does not
satisfy) conflates inequivalent paths and can produce inaccurate simulations.

---

## 2. Two Sources of Symmetry

### 2a. Crystal Geometric Symmetry (D6h — intrinsic)

A hexagonal ice crystal belongs to the D6h point group, which contains:

- **C6**: six-fold rotation about the c-axis (60° steps)
- **σh**: one horizontal mirror plane (perpendicular to the c-axis)
- **σv / σd**: six vertical mirror planes (alternating through prism faces and prism edges)
- Plus the combination of these (improper axes, inversion)

This geometric symmetry is **intrinsic** — it holds for any ice crystal regardless of how it
is oriented in the atmosphere.

Face numbering convention:

| Group | Faces |
|-------|-------|
| Basal (top/bottom) | 1 (top), 2 (bottom) |
| Prism (side) | 3 – 8 (six faces, 60° apart) |
| Upper pyramid | 13 – 18 |
| Lower pyramid | 23 – 28 |

### 2b. Orientation Distribution Symmetry (ensemble — conditional)

When a population of crystals has a specific orientation distribution, additional raypath
equivalences can arise at the **ensemble** level. These depend on how azimuth, zenith, and
roll are distributed:

- **Azimuth uniform 360°**: all orientations about the vertical are equally likely →
  C6 equivalence holds in the ensemble.
- **Zenith distribution symmetric** (e.g., plate crystals with both faces up/down equally):
  σh (horizontal mirror) equivalence holds.
- **Roll mean at a multiple of 30°** combined with azimuth uniform 360°: a specific vertical
  mirror σv or σd applies across the ensemble.

**The key insight**: P, B, and D are not properties of a single crystal. They are properties
of the *ensemble* of crystals described by the axis distribution. Only when the ensemble
distribution respects a symmetry does it make sense to enable the corresponding filter toggle.

---

## 3. P, B, D Toggle Semantics

### P — C6 Rotational Equivalence

Applies the six-fold rotational symmetry of the hexagonal prism about the c-axis. Under C6,
each prism face maps to the next (3→4→5→6→7→8→3), and the basal and pyramid faces rotate
correspondingly.

**Enabling condition**: azimuth distribution is uniform over 360°. This is the most common
case (freely falling or lightly oriented crystals) and is nearly always safe to enable.

**Effect**: the canonical raypath uses the smallest face permutation representative; six
rotationally equivalent paths collapse to one.

### B — Horizontal Mirror (σh)

Applies the horizontal mirror plane through the crystal's equator. Under σh:

- Basal faces: 1 ↔ 2
- Prism faces 3–8: unchanged (they straddle the mirror plane)
- Upper pyramid faces: 13 ↔ 23, 14 ↔ 24, 15 ↔ 25, 16 ↔ 26, 17 ↔ 27, 18 ↔ 28

**Enabling condition**: the zenith distribution is symmetric about 90° (neither top nor bottom
basal face is preferentially up). This is typically satisfied for column crystals with a
broad zenith distribution and for completely random orientations.

**Effect**: paths entering through the top basal become equivalent to paths entering through
the bottom basal; upper-pyramid paths become equivalent to lower-pyramid paths.

### D — Vertical Mirror (σv or σd)

Applies a single vertical mirror plane whose orientation is determined by the **roll mean**
of the axis distribution. Depending on the roll mean, the mirror is either a σv plane
(passing through two opposite prism face centers) or a σd plane (passing through two opposite
prism edges).

Under D, basal faces 1 and 2 are always fixed. Prism faces map according to the
σ-by-roll-mean formula (see §4). Pyramid faces follow the same prism mapping.

**Enabling condition**: see §4.

---

## 4. D Enabling Condition and σ Derivation

### Geometric Basis

In the Lumice rotation chain `R = Rz(az−π) · Ry(−zenith) · Rz(roll)`, the world-up
direction `(0,0,1)` maps to the crystal-frame vector `(cos roll, −sin roll, 0)`. This vector
rotates in the crystal XY plane as roll changes, sweeping through the prism faces in the
reverse C6 order: face 3 → 8 → 7 → 6 → 5 → 4 → 3 every 360° (or equivalently every 180°,
since faces 3 and 6 share the same mirror-plane axis).

When azimuth is uniform over 360°, the ensemble average is rotationally symmetric about the
vertical axis. If the roll distribution is also concentrated at a specific value (or
distributed symmetrically about it), the combined ensemble may respect a particular vertical
mirror plane determined by the roll mean.

### Enabling Condition

D is applicable when **both** of the following hold:

1. **Azimuth is uniform 360°**: `az_dist.type == Uniform && az_dist.std ≈ 360°`
2. **Roll mean is a multiple of 30°**: `roll_dist.mean mod 30° ≈ 0°`
   (tolerance ε ≈ 0.001°; applies regardless of roll distribution type)

When D is not applicable, the D toggle in the filter has no effect on the simulation even if
it is checked — the symmetry reduction is simply skipped.

### σ-by-Roll-Mean Formula

Let `mean` be the roll mean in degrees. Compute:

```
n = round(mean / 30°) mod 6       -- which 30° sector (0..5)
a = (6 − n) mod 6                 -- mirror parameter (0..5)
```

The D mirror maps each prism face as:

```
pri_idx     = face − 3            -- 0-based prism index (0..5)
pri_idx_new = (a − pri_idx) mod 6
face_new    = pri_idx_new + 3
```

Basal faces (1, 2) and the pyramid position indicator are unchanged; pyramid faces 13–18 and
23–28 follow the same prism mapping (shift face by ±10 as needed, preserve 1x/2x prefix).

### Reference Table

| n | roll mean (mod 180°) | a | Mirror type | Through | Prism fixed | Prism swaps |
|---|---|---|---|---|---|---|
| 0 | 0° | 0 | σv | face-3 axis | 3, 6 | 4↔8, 5↔7 |
| 1 | 30° | 5 | σd | edge 3–8 | — | 3↔8, 4↔7, 5↔6 |
| 2 | 60° | 4 | σv | face-8 axis | 5, 8 | 3↔7, 4↔6 |
| 3 | 90° | 3 | σd | edge 8–7 | — | 3↔6, 4↔5, 7↔8 |
| 4 | 120° | 2 | σv | face-7 axis | 4, 7 | 3↔5, 6↔8 |
| 5 | 150° | 1 | σd | edge 7–6 | — | 3↔4, 5↔8, 6↔7 |

Period is 180°: n=0 and n=6 (roll=0° and roll=180°) yield the same σv through the face-3
axis.

---

## 5. Typical Scenario Reference

| Crystal type | Az | Zenith | Roll mean | P | B | D |
|---|---|---|---|---|---|---|
| Plate, free falling (22° halo ring) | uniform 360° | Gauss ≈ 0° | uniform 360° | ✓ | ✓ | ✓ (D geometrically coincides with P; enabling both is harmless) |
| Plate, Parry arc | uniform 360° | Gauss ≈ 0° | 0° | ✓ | — | ✓ (roll mean = 0°, σv through face 3) |
| Column, random | uniform 360° | uniform 90° | uniform 360° | ✓ | ✓ | ✓ (D geometrically coincides with P) |
| Column, oriented (tangent arcs) | uniform 360° | Gauss ≈ 90° | Gauss σ ≈ 0°, mean = 0° | ✓ | — | ✓ |
| Pyramid (odd-roll config) | uniform 360° | Gauss ≈ 45° | Gauss mean = 15° | ✓ | — | ✗ (15° not a multiple of 30°) |
| Any, non-uniform azimuth | Gauss | any | any | ✗ | — | ✗ |

Notes:
- "–" means the toggle is typically not meaningful for this crystal type.
- When roll distribution is `uniform 360°`, D is geometrically equivalent to P (the ensemble
  has full rotational symmetry); enabling D alongside P is harmless but redundant.
- B is generally only meaningful when the zenith distribution treats up/down equally.

---

## 6. GUI Behavior

**D checkbox**: always remains enabled (never greyed out). The checkbox controls whether the
simulator attempts to apply D symmetry reduction; the simulator internally checks the
enabling condition and skips D if it is not satisfied.

**Informational indicator**: when the current axis configuration does not satisfy D's enabling
condition (az not uniform 360°, or roll mean not a multiple of 30°), a transparent `(i)`
button appears to the right of the D checkbox. Hovering over it shows the tooltip:

> D applies when azimuth = uniform 360° and roll mean is a multiple of 30°.
> Current config does not meet this condition, so D has no effect.

This design follows a "weak hint" principle: the user retains control and is not blocked from
checking D, but is informed when the toggle has no practical effect.

---

## 7. Out of Scope

The following are **not** covered by P, B, or D:

- **Complex filter symmetry**: filters of type `complex` compose multiple sub-filters;
  symmetry reduction across sub-filter boundaries is not implemented.
- **Entry/exit pairs with multiple values**: when an `entry_exit` filter lists multiple entry
  or exit faces, cross-face symmetry beyond what P/B/D express is not handled.
- **Non-standard crystal types**: custom face geometries (pyramidal crystals with unusual
  upper/lower index lists) may not map correctly under B; verify against your crystal config.

---

## See Also

- [Configuration Guide — filter section](configuration.md#filter)
- [Crystal Orientation Sampling](crystal-orientation-sampling.md)
- [Coordinate Convention](coordinate-convention.md)
