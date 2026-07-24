# Geometry Randomization: Value & Measurement

> Status: **framework + measurement methodology + stage-1 findings**, plus a **blueprint**
> variance law (§3) not yet fully built. Read this before designing any experiment that claims
> to measure "how good" geometry randomization is, or before adding a geometry-randomization
> quality/throughput gate. Companion to `geometry-randomization-perf.md` (the cost/throughput
> side) and `crystal-geometry-representation.md` (the closed-form representation this rests on).
> 中文速记见末尾。

## Why this document exists

Crystal geometry randomization — sampling each crystal's shape from a `Distribution` on
`height`/`face_distance` instead of freezing one shape — is a product differentiator: real
halos come from *populations* of slightly-different crystals, not one perfect crystal. The
capability has landed on all three backends (closed-form representation PR #214; per-ray
K-shape pool productized as `scene.geom_clock`, Metal PR #217, CUDA PR #218). But the thing
that makes it *worth having* had, until this work, never been measured. This document names
the two orthogonal axes it must be measured on, gives the methodology for the axis nobody had
measured, and records what that measurement found.

## 1. Two orthogonal axes — and the measurement sin of conflating them

There are **two independent questions**, and mixing them is the original measurement error:

| Axis | Question | Knob | What it is |
|---|---|---|---|
| **Variance / efficiency** | how *fast* do we converge? | `K` = rays sharing one shape (`geom_clock`) | cluster sampling: unbiased mean, `K` only inflates variance |
| **Bias / correctness** | do we converge to the *right* image? | randomization ON vs OFF (deterministic) | deterministic converges to a physically *wrong* image; randomization to the right one |

- `K = D` (deterministic-per-dispatch) vs `K = 64` is a **variance** comparison: both arms
  converge to the *same* mean image; only the convergence *speed* differs.
- Randomization **ON vs OFF** is a **bias** comparison: a single deterministic crystal
  converges to an over-sharp image with spurious caustics; a randomized population converges
  to the correctly-broadened, filled-in image that real polydisperse clouds produce.

**The moat number measured before this work is a variance-axis number.** The headline
"equal-error speedup" (filtered scalar variance ratio → ≈28–40× on Metal, ≈15× equal-dispatch
on CUDA) answers "how fast", **not** "is it right". The product sell — physical accuracy —
lives entirely on the unmeasured bias axis. Any experiment that renders one image and computes
an RMSE against that arm's own converged mean is measuring variance, and is *structurally
blind* to the bias axis (it never compares the two different converged means to each other).

## 2. What the bias axis measures, and where the effect lives

Deterministic geometry does not merely produce a *noisier* image — it produces a *different,
physically wrong* one: over-sharp caustics, too-narrow feature widths, spurious structure.
Shape randomization is the physical correction. Measured on the textbook 22° halo (random
orientation, monochromatic to isolate the effect from chromatic dispersion), shape
randomization at `face_distance` σ = 0.5:

| Observable | OFF → ON | direction |
|---|---|---|
| 22° halo **peak** | ×0.79 (−21%) | softened |
| inside the halo [10–20°] (dark region) | ×1.5 (+50%) | filled in |
| antisolar weak peak [150–158°] | ×0.43–0.52 (halved) | suppressed |

Two conclusions that reshape the sell:

1. **The effect is clean polydispersity, not a degenerate-crystal artifact.** A `gauss` and a
   bounded `uniform` `face_distance` at *matched statistical σ* give value-for-value identical
   profiles. Since `uniform` has a far lower degenerate-crystal rate than `gauss`'s long tail,
   any degenerate-scatter contribution would diverge at large σ; it does not. The softening /
   filling / suppression are genuine ensemble physics.
2. **The effect is smallest on the brightest feature.** On the dominant 22° peak it is only
   −21%; in the dark region and the faint antisolar feature it is +50% / −50%. This is the
   *same* conclusion the variance axis reached ("full image is a wash; the filtered/faint
   observable is where it matters"): **both axes point at faint / away-from-peak observables.**
   To *demonstrate* the sell, pick a faint feature, not the 22° peak.

### 2.1 Plate parhelic circle — a richer bias載体

Plate orientation (c-axis vertical) with the sun on the horizon produces a parhelic circle
along the horizon; measuring energy vs azimuth along it (≡ scattering angle along the circle)
shows an even richer set of ON/OFF differences than random orientation: the sundog softens
(×0.85), the parhelic-circle mid-section fills (×1.88 over 50–100°), the 145–160° weak peak is
suppressed (×0.58), and energy redistributes toward the antisolar point (×2.5 over 163–180°).

One feature runs **opposite** to all the others: the **120° parhelion brightens** (~×6.76
integrated) under randomization. It is a real brightness difference (visible in rendered
images, not a binning artifact), but it is a *vertical* feature and the horizon-band measure
captures only a slice of it, so the exact factor is band-dependent and the mechanism (broken
hexagonal symmetry opening new ray paths vs. elevation-broadening into the band) is unresolved
— tracked as an open item, not a load-bearing claim.

## 3. The variance law (blueprint)

> This section is a **blueprint**: the law and its invariants are derived and checked against
> the current code, but a `K*`-calibration harness built on it does not yet exist.

Let total rays be `N`, a shape shared across `K` rays, so `M = N/K` distinct shapes over the
run. Split randomness into the **shape** `s` and everything resampled per ray `w` (wavelength,
orientation, incidence). The law of total variance gives, per pixel:

```
Var(K) = ( σ̄²_w + K · σ²_b ) / N
```

- `σ̄²_w` = within-group variance (variance at fixed shape); `σ²_b` = between-group variance
  (how a pixel's expectation moves with the shape).
- `K = 1` is the variance optimum (no pooling penalty, most expensive). Pooling penalizes only
  the between-shape term, linearly in `K`; the within-shape term is `K`-independent.
- Deterministic geometry is `K → ∞`: `σ²_b` degenerates into an **irreducible bias** (the wrong
  converged image). This is the one regime where adding rays does not help — the reason to keep
  randomization ON even at `K = D`.

Efficiency × convergence is a product. Wall-clock to reach target variance `V`, with `C` =
per-shape construction cost and `c` = per-ray trace cost:

```
W(K) = (1/V) · (σ̄²_w + K·σ²_b) · (C/K + c)   ⇒   K* = sqrt( (σ̄²_w / σ²_b) · (C/c) )
```

**Optimal `K` = geometric mean of the variance ratio and the cost ratio.** Costly construction
(CUDA H2D) → larger `K*`; shape-dominated observable (faint/filtered, large `σ²_b`) → smaller
`K*`. This *unifies* the observable-dependence: the "degree to which `K` matters" is exactly
`σ²_b / σ̄²_w`. The bias-axis multi-seed data is *simultaneously* the `K*`-calibration data
(fixed-shape-across-other-randomness separates `σ̄²_w`; shape-mean spread separates `σ²_b`).

### 3.1 The three structural invariants (guardrail red-lines)

The clean placement of `K` as a linear coefficient on the between-shape term survives the real
nested-clock pipeline (wavelength pool, per-`(MS-layer, crystal)` shape pool, orientation LUT)
**only** while these hold — verified against the current code:

1. **Shape pool is i.i.d.-drawn** (each slot an independent `MakeCrystal`).
2. **Per-MS-layer pools are independent, shapes resampled each layer** (transit does not carry
   a shape across layers — only the wavelength is a photon-lifetime label). Shapes decorrelate
   with scattering depth.
3. **Non-shape random streams are PCG-isolated from the shape stream** (e.g. the wavelength
   stream's own nonce). Sampling-level `wl ⊥ shape`; any `f(wl, shape)` coupling lives only in
   the integrand and is absorbed by the total-variance identity.

Changes that would **break** the clean placement (do not do these without re-deriving the law):
- **Stream collision** — sharing PCG state across wl / orientation / shape (this has bitten
  before: a green tint from a wavelength draw colliding with the orientation stream, fixed by an
  independent nonce). Stream isolation is the survival charm.
- **Cross-layer shape reuse** — letting a photon carry its shape across MS layers to save
  construction cost breaks invariant 2 and destroys depth decorrelation.
- **Non-shape sampling that depends on the shape assignment.**

The only genuine coupling is on the *efficiency* side, not the variance side: `K*`'s throughput
model assumes `C` is per-shape constant; degenerate geometry that triggers a fallback makes `C`
shape-dependent and spiky, distorting `T(K)` and `K*` (not `K`'s variance placement).

## 4. Measurement methodology

The reusable recipes, so the next person does not re-derive them:

- **Radial profile in scattering-angle space, not pixel registration.** Bin raw linear-Y by
  angular distance from the sun; feature geometry (inner-edge sharpness, FWHM, peak) is
  calibration- and projection-invariant and comparable to literature, whereas per-pixel
  registration against a photo is defeated by unknown radiometric calibration, haze, and crystal
  population.
- **Sun at zenith + `rectangular` projection ⇒ scattering angle = 90 − latitude**, independent
  of longitude, so every image *row* is one scattering angle sampled by the full image width.
  This gives uniform sampling across the whole [0,180°] range and eliminates the few-pixel
  pole artifact that a sun-on-horizon setup produces at the antisolar point. Projection math:
  `rectangular` maps `lat = (H/2 − py)/scale`, `scale = min(W/2,H)/π`
  (`lens_proj_build.hpp` `ComputeScaleAz0`).
- **Plate + sun on horizon ⇒ parhelic circle = the equator row**; energy vs azimuth is a direct
  read of that row. Plate orientation predicate: zenith gauss mean 0, std < 10, azimuth full
  360 uniform (`src/gui/axis_presets.hpp` `ClassifyAxisPreset`).
- **`uniform` vs `gauss` at matched statistical σ** to separate clean polydispersity from
  degenerate-tail artifacts. `uniform`'s `std` parameter is the *full width*
  (`(GetUniform()−0.5)·std + mean`, `src/core/math.cpp`), so match a `gauss` σ with a `uniform`
  full width of `σ·√12`.
- **Compare converged means with enough rays** (the bias axis compares two different converged
  images; at ~20–40M rays the mono profile is seed-stable — 3-seed ≡ 1-seed).
- **Integrated energy over a feature window, not the peak bin**, and **check per-bin pixel
  population** before trusting a value. A razor-sharp caustic reads as a single-bin spike that
  windowed integration and a pixel-count sanity check will expose (this caught a spurious
  antisolar "peak" that was 8 pixels at the θ=180° pole).
- **Raw un-clipped XYZ** via the C API (`LUMICE_GetRawXyzResults`) with `sim_seed` control, not
  the 8-bit JPG/PNG the CLI writes. **Trap:** on a *no-filter* scene the C API's
  `unfiltered_xyz_buffer` is a `0x1` sentinel (non-null junk) — read `xyz_buffer` (which is the
  full image when nothing is filtered); dereferencing the sentinel segfaults.

## 5. The two configs this pins down

- **Performance-measurement config must include a multi-crystal, multi-MS scene.** The K=64
  throughput penalty is scene-dependent and *worst* on multi-crystal scenes (§ in
  `geometry-randomization-perf.md`); a single-crystal single-MS bench understates it.
- **Physical-correctness config** = `rectangular` full-sky + sun at zenith (radial θ_s profile)
  or sun on horizon with plate orientation (parhelic circle), with feature loci at the 22°
  halo's dark interior, the sundog, the antisolar weak peak, and the parhelic circle — the
  faint/away-from-peak observables where the bias effect is largest.

## 6. Open items

- **Stage-2 external anchor.** Stage-1 shows randomization *softens* toward the right direction;
  to show it converges to the *correct* profile needs an external ground truth — a literature
  22° halo radial intensity curve is a cleaner first anchor than a real photo (no unknown
  radiometric calibration / crystal population).
- **120° parhelion anomaly** (§2.1) — mechanism unresolved; low priority.
- **`K*(config)` auto-calibration** from §3's law, reusing the bias-axis multi-seed harness.
- **Shrink `MakeCrystal` object construction** — the cross-backend construction wall's real
  lever; its ROI is largest exactly on the multi-crystal scenes where K=64 is worst (see
  `geometry-randomization-perf.md`).

---

## 中文速记

- **两根正交轴，混轴是度量原罪**：方差/效率（`K`=D vs 64，收敛多快，同一均值）vs 偏置/正确性（ON vs OFF，
  确定性收敛到物理错的图）。**已有 moat 数字（28–40× / 15×）是方差轴的**，卖点（物理准确）在没测过的偏置轴。
- **偏置发现**：确定性 = 过锐/伪焦散；随机化软化主峰（−21%）/ 填充暗区（+50%）/ 抑制对日弱峰（减半）。
  **gauss≡uniform 同 σ 逐值 ⇒ 干净多分散非退化背景**；**效应集中 faint/away-from-peak**（两轴同指向）。
  plate 幻日环更丰富；120° 幻日反常变亮（未解，记 open item）。
- **变量定律（blueprint）**：`Var(K)=(σ̄²_w+K·σ²_b)/N`，`K*=√((σ̄²_w/σ²_b)·(C/c))`；靠三结构不变量
  （i.i.d. 抽 / 各 MS 层独立重采 / PCG 流隔离）保 `K` 干净线性定位；护栏红线 = 流碰撞 / 跨层复用形状 / 采样依赖形状。
- **方法论**：散射角空间剖面（非像素配准）/ 太阳天顶→θ_s=90−lat 逐行满采样 / plate+地平线→幻日环=赤道行 /
  uniform-vs-gauss 同 σ disentangle / 积分能量+像素数纪律 / C API 原始 XYZ（no-filter 读 `xyz_buffer`，
  `unfiltered_xyz_buffer` 是 0x1 哨兵）。
- **两 config**：性能须含多晶多MS；正确性 = rectangular+太阳（天顶/地平线），特征位取暗区/sundog/对日弱峰/幻日环。
