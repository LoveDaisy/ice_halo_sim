# Geometry-Randomization Performance — Cost Model & Frontier Map

Crystal geometry randomization (per-crystal shape sampled from a `Distribution`, e.g.
`height`/`face_distance` drawn per crystal) is a product differentiator: users want to
reproduce halo patterns from *populations* of slightly-different crystals, not one frozen
shape. Making that fast — reaching a target statistical fidelity in the least wall-clock —
is a moat. This document is the cost model and the lever map: read it before optimizing the
geometry-randomization path so you don't chase a phantom.

> **Read `geometry-randomization-value-and-measurement.md` alongside this.** That doc owns the
> *value* side (does randomization converge to the right image, and where the effect lives);
> this doc owns the *cost* side. The "equal-error throughput" metric below is a **variance-axis**
> quantity — it measures how fast, not whether it is right.

## Update — post closed-form landing (PR #214/#217/#218): C is superseded, the wall moved

The body below (lever map, "B and C inseparable", the topology-reuse analysis) is the reasoning
*as it stood on the old pipeline*. The closed-form crystal representation has since landed
(PR #214; see `crystal-geometry-representation.md`), which changes two load-bearing facts:

- **C (topology reuse) is not merely blocked — it is the wrong lever.** Closed-form construction
  makes the geometry *compute* ~92 ns, which is only ~5% of `MakeCrystal`'s cost; the other ~95%
  is `Crystal` *object* construction (~1.66 µs on Metal, ~2.5 µs on CUDA — host build 93%, H2D
  upload 7% measured warm). C optimized the 5%. The premise that made "B and C inseparable" —
  that construction *is* the reusable topology derivation — is dissolved: B alone now delivers a
  larger equal-error moat (≈28–40× on Metal) than the old projection expected. Do not re-attempt
  C. The real remaining lever is **shrinking `MakeCrystal` object construction** (cross-backend);
  its ROI is largest on multi-crystal scenes (see next).
- **The cost-model numbers below (10.12 µs `MakeCrystal`, 98.4% mesh re-derivation) are the old
  pipeline.** Post closed-form the wall is object construction, not topology re-derivation.

### K=64 penalty is scene-dependent — measured across scenes (not just single-crystal single-MS)

`geom_clock` was measured for its cost only on the single-crystal single-MS bench (≈0.37×). It
is **not** a universal factor. Across a scene spread (Metal `--benchmark`, best-of-3):

| scene | K=D / det | K=64 / det |
|---|---|---|
| single-MS 1-crystal (2048) | 1.17 | 0.36 |
| 2-MS 1-crystal (256) | 0.98 | 0.43 |
| 3-MS 7-crystal (2048) | 0.96 | 0.28 |
| 3-MS 8-crystal mixed-pyramid (2048) | 0.89 | 0.25 |

- **"Turning randomization on is ≈ free" (K=D / det ≈ 1) holds across scenes.**
- **The K=64 penalty is worse on multi-crystal / pyramid scenes** (0.25 vs 0.36). Construction is
  paid per `(crystal × MS-layer)` shape pool, so pool count grows *faster* than per-ray trace
  cost on heavy scenes — refuting the naive "heavier trace dilutes the penalty" expectation. A
  performance-measurement config must therefore include a multi-crystal multi-MS scene, and the
  "shrink object construction" lever's ROI is largest exactly there.

## The one thing to internalize first: the metric is equal-error throughput

Raw ray throughput is **not** the objective and is **already adequate**. The objective is
**equal-error throughput** — the fastest route to a target error in a *filtered* observable
(filter match-count or full-sphere energy; never a windowed image energy, which mismeasures
directional signal). Two facts make this the right metric:

- Geometry randomization's value lives *below the filter*. On the full image, randomizing
  `height`/`face_distance` versus freezing them is a ~26 dB noise floor with flat variance —
  the halo *angles* are mathematically immune to `h`/`d` because those parameters only shift
  plane offsets, never rotate face normals (see `FillHexCrystalCoef` in `src/core/`). Add a
  raypath filter and the reweighting that was cancelling out across the whole image becomes
  the entire signal.
- The geometry clock is a **variance** knob, not a bias knob. Sharing one sampled shape
  across `K` rays leaves the *mean* unbiased (cluster sampling is exact) and only inflates
  *variance*; variance is bought back with more rays. So low-fidelity geometry and extra rays
  are the same currency, and can be traded directly.

Notation (used throughout): **`K` = rays sharing one sampled shape** (physics; ideal `K`=1;
the physics-optimal `K*` measured 8–64, backend-independent). **`D` = rays per dispatch per
backend. `D/K` = crystals per batch = cost.**

## The cost model

At the physics optimum `K*`=64 with a GPU dispatch `D`=262144, a batch needs `D/K*` = 4096
**distinct** crystal shapes. Today both GPU backends run at `K = D` — one shape per dispatch
(`D/K` ≡ 1) — which is far above `K*`, so variance-per-ray is high. Closing that gap means
building thousands of shapes per batch, and **host `MakeCrystal` is the wall**: ~10.12 µs per
prism, of which 98.4% is mesh construction re-deriving a topology that never changes across
random draws. 4096 shapes/batch of that is tens of milliseconds of pure host construction,
dwarfing the trace.

## ⚠️ Premise correction: the "15.7× slower when randomized" number is a measurement artifact

When the CUDA geometry-randomization path was unfrozen (PR #209), a naive `--benchmark`
comparison showed random configs running ~15.7× slower than deterministic ones (296k vs
4.65M rays/s). **That gap is not a per-batch geometry cost — it is a cold-vs-warm CUDA
context-init measurement artifact.** Mechanism:

- The process's first CUDA call (a `cudaFree` in the pool build) triggers lazy CUDA context
  initialization: ~1.3s cold, ~62ms warm (a subsequent process on an already-warm driver).
- `--benchmark`'s `active_sec` reads 0 on short / too-few-drains runs, so `rate_basis` falls
  back to `wall` (`src/main.cpp`), and `wall` includes that context init. The "15.7×" was a
  *cold* randomized run (paying the 1.3s init) compared against a *warm* deterministic run
  (which had already paid it).
- Warm-vs-warm at the current one-shape-per-dispatch clock, randomized ≈ deterministic (a 4M
  run: 43.9M vs 39.5M rays/s; randomized even nominally faster). The steady per-batch rebuild
  is ~50 µs, and the malloc/free churn inside it is ~17 µs — negligible.

**Consequence:** a "persistent device buffer to kill the per-batch malloc/free churn" would
optimize a cost that does not exist (~17 µs/batch at `K`=1, ~2.6% even at `K*`=64 where
`MakeCrystal` dominates). Do not build it. The real levers are below.

## Lever map

| Lever | Ceiling | Risk | Stage |
|-------|---------|------|-------|
| Trustworthy measurement (warm context, active/steady basis; reject short-run `wall_fallback`) | removes the artifact | low | 0 — enabling, cheap |
| ~~Persistent buffer to kill per-batch churn~~ | targets a cost that isn't there | low | dropped |
| **B — per-ray K-shape pool** (decouple `K` from `D`; build `D/K*` shapes/batch, index per ray) | reaches `K*`=64 | medium (engineering) | 1 |
| **C — topology reuse** (cache plane-triple→vertex incidence; 10.12 µs → 127 ns/prism) | ~79×/prism; makes `K*`=64 construction ~5.7% of trace instead of ~453% | high — geometry predicates in `src/core/math.cpp`, same numerical-robustness zone as prior missing-face bugs (read `numerical-robustness.md` first) | 1 (inseparable from B) |
| Async double-buffer (hide residual construction + H2D behind the trace window) | hides residual | medium | 2 |

**B and C are inseparable.** B alone cannot deliver the win — building `D/K*` shapes per batch
runs straight into the host construction wall, so B's throughput is eaten unless C makes each
shape cheap. Direct measurement: at `K*`=64, host construction is a ~13× loss without reuse;
topology reuse cuts a single construction from 10.12 µs to 127 ns, dropping `K*`=64
construction from ~453% of trace to ~5.7%.

## "Have we squeezed it to the limit?" — the checklist

1. **Physics-optimal `K*` is 8–64** (backend-independent; the payoff is a physical quantity). ✅ measured.
2. **Construction wall at `K*`=64 is ~453% of trace, dropping to ~5.7% with topology reuse.** ✅ measured (`bench/bench_crystal.cpp` is the acceptance tool for the reuse ceiling).
3. **Per-crystal construction cost is ~10.12 µs (98.4% mesh construction of a fixed topology); ceiling 127 ns.** ✅ measured.
4. **CUDA per-batch H2D bandwidth at `K*`=64** (~665 KB/batch, ~12 GB over a full run; topology reuse does not reduce bandwidth — orthogonal). ✅ measured: ~210 MB/s, ~1% of PCIe — not a constraint.
5. **Residual of the 79× topology-reuse ceiling once a *sufficient* validity predicate is paid** — face-count conservation is necessary but not sufficient (the hyperplane arrangement's combinatorial structure can change with face count fixed). ✅ measured, and it **found the blocker** — see below.

## B is built; C is blocked on a correctness question, not a performance one

**B (per-ray K-shape pool) is implemented and verified on both GPU backends** behind
`LUMICE_GPU_GEOM_CLOCK` (default 0 = disabled = the historical one-shape-per-dispatch
behavior, bit-for-bit). Building it surfaced and fixed two real defects: absolute pool
indices leaking into consumers expecting per-crystal local indices (silently corrupting
entry/exit filters once a batch held more than one shape), and a CUDA pool that stayed
frozen at a degenerate size for `K > 0` sessions.

**C (topology reuse) measured well and still cannot land as designed.** Cost side, on the
`bench/bench_crystal.cpp` prototype: a candidate validity predicate (re-solve each cached
plane triple, then half-space-test every cached vertex against every plane) costs only
+62 ns for a prism / +279 ns for a pyramid, leaving a **residual of 53.6× (prism) / 92.8×
(pyramid)** against full construction. The predicate is emphatically not the bottleneck.

Sufficiency side, which decides whether it may ship: fuzzing the predicate's verdict against
the production solver over 20000 perturbed samples per σ found **false accepts** — the
predicate declares the cache valid while the topology has genuinely changed (confirmed as a
real 12 → 10 vertex count change, two vertices merging while both stay inside every
half-space, so a half-space test cannot see it). Augmenting it with plane-concurrency and
vertex-coincidence checks narrowed the leak but did not close it; the remainder sits on the
degenerate boundary where the predicate's tolerance and the production solver's dedup
tolerance disagree. Separately, a pyramid apex is a vertex where more than three planes meet
by construction, which breaks the "cache one plane triple per vertex" model outright.

Tuning the fuzz until the count reaches zero would be overfitting one seed, not a sufficiency
proof. A provably sufficient predicate would have to align exactly with the production
solver's degenerate dedup behavior — and that solver's tolerance is itself an empirically
swept constant, so this is a regress, not an engineering task with an end.

**Conclusion: the blocker is structural, not numerical-hygiene.** The validity question only
exists because the crystal's combinatorial structure is discovered numerically rather than
being known from the parametrization. See `crystal-geometry-representation.md` for that
diagnosis and the redesign direction, under which the predicate — and the cache it guards —
have no reason to exist, at a construction floor below the 127 ns reuse was reaching for.
C should not be re-attempted on the current representation.

## Measurement discipline (learned the hard way)

- **Warm the CUDA context** before timing (a throwaway iteration), or the first-call init pollutes the number.
- **Interleave** the two arms; never compare back-to-back grouped runs on a shared machine.
- **Use a large enough `ray_num`** that trace dominates fixed overhead.
- **Trust only `rate_basis` = `steady`/`active`; treat `wall_fallback` as setup-polluted and discard it.**
- Perf baseline is legacy CPU (state the denominator); the GPU throughput bar is hardware/competitor class, not merely "beats legacy".

See also: `seam-design.md` (§3.2 per-ray shape pool, §5 single-engine three-clock target),
`gpu-single-engine-implementation.md` (as-built), `numerical-robustness.md` (the geometry-predicate
high-risk zone), `performance-testing.md` (benchmark caveats), `crystal-orientation-sampling.md`.
