# Numerical Robustness Conventions (Geometry)

> Status: conventions distilled from the extreme-wedge pyramid bug family (PR #132/#133/#135)
> and the follow-up predicate audit (PR #137). Read before adding or modifying any
> geometric predicate (coplanarity, face matching, intersection, degeneracy test).
> 中文速记见末尾。
>
> ⚠️ **These conventions treat the symptom.** They describe how to survive the current
> geometry pipeline, in which the combinatorial structure of the crystal is discovered
> numerically and therefore needs tolerances at every step. The *structural* cause of that
> need — and a representation that would not require it — is in
> `crystal-geometry-representation.md`. Read that one before redesigning; read this one
> before editing.

## The failure family

Four bugs in weeks shared one root cause: **an absolute epsilon applied to a
scale-variant geometric quantity, which breaks in extreme or degenerate geometry.**

Extreme ice-crystal configs make geometric quantities collapse toward the tolerance:
- **Inter-face angles shrink** with wedge angle: at wedge 88° adjacent upper-cone face
  normals dot to `0.99939`, defeating a `dot > 1 - 1e-3` (=0.999) "same plane" test.
- **Crystal thickness** is `prism_h=0` pyramids is `~0.02-0.04` — comparable to the
  `kFloatEps = 1e-5` length tolerances used in membership tests.
- **Areas scale as length²**, determinants scale as `|n1||n2||n3|` — doubly/triply
  sensitive to shrinking geometry.

The bugs and their fixes:

| Bug | Site | Anti-pattern | Fix |
|---|---|---|---|
| task-276 (#133) | `crystal.cpp::BuildPolygonFaceData` | `dot > 1-1e-3` first/threshold triangle→plane grouping | **argmax** + sanity floor; geometry-gen in **double** |
| task-278 (#135) | `simulator.cpp::PolygonFaceOfTri` | same `dot > 1-1e-3` first-match (missed sibling) | **argmax** |
| task-275 (#132, superseded) | next-face `denom > kFloatEps` | half-space test, thin-slab scale | root cause was 276/278; membership-check deferred to backlog |
| task-277 (canceled) | proxy metric (`abs(-0.002)` "inside hull") | absolute tolerance as proxy in thin slab | use ground truth, scale-relative measures |

## The conventions (best practices)

### 1. Prefer **argmax over absolute thresholds** for "find the matching X"
When assigning a triangle to its plane / face, or classifying a normal among candidates,
pick the **single best match (max dot)**, not "the first / any candidate above a fixed
bar." Argmax needs no knowledge of the geometry's scale and is robust as angular
separations shrink. Keep a **sanity floor** (`kFaceCoplanarFloor = 1e-2`, ~8.1° slack)
only to reject genuine non-matches, never as the selector.

✅ Golden templates already in the tree:
- `crystal.cpp::BuildPolygonFaceData` — argmax triangle→plane assignment.
- `simulator.cpp::PolygonFaceOfTri` — argmax triangle→polygon-face.
- `crystal.cpp::FillHexFnMap` pri loop — `p > p_comp` is argmax over the 6 prism refs.

### 2. Prefer **relative / normalized tolerances** for scale-variant quantities
- **Areas**: compare relative to a characteristic area, not an absolute floor.
  ✅ `crystal.cpp:699`: `face_area < 1e-6f * max_tri_area`.
- **Determinants** (triple/cross products): normalize by the operand magnitudes —
  `|det| / (|n1||n2||n3|)` — or guarantee unit-normalized inputs before the absolute
  test. A bare `|det| < 1e-10` on unnormalized plane normals is scale-variant.
  ⚠️ Known debt: `SolvePlanes`/`SolvePlanesD` (`math.cpp`), inline `solve_planes_d`
  (`geo3d.cpp`) still use absolute det thresholds (see explore-279 catalog A1-A4).
- **Half-space / point-on-plane** tests assume unit-normalized plane normals; if that
  isn't enforced upstream, the absolute distance tolerance is in the wrong units.

### 3. Geometry **generation** runs in **double**, hot tracing stays float
Mesh assembly / vertex solving is not a hot path. Doing it in double pushes the
breakdown threshold far out (task-276: extreme-wedge vertex collapse cliff moved from
~88.5° to ~89.99°). The per-ray trace loop stays float for throughput.

### 4. **One predicate, one implementation** (no missed siblings)
task-278 existed only because `PolygonFaceOfTri` was a hand-copied sibling of
`BuildPolygonFaceData` that didn't get the argmax fix. When the same geometric question
("which plane does this triangle belong to?") is answered in two places, they **will**
drift. Prefer a shared helper; if a backend boundary forces duplication (e.g. the Metal
`tri_to_poly` kernel mirroring the CPU function), add a `// must match X` comment and a
parity test, and fix both together.

Convergence log (geometry-gen double-precision predicates):
- `SolvePlanesD` — converged in task-280.4 (`geo3d.cpp` inline copy removed; single source `math.hpp::SolvePlanesD`).
- `IsInPolyhedron3D` — converged in task-geometry-predicate-single-source (`geo3d.cpp::is_in_polyhedron_d` lambda removed; single source `math.hpp::IsInPolyhedron3D` with `kIncidenceEpsD = 1e-5`, replacing the lambda's drifted `1e-10`).

### 5. **Guard normalizations and divisions** by potentially-zero quantities
`Normalize3`, `cross / |cross|`, `x / det`, `x / extent`, perspective divide — all
produce NaN/Inf on degenerate inputs (collinear vertices, parallel normals, edge-on
faces). Guard the magnitude before dividing; use `< eps` not `== 0` (near-singular still
explodes). GUI degeneracy thresholds should scale with the (AABB-normalized) geometry,
not be a fixed `1e-12`.

### 6. **Verify on ground truth, distrust proxy metrics** (process)
task-277 burned a full task on a proxy (`abs` half-space value) that itself had the
thin-geometry disease. A correlation/metric that "looks fixed" can mask the bug — confirm
against the actual rendered/physical result. And reverse-validate regression sentinels:
a sentinel that passes on the *buggy* code (no detection power) is worse than none.

### 7. **Cover the extreme tail in tests**
Bugs live at wedge ≥ 87.4°, near-degenerate faces, `prism_h=0`. A sentinel at wedge 88°
does not cover 89.x°. Sweep the tail; anchor sentinels to the bug's reproduction config,
not a synthetic case.

## Where the absolute tolerances live (audit pointers)

See `scratchpad/explore-geometry-numerical-robustness-audit/catalog.md` for the full
predicate catalog with per-site reachability verdicts. Constants: `kFloatEps = 1e-5`
(`math.hpp`), `kIncidenceEpsD = 1e-5`, `kSingularDetD = 1e-10` (`math.cpp`),
`kFaceCoplanarFloor = 1e-2` (`crystal.cpp`).

## Proposed automation gate (a04: harden conventions into gates)

A soft convention rots. Candidate gate: a lint/test check that flags new
`dot > 1 - <const>` or bare `fabs(det) < <const>` patterns in `src/core/*` geometry
files, pointing the author here. Scope TBD in the hardening backlog.

---

## 中文速记

- **病灶族**：绝对 epsilon 作用在尺度可变的几何量（夹角随 wedge 缩、薄片厚 0.02-0.04、面积∝长²、det∝模长积），极端几何下失效。
- **七条约定**：①匹配用 argmax 不用绝对阈值 ②尺度可变量用相对/归一化容差 ③几何生成走 double、热追踪保 float ④谓词单一真源(防漏网兄弟) ⑤归一化/除法加零保护、用 `<eps` 不用 `==0` ⑥信 ground truth 不信 proxy 指标、哨兵要反向验证有检测力 ⑦测试覆盖极端尾部(wedge 89°+、近退化)。
- **金标准模板**：`crystal.cpp:699` 相对面积、`BuildPolygonFaceData`/`PolygonFaceOfTri`/`FillHexFnMap` pri 的 argmax。
- **已知欠账**：SolvePlanes 系列 det 绝对阈值未归一化(脆但当前未触发)。
