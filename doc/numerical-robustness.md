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

### 8. **Free-symbol algebra is structurally blind to degeneracy** (choosing the model)

The prior seven conventions treat *floating-point* tolerance. This one is upstream of
them: it is about picking the wrong **algebraic model** for an exactness claim, a mistake
no tolerance can rescue.

Free / transcendental symbolic algebra is the algebra of the **generic point**. Treating
a shape parameter as a free symbol `α` (rather than a specific number) can only decide
identities that hold for *all* values of `α` — it answers `p(α) ≡ 0 ⟺ p is the zero
polynomial`. Geometric degeneracy is the opposite kind of fact: it is a **coincidence at
a specific value** — `a1 = a2 ⟹ α = β`, three faces meeting at a point, an edge
vanishing first. A free-symbol model is structurally blind to these, because to it the
parameters are never equal. This is not an implementation bug you can patch; it is a
**category error** — the model cannot see the phenomenon being asked about.

The concrete failure this is distilled from: a pyramid exactness oracle carried the cone
angles as free symbols and, when a regular cone (`a1 = a2`) made a factor `k·(α − β)`
truly zero, could not know it — so it fell back to an embedded double-Horner filter with a
128-ULP refuse margin. That filter's ambiguous→refuse verdict is FMA-contraction
sensitive and therefore platform-dependent: the same source gave the correct 14-vertex
topology on one platform and an all-wrong 18 on another (Ampere ARM64). The three prior
"passing" platforms were luck — rounding happened to land just past the margin. No amount
of margin tuning fixes this; the algebra was wrong, not the epsilon.

**The criterion.** For an exactness claim you have exactly two honest options:
- **Pin the parameters into a concrete number field** that is closed under the operations
  *and* has a decidable zero-test, then do exact integer/rational arithmetic. For this
  repo's hexagonal-crystal family that field is `QS3 = ℚ(√3)` — but note the input
  parameters must *themselves* land in that field, not just the arithmetic downstream.
- **Honestly admit it is a double approximation** and use a scale-relative tolerance per
  convention #2, with fixtures kept off the exact boundary so platform rounding never
  flips a verdict.

The trap is the middle path — a free symbol *pretending* to be exact. It gets neither:
not genuine exactness (blind to degeneracy), not honest approximation (it advertises a
precision it does not have).

**When you genuinely cannot avoid exact computation**, there are two correct landings by
regime:
- *Test / build time*: compute it offline with a CAS over true algebraic numbers (e.g.
  sympy's `Fraction`/`sqrt`), evaluate once, and store the constants. No live oracle, no
  per-platform arithmetic on the hot boundary.
- *Runtime / production*: either pin the number field and do exact integer arithmetic, or
  admit the approximation and degrade gracefully. This repo has **never** had a runtime
  path that must be bit-exact — production `geo3d_closedform.*` is all `double`, and its
  degenerate output (a near-zero-area face contributing nothing to the MC path) is an
  accepted contract, not a bug.

Landing example: `test/golden-analytic/core/test_closed_form_pyramid.cpp` replaced the
free-symbol live oracle with three tests that each assert a contract the production code
*actually holds*, none of which arbitrates "which is more precise" on a degenerate input:
`TopologyMatchesGoldenConstants` (post-closed-form topology is a stored constant — zero
computation, zero platform surface), `VertexPlaneSelfConsistency` (the shape's own
vertices satisfy its own half-space inequalities under a scale-relative tolerance), and
`DegenerateContractSafe` (degenerate inputs stay bounded, finite, and collapse to
near-zero area — the graceful-degradation contract, not an exact apex).

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
- **八条约定**：①匹配用 argmax 不用绝对阈值 ②尺度可变量用相对/归一化容差 ③几何生成走 double、热追踪保 float ④谓词单一真源(防漏网兄弟) ⑤归一化/除法加零保护、用 `<eps` 不用 `==0` ⑥信 ground truth 不信 proxy 指标、哨兵要反向验证有检测力 ⑦测试覆盖极端尾部(wedge 89°+、近退化) ⑧自由符号代数结构性盲于退化——选错代数模型，任何容差救不了。
- **第 8 条（选模型，不是调 ε）**：自由/超越符号是"通用点"的代数，只判"对所有取值成立的恒等式"(`p(α)≡0 ⟺ p 是零多项式`)；几何退化是"特定取值下的巧合"(`a1=a2 ⟹ α=β`、三面共点、某边先消失)，自由符号模型按定义看不见——这是**范畴错误**，不是可打补丁的实现 bug。病例：pyramid oracle 把锥角当自由符号 α/β，正规锥体 `k·(α−β)` 真零却判不出，落到内嵌 double-Horner + 128-ULP refuse 滤波器，其 ambiguous→refuse 受 FMA 收缩影响、平台相关(同码 macOS 出 14 顶点、Ampere ARM64 出全错 18；前三平台绿是舍入侥幸)。**判据 = 二选一**：要么把参数**钉进对运算封闭且零测试可判定的数域**(六方族 = `QS3 = ℚ(√3)`，且输入参数本身也须落域内)做精确整数/有理算术；要么**老实认 double 近似**、按约定②用尺度相对容差 + fixture 离精确边界。中间"自由符号假装精确"两头不占。**绕不开精确计算的两种正解**：测试/构建期 → 离线 CAS(sympy 真·代数数)一次算死存盘；运行时生产 → 钉数域精确整数 or 认近似(本仓库生产 `geo3d_closedform.*` 全 double、退化产近零面积面即可，从未需运行时精确)。落地示例：`test_closed_form_pyramid.cpp` 的三件契约测试(`TopologyMatchesGoldenConstants`/`VertexPlaneSelfConsistency`/`DegenerateContractSafe`)各断言生产真持有的契约，均不对退化情形仲裁"谁更精确"。
- **金标准模板**：`crystal.cpp:699` 相对面积、`BuildPolygonFaceData`/`PolygonFaceOfTri`/`FillHexFnMap` pri 的 argmax。
- **已知欠账**：SolvePlanes 系列 det 绝对阈值未归一化(脆但当前未触发)。
