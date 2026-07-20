# Crystal Geometry: Generation and In-Memory Representation

> Status: **diagnosis + redesign direction**, not as-built. The pipeline described in §1 is
> what the tree does today; §4 is a proposed target that has not been built or validated.
> Read this before touching crystal construction, the polygon-face tables, the face-number
> map, or any backend's geometry upload — and read it *together with*
> `numerical-robustness.md`, which gives the conventions for surviving the current design.
> 中文速记见末尾。

## Why this document exists

Between PR #132 and PR #209 the geometry layer produced a steady stream of defects: missing
faces on extreme-wedge pyramids, a spurious flat basal face, broken 6-fold symmetry in the
face-number distribution, malformed meshes with `V − E + F = 6`, three separate SIGSEGVs, and
a 26% energy deficit on one GPU backend. Each was root-caused and fixed on its own terms, and
`numerical-robustness.md` distilled the shared *symptom* — an absolute epsilon applied to a
scale-variant geometric quantity.

That naming is correct but stops one level too high. The epsilons are not an unlucky coding
habit; they are **structurally required by the shape of the pipeline**. This document names
the structural cause, and sketches what a representation that does not require them looks
like.

## 1. The pipeline as built: information is discarded, then numerically rebuilt

```
parameters (h, d[6], wedge angles)
  → plane equations (float)                       FillHexCrystalCoef        (geo3d.cpp)
  → ✂ discard "which plane is which face"
  → brute-force C(n,3) triples + ε dedup          SolveConvexPolyhedronVtxD (math.cpp)
  → vertex cloud
  → ✂ discard "which vertex lies on which plane"
  → re-group by numeric coplanarity               CollectSurfaceVtxD        (math.cpp)
  → polygon faces → triangulation                 Triangulate               (math.cpp)
  → ✂ discard "which triangle came from which face"
  → recover by normal matching                    BuildPolygonFaceData      (crystal.cpp)
                                                  PolygonFaceOfTri          (simulator.cpp)
                                                  tri_to_poly               (each GPU backend)
  → ✂ discard "what is this face's number"
  → recover by normal matching                    FillHexFnMap              (crystal.cpp)
```

Every `✂` is followed by a numerical reconstruction. Every reconstruction needs a tolerance.
Every one of those tolerances has produced at least one real defect:

| Discarded at | Rebuilt by | Defects it produced |
|---|---|---|
| planes → vertices | `C(n,3)` + dedup tolerance | malformed mesh → SIGSEGV (fixed in 23a63386 by making dedup scale-relative, PR #206); inner dedup reachable only at `size()==3`; **the topology-reuse validity predicate that blocks the geometry-randomization moat** (§4) |
| vertices → faces | coplanarity `dot` threshold | spurious flat basal face at wedge ≥ 87.44°, face count 12→14 (PR #133); a full task burned on a proxy metric that carried the same disease (PR #132, superseded) |
| faces → triangles → faces | normal `argmax` / stride bookkeeping | non-`argmax` first-match sibling breaking 6-fold face symmetry (PR #135); polygon-face count/stride mismatch → SIGSEGV (3c3bc3fb, PR #208) |
| faces → face numbers | `fn_map_` double indexing | out-of-range inner index read on malformed input (mitigated by a bounds check in a97fedb5, explicitly labelled depth-of-defense, not a root fix) |
| host → device | each backend re-packs | 26% energy deficit from an independently invented traversal (fixed in a7c63c86 by porting the polygon-slab algorithm); absolute-vs-local index leakage and a `uint16` overflow in the shape pool |

**The information destroyed at each `✂` is known a priori.** A hexagonal prism has 8 planes;
plane *k* bounds face *k*; face *k*'s number is a constant; the set of planes meeting at each
vertex is fixed by the parametrization. None of it needs to be discovered from coordinates.

## 2. The structural cause

> The current design treats a **parametrized, combinatorially known, closed family of at most
> 20 planes** as an **arbitrary convex polyhedron whose combinatorial structure must be
> discovered from a set of half-spaces**. Every epsilon is the tax on that discovery.

The generic machinery is not serving a generic need. `CreateConvexPolyhedronMesh` — the
`C(n,3)` solver, the coplanarity grouping, the triangulation — has exactly three call sites in
the tree (`crystal.cpp:155`, `:166`, `:182`), and all three are `Crystal::CreatePrism` /
`Crystal::CreatePyramid`. There is no live arbitrary-polyhedron path; `CrystalType::kCustom`
is not reachable through it.

A second inversion compounds it. Since the triangle-granular propagation path was removed, the
physics traverses **polygon faces**, not triangles; triangles remain only for GUI mesh display.
But the in-memory representation still centres on `Mesh` (vertices + triangle indices), with
the polygon-face tables (`poly_face_n_`, `poly_face_d_`, `poly_face_tri_id_`) derived *back out*
of the triangle mesh by normal matching. The primary consumer reads a secondary structure
reconstructed from a representation nothing physical consumes. The whole index-defect family —
count/stride mismatch, absolute-vs-local pool indices, the `int` stored inside a `float` array
in `poly_face_data_` — lives on that back-mapping.

## 3. What the physics actually consumes

The inner traversal loop asks one question: given a ray and its current face, which face does
it hit next? It answers it per plane, `t = -(n·o + d)/(n·u)`, takes the smallest positive `t`,
and confirms the hit point is inside the face — which for a convex body is a half-space test.

**That needs plane equations and a face-existence mask. It needs neither vertices nor
triangles.** Vertices are required by exactly two consumers: GUI preview rendering (which may
be slow), and uniform point sampling on the entry face (which needs the face's polygon outline).
Face numbers, symmetry reduction and raypath filters need a per-face integer — a constant of
the parametrization, not a matching result.

## 4. Redesign direction

Target representation — flat, POD, no pointers, identical on host and device:

```
Crystal = { plane equations [≤20]        // generated in double, stored float
          , face_present mask            // analytic predicate on (h, d[6], wedge)
          , face number [≤20]            // constant table, not a matching result
          , face outline vertices        // closed form, grouped per face, derived/lazy
          , triangles                    // GUI only, fan-triangulated from outlines
          }
```

What this dissolves, in the order the tree suffered from it:

- **No coplanarity grouping, no normal matching, no `tri_to_poly`.** Face membership is
  primary data, so the `argmax` predicates and their sibling-drift risk have nothing to
  classify. The index-bookkeeping family loses its substrate.
- **No `C(n,3)` search and no vertex dedup.** Vertices are a closed-form function of the
  parameters, so the dedup tolerance — the mechanism behind the malformed-mesh SIGSEGV — is
  not merely retuned, it is absent.
- **Face loss becomes a first-class state instead of an accident.** Under randomized
  `face_distance` a large fraction of crystals legitimately lose faces; today that is inferred
  from coordinates and silently mismatches filters that reference the face by number. With an
  explicit `face_present` mask the filter layer can see it.
- **The topology-reuse blocker disappears.** The geometry-randomization moat (see
  `geometry-randomization-perf.md`) stalled on a validity predicate: caching the plane-triple →
  vertex incidence requires deciding, per resample, whether the cached combinatorics still
  hold, and a candidate predicate measured a non-zero false-accept rate against the production
  solver. That question exists **only because the topology was discovered numerically**. If
  face existence is an analytic predicate and vertices are closed form, there is no cache, no
  validity question, and no epsilon to align — and the construction floor is below the 127 ns
  that reuse was reaching for.
- **Pooling becomes concatenation.** A flat POD shared by host and both GPU backends removes
  the per-backend re-pack stage that produced the traversal and index defects.

## 5. What it does **not** dissolve (honest limits)

- **Concave pyramids** are not a half-space intersection. `CreateConcavePyramidMesh` needs
  separate modelling (it is a composition of convex blocks); it is extra work, not a free ride.
- **Numerical care at ray–geometry intersection remains.** Grazing rays and near-singular
  determinants do not care how the geometry was built. The gain is one of *containment*:
  epsilons shrink from "every grouping and dedup step of construction" to "the intersection
  test", where a physical scale is available to normalize against.
- **Face-loss semantics is a product question, not a representation question.** What a filter
  referencing an absent face *should* do still needs deciding; the mask only makes it visible.
- **Tension with an existing ruling.** The signed-`face_distance` fix (730d54a9, PR #206)
  deliberately chose to **validate the output, not the input** — construct normally, then gate
  on a closed-mesh Euler check — precisely to avoid deriving a validity domain over the
  parameters, which was judged high-risk. An analytic `face_present` predicate is exactly such
  a derivation. The two are reconcilable (that ruling was made under time pressure while fixing
  a live crash; this is a deliberate redesign over a small closed family) but the tension is
  real and should be re-decided explicitly, not assumed away.

## 6. What must be answered before committing to the rewrite

The closed-form derivation is bounded manual work and is *not* where the risk lives. Three
other questions are:

1. **Consumer inventory.** Enumerate precisely who needs planes, outlines, triangles, face
   numbers and existence, across tracing, entry-face sampling, `Crystal::GetFn`, raypath
   filters, symmetry reduction, GUI preview and each backend's upload. Getting the
   representation wrong is far more expensive than getting a generation algorithm wrong; this
   inventory, not the algebra, defines the minimum sufficient representation and the blast
   radius.
2. **Where the equivalence oracle comes from.** The new generator must be checked against a
   ground truth, and **the existing solver cannot be it** — that solver's dedup tolerance is an
   empirically swept constant (`5e-5 × char_len`, `math.cpp`), i.e. it is the patient. It is a
   valid oracle in the well-conditioned regime, where old and new must agree face-by-face; in
   the degenerate regime the oracle must be independent (exact/rational arithmetic, or hand-
   derived solutions for known configurations). Without this, a rewrite can only be shown to
   *differ* from the old pipeline, never to be *more correct* than it. This is the same trap
   that stalled the topology-reuse predicate.
3. **The shape of the face-existence predicate.** Not "can it be derived" but "does it come out
   as clean analytic inequalities, or as a case explosion". If strongly skewed `d` fragments it
   into many branches, the design has traded epsilons for branches and the benefit shrinks. The
   8-plane prism is enough to judge; the positive-volume condition on opposite face pairs
   (`d_i + d_{i+3} > 0`, established while diagnosing the randomized-`face_distance` crash) is
   an encouraging sample of the expected form.

---

## 中文速记

- **诊断**：几何缺陷族的机制层根因不是"用错了 ε"，而是**管线把先验已知的信息在每一步丢弃、再用数值方法重建**（平面→顶点→面归组→三角化→反推面归属→反推面号）。每个重建需要一个容差，每个容差都产出过真缺陷（对照表见 §1）。
- **结构性错配**：把一个**参数化的、组合结构已知的、≤20 面的封闭小族**，当作**任意凸多面体、需从半空间集合发现组合结构**来处理。证据：通用求解器 `CreateConvexPolyhedronMesh` 全仓库仅 3 个调用点，全部是六方晶体工厂；无存活的自定义多面体路径。
- **第二重倒置**：物理消费的是多边形面，三角网格只剩 GUI 预览需要；但表达仍以三角网格为主、多边形面表由它**反推**。整个索引缺陷族长在这个反推映射上。
- **目标表达**：平面方程 + 面存在掩码 + 面号常数表 + 闭式面轮廓（三角形仅 GUI 派生）；扁平 POD、host/device 同构、池化即拼接。
- **消解了什么**：共面归组/法向匹配/`tri_to_poly` 无对象可分类；顶点去重容差不复存在；掉面从"意外"变"一等状态"；**拓扑复用的失效判据问题消失**（判据难题只在拓扑靠数值发现时才存在），构造地板低于 127 ns。
- **不消解什么**：凹锥晶体需单独建模；光线–几何求交的数值问题仍在（收益是**收敛**ε 的作用域）；掉面语义是产品问题；⚠️ 与"校验输出不校验输入"的既定判据存在真实张力，须显式重新决定。
- **动手前必须先回答的三件事**：①消费者清点（定最小充分表达，比算法更贵） ②等价性 gt 从哪来（**旧求解器不能当 gt，它就是病人**；良态区可对照，退化区需独立 gt——这正是拓扑复用卡死的同一个坑） ③面存在谓词的**形态**（干净不等式 vs case 爆炸）。
