#include "core/crystal.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "config/filter_config.hpp"
#include "core/def.hpp"
#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"
#include "util/logger.hpp"

namespace lumice {

// Bridge Crystal's flat-POD capacity to the closed-form evaluator's per-face
// counts. If the closed-form path ever gains a face-count larger than 20 (a
// new crystal family or a widened cone slot layout), CrystalGeom's fixed arrays
// would silently truncate — catch that at compile time here rather than at a
// runtime read-past-end site.
static_assert(kCrystalGeomMaxFaces >= kClosedFormPrismFaceCnt,
              "CrystalGeom face capacity must accommodate the prism closed-form family");
static_assert(kCrystalGeomMaxFaces >= kClosedFormPyramidFaceCnt,
              "CrystalGeom face capacity must accommodate the pyramid closed-form family");
static_assert(kCrystalGeomMaxVtxPerFace >= kClosedFormPyramidMaxFaceVtx,
              "CrystalGeom per-face vertex capacity must accommodate pyramid cone faces");

namespace {
// Process-global count of degenerate polygon faces dropped by
// BuildPolygonFaceData (the count/stride "shrink" path). Test observability
// only; see Crystal::DegenerateShrinkCount().
std::atomic<uint64_t> g_degenerate_shrink_count{ 0 };
}  // namespace

uint64_t Crystal::DegenerateShrinkCount() {
  return g_degenerate_shrink_count.load(std::memory_order_relaxed);
}

void Crystal::ResetDegenerateShrinkCount() {
  g_degenerate_shrink_count.store(0, std::memory_order_relaxed);
}

// Legal face-number sets mirror FillHexFnMap below:
//   basal:         1, 2
//   prism lateral: 3..8
//   upper pyramid: 13..18
//   lower pyramid: 23..28
// See crystal_kind.hpp for rationale on why this GUI-facing coarse enum is
// intentionally distinct from the core CrystalType.
bool IsLegalFace(CrystalKind kind, int face) {
  auto is_basal = [](int f) { return f == 1 || f == 2; };
  auto is_prism_lateral = [](int f) { return f >= 3 && f <= 8; };
  auto is_upper_pyramid = [](int f) { return f >= 13 && f <= 18; };
  auto is_lower_pyramid = [](int f) { return f >= 23 && f <= 28; };

  switch (kind) {
    case CrystalKind::kPrism:
      return is_basal(face) || is_prism_lateral(face);
    case CrystalKind::kPyramid:
      return is_basal(face) || is_prism_lateral(face) || is_upper_pyramid(face) || is_lower_pyramid(face);
  }
  // Unhandled CrystalKind — new enum values must extend the switch above.
  assert(false && "IsLegalFace: unhandled CrystalKind");
  return false;
}

void FillHexFnMap(size_t face_cnt, const float* face_n, IdType* fn_map) {
  using math::kSqrt3_2;

  std::fill(fn_map, fn_map + face_cnt, kInvalidId);

  float ref_norms[9 * 3]{
    0.0f,  0.0f,      0.0f,   // placeholder
    0.0f,  0.0f,      1.0f,   // 1
    0.0f,  0.0f,      -1.0f,  // 2
    1.0f,  0.0f,      0.0f,   // 3
    0.5f,  kSqrt3_2,  0.0f,   // 4
    -0.5f, kSqrt3_2,  0.0f,   // 5
    -1.0f, 0.0f,      0.0f,   // 6
    -0.5,  -kSqrt3_2, 0.0f,   // 7
    0.5f,  -kSqrt3_2, 0.0f,   // 8
  };

  for (size_t i = 0; i < face_cnt; i++) {
    const float* curr_norm = face_n + i * 3;
    IdType pri = 0;
    float p_comp = -1;
    for (IdType j = 3; j < 9; j++) {
      if (auto p = Dot3(curr_norm, ref_norms + j * 3); p > math::kFloatEps && p > p_comp) {
        pri = j;
        p_comp = p;
      }
    }

    IdType pyr = 0;
    if (Dot3(curr_norm, ref_norms + 3) > math::kFloatEps) {
      pyr = 1;
    } else if (Dot3(curr_norm, ref_norms + 6) > math::kFloatEps) {
      pyr = 2;
    }

    if (pri == 0 && pyr > 0) {
      fn_map[i] = pyr;  // basal: 1 or 2
    } else if (pri > 0) {
      fn_map[i] = pyr * 10 + pri;  // prism/pyramidal: 3-8, 13-18, 23-28
    } else {
      LOG_WARNING("Unrecognized face number!");
    }
  }
}

bool IsClosedTriMesh(size_t v, size_t f) {
  if (v == 0 || f == 0 || f % 2 != 0) {
    return false;
  }
  const auto vi = static_cast<int64_t>(v);
  const auto fi = static_cast<int64_t>(f);
  return vi - (3 * fi / 2) + fi == 2;
}

namespace {

// Fires when the numerical-geometry pipeline produced a mesh that fails the
// closed-manifold Euler check under extreme random face_distance combinations
// that the vertex-dedup relative tolerance still misses (empirically ~14 in
// 200k under a gauss(1, 0.5) fuzz sweep). Downstream BuildPolygonFaceData
// relies on a well-formed input mesh; feeding it a mesh that fails this check
// leaves polygon-face slots partially initialized and the polygon-indexed
// GetFn(IdType) reads through garbage tri ids into fn_map_. Rejecting the
// mesh at the factory boundary matches FillHexCrystalCoef's existing
// "zero-volume degenerate → returns 0 planes" pattern: caller-visible
// contract is unchanged, downstream sees a Crystal with 0 triangles that
// contributes nothing to raypath sampling.
//
// Retained for the pyramid path (which still walks the numerical pipeline in
// Step 2's window); prism now goes through the closed-form gate below.
Mesh RejectMalformed(Mesh mesh, const char* factory) {
  const size_t v = mesh.GetVtxCnt();
  const size_t f = mesh.GetTriangleCnt();
  if (IsClosedTriMesh(v, f)) {
    return mesh;
  }
  // Silent when the upstream pipeline already returned an empty mesh (e.g.
  // FillHexCrystalCoef's zero-volume early-return path emits its own warning).
  // Emit only for the real "constructed something but it failed the
  // closed-mesh check" case that this gate is designed to catch. This check
  // is necessary but not sufficient for manifold-ness (see IsClosedTriMesh
  // doc comment in crystal.hpp) — the log message deliberately says "Euler
  // check" rather than "non-manifold" so it does not overclaim detection
  // power it does not have.
  if (v != 0 || f != 0) {
    LOG_WARNING("{}: failed closed-mesh Euler check (V={}, F={}); treating as degenerate", factory, v, f);
  }
  return Mesh(0, 0);
}

// 386.2 validity gate for the prism closed-form path. The malformed families
// the Euler check used to catch (opposite-pair-sum ≤ 0, wedge-collapse
// pyramid-adjacent inputs) collapse the 2D cross-section corner ring to fewer
// than three distinct corners; the closed-form solver produces that fact
// directly (corner_cnt) — no reverse-engineering the topology from a triangle
// mesh required.
bool IsValidClosedFormPrism(const ClosedFormPrismResult& r, float h) {
  return h > math::kFloatEps && r.corner_cnt >= 3;
}

// 386.2 validity gate for the pyramid closed-form path. The closed-form
// evaluator already resolves face-presence per slot (accounting for cone
// dropout at illegal alpha, shoulder-vs-apex layering, and corner-death
// events); a valid solid needs at least a tetrahedron's worth of faces
// (4 present). Zero-volume inputs (h1=h2=h3=0 or all cones dropped with h2=0)
// short-circuit to vtx_cnt == 0 inside the evaluator, which trivially fails
// the count check.
bool IsValidClosedFormPyramid(const ClosedFormPyramidResult& r) {
  int present_cnt = 0;
  for (int i = 0; i < kClosedFormPyramidFaceCnt; i++) {
    if (r.face_present[i]) {
      present_cnt++;
    }
  }
  return present_cnt >= 4;
}

// Convert ClosedFormPrismResult (single CCW 2D corner ring + h) into per-face
// 3D CCW vertex lists in CrystalGeom.
//   - Upper basal (slot 0, normal +z): ring CCW at z = +h/2 (ring is emitted
//     CCW as seen from +z by SolveHexCrossSection; no re-ordering).
//   - Lower basal (slot 1, normal -z): ring reversed at z = -h/2 so viewer at
//     -z sees CCW.
//   - Side face (slot 2+i, present): 4-vert rectangle
//       (c_prev, z_bot), (c_curr, z_bot), (c_curr, z_top), (c_prev, z_top)
//     with c_prev, c_curr the two ring corners the side lies between (walking
//     present sides in i-increasing order). cross(v1-v0, v2-v0) aligns with
//     the outward horizontal normal.
void AdaptClosedFormPrismToCrystalGeom(const ClosedFormPrismResult& r, float h, CrystalGeom& g) {
  g = CrystalGeom{};
  g.face_cnt = kClosedFormPrismFaceCnt;

  std::memcpy(g.plane_coef, r.plane_coef, sizeof(r.plane_coef));
  std::memcpy(g.face_normal, r.face_normal, sizeof(r.face_normal));
  std::memcpy(g.face_number, r.face_number, sizeof(r.face_number));
  for (int i = 0; i < kClosedFormPrismFaceCnt; i++) {
    g.face_present[i] = r.face_present[i];
  }

  if (r.corner_cnt < 3) {
    return;
  }

  const int n = r.corner_cnt;
  const float z_top = 0.5f * h;
  const float z_bot = -0.5f * h;

  if (r.face_present[0]) {
    g.face_vtx_cnt[0] = n;
    float* base = g.face_vtx + 0 * kCrystalGeomMaxVtxPerFace * 3;
    for (int k = 0; k < n; k++) {
      base[k * 3 + 0] = r.corner_x[k];
      base[k * 3 + 1] = r.corner_y[k];
      base[k * 3 + 2] = z_top;
    }
  }
  if (r.face_present[1]) {
    g.face_vtx_cnt[1] = n;
    float* base = g.face_vtx + 1 * kCrystalGeomMaxVtxPerFace * 3;
    for (int k = 0; k < n; k++) {
      const int rev = n - 1 - k;
      base[k * 3 + 0] = r.corner_x[rev];
      base[k * 3 + 1] = r.corner_y[rev];
      base[k * 3 + 2] = z_bot;
    }
  }

  int present_idx[kClosedFormPrismSideCnt];
  int p_n = 0;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    if (r.face_present[2 + i]) {
      present_idx[p_n++] = i;
    }
  }
  // p_n must equal n: the corner ring is built by walking present sides and
  // emitting one corner per adjacent pair, and a bounded 2D polygon has as
  // many edges as vertices.
  assert(p_n == n);

  for (int k = 0; k < p_n; k++) {
    const int side_i = present_idx[k];
    const int slot = 2 + side_i;
    const int c_prev = (k - 1 + p_n) % p_n;
    const int c_curr = k;

    const float px = r.corner_x[c_prev];
    const float py = r.corner_y[c_prev];
    const float cx = r.corner_x[c_curr];
    const float cy = r.corner_y[c_curr];

    g.face_vtx_cnt[slot] = 4;
    float* base = g.face_vtx + slot * kCrystalGeomMaxVtxPerFace * 3;
    base[0 * 3 + 0] = px;
    base[0 * 3 + 1] = py;
    base[0 * 3 + 2] = z_bot;
    base[1 * 3 + 0] = cx;
    base[1 * 3 + 1] = cy;
    base[1 * 3 + 2] = z_bot;
    base[2 * 3 + 0] = cx;
    base[2 * 3 + 1] = cy;
    base[2 * 3 + 2] = z_top;
    base[3 * 3 + 0] = px;
    base[3 * 3 + 1] = py;
    base[3 * 3 + 2] = z_top;
  }
}

// Convert ClosedFormPyramidResult (per-slot CCW vertex indices into the
// evaluator's global 3D pool) into per-face 3D CCW vertex coord lists in
// CrystalGeom. The evaluator already sorts each face's vertices CCW as seen
// from OUTSIDE the solid (geo3d_closedform.cpp:923-988), so no re-ordering is
// needed here — just an index→coord scatter.
void AdaptClosedFormPyramidToCrystalGeom(const ClosedFormPyramidResult& r, CrystalGeom& g) {
  g = CrystalGeom{};
  g.face_cnt = kClosedFormPyramidFaceCnt;

  std::memcpy(g.plane_coef, r.plane_coef, sizeof(r.plane_coef));
  std::memcpy(g.face_normal, r.face_normal, sizeof(r.face_normal));
  std::memcpy(g.face_number, r.face_number, sizeof(r.face_number));
  for (int i = 0; i < kClosedFormPyramidFaceCnt; i++) {
    g.face_present[i] = r.face_present[i];
  }

  for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
    if (!r.face_present[slot]) {
      continue;
    }
    const int fn = r.face_vtx_cnt[slot];
    assert(fn <= kCrystalGeomMaxVtxPerFace);
    g.face_vtx_cnt[slot] = fn;
    float* base = g.face_vtx + slot * kCrystalGeomMaxVtxPerFace * 3;
    for (int k = 0; k < fn; k++) {
      const int vi = r.face_vtx[slot][k];
      base[k * 3 + 0] = r.vtx[vi * 3 + 0];
      base[k * 3 + 1] = r.vtx[vi * 3 + 1];
      base[k * 3 + 2] = r.vtx[vi * 3 + 2];
    }
  }
}

// Dedup-linear-search 3D vertex pool + fixed-fan triangulation. Small n
// (≤ 24 verts for prism, ≤ 96 for the pyramid worst case), so O(n²) dedup is
// fine and matches the tolerance choice already used inside
// geo3d_closedform.cpp's 2D corner dedup.
struct BuiltMesh {
  Mesh mesh;
  std::vector<int> tri_face_slot;  // len == mesh.GetTriangleCnt()
};

BuiltMesh BuildMeshFromCfGeom(const CrystalGeom& g) {
  constexpr float kDedupTol = 1e-6f;

  std::vector<float> vtx_pool;
  vtx_pool.reserve(static_cast<size_t>(kCrystalGeomMaxFaces) * kCrystalGeomMaxVtxPerFace * 3);
  auto add_or_find = [&](float x, float y, float z) -> int {
    const size_t n = vtx_pool.size() / 3;
    for (size_t i = 0; i < n; i++) {
      const float dx = vtx_pool[i * 3 + 0] - x;
      const float dy = vtx_pool[i * 3 + 1] - y;
      const float dz = vtx_pool[i * 3 + 2] - z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < kDedupTol) {
        return static_cast<int>(i);
      }
    }
    vtx_pool.push_back(x);
    vtx_pool.push_back(y);
    vtx_pool.push_back(z);
    return static_cast<int>(n);
  };

  std::vector<int> tri_idx;
  std::vector<int> tri_face_slot;
  int face_to_global[kCrystalGeomMaxVtxPerFace];

  for (int slot = 0; slot < g.face_cnt; slot++) {
    if (!g.face_present[slot]) {
      continue;
    }
    const int fn = g.face_vtx_cnt[slot];
    if (fn < 3) {
      continue;
    }
    const float* base = g.face_vtx + slot * kCrystalGeomMaxVtxPerFace * 3;
    for (int k = 0; k < fn; k++) {
      face_to_global[k] = add_or_find(base[k * 3 + 0], base[k * 3 + 1], base[k * 3 + 2]);
    }
    // Fan (v[0], v[i-1], v[i]) for i = 2..n-1 → n-2 triangles.
    for (int i = 2; i < fn; i++) {
      tri_idx.push_back(face_to_global[0]);
      tri_idx.push_back(face_to_global[i - 1]);
      tri_idx.push_back(face_to_global[i]);
      tri_face_slot.push_back(slot);
    }
  }

  const size_t vtx_cnt = vtx_pool.size() / 3;
  const size_t tri_cnt = tri_face_slot.size();

  auto vtx_buf = std::make_unique<float[]>(vtx_cnt * 3);
  if (vtx_cnt > 0) {
    std::memcpy(vtx_buf.get(), vtx_pool.data(), vtx_cnt * 3 * sizeof(float));
  }
  auto tri_buf = std::make_unique<int[]>(tri_cnt * 3);
  if (tri_cnt > 0) {
    std::memcpy(tri_buf.get(), tri_idx.data(), tri_cnt * 3 * sizeof(int));
  }

  return BuiltMesh{ Mesh(vtx_cnt, std::move(vtx_buf), tri_cnt, std::move(tri_buf)), std::move(tri_face_slot) };
}

}  // namespace

// Populate fn_map_ and poly_face_data_ directly from the closed-form output,
// skipping FillHexFnMap's argmax-vs-reference-normals reversal and
// BuildPolygonFaceData's argmax triangle-to-plane grouping. The closed-form
// output already carries the parametric face-number and one-tri-per-face slot
// membership; there is nothing to reverse-engineer. This is the payoff of the
// representation swap: the three argmax reversals (CPU / Metal / CUDA) become
// straight assignments from the same source.
void Crystal::PopulateFromCfGeom(const std::vector<int>& tri_face_slot) {
  const size_t tri_cnt = mesh_.GetTriangleCnt();
  assert(tri_face_slot.size() == tri_cnt);
  for (size_t t = 0; t < tri_cnt; t++) {
    fn_map_[t] = static_cast<IdType>(cf_geom_.face_number[tri_face_slot[t]]);
  }

  // slot → poly-face-index (0..present_cnt-1). kInvalidId for absent slots.
  IdType slot_to_poly[kCrystalGeomMaxFaces];
  for (int i = 0; i < kCrystalGeomMaxFaces; i++) {
    slot_to_poly[i] = kInvalidId;
  }
  size_t present = 0;
  for (int slot = 0; slot < cf_geom_.face_cnt; slot++) {
    if (cf_geom_.face_present[slot]) {
      slot_to_poly[slot] = static_cast<IdType>(present);
      present++;
    }
  }
  poly_face_cnt_ = present;

  poly_face_of_tri_ = std::make_unique<IdType[]>(tri_cnt);
  for (size_t t = 0; t < tri_cnt; t++) {
    const int slot = tri_face_slot[t];
    poly_face_of_tri_[t] = (slot >= 0 && slot < kCrystalGeomMaxFaces) ? slot_to_poly[slot] : kInvalidId;
  }

  if (poly_face_cnt_ == 0) {
    poly_face_data_.reset();
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
    return;
  }
  poly_face_data_ = std::make_unique<float[]>(poly_face_cnt_ * 5);
  poly_face_n_ = poly_face_data_.get();
  poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
  poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);

  size_t p = 0;
  for (int slot = 0; slot < cf_geom_.face_cnt; slot++) {
    if (!cf_geom_.face_present[slot]) {
      continue;
    }
    // Representative triangle: first tri assigned to this slot. This is the
    // same contract BuildPolygonFaceData provided (a triangle definitely
    // coplanar with the polygon plane); the specific triangle chosen doesn't
    // matter beyond that — GetFn(IdType) only reads fn_map_[rep_tri], and both
    // paths land the tri on the same face.
    int rep_tri = -1;
    for (size_t t = 0; t < tri_cnt; t++) {
      if (tri_face_slot[t] == slot) {
        rep_tri = static_cast<int>(t);
        break;
      }
    }
    assert(rep_tri >= 0);

    poly_face_n_[p * 3 + 0] = cf_geom_.face_normal[slot * 3 + 0];
    poly_face_n_[p * 3 + 1] = cf_geom_.face_normal[slot * 3 + 1];
    poly_face_n_[p * 3 + 2] = cf_geom_.face_normal[slot * 3 + 2];

    // Normalize the plane's d by |(a, b, c)| so the stored plane matches the
    // "unit normal" convention BuildPolygonFaceData used. The closed-form
    // plane_coef layout mirrors FillHexCrystalCoef's, whose per-slot norms are
    // {1, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5} for prism (basal unit, side 0.5).
    const float* coef = cf_geom_.plane_coef + slot * 4;
    const float norm = Norm3(coef);
    poly_face_d_[p] = (norm > math::kFloatEps) ? (coef[3] / norm) : 0.0f;
    poly_face_tri_id_[p] = rep_tri;
    p++;
  }
}

Crystal Crystal::MakePrismClosedForm(float h, const float dist[6], const char* factory) {
  ClosedFormPrismResult r = ComputeClosedFormPrism(h, dist);
  if (!IsValidClosedFormPrism(r, h)) {
    // Silent for the FillHexCrystalCoef-mirroring zero-volume path (h ≤ eps).
    // Warn on the "constructed something but it failed the closed-form gate"
    // case — matches the RejectMalformed semantics for what used to be the
    // "closed-mesh Euler check" family: caller-visible contract is unchanged,
    // downstream sees a zero-triangle Crystal that contributes nothing.
    if (h > math::kFloatEps) {
      LOG_WARNING("{}: failed closed-form validity gate (h={:.4e}, corner_cnt={}); treating as degenerate",  //
                  factory, static_cast<double>(h), r.corner_cnt);
    }
    return Crystal(Mesh(0, 0));
  }
  CrystalGeom g;
  AdaptClosedFormPrismToCrystalGeom(r, h, g);
  BuiltMesh built = BuildMeshFromCfGeom(g);
  Crystal c(std::move(built.mesh));
  c.cf_geom_ = g;
  c.fn_period_ = 6;
  c.PopulateFromCfGeom(built.tri_face_slot);
  return c;
}

Crystal Crystal::CreatePrism(float h) {
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  return MakePrismClosedForm(h, dist, "CreatePrism(h)");
}

Crystal Crystal::CreatePrism(float h, const float* fd) {
  return MakePrismClosedForm(h, fd, "CreatePrism(h, fd)");
}

Crystal Crystal::CreatePyramid(float h1, float h2, float h3) {
  float alpha = std::atan(math::kSqrt3_2 / kIceCrystalC) * math::kRadToDegree;
  return CreatePyramid(alpha, alpha, h1, h2, h3);
}

Crystal Crystal::MakePyramidClosedForm(float upper_alpha, float lower_alpha, float h1, float h2, float h3,
                                       const float dist[6], const char* factory) {
  ClosedFormPyramidResult r = ComputeClosedFormPyramid(upper_alpha, lower_alpha, h1, h2, h3, dist);
  if (!IsValidClosedFormPyramid(r)) {
    // Silent when the evaluator returned an all-empty result (zero-volume
    // short-circuit — matches FillHexCrystalCoef's own zero-volume path,
    // which emits its own warning). Warn only when it produced *some*
    // vertices but not enough face slots to bound a solid — the RejectMalformed
    // analog for the closed-form path.
    if (r.vtx_cnt > 0) {
      LOG_WARNING("{}: failed closed-form validity gate (present face count < 4, vtx_cnt={})", factory, r.vtx_cnt);
    }
    return Crystal(Mesh(0, 0));
  }
  CrystalGeom g;
  AdaptClosedFormPyramidToCrystalGeom(r, g);
  BuiltMesh built = BuildMeshFromCfGeom(g);
  Crystal c(std::move(built.mesh));
  c.cf_geom_ = g;
  c.fn_period_ = 6;
  c.PopulateFromCfGeom(built.tri_face_slot);
  return c;
}

Crystal Crystal::CreatePyramid(float upper_alpha, float lower_alpha, float h1, float h2, float h3, const float* dist) {
  return MakePyramidClosedForm(upper_alpha, lower_alpha, h1, h2, h3, dist, "CreatePyramid");
}

Crystal Crystal::CreatePyramid(float upper_alpha, float lower_alpha, float h1, float h2, float h3) {
  float dist[6]{ 1, 1, 1, 1, 1, 1 };
  return CreatePyramid(upper_alpha, lower_alpha, h1, h2, h3, dist);
}

Crystal Crystal::CreatePyramid(int upper_i1, int upper_i4, int lower_i1, int lower_i4,  // Miller index
                               float h1, float h2, float h3,                            // height
                               const float* dist) {                                     // face distance
  // Guard against i1=0: although float division yields inf (not UB), we prefer an explicit guard
  // over relying on atan(inf)=90° being filtered by the alpha range check in FillHexCrystalCoef.
  float upper_alpha =
      upper_i1 != 0 ? std::atan(math::kSqrt3_2 * upper_i4 / upper_i1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
  float lower_alpha =
      lower_i1 != 0 ? std::atan(math::kSqrt3_2 * lower_i4 / lower_i1 / kIceCrystalC) * math::kRadToDegree : 0.0f;
  return CreatePyramid(upper_alpha, lower_alpha, h1, h2, h3, dist);
}


enum CrystalCachOffset {
  kVtx = 0,
  kNormal = 9,
  kArea = 12,
  kTotal = 13,
};

Crystal::Crystal() {}

Crystal::Crystal(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx)
    : mesh_(vtx_cnt, std::move(vtx), triangle_cnt, std::move(triangle_idx)),
      cache_data_(std::make_unique<float[]>(triangle_cnt * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kVtx),
      face_n_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + triangle_cnt * CrystalCachOffset::kArea),
      fn_map_(std::make_unique<IdType[]>(triangle_cnt)), fn_period_(-1) {
  ComputeCacheData();
}

Crystal::Crystal(Mesh mesh)
    : mesh_(std::move(mesh)),
      cache_data_(std::make_unique<float[]>(mesh_.GetTriangleCnt() * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kVtx),
      face_n_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kArea),
      fn_map_(std::make_unique<IdType[]>(mesh_.GetTriangleCnt())), fn_period_(-1) {
  ComputeCacheData();
}

Crystal::Crystal(const Crystal& other)
    : config_id_(other.config_id_), mesh_(other.mesh_),
      cache_data_(std::make_unique<float[]>(other.TotalTriangles() * CrystalCachOffset::kTotal)),
      face_v_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kVtx),
      face_n_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + other.TotalTriangles() * CrystalCachOffset::kArea),
      fn_map_(std::make_unique<IdType[]>(other.TotalTriangles())), fn_period_(other.fn_period_),
      poly_face_cnt_(other.poly_face_cnt_), cf_geom_(other.cf_geom_) {
  auto tri_cnt = other.TotalTriangles();
  std::memcpy(cache_data_.get(), other.cache_data_.get(), tri_cnt * CrystalCachOffset::kTotal * sizeof(float));
  std::memcpy(fn_map_.get(), other.fn_map_.get(), tri_cnt * sizeof(IdType));

  if (poly_face_cnt_ > 0) {
    poly_face_data_ = std::make_unique<float[]>(poly_face_cnt_ * 5);
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
    std::memcpy(poly_face_data_.get(), other.poly_face_data_.get(), poly_face_cnt_ * 5 * sizeof(float));
  }
  if (other.poly_face_of_tri_) {
    poly_face_of_tri_ = std::make_unique<IdType[]>(tri_cnt);
    std::memcpy(poly_face_of_tri_.get(), other.poly_face_of_tri_.get(), tri_cnt * sizeof(IdType));
  }
}

Crystal::Crystal(Crystal&& other) noexcept
    : config_id_(other.config_id_), mesh_(std::move(other.mesh_)), cache_data_(std::move(other.cache_data_)),
      face_v_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kVtx),
      face_n_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kNormal),
      face_area_(cache_data_.get() + mesh_.GetTriangleCnt() * CrystalCachOffset::kArea),
      fn_map_(std::move(other.fn_map_)), fn_period_(other.fn_period_), poly_face_cnt_(other.poly_face_cnt_),
      poly_face_data_(std::move(other.poly_face_data_)), poly_face_of_tri_(std::move(other.poly_face_of_tri_)),
      cf_geom_(other.cf_geom_) {
  other.face_v_ = nullptr;
  other.face_n_ = nullptr;
  other.face_area_ = nullptr;

  if (poly_face_cnt_ > 0) {
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
  }
  other.poly_face_cnt_ = 0;
  other.poly_face_n_ = nullptr;
  other.poly_face_d_ = nullptr;
  other.poly_face_tri_id_ = nullptr;
}

Crystal& Crystal::operator=(const Crystal& other) {
  if (&other == this) {
    return *this;
  }

  config_id_ = other.config_id_;
  mesh_ = other.mesh_;

  auto n = other.TotalTriangles();
  cache_data_ = std::make_unique<float[]>(n * CrystalCachOffset::kTotal);
  face_v_ = cache_data_.get() + n * CrystalCachOffset::kVtx;
  face_n_ = cache_data_.get() + n * CrystalCachOffset::kNormal;
  face_area_ = cache_data_.get() + n * CrystalCachOffset::kArea;
  fn_map_ = std::make_unique<IdType[]>(n);

  std::memcpy(cache_data_.get(), other.cache_data_.get(), n * CrystalCachOffset::kTotal * sizeof(float));
  std::memcpy(fn_map_.get(), other.fn_map_.get(), n * sizeof(IdType));

  fn_period_ = other.fn_period_;

  poly_face_cnt_ = other.poly_face_cnt_;
  if (poly_face_cnt_ > 0) {
    poly_face_data_ = std::make_unique<float[]>(poly_face_cnt_ * 5);
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
    std::memcpy(poly_face_data_.get(), other.poly_face_data_.get(), poly_face_cnt_ * 5 * sizeof(float));
  } else {
    poly_face_data_.reset();
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
  }
  if (other.poly_face_of_tri_) {
    poly_face_of_tri_ = std::make_unique<IdType[]>(n);
    std::memcpy(poly_face_of_tri_.get(), other.poly_face_of_tri_.get(), n * sizeof(IdType));
  } else {
    poly_face_of_tri_.reset();
  }
  cf_geom_ = other.cf_geom_;
  return *this;
}

Crystal& Crystal::operator=(Crystal&& other) noexcept {
  if (&other == this) {
    return *this;
  }

  config_id_ = other.config_id_;
  mesh_ = std::move(other.mesh_);

  auto n = mesh_.GetTriangleCnt();
  cache_data_ = std::move(other.cache_data_);
  face_v_ = cache_data_.get() + n * CrystalCachOffset::kVtx;
  face_n_ = cache_data_.get() + n * CrystalCachOffset::kNormal;
  face_area_ = cache_data_.get() + n * CrystalCachOffset::kArea;
  other.face_v_ = nullptr;
  other.face_n_ = nullptr;
  other.face_area_ = nullptr;

  fn_map_ = std::move(other.fn_map_);
  fn_period_ = other.fn_period_;

  poly_face_cnt_ = other.poly_face_cnt_;
  poly_face_data_ = std::move(other.poly_face_data_);
  if (poly_face_cnt_ > 0) {
    poly_face_n_ = poly_face_data_.get();
    poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
    poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);
  } else {
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
  }
  other.poly_face_cnt_ = 0;
  other.poly_face_n_ = nullptr;
  other.poly_face_d_ = nullptr;
  other.poly_face_tri_id_ = nullptr;

  poly_face_of_tri_ = std::move(other.poly_face_of_tri_);

  cf_geom_ = other.cf_geom_;
  other.cf_geom_ = CrystalGeom{};

  return *this;
}

void Crystal::ComputeCacheData() {
  auto triangle_cnt = mesh_.GetTriangleCnt();
  const auto* vtx = mesh_.GetVtxPtr(0);
  const auto* triangle_idx = mesh_.GetTrianglePtr(0);
  float ev[6];
  for (size_t i = 0; i < triangle_cnt; i++) {
    // face_v_
    for (int j = 0; j < 3; j++) {
      std::memcpy(face_v_ + i * 9 + j * 3, vtx + triangle_idx[i * 3 + j] * 3, 3 * sizeof(float));
    }

    // edge vectors (local)
    for (int j = 0; j < 3; j++) {
      ev[j + 0] = face_v_[i * 9 + j + 3] - face_v_[i * 9 + j + 0];
      ev[j + 3] = face_v_[i * 9 + j + 6] - face_v_[i * 9 + j + 0];
    }

    // face_n_ && face_area_
    Cross3(ev + 0, ev + 3, face_n_ + i * 3);
    face_area_[i] = Norm3(face_n_ + i * 3) / 2.0f;
    Normalize3(face_n_ + i * 3);
  }
}

size_t Crystal::TotalTriangles() const {
  return mesh_.GetTriangleCnt();
}

const float* Crystal::GetTriangleVtx() const {
  return face_v_;
}

const float* Crystal::GetTriangleNormal() const {
  return face_n_;
}

const float* Crystal::GetTirangleArea() const {
  return face_area_;
}

IdType Crystal::PolygonFaceOfTri(int tri_id) const {
  if (tri_id < 0 || tri_id >= static_cast<int>(mesh_.GetTriangleCnt())) {
    return kInvalidId;
  }
  if (!poly_face_of_tri_) {
    // Mesh-only Crystal (custom-mesh or default-constructed): tri→poly-face
    // mapping is not derivable without the parametric slot table. Callers
    // must treat kInvalidId as "no polygon face for this triangle".
    return kInvalidId;
  }
  return poly_face_of_tri_[tri_id];
}

IdType Crystal::GetFn(IdType poly_idx) const {
  if (poly_idx == kInvalidId || poly_idx >= poly_face_cnt_) {
    return kInvalidId;
  }
  // Depth-of-defense: the inner index poly_face_tri_id_[poly_idx] is populated
  // by BuildPolygonFaceData; if any future upstream regression leaves a slot
  // uninitialized (the failure mode diagnosed on hex prisms with random
  // face_distance, now guarded at the mesh factory), this bound check turns a
  // wild fn_map_[garbage] read into a well-defined kInvalidId return. This is
  // NOT a root-cause fix — the real fix is at the mesh-construction boundary
  // (SolveConvexPolyhedronVtxD scale-relative dedup + Crystal factory Euler
  // gate). Leaving the defense here so a future upstream drift surfaces as a
  // detectable "no fn" symptom rather than a silent crash; do not treat this
  // bound check as license to relax the mesh-side invariants.
  const int tri = poly_face_tri_id_[poly_idx];
  if (tri < 0 || static_cast<size_t>(tri) >= mesh_.GetTriangleCnt()) {
    return kInvalidId;
  }
  return fn_map_[tri];
}

std::vector<IdType> Crystal::PCanonicalShift(const std::vector<IdType>& rp) const {
  std::vector<IdType> result = rp;
  IdType first_pri = kInvalidId;
  for (auto& x : result) {
    if (x < 3) {
      continue;
    }
    IdType pyr = x / 10;
    IdType pri = x % 10;
    if (first_pri == kInvalidId) {
      first_pri = pri;
    }
    pri += fn_period_ - first_pri;
    pri %= fn_period_;
    pri += 3;
    x = pyr * 10 + pri;
  }
  return result;
}

std::vector<IdType> Crystal::ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  return ReduceRaypath(rp, symmetry, 0, false);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<IdType> Crystal::ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                                           bool d_applicable) const {
  if (symmetry == FilterConfig::kSymNone || fn_period_ < 0) {
    return rp;
  }

  std::vector<IdType> reduced_rp = rp;
  if (symmetry & FilterConfig::kSymP) {
    reduced_rp = PCanonicalShift(reduced_rp);
  }

  if ((symmetry & FilterConfig::kSymD) && d_applicable) {
    // σ-clean: reflect every prism face using formula face_id_new = (sigma_a - (face_id-3) + fn_period_) % fn_period_ +
    // 3
    std::vector<IdType> rp_reflected = reduced_rp;
    for (auto& x : rp_reflected) {
      if (x < 3) {
        continue;  // skip basal faces; pyramid faces use same pyr/pri decomposition
      }
      IdType pyr = x / 10;
      IdType pri = x % 10 - 3;
      pri = (sigma_a - pri + fn_period_) % fn_period_;
      x = pyr * 10 + pri + 3;
    }
    // When kSymP is also enabled, the D-image may no longer be P-canonical
    // (first pri shifted by sigma_a); re-canonicalize before lex comparison
    // so same orbit always reduces to the same representative.
    if (symmetry & FilterConfig::kSymP) {
      rp_reflected = PCanonicalShift(rp_reflected);
    }
    if (rp_reflected < reduced_rp) {
      reduced_rp = rp_reflected;
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    // B reflection: basal 1↔2 and pyramid upper[13..18]↔lower[23..28], applied together.
    // Generate the B-reflected candidate and keep the lexicographically smaller one.
    std::vector<IdType> rp_b_reflected = reduced_rp;
    bool changed = false;
    for (auto& x : rp_b_reflected) {
      if (x <= 2) {
        x = 3 - x;
        changed = true;
      } else if (x >= 13 && x <= 18) {
        x += 10;
        changed = true;
      } else if (x >= 23 && x <= 28) {
        x -= 10;
        changed = true;
      }
    }
    if (changed && rp_b_reflected < reduced_rp) {
      reduced_rp = rp_b_reflected;
    }
  }

  return reduced_rp;
}

std::vector<std::vector<IdType>> Crystal::ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const {
  return ExpandRaypath(rp, symmetry, 0, false);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<std::vector<IdType>> Crystal::ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                                                        bool d_applicable) const {
  std::vector<std::vector<IdType>> result;
  result.emplace_back(rp);
  if (symmetry == FilterConfig::kSymNone || fn_period_ < 0) {
    return result;
  }

  if (symmetry & FilterConfig::kSymP) {
    for (int i = 1; i < fn_period_; i++) {
      std::vector<IdType> curr_rp{ rp };
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x < 3) {
          continue;
        }

        IdType pyr = x / 10;
        IdType pri = x % 10;
        pri += fn_period_ - 3;
        pri += i;
        pri %= fn_period_;
        pri += 3;
        x = pyr * 10 + pri;
        changed = true;
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  if ((symmetry & FilterConfig::kSymD) && d_applicable) {
    // σ-clean: for each existing variant, generate σ-reflected copy
    auto size = result.size();
    for (size_t i = 0; i < size; i++) {
      auto curr_rp = result[i];
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x < 3) {
          continue;  // skip basal faces; pyramid faces use same pyr/pri decomposition
        }
        IdType pyr = x / 10;
        IdType pri = x % 10 - 3;
        IdType pri_new = (sigma_a - pri + fn_period_) % fn_period_;
        IdType x_new = pyr * 10 + pri_new + 3;
        if (x_new != x) {
          x = x_new;
          changed = true;
        }
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  if (symmetry & FilterConfig::kSymB) {
    // B reflection: basal 1↔2, pyramid upper[13..18]↔lower[23..28], prism unchanged.
    // Both swaps are part of the same transformation and applied together.
    auto size = result.size();
    for (size_t i = 0; i < size; i++) {
      auto curr_rp = result[i];
      bool changed = false;
      for (auto& x : curr_rp) {
        if (x <= 2) {
          x = 3 - x;
          changed = true;
        } else if (x >= 13 && x <= 18) {
          x += 10;
          changed = true;
        } else if (x >= 23 && x <= 28) {
          x -= 10;
          changed = true;
        }
      }
      if (changed) {
        result.emplace_back(curr_rp);
      }
    }
  }

  return result;
}

size_t Crystal::PolygonFaceCount() const {
  return poly_face_cnt_;
}

const float* Crystal::GetPolygonFaceNormal() const {
  return poly_face_n_;
}

const float* Crystal::GetPolygonFaceDist() const {
  return poly_face_d_;
}

const int* Crystal::GetPolygonFaceTriId() const {
  return poly_face_tri_id_;
}

// Triangle→argmax-plane grouping (task-geometry-gen-numerical-robustness Step 2):
// each triangle is assigned to the SINGLE plane with the highest |n_tri · n_plane| dot
// (argmax over planes per triangle). Previously the loop was plane-driven: each plane
// claimed every triangle with dot > 1-1e-3, which let the always-present basal plane
// (0,0,±1) steal near-horizontal pyramid triangles whose z-component crept above 0.999
// at wedge ≥ 87.44° — producing a phantom flat basal face on extreme-wedge bipyramids
// (B-ring 87.5° bug). A geometric triangle lies on exactly one plane; argmax enforces
// that. A planar floor (best_dot > 1 - kFaceCoplanarFloor) still rejects triangles that
// match no plane within float noise (signals geometry-build inconsistency).
//
// Phantom-plane safety: planes with no triangle whose argmax lands on them are silently
// dropped (no entry in poly_face_*). Same external contract as before, just the
// pigeon-holing direction is reversed.
void Crystal::BuildPolygonFaceData(const float* plane_coef, size_t plane_cnt) {
  static_assert(sizeof(int) == sizeof(float), "BuildPolygonFaceData requires sizeof(int)==sizeof(float)");

  auto tri_cnt = TotalTriangles();

  // kFaceCoplanarFloor: a triangle whose best-match plane dot is below (1 - this) is
  // considered to match no plane (≈8.1° angular slack). The basal-plane phantom-grab at
  // wedge ≥ 87.44° produced dots of ~0.9990; the previous threshold (1 - 1e-3 = 0.999)
  // sat right on this number and accepted it. The floor here is only a sanity gate
  // against degenerate mesh build — the assignment itself is by argmax, not by absolute
  // threshold, so it is robust to which side of 0.999 the wedge happens to land on.
  constexpr float kFaceCoplanarFloor = 1e-2f;

  // Pre-normalize plane equations once; skip degenerate (zero-norm) planes.
  std::vector<float> plane_n(plane_cnt * 3, 0.0f);
  std::vector<float> plane_d(plane_cnt, 0.0f);
  std::vector<char> plane_active(plane_cnt, 0);
  for (size_t p = 0; p < plane_cnt; p++) {
    const float* coef = plane_coef + p * 4;
    float norm_len = Norm3(coef);
    if (norm_len < math::kFloatEps) {
      continue;
    }
    plane_n[p * 3 + 0] = coef[0] / norm_len;
    plane_n[p * 3 + 1] = coef[1] / norm_len;
    plane_n[p * 3 + 2] = coef[2] / norm_len;
    plane_d[p] = coef[3] / norm_len;
    plane_active[p] = 1;
  }

  // For each triangle, find its argmax plane (the single best match). Track per-plane
  // the representative triangle = the one with the largest dot among triangles assigned
  // to that plane.
  std::vector<int> plane_best_tri(plane_cnt, -1);
  std::vector<float> plane_best_dot(plane_cnt, -1.0f);
  for (size_t t = 0; t < tri_cnt; t++) {
    const float* n_tri = face_n_ + t * 3;
    int tri_best_plane = -1;
    float tri_best_dot = -1.0f;
    for (size_t p = 0; p < plane_cnt; p++) {
      if (!plane_active[p]) {
        continue;
      }
      float dot = Dot3(plane_n.data() + p * 3, n_tri);
      if (dot > tri_best_dot) {
        tri_best_dot = dot;
        tri_best_plane = static_cast<int>(p);
      }
    }
    if (tri_best_plane < 0 || tri_best_dot < 1.0f - kFaceCoplanarFloor) {
      LOG_WARNING("BuildPolygonFaceData: triangle {} has no coplanar plane (best_dot={:.4f})", t,
                  static_cast<double>(tri_best_dot));
      continue;
    }
    if (tri_best_dot > plane_best_dot[tri_best_plane]) {
      plane_best_dot[tri_best_plane] = tri_best_dot;
      plane_best_tri[tri_best_plane] = static_cast<int>(t);
    }
  }

  // Degenerate-face safety net: reject planes whose representative triangle has
  // near-zero area relative to the largest triangle. This signals an upstream
  // geometry-gen artifact (a polygon plane survived without a real surface); in
  // current pipeline Triangulate already skips faces with <3 vertices, so this
  // gate is belt-and-suspenders for future configs (asymmetric d[6], near-degenerate
  // apex collapse, etc.). max_tri_area only depends on face_area_ which is populated
  // by ComputeCacheData before this function runs, so we can compute it up-front —
  // it does not depend on the accept/reject decision.
  float max_tri_area = 0.0f;
  for (size_t t = 0; t < tri_cnt; t++) {
    max_tri_area = std::max(max_tri_area, face_area_[t]);
  }

  // Single-pass acceptance: collect surviving (plane_idx, rep_tri) pairs. The
  // acceptance predicate — "plane won a triangle" AND "representative triangle
  // is not degenerate" — is evaluated exactly once here. A previous two-phase
  // form evaluated only the first half to size the allocation, then re-applied
  // the second half in the fill loop and rewrote poly_face_cnt_ to the final
  // written count; that left the allocated memory laid out at the larger
  // stride while poly_face_cnt_ shrank, so copy/move ctors of Crystal
  // re-derived the pointer offsets from the shrunk count and read
  // poly_face_d_ / poly_face_tri_id_ from wrong regions of poly_face_data_,
  // causing Metal SIGSEGV and CPU silent-wrong-fn on pyramid+random
  // face_distance configs. Now poly_face_cnt_ ≡ accepted.size() and equals
  // the actual allocated stride from first assignment onward.
  struct AcceptedFace {
    size_t plane_idx;
    int rep_tri;
  };
  std::vector<AcceptedFace> accepted;
  accepted.reserve(plane_cnt);
  for (size_t p = 0; p < plane_cnt; p++) {
    if (plane_best_tri[p] < 0) {
      continue;
    }
    int rep_tri = plane_best_tri[p];
    if (max_tri_area > 0.0f && face_area_[rep_tri] < 1e-6f * max_tri_area) {
      LOG_WARNING("BuildPolygonFaceData: plane {} has degenerate rep triangle {} (area={:.4e}, max={:.4e})", p, rep_tri,
                  static_cast<double>(face_area_[rep_tri]), static_cast<double>(max_tri_area));
      g_degenerate_shrink_count.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    accepted.push_back({ p, rep_tri });
  }

  poly_face_cnt_ = accepted.size();
  if (poly_face_cnt_ == 0) {
    poly_face_data_.reset();
    poly_face_n_ = nullptr;
    poly_face_d_ = nullptr;
    poly_face_tri_id_ = nullptr;
    return;
  }

  // Allocate: normals(3*cnt) + dist(cnt) + tri_id as int(cnt) = 5*cnt floats.
  // stride is fixed by poly_face_cnt_ (= accepted.size()) and never re-derived
  // from a shrunk count elsewhere.
  poly_face_data_ = std::make_unique<float[]>(poly_face_cnt_ * 5);
  poly_face_n_ = poly_face_data_.get();
  poly_face_d_ = poly_face_data_.get() + poly_face_cnt_ * 3;
  poly_face_tri_id_ = reinterpret_cast<int*>(poly_face_data_.get() + poly_face_cnt_ * 4);

  for (size_t i = 0; i < poly_face_cnt_; i++) {
    const auto& a = accepted[i];
    poly_face_n_[i * 3 + 0] = plane_n[a.plane_idx * 3 + 0];
    poly_face_n_[i * 3 + 1] = plane_n[a.plane_idx * 3 + 1];
    poly_face_n_[i * 3 + 2] = plane_n[a.plane_idx * 3 + 2];
    poly_face_d_[i] = plane_d[a.plane_idx];
    poly_face_tri_id_[i] = a.rep_tri;
  }
}

float Crystal::GetRefractiveIndex(float wl) const {
  return IceRefractiveIndex::Get(wl);
}


namespace detail {

bool IsRollMeanAtMultipleOf30(const AxisDistribution& d) {
  float mean = d.roll_dist.mean;
  float remainder = std::fmod(std::fmod(mean, 30.0f) + 30.0f, 30.0f);
  return FloatEqual(remainder, 0.0f) || FloatEqual(remainder, 30.0f);
}

int ComputeSigmaA(float roll_mean_deg) {
  if (std::fabs(roll_mean_deg) > 1e6f) {
    return 0;
  }
  int n = (static_cast<int>(std::round(roll_mean_deg / 30.0f)) % 6 + 6) % 6;
  return (6 - n) % 6;
}

bool IsDApplicable(const AxisDistribution& d) {
  return d.IsAzRotationallySymmetric() && IsRollMeanAtMultipleOf30(d);
}

}  // namespace detail


}  // namespace lumice
