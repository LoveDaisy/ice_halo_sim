// Analytic ground-truth oracle for crystal entry-point incidence sampling.
// Header-only test support (mirrors test/support/exact_prism_oracle.hpp).
//
// Purpose (口径 B — distribution invariance, NOT bit-exact): the entry-point
// sampler `InitRay_p_fid` picks a face area-weighted by projected area and then
// a point uniformly within the chosen face. The correct target distribution is
// a CLOSED FORM of the crystal geometry alone, independent of how the sampler is
// implemented (triangle-granularity today, polygon-granularity after T2/T3). So
// its correctness can be judged against an analytic oracle rather than against a
// frozen reference sampler. This header IS that oracle, plus the statistical
// comparators that turn "observed samples" into a pass/fail + quantified
// deviation.
//
// Three layers, deliberately decoupled so a bug in one cannot mask a bug in
// another (doc/numerical-robustness.md rule 4 "one predicate, one owner"):
//   1. Analytic math (Newell area / 2D polygon + triangle moments) — pure
//      geometry, no sampling, self-checkable against hand-computed values.
//   2. Target distributions built from `CrystalGeom` (present-slot compact order,
//      matching `Crystal::PolygonFaceOfTri` / `RaySeg::to_face_`).
//   3. Comparators (AC1 per-face projected-area distribution, AC2 in-face
//      uniformity) that consume an observed sample set and emit a verdict.
//
// Fan-triangulation target: the target is defined over the FAN-TRIANGULATION of
// each face (fanned from corner 0, matching BuildMeshFromCfGeom), not a planar
// polygon. For a planar convex face this is IDENTICAL to the whole-polygon target
// (projected area = shoelace × cos; in-face = uniform-over-polygon); for a
// degenerate, slightly non-planar face (some closed-form pyramid cone faces) the
// fan is the honest description of the actual bent surface both the current
// triangle sampler AND the future polygon sampler operate on. So it is the 口径-B
// distribution-invariant target — see the DECISION note in the task progress log.
//
// Independence discipline (a02): the oracle computes each fan triangle's normal
// from its corners with the SAME winding as the mesh (Cross3(v1-v0, v2-v0)) and
// its area from the corners — geometry the sampler also consumes, but the oracle
// re-derives it independently and judges the *distribution*. The red-state test
// (a deliberately sign-flipped weight) confirms the comparator still has teeth,
// so this is not a tautology.
//
// Present-slot compact ordering: both `PopulateFromCfGeom` (which feeds
// `PolygonFaceOfTri` → `to_face_`) and the oracle iterate `CrystalGeom` slots in
// ascending order, skipping absent slots and numbering the survivors 0,1,2,…
// So `ComputeProjectedFaceAreaDistribution` returns a vector whose index i is
// exactly the `to_face_` value the sampler writes — no mapping table needed.

#ifndef LUMICE_TEST_SUPPORT_INCIDENCE_SAMPLING_ORACLE_HPP_
#define LUMICE_TEST_SUPPORT_INCIDENCE_SAMPLING_ORACLE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/trace_ops.hpp"

namespace lumice {
namespace test_support {

// ---------------------------------------------------------------------------
// Layer 1 — pure analytic geometry (double precision; no sampling).
// All vertex inputs are flat [k * 3 + xyz] double arrays so the self-check can
// feed hand-written literal polygons (unit square, regular hexagon).
// ---------------------------------------------------------------------------

inline double OracleDot3(const double a[3], const double b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void OracleCross3(const double a[3], const double b[3], double out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

inline double OracleNorm3(const double a[3]) {
  return std::sqrt(OracleDot3(a, a));
}

// Newell area of a planar 3D polygon: 0.5 * |Σ_i V_i × V_{i+1}|. Matches the
// vector-area convention used by test_closed_form_pyramid.cpp::FacePolygonArea.
inline double PolygonArea3D(const double* vertices, int vtx_cnt) {
  if (vtx_cnt < 3) {
    return 0.0;
  }
  double acc[3] = { 0.0, 0.0, 0.0 };
  for (int k = 0; k < vtx_cnt; k++) {
    const double* a = vertices + k * 3;
    const double* b = vertices + ((k + 1) % vtx_cnt) * 3;
    double c[3];
    OracleCross3(a, b, c);
    acc[0] += c[0];
    acc[1] += c[1];
    acc[2] += c[2];
  }
  return 0.5 * OracleNorm3(acc);
}

// Newell unit normal (un-oriented; direction follows vertex winding).
inline void PolygonNewellNormal(const double* vertices, int vtx_cnt, double out_unit_normal[3]) {
  double acc[3] = { 0.0, 0.0, 0.0 };
  for (int k = 0; k < vtx_cnt; k++) {
    const double* a = vertices + k * 3;
    const double* b = vertices + ((k + 1) % vtx_cnt) * 3;
    double c[3];
    OracleCross3(a, b, c);
    acc[0] += c[0];
    acc[1] += c[1];
    acc[2] += c[2];
  }
  const double n = OracleNorm3(acc);
  if (n > 0.0) {
    out_unit_normal[0] = acc[0] / n;
    out_unit_normal[1] = acc[1] / n;
    out_unit_normal[2] = acc[2] / n;
  } else {
    out_unit_normal[0] = out_unit_normal[1] = out_unit_normal[2] = 0.0;
  }
}

// Orthonormal 2D basis on a face: origin = vertex 0, e1 along the first edge,
// e2 = normal × e1. `unit_normal` must be a unit vector normal to the face.
struct FaceBasis2D {
  double origin[3];
  double e1[3];
  double e2[3];
};

inline FaceBasis2D BuildFaceLocalBasis(const double* vertices, int vtx_cnt, const double unit_normal[3]) {
  FaceBasis2D b{};
  b.origin[0] = vertices[0];
  b.origin[1] = vertices[1];
  b.origin[2] = vertices[2];
  double edge[3] = { 0, 0, 0 };
  if (vtx_cnt >= 2) {
    edge[0] = vertices[3] - vertices[0];
    edge[1] = vertices[4] - vertices[1];
    edge[2] = vertices[5] - vertices[2];
  }
  const double en = OracleNorm3(edge);
  if (en > 0.0) {
    b.e1[0] = edge[0] / en;
    b.e1[1] = edge[1] / en;
    b.e1[2] = edge[2] / en;
  }
  OracleCross3(unit_normal, b.e1, b.e2);
  const double e2n = OracleNorm3(b.e2);
  if (e2n > 0.0) {
    b.e2[0] /= e2n;
    b.e2[1] /= e2n;
    b.e2[2] /= e2n;
  }
  return b;
}

inline void ProjectToLocal2D(const double p[3], const FaceBasis2D& basis, double* u, double* v) {
  const double r[3] = { p[0] - basis.origin[0], p[1] - basis.origin[1], p[2] - basis.origin[2] };
  *u = OracleDot3(r, basis.e1);
  *v = OracleDot3(r, basis.e2);
}

// Analytic moments of the UNIFORM distribution over a simple 2D polygon.
// Standard shoelace-based formulas. Returns centroid (cu, cv) and the central
// second moments (var_u, var_v, cov_uv) of a uniformly-drawn point — i.e. the
// expected sample mean and covariance.
struct Polygon2DMoments {
  double area = 0.0;
  double cu = 0.0, cv = 0.0;
  double var_u = 0.0, var_v = 0.0, cov_uv = 0.0;
};

inline Polygon2DMoments ComputePolygon2DMoments(const double* uv, int n) {
  Polygon2DMoments m;
  if (n < 3) {
    return m;
  }
  double a2 = 0.0;  // 2*area
  double cx = 0.0;
  double cy = 0.0;
  double ixx = 0.0;  // ∫ x² over polygon
  double iyy = 0.0;  // ∫ y² over polygon
  double ixy = 0.0;  // ∫ xy over polygon
  for (int i = 0; i < n; i++) {
    const double x0 = uv[i * 2 + 0];
    const double y0 = uv[i * 2 + 1];
    const double x1 = uv[((i + 1) % n) * 2 + 0];
    const double y1 = uv[((i + 1) % n) * 2 + 1];
    const double cross = x0 * y1 - x1 * y0;
    a2 += cross;
    cx += (x0 + x1) * cross;
    cy += (y0 + y1) * cross;
    ixx += (x0 * x0 + x0 * x1 + x1 * x1) * cross;
    iyy += (y0 * y0 + y0 * y1 + y1 * y1) * cross;
    ixy += (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) * cross;
  }
  const double area = 0.5 * a2;
  m.area = std::abs(area);
  if (std::abs(area) < 1e-30) {
    return m;
  }
  m.cu = cx / (6.0 * area);
  m.cv = cy / (6.0 * area);
  // Second moments about the origin, per-unit-area, then shift to centroid.
  const double mxx = (ixx / 12.0) / area;  // E[x²]
  const double myy = (iyy / 12.0) / area;  // E[y²]
  const double mxy = (ixy / 24.0) / area;  // E[xy]
  m.var_u = mxx - m.cu * m.cu;
  m.var_v = myy - m.cv * m.cv;
  m.cov_uv = mxy - m.cu * m.cv;
  return m;
}

// Moments of the UNIFORM distribution over one 2D triangle. q = 3 corners
// (u, v). Closed form: mean = vertex average; central second moments =
// (1/36)[3·Σ qᵢ² − (Σ qᵢ)²]. (Verified against the unit right triangle: var =
// 1/18.) For a convex polygon fanned from vertex 0, the area-weighted mixture of
// these per-triangle moments equals ComputePolygon2DMoments — the fan and the
// whole-polygon computation agree (asserted in OracleMathSelfProof).
inline Polygon2DMoments ComputeTriangle2DMoments(const double q[6]) {
  Polygon2DMoments m;
  const double x0 = q[0];
  const double y0 = q[1];
  const double x1 = q[2];
  const double y1 = q[3];
  const double x2 = q[4];
  const double y2 = q[5];
  m.area = 0.5 * std::abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
  m.cu = (x0 + x1 + x2) / 3.0;
  m.cv = (y0 + y1 + y2) / 3.0;
  const double sx = x0 + x1 + x2;
  const double sy = y0 + y1 + y2;
  m.var_u = (3.0 * (x0 * x0 + x1 * x1 + x2 * x2) - sx * sx) / 36.0;
  m.var_v = (3.0 * (y0 * y0 + y1 * y1 + y2 * y2) - sy * sy) / 36.0;
  m.cov_uv = (3.0 * (x0 * y0 + x1 * y1 + x2 * y2) - sx * sy) / 36.0;
  return m;
}

// ---------------------------------------------------------------------------
// Layer 2 — target distributions & per-face geometry from CrystalGeom.
//
// The target is defined over the FAN-TRIANGULATION of each face (fanned from
// corner 0, matching BuildMeshFromCfGeom), NOT a planar polygon. For a planar
// convex face this is identical to the whole-polygon target; for a degenerate,
// slightly non-planar face (some closed-form pyramid cone faces) the fan is the
// honest description of the actual bent surface that BOTH the current triangle
// sampler and the future polygon sampler operate on. So it is the 口径-B
// distribution-invariant target, not a mirror of any one sampler.
// ---------------------------------------------------------------------------

// One fan triangle of a face: 3D corners + outward unit normal + 3D area, plus
// the corners projected into the face's 2D basis (for the AC2 moment target).
struct OracleFanTri {
  double n[3] = {};  // outward unit normal (3D, from corners; oriented by plane_coef)
  double area3d = 0.0;
  double uv[6] = {};  // 3 corners in the face 2D basis
};

// Per-present-face analytic geometry, indexed by compact present order (== the
// `to_face_` value the sampler writes).
struct OracleFaceGeom {
  int slot = -1;
  int vtx_cnt = 0;
  std::vector<double> vtx;         // vtx_cnt * 3, double (face corners)
  double normal[3] = {};           // OUTWARD unit normal from plane_coef (basis + orient)
  FaceBasis2D basis;               // 2D basis (for AC2 projection)
  std::vector<OracleFanTri> tris;  // fan triangles from corner 0
};

inline void GatherFaceVertices(const CrystalGeom& cf, int slot, std::vector<double>* out) {
  const int cnt = cf.face_vtx_cnt[slot];
  out->resize(static_cast<size_t>(cnt) * 3);
  for (int k = 0; k < cnt; k++) {
    const float* v = cf.face_vtx + (static_cast<size_t>(slot) * kCrystalGeomMaxVtxPerFace + k) * 3;
    (*out)[k * 3 + 0] = static_cast<double>(v[0]);
    (*out)[k * 3 + 1] = static_cast<double>(v[1]);
    (*out)[k * 3 + 2] = static_cast<double>(v[2]);
  }
}

// Outward unit normal of a face from its plane coefficients. CrystalGeom's
// convention is `a·x + b·y + c·z + d ≤ 0` for the bounded (interior) half-space,
// so the gradient (a, b, c) points toward increasing value = outside = outward.
// Used to orient per-triangle normals and to build the 2D basis. Independent of
// the SAMPLER's mesh-triangle normals (a different route to the same geometry).
inline void PlaneOutwardNormal(const CrystalGeom& cf, int slot, double out_unit[3]) {
  const float* coef = cf.plane_coef + static_cast<size_t>(slot) * 4;
  double n[3] = { static_cast<double>(coef[0]), static_cast<double>(coef[1]), static_cast<double>(coef[2]) };
  const double len = OracleNorm3(n);
  if (len > 0.0) {
    out_unit[0] = n[0] / len;
    out_unit[1] = n[1] / len;
    out_unit[2] = n[2] / len;
  } else {
    out_unit[0] = out_unit[1] = out_unit[2] = 0.0;
  }
}

inline std::vector<OracleFaceGeom> BuildPresentFaceGeom(const CrystalGeom& cf) {
  std::vector<OracleFaceGeom> faces;
  for (int slot = 0; slot < cf.face_cnt; slot++) {
    if (!cf.face_present[slot]) {
      continue;
    }
    OracleFaceGeom g;
    g.slot = slot;
    g.vtx_cnt = cf.face_vtx_cnt[slot];
    GatherFaceVertices(cf, slot, &g.vtx);
    PlaneOutwardNormal(cf, slot, g.normal);
    g.basis = BuildFaceLocalBasis(g.vtx.data(), g.vtx_cnt, g.normal);
    // Fan from corner 0: triangles (0, k, k+1). Matches BuildMeshFromCfGeom.
    for (int k = 1; k + 1 < g.vtx_cnt; k++) {
      const double* p0 = g.vtx.data() + 0;
      const double* p1 = g.vtx.data() + k * 3;
      const double* p2 = g.vtx.data() + (k + 1) * 3;
      OracleFanTri t;
      // 3D area + normal from the triangle edges.
      double e1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
      double e2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
      double cr[3];
      OracleCross3(e1, e2, cr);
      const double crn = OracleNorm3(cr);
      t.area3d = 0.5 * crn;
      if (crn > 0.0) {
        // Raw fan-winding normal — IDENTICAL to the mesh's per-triangle normal
        // (crystal.cpp::ComputeCacheData computes Cross3(v1-v0, v2-v0) with the
        // same fan winding, no outward reorientation). Reorienting to plane_coef
        // here would disagree with the mesh on strongly-bent degenerate faces
        // (a fan triangle whose winding normal is >90° from the face plane).
        t.n[0] = cr[0] / crn;
        t.n[1] = cr[1] / crn;
        t.n[2] = cr[2] / crn;
      }
      ProjectToLocal2D(p0, g.basis, &t.uv[0], &t.uv[1]);
      ProjectToLocal2D(p1, g.basis, &t.uv[2], &t.uv[3]);
      ProjectToLocal2D(p2, g.basis, &t.uv[4], &t.uv[5]);
      g.tris.push_back(t);
    }
    faces.push_back(std::move(g));
  }
  return faces;
}

// Per-triangle projected area weight (the sampler's per-triangle weight):
// max(-d·n_tri · A_tri, 0).
inline double FanTriProjectedArea(const OracleFanTri& t, const double d[3]) {
  double proj = -OracleDot3(d, t.n) * t.area3d;
  return (proj > 0.0) ? proj : 0.0;
}

// Target per-face selection probability: normalized sum of the fan triangles'
// projected areas, in present-slot compact order. Index i == `to_face_` == i.
inline std::vector<double> ComputeProjectedFaceAreaDistribution(const CrystalGeom& cf, const float d[3]) {
  const double dd[3] = { static_cast<double>(d[0]), static_cast<double>(d[1]), static_cast<double>(d[2]) };
  std::vector<OracleFaceGeom> faces = BuildPresentFaceGeom(cf);
  std::vector<double> w(faces.size(), 0.0);
  double sum = 0.0;
  for (size_t i = 0; i < faces.size(); i++) {
    double proj = 0.0;
    for (const OracleFanTri& t : faces[i].tris) {
      proj += FanTriProjectedArea(t, dd);
    }
    w[i] = proj;
    sum += proj;
  }
  if (sum > 0.0) {
    for (double& x : w) {
      x /= sum;
    }
  }
  return w;
}

// Analytic in-face target moments (2D, in the face basis) of the sampler's
// conditional point distribution: an area-weighted mixture over the fan
// triangles, weighted by each triangle's PROJECTED area (the same weight the
// global selection uses). For a planar face all triangles share a normal, so the
// weights reduce to plain area and the mixture is uniform over the polygon.
inline Polygon2DMoments ComputeInFaceTargetMoments(const OracleFaceGeom& g, const double d[3]) {
  double w_sum = 0.0;
  double mu_u = 0.0;
  double mu_v = 0.0;
  // First pass: weighted mean.
  std::vector<double> w(g.tris.size(), 0.0);
  std::vector<Polygon2DMoments> tm(g.tris.size());
  for (size_t i = 0; i < g.tris.size(); i++) {
    w[i] = FanTriProjectedArea(g.tris[i], d);
    tm[i] = ComputeTriangle2DMoments(g.tris[i].uv);
    w_sum += w[i];
    mu_u += w[i] * tm[i].cu;
    mu_v += w[i] * tm[i].cv;
  }
  Polygon2DMoments out;
  if (w_sum <= 0.0) {
    return out;
  }
  mu_u /= w_sum;
  mu_v /= w_sum;
  // Second pass: mixture covariance = Σ wᵢ (Covᵢ + gᵢgᵢᵀ) / Σw − μμᵀ.
  double e_uu = 0.0;
  double e_vv = 0.0;
  double e_uv = 0.0;
  for (size_t i = 0; i < g.tris.size(); i++) {
    e_uu += w[i] * (tm[i].var_u + tm[i].cu * tm[i].cu);
    e_vv += w[i] * (tm[i].var_v + tm[i].cv * tm[i].cv);
    e_uv += w[i] * (tm[i].cov_uv + tm[i].cu * tm[i].cv);
  }
  e_uu /= w_sum;
  e_vv /= w_sum;
  e_uv /= w_sum;
  out.area = w_sum;  // reuse: total projected weight (not geometric area)
  out.cu = mu_u;
  out.cv = mu_v;
  out.var_u = e_uu - mu_u * mu_u;
  out.var_v = e_vv - mu_v * mu_v;
  out.cov_uv = e_uv - mu_u * mu_v;
  return out;
}

// ---------------------------------------------------------------------------
// Layer 3 — sample drivers & comparators.
// ---------------------------------------------------------------------------

// One captured entry-point sample: the face hit (to_face_) and the 3D point.
struct EntrySamples {
  std::vector<IdType> face;                 // to_face_ per ray (kInvalidId if unmapped)
  std::vector<std::array<float, 3>> point;  // p_ per ray
};

// Drive the REAL production sampler `InitRay_p_fid` for `n` rays, all with
// crystal-local direction `d`. Seeds the global RNG singleton (the production
// primitives `RandomSample` / `SampleTrianglePoint` read
// `RandomNumberGenerator::GetInstance()`, so reproducibility comes from seeding
// that instance — a passed-in RNG would be ignored by the production path).
inline EntrySamples DriveEntrySampling(const Crystal& crystal, const float d[3], size_t n, uint32_t seed) {
  RandomNumberGenerator::GetInstance().SetSeed(seed);
  RayBuffer buf(n);
  buf.size_ = n;
  for (size_t i = 0; i < n; i++) {
    RaySeg& r = buf[i];
    r.d_[0] = d[0];
    r.d_[1] = d[1];
    r.d_[2] = d[2];
    r.w_ = 1.0f;
    r.from_face_ = kInvalidId;
    r.to_face_ = kInvalidId;
  }
  InitRay_p_fid(crystal, &buf);
  EntrySamples out;
  out.face.resize(n);
  out.point.resize(n);
  for (size_t i = 0; i < n; i++) {
    out.face[i] = buf[i].to_face_;
    out.point[i] = { buf[i].p_[0], buf[i].p_[1], buf[i].p_[2] };
  }
  return out;
}

// ---- AC1: per-face projected-area distribution ----
struct Ac1Verdict {
  bool pass = false;
  size_t sample_n = 0;
  double max_abs_z = 0.0;                 // max |z-score| over positive-weight faces
  double max_relative_dev = 0.0;          // max |p_hat - p| / p over positive-weight faces
  int worst_face = -1;                    // face with max |z|
  bool zero_weight_leak = false;          // a zero-target face received samples
  std::vector<double> expected_prob;      // per compact-present face
  std::vector<long long> observed_count;  // per compact-present face
};

// Compare an observed face-histogram against the analytic target. `k_sigma` is
// the two-sided binomial acceptance width. `min_expected` gates the z-test to
// faces with a reliable normal approximation; below it we require only that the
// observed count stays within the same loose binomial band (still catches gross
// bias) without demanding sub-count precision.
inline Ac1Verdict CheckProjectedAreaDistribution(const Crystal& crystal, const float d[3],
                                                 const std::vector<IdType>& observed_faces, double k_sigma,
                                                 double min_expected = 30.0) {
  Ac1Verdict v;
  v.sample_n = observed_faces.size();
  v.expected_prob = ComputeProjectedFaceAreaDistribution(crystal.CfGeom(), d);
  const size_t face_cnt = v.expected_prob.size();
  v.observed_count.assign(face_cnt, 0);
  for (IdType f : observed_faces) {
    if (f != kInvalidId && f < face_cnt) {
      v.observed_count[f]++;
    }
  }
  const double n = static_cast<double>(v.sample_n);
  v.pass = true;
  for (size_t f = 0; f < face_cnt; f++) {
    const double p = v.expected_prob[f];
    const long long o = v.observed_count[f];
    const double e = n * p;
    if (p <= 0.0) {
      // A zero-target (back / grazing-out) face must NEVER be selected: the
      // area-weight is exactly 0, so RandomSample cannot pick it.
      if (o > 0) {
        v.zero_weight_leak = true;
        v.pass = false;
      }
      continue;
    }
    const double sigma = std::sqrt(n * p * (1.0 - p));
    const double z = (sigma > 0.0) ? (static_cast<double>(o) - e) / sigma : 0.0;
    const double az = std::abs(z);
    const double rel = std::abs(static_cast<double>(o) / n - p) / p;
    if (az > v.max_abs_z) {
      v.max_abs_z = az;
      v.worst_face = static_cast<int>(f);
    }
    if (rel > v.max_relative_dev) {
      v.max_relative_dev = rel;
    }
    if (e >= min_expected && az > k_sigma) {
      v.pass = false;
    } else if (e < min_expected) {
      // Loose band for rarely-selected faces: reject only gross departures.
      if (std::abs(static_cast<double>(o) - e) > k_sigma * (sigma + std::sqrt(e) + 1.0)) {
        v.pass = false;
      }
    }
  }
  return v;
}

inline Ac1Verdict SampleAndCheckProjectedAreaDistribution(const Crystal& crystal, const float d[3], size_t n,
                                                          uint32_t seed, double k_sigma) {
  EntrySamples s = DriveEntrySampling(crystal, d, n, seed);
  return CheckProjectedAreaDistribution(crystal, d, s.face, k_sigma);
}

// ---- AC2: in-face uniformity ----
struct Ac2FaceResult {
  int face = -1;
  long long count = 0;
  double centroid_dev_sigma = 0.0;   // |sample_mean - centroid| in units of SE
  double moment_relative_dev = 0.0;  // |tr(sample_cov) - tr(analytic_cov)| / tr(analytic_cov)
  double moment_dev_sigma = 0.0;     // moment_relative_dev in units of its count-scaled SE
  bool checked = false;              // had enough samples to judge
  bool pass = true;
};

struct Ac2Verdict {
  bool pass = false;
  double max_centroid_dev_sigma = 0.0;
  double max_moment_relative_dev = 0.0;
  double max_moment_dev_sigma = 0.0;
  int worst_face = -1;
  std::vector<Ac2FaceResult> per_face;
};

// Check that samples on each face match the analytic in-face target: an
// area-weighted mixture over the fan triangles (uniform-over-polygon for planar
// faces). Both tests are count-scaled z-scores so a face with few samples is not
// held to the same absolute precision as a dominant one: centroid via the
// per-axis standard error of the mean; covariance trace via its estimator SE
// (∝ 1/√count). `min_face_samples` gates faces too sparse to judge. `d` is
// needed because the in-face mixture weights are per-triangle projected areas.
inline Ac2Verdict CheckInFaceUniformity(const Crystal& crystal, const float d[3], const EntrySamples& samples,
                                        double centroid_k_sigma, double moment_k_sigma,
                                        size_t min_face_samples = 2000) {
  Ac2Verdict v;
  const double dd[3] = { static_cast<double>(d[0]), static_cast<double>(d[1]), static_cast<double>(d[2]) };
  std::vector<OracleFaceGeom> faces = BuildPresentFaceGeom(crystal.CfGeom());
  const size_t face_cnt = faces.size();
  v.per_face.resize(face_cnt);
  v.pass = true;

  // Accumulators per face: n, Σu, Σv, Σu², Σv², Σuv.
  struct Acc {
    long long n = 0;
    double su = 0, sv = 0, suu = 0, svv = 0, suv = 0;
  };
  std::vector<Acc> acc(face_cnt);
  for (size_t i = 0; i < samples.face.size(); i++) {
    const IdType f = samples.face[i];
    if (f == kInvalidId || f >= face_cnt) {
      continue;
    }
    const double p[3] = { static_cast<double>(samples.point[i][0]), static_cast<double>(samples.point[i][1]),
                          static_cast<double>(samples.point[i][2]) };
    double u = 0.0;
    double vv = 0.0;
    ProjectToLocal2D(p, faces[f].basis, &u, &vv);
    Acc& a = acc[f];
    a.n++;
    a.su += u;
    a.sv += vv;
    a.suu += u * u;
    a.svv += vv * vv;
    a.suv += u * vv;
  }

  for (size_t f = 0; f < face_cnt; f++) {
    Ac2FaceResult r;
    r.face = static_cast<int>(f);
    r.count = acc[f].n;
    const Polygon2DMoments m = ComputeInFaceTargetMoments(faces[f], dd);
    if (acc[f].n < static_cast<long long>(min_face_samples) || m.area <= 0.0) {
      r.checked = false;
      v.per_face[f] = r;
      continue;
    }
    r.checked = true;
    const double n = static_cast<double>(acc[f].n);
    const double mu_u = acc[f].su / n;
    const double mu_v = acc[f].sv / n;
    const double var_u = acc[f].suu / n - mu_u * mu_u;
    const double var_v = acc[f].svv / n - mu_v * mu_v;

    // Centroid: standard error of the mean = sqrt(var / n) per axis.
    const double se_u = std::sqrt(std::max(m.var_u, 1e-30) / n);
    const double se_v = std::sqrt(std::max(m.var_v, 1e-30) / n);
    const double zu = (mu_u - m.cu) / se_u;
    const double zv = (mu_v - m.cv) / se_v;
    r.centroid_dev_sigma = std::sqrt(zu * zu + zv * zv);

    // Covariance-trace relative deviation, converted to a count-scaled z-score.
    // The variance-trace estimator's relative standard error scales as 1/√count;
    // sqrt(2/count) is the Gaussian reference (exact factor absorbed into the
    // calibrated k). Judging the z-score (not the raw relative deviation) keeps a
    // sparsely-sampled face from failing on noise a dominant face never shows.
    const double tr_sample = var_u + var_v;
    const double tr_analytic = m.var_u + m.var_v;
    r.moment_relative_dev = std::abs(tr_sample - tr_analytic) / std::max(tr_analytic, 1e-30);
    const double moment_se = std::sqrt(2.0 / n);
    r.moment_dev_sigma = r.moment_relative_dev / moment_se;

    r.pass = (r.centroid_dev_sigma <= centroid_k_sigma) && (r.moment_dev_sigma <= moment_k_sigma);
    if (!r.pass) {
      v.pass = false;
    }
    if (r.centroid_dev_sigma > v.max_centroid_dev_sigma) {
      v.max_centroid_dev_sigma = r.centroid_dev_sigma;
      v.worst_face = static_cast<int>(f);
    }
    if (r.moment_relative_dev > v.max_moment_relative_dev) {
      v.max_moment_relative_dev = r.moment_relative_dev;
    }
    if (r.moment_dev_sigma > v.max_moment_dev_sigma) {
      v.max_moment_dev_sigma = r.moment_dev_sigma;
    }
    v.per_face[f] = r;
  }
  return v;
}

inline Ac2Verdict SampleAndCheckInFaceUniformity(const Crystal& crystal, const float d[3], size_t n, uint32_t seed,
                                                 double centroid_k_sigma, double moment_k_sigma) {
  EntrySamples s = DriveEntrySampling(crystal, d, n, seed);
  return CheckInFaceUniformity(crystal, d, s, centroid_k_sigma, moment_k_sigma);
}

}  // namespace test_support
}  // namespace lumice

#endif  // LUMICE_TEST_SUPPORT_INCIDENCE_SAMPLING_ORACLE_HPP_
