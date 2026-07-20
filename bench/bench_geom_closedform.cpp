// EXPLORE SCAFFOLDING — closed-form prism geometry vs the production solver.
//
// Not a benchmark and not a proposed implementation. It exists to answer three
// questions about replacing "discover the polyhedron's combinatorics
// numerically" with "evaluate them from the parametrization":
//
//   Q1 (conditioning) The production path solves C(8,3)=56 plane triples whose
//      determinants range down to near-singular, which is why it needs a
//      det threshold and a dedup tolerance. A prism's prism-face set has fixed
//      directions 60 deg apart, so every non-opposite pair should have a
//      CONSTANT determinant. Measured here, not assumed.
//   Q2 (agreement)   In the well-conditioned regime the closed form must agree
//      with the production solver vertex-for-vertex. Disagreement there is a
//      bug in the closed form, not evidence about the solver.
//   Q3 (oracle)      In the degenerate regime the production solver cannot be
//      the ground truth (its dedup tolerance is an empirically swept constant).
//      An EXACT integer oracle is built instead (see ExactPrismCornerCount):
//      after x = sqrt(3)*u the constraints are rational in dist[], so the whole
//      predicate runs in __int128 with no tolerance at all. NOTE: precision
//      escalation via `long double` is NOT usable here -- on arm64 macOS
//      long double IS double (53-bit), so such a test passes vacuously.
//
// Derivation being tested (from FillHexCrystalCoef, geo3d.cpp):
//   prism face i has normal 0.5*(cos t_i, sin t_i, 0), t_i = i*60deg, and
//   constant term -dist[i]*sqrt(3)/8, so the half-space is
//       x*cos t_i + y*sin t_i <= r_i,   r_i = (sqrt(3)/4)*dist[i].
//   Basal planes are z <= h/2 and -z <= h/2.
//   => the prism is a 2D half-plane intersection extruded in z.

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "core/geo3d.hpp"
#include "core/math.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace) bench code

namespace {

constexpr int kPrismFaces = 6;

// ---- closed form, templated on the working precision (Q3) -------------------

template <typename T>
struct ClosedFormResult {
  int vtx_cnt = 0;
  T x[12]{};  // at most C(6,2) - 3 = 12 feasible corners
  T y[12]{};
  bool face_present[kPrismFaces]{};
  T min_abs_det = std::numeric_limits<T>::max();
};

// Half-plane intersection over the 6 fixed prism directions. Every candidate
// corner is the intersection of two non-opposite faces; opposite faces (j==i+3)
// are parallel and skipped. Feasibility uses a tolerance RELATIVE to the
// crystal's own scale (doc/numerical-robustness.md rule 2).
template <typename T>
ClosedFormResult<T> ClosedFormPrism(const float* dist, T feas_tol_rel) {
  ClosedFormResult<T> out;
  const T k = std::sqrt(static_cast<T>(3)) / static_cast<T>(4);
  T r[kPrismFaces], cs[kPrismFaces], sn[kPrismFaces];
  T scale = 0;
  for (int i = 0; i < kPrismFaces; i++) {
    const T t = static_cast<T>(i) * static_cast<T>(M_PI) / static_cast<T>(3);
    cs[i] = std::cos(t);
    sn[i] = std::sin(t);
    r[i] = k * static_cast<T>(dist[i]);
    scale = std::max(scale, std::fabs(r[i]));
  }
  const T tol = feas_tol_rel * std::max(scale, static_cast<T>(1));

  for (int i = 0; i < kPrismFaces; i++) {
    for (int j = i + 1; j < kPrismFaces; j++) {
      if (j == i + 3) {
        continue;  // opposite faces: parallel, no corner
      }
      const T det = cs[i] * sn[j] - cs[j] * sn[i];
      out.min_abs_det = std::min(out.min_abs_det, std::fabs(det));
      const T px = (r[i] * sn[j] - r[j] * sn[i]) / det;
      const T py = (r[j] * cs[i] - r[i] * cs[j]) / det;
      bool feasible = true;
      for (int m = 0; m < kPrismFaces; m++) {
        if (px * cs[m] + py * sn[m] > r[m] + tol) {
          feasible = false;
          break;
        }
      }
      if (!feasible) {
        continue;
      }
      bool dup = false;
      for (int v = 0; v < out.vtx_cnt; v++) {
        const T dx = out.x[v] - px, dy = out.y[v] - py;
        if (std::sqrt(dx * dx + dy * dy) < tol) {
          dup = true;
          break;
        }
      }
      if (!dup && out.vtx_cnt < 12) {
        out.x[out.vtx_cnt] = px;
        out.y[out.vtx_cnt] = py;
        out.vtx_cnt++;
      }
    }
  }
  // Face i bounds the body iff at least two distinct feasible corners lie on it.
  for (int i = 0; i < kPrismFaces; i++) {
    int on = 0;
    for (int v = 0; v < out.vtx_cnt; v++) {
      if (std::fabs(out.x[v] * cs[i] + out.y[v] * sn[i] - r[i]) <= tol) {
        on++;
      }
    }
    out.face_present[i] = (on >= 2);
  }
  return out;
}

// ---- EXACT oracle: integer arithmetic, zero tolerance ----------------------
//
// This is the independent ground truth the topology-reuse work lacked. It does
// NOT consult the production solver, and it contains no epsilon: after the
// change of variables x = sqrt(3)*u the six prism constraints are rational in
// dist[], and every float is exactly a dyadic rational, so scaling by a common
// power of two makes the entire feasibility test exact in integers.
//
// Constraint form after scaling by 4 * 2^SHIFT (a_i*u + b_i*v <= c_i):
//   i=0: ( 4, 0, D0)   i=1: ( 2, 2, D1)   i=2: (-2, 2, D2)
//   i=3: (-4, 0, D3)   i=4: (-2,-2, D4)   i=5: ( 2,-2, D5)
// (faces 0/3 carry the 4 because their constraint is u <= d/4, while the four
//  oblique faces reduce to u+-v <= d/2 -- getting this scaling wrong makes 0/3
//  redundant and silently turns every hexagon into a quadrilateral.)
// with Di = round(dist[i] * 2^SHIFT) exact for the float mantissa.
struct ExactLine {
  __int128 a, b, c;
};

int ExactPrismCornerCount(const float* dist) {
  // 2^30 keeps |dist| < 2^33 exactly representable and leaves ample headroom in
  // __int128 for the 2x2 solves and the six substitutions below.
  constexpr int kShift = 30;
  const auto S = static_cast<__int128>(1) << kShift;
  __int128 D[kPrismFaces];
  for (int i = 0; i < kPrismFaces; i++) {
    D[i] = static_cast<__int128>(std::llround(static_cast<double>(dist[i]) * static_cast<double>(S)));
  }
  const ExactLine L[kPrismFaces] = {
    { 4, 0, D[0] }, { 2, 2, D[1] }, { -2, 2, D[2] }, { -4, 0, D[3] }, { -2, -2, D[4] }, { 2, -2, D[5] },
  };

  int cnt = 0;
  __int128 vx[16], vy[16], vd[16];  // corner = (vx/vd, vy/vd), vd > 0
  for (int i = 0; i < kPrismFaces; i++) {
    for (int j = i + 1; j < kPrismFaces; j++) {
      __int128 det = L[i].a * L[j].b - L[j].a * L[i].b;
      if (det == 0) {
        continue;  // parallel (opposite faces) — exactly zero, not "small"
      }
      __int128 px = L[i].c * L[j].b - L[j].c * L[i].b;
      __int128 py = L[i].a * L[j].c - L[j].a * L[i].c;
      if (det < 0) {
        det = -det;
        px = -px;
        py = -py;
      }
      bool feasible = true;
      for (int m = 0; m < kPrismFaces; m++) {
        // (a*px + b*py)/det <= c   <=>   a*px + b*py <= c*det   (det > 0)
        if (L[m].a * px + L[m].b * py > L[m].c * det) {
          feasible = false;
          break;
        }
      }
      if (!feasible) {
        continue;
      }
      bool dup = false;
      for (int v = 0; v < cnt; v++) {
        // exact cross-multiplied equality — no distance, no tolerance
        if (vx[v] * det == px * vd[v] && vy[v] * det == py * vd[v]) {
          dup = true;
          break;
        }
      }
      if (!dup && cnt < 16) {
        vx[cnt] = px;
        vy[cnt] = py;
        vd[cnt] = det;
        cnt++;
      }
    }
  }
  return cnt;
}

// ---- production path, for comparison ---------------------------------------

struct ProdResult {
  int vtx_cnt = 0;
  std::vector<float> vtx;  // 3 * vtx_cnt
  double min_abs_det_norm = std::numeric_limits<double>::max();
};

ProdResult ProductionPrism(float h, const float* dist) {
  ProdResult out;
  float coef[kMaxHexCrystalPlanes * 4];
  const int plane_cnt = static_cast<int>(FillHexCrystalCoef(0, 0, 0, h, 0, dist, coef));
  if (plane_cnt == 0) {
    return out;
  }
  auto [vtx, cnt] = SolveConvexPolyhedronVtxD(plane_cnt, coef);
  out.vtx_cnt = cnt;
  out.vtx.assign(vtx.get(), vtx.get() + cnt * 3);

  // Q1: conditioning actually seen by the generic triple loop. Normalized by
  // the operand magnitudes, which is what makes a det threshold scale-free.
  for (int i = 0; i < plane_cnt; i++) {
    for (int j = i + 1; j < plane_cnt; j++) {
      for (int k = j + 1; k < plane_cnt; k++) {
        const float* a = coef + i * 4;
        const float* b = coef + j * 4;
        const float* c = coef + k * 4;
        const double det = static_cast<double>(a[0]) * (b[1] * c[2] - b[2] * c[1]) -
                           static_cast<double>(a[1]) * (b[0] * c[2] - b[2] * c[0]) +
                           static_cast<double>(a[2]) * (b[0] * c[1] - b[1] * c[0]);
        const double na = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        const double nb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        const double nc = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
        const double dn = std::fabs(det) / (na * nb * nc);
        if (dn > 1e-12) {  // exclude exactly-parallel triples: not solved either way
          out.min_abs_det_norm = std::min(out.min_abs_det_norm, dn);
        }
      }
    }
  }
  return out;
}

// Q2/Q3: do the two vertex sets describe the same solid? The closed form is 2D
// (each corner lifts to z = +-h/2), so compare against the production 3D set.
bool VertexSetsAgree(const ClosedFormResult<double>& cf, float h, const ProdResult& pr, double tol, int* cf_expanded) {
  const int expect = cf.vtx_cnt * 2;
  *cf_expanded = expect;
  if (pr.vtx_cnt != expect) {
    return false;
  }
  for (int v = 0; v < cf.vtx_cnt; v++) {
    for (int s = 0; s < 2; s++) {
      const double z = (s == 0 ? 0.5 : -0.5) * static_cast<double>(h);
      bool matched = false;
      for (int p = 0; p < pr.vtx_cnt; p++) {
        const double dx = pr.vtx[p * 3 + 0] - cf.x[v];
        const double dy = pr.vtx[p * 3 + 1] - cf.y[v];
        const double dz = pr.vtx[p * 3 + 2] - z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < tol) {
          matched = true;
          break;
        }
      }
      if (!matched) {
        return false;
      }
    }
  }
  return true;
}

struct SweepStats {
  long samples = 0;
  long agree = 0;
  long count_diff = 0;
  long prod_empty = 0;
  long closedform_vs_exact_diff = 0;  // double closed form vs exact integer oracle
  long production_vs_exact_diff = 0;  // production solver vs exact integer oracle
  double min_prod_det = std::numeric_limits<double>::max();
  double min_cf_det = std::numeric_limits<double>::max();
  long near_boundary = 0;  // a corner sits within 1e-6 of another face: genuinely ambiguous
};

SweepStats RunSweep(double sigma, unsigned seed, int samples) {
  SweepStats st;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(1.0, sigma);
  const float h = 1.0f;
  for (int s = 0; s < samples; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(noise(rng));
    }

    auto cf_d = ClosedFormPrism<double>(dist, 1e-12);
    auto pr = ProductionPrism(h, dist);

    st.samples++;
    st.min_cf_det = std::min(st.min_cf_det, cf_d.min_abs_det);
    if (pr.min_abs_det_norm < st.min_prod_det) {
      st.min_prod_det = pr.min_abs_det_norm;
    }

    // Q3 oracle: EXACT integer arithmetic, no tolerance anywhere.
    // Substituting x = sqrt(3)*u leaves every constraint rational in dist[]:
    //   u <= d0/4,  u+v <= d1/2,  -u+v <= d2/2,
    //  -u <= d3/4, -u-v <= d4/2,   u-v <= d5/2.
    // Scaling by 4 and clearing denominators with the exact dyadic value of each
    // float dist[i] makes the whole predicate exact in __int128.
    const int exact_cnt = ExactPrismCornerCount(dist);
    if (exact_cnt != cf_d.vtx_cnt) {
      st.closedform_vs_exact_diff++;
      if (st.closedform_vs_exact_diff <= 5) {
        std::fprintf(stderr, "[CF!=EXACT sigma=%.2f] closed-form=%d exact=%d\n", sigma, cf_d.vtx_cnt, exact_cnt);
      }
    }
    if (pr.vtx_cnt != 0 && pr.vtx_cnt != exact_cnt * 2) {
      st.production_vs_exact_diff++;
      if (st.production_vs_exact_diff <= 5) {
        std::fprintf(stderr, "[PROD!=EXACT sigma=%.2f] production=%d exact(lifted)=%d  d=", sigma, pr.vtx_cnt,
                     exact_cnt * 2);
        for (float d : dist) {
          std::fprintf(stderr, "%a,", static_cast<double>(d));
        }
        std::fprintf(stderr, "\n");
      }
    }

    // How often is the configuration genuinely ambiguous — i.e. a corner lies on
    // a face OTHER than the two that define it (a 3-face concurrency)? That is
    // the only regime where "which topology is correct" has no
    // precision-independent answer. Faces the corner sits on by construction
    // must be excluded, or this counts every corner and measures nothing.
    {
      const double k = std::sqrt(3.0) / 4.0;
      bool flagged = false;
      for (int v = 0; v < cf_d.vtx_cnt && !flagged; v++) {
        for (int m = 0; m < kPrismFaces; m++) {
          const double t = m * M_PI / 3.0;
          const double slack = k * dist[m] - (cf_d.x[v] * std::cos(t) + cf_d.y[v] * std::sin(t));
          if (std::fabs(slack) < 1e-9) {
            continue;  // a defining face of this corner
          }
          if (slack > 0 && slack < 1e-6) {
            st.near_boundary++;
            flagged = true;
            break;
          }
        }
      }
    }

    if (pr.vtx_cnt == 0) {
      st.prod_empty++;
      continue;
    }
    int expanded = 0;
    if (VertexSetsAgree(cf_d, h, pr, 1e-4, &expanded)) {
      st.agree++;
    } else {
      st.count_diff++;
      if (st.count_diff <= 5) {
        std::fprintf(stderr, "[DISAGREE sigma=%.2f] closed-form corners=%d (lifted %d) vs production vtx=%d\n", sigma,
                     cf_d.vtx_cnt, expanded, pr.vtx_cnt);
      }
    }
  }
  return st;
}

void BM_ClosedFormVsProduction(benchmark::State& state) {
  const double sigma = static_cast<double>(state.range(0)) / 100.0;
  SweepStats st = RunSweep(sigma, 20260720u, 5000);
  for (auto _ : state) {
    benchmark::DoNotOptimize(st.agree);
  }
  state.counters["sigma"] = sigma;
  state.counters["samples"] = static_cast<double>(st.samples);
  state.counters["AGREE"] = static_cast<double>(st.agree);
  state.counters["count_diff"] = static_cast<double>(st.count_diff);
  state.counters["prod_empty"] = static_cast<double>(st.prod_empty);
  state.counters["CF_vs_EXACT_diff"] = static_cast<double>(st.closedform_vs_exact_diff);
  state.counters["PROD_vs_EXACT_diff"] = static_cast<double>(st.production_vs_exact_diff);
  state.counters["min_det_PRODUCTION"] = st.min_prod_det;
  state.counters["min_det_closedform"] = st.min_cf_det;
  state.counters["near_boundary"] = static_cast<double>(st.near_boundary);
}
BENCHMARK(BM_ClosedFormVsProduction)->Arg(5)->Arg(10)->Arg(20)->Arg(30)->Arg(50);

// Cost of the closed form itself, against the ~10.12 us full construction and
// the 127 ns topology-reuse floor.
void BM_ClosedFormPrismCost(benchmark::State& state) {
  float dist[kPrismFaces]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (auto _ : state) {
    auto r = ClosedFormPrism<double>(dist, 1e-12);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_ClosedFormPrismCost);

// ---- Experiment 2: how often is the PRODUCTION solver's topology wrong? -----
//
// Adjudicated by the exact integer oracle, never by the other float path. This
// is the question the topology-reuse work could not ask, because it used the
// production solver AS the ground truth: when a candidate cache-validity
// predicate "false accepted" (predicate said 12 vertices, solver said 10), that
// was recorded as the predicate being wrong. If the solver is the one that is
// wrong there, the blocker was an artifact.
struct AdjudicationStats {
  long samples = 0;
  long prod_empty = 0;
  long prod_correct = 0;
  long prod_over_merge = 0;   // production reports FEWER corners than exist
  long prod_under_merge = 0;  // production reports MORE corners than exist
  long cf_wrong = 0;
};

AdjudicationStats RunAdjudication(double sigma, unsigned seed, int samples) {
  AdjudicationStats st;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(1.0, sigma);
  for (int s = 0; s < samples; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(noise(rng));
    }
    const int exact = ExactPrismCornerCount(dist);
    const auto cf = ClosedFormPrism<double>(dist, 1e-12);
    const auto pr = ProductionPrism(1.0f, dist);
    st.samples++;
    if (cf.vtx_cnt != exact) {
      st.cf_wrong++;
    }
    if (pr.vtx_cnt == 0) {
      st.prod_empty++;
      continue;
    }
    const int prod_corners = pr.vtx_cnt / 2;  // each 2D corner lifts to z = +-h/2
    if (pr.vtx_cnt == exact * 2) {
      st.prod_correct++;
    } else if (prod_corners < exact) {
      st.prod_over_merge++;
    } else {
      st.prod_under_merge++;
    }
  }
  return st;
}

void BM_ProductionTopologyErrorRate(benchmark::State& state) {
  const double sigma = static_cast<double>(state.range(0)) / 100.0;
  AdjudicationStats st = RunAdjudication(sigma, 777u, 40000);
  for (auto _ : state) {
    benchmark::DoNotOptimize(st.prod_correct);
  }
  state.counters["sigma"] = sigma;
  state.counters["samples"] = static_cast<double>(st.samples);
  state.counters["prod_OK"] = static_cast<double>(st.prod_correct);
  state.counters["prod_OVERMERGE"] = static_cast<double>(st.prod_over_merge);
  state.counters["prod_UNDERMERGE"] = static_cast<double>(st.prod_under_merge);
  state.counters["prod_empty"] = static_cast<double>(st.prod_empty);
  state.counters["closedform_WRONG"] = static_cast<double>(st.cf_wrong);
  const double denom = static_cast<double>(st.samples - st.prod_empty);
  state.counters["prod_err_rate"] =
      denom > 0 ? static_cast<double>(st.prod_over_merge + st.prod_under_merge) / denom : 0.0;
}
BENCHMARK(BM_ProductionTopologyErrorRate)->Arg(10)->Arg(20)->Arg(30)->Arg(40)->Arg(50);

// ---- Experiment 3: re-adjudicate the topology-reuse criterion ---------------
//
// Reproduces the measurement that blocked topology reuse, in a space where an
// exact oracle exists. The reuse cache stores, per vertex, the plane triple that
// produces it; criterion #1 re-solves those triples and requires every resulting
// vertex to stay inside every half-space. For a prism the cache is the six
// adjacent face pairs (0,1),(1,2)...(5,0).
//
// The original measurement scored a sample as a FALSE ACCEPT when the criterion
// said "cache still valid" but the PRODUCTION SOLVER reported a different vertex
// count -- and concluded the criterion was insufficient. Here each such sample is
// put to the exact integer oracle instead, which answers who was actually right.
struct ReuseAdjudication {
  long samples = 0;
  long criterion_accept = 0;
  long flagged_false_accept = 0;   // as the original measurement scored it
  long truly_criterion_wrong = 0;  // exact oracle agrees with production
  long actually_solver_wrong = 0;  // exact oracle agrees with the criterion
};

ReuseAdjudication RunReuseAdjudication(double sigma, unsigned seed, int samples) {
  ReuseAdjudication st;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(1.0, sigma);
  const double k = std::sqrt(3.0) / 4.0;
  for (int s = 0; s < samples; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(noise(rng));
    }
    double r[kPrismFaces], cs[kPrismFaces], sn[kPrismFaces];
    for (int i = 0; i < kPrismFaces; i++) {
      const double t = i * M_PI / 3.0;
      cs[i] = std::cos(t);
      sn[i] = std::sin(t);
      r[i] = k * static_cast<double>(dist[i]);
    }
    // criterion #1 over the six cached adjacent-face corners
    bool cache_valid = true;
    int cached_corners = 0;
    for (int i = 0; i < kPrismFaces && cache_valid; i++) {
      const int j = (i + 1) % kPrismFaces;
      const double det = cs[i] * sn[j] - cs[j] * sn[i];
      const double px = (r[i] * sn[j] - r[j] * sn[i]) / det;
      const double py = (r[j] * cs[i] - r[i] * cs[j]) / det;
      cached_corners++;
      for (int m = 0; m < kPrismFaces; m++) {
        if (px * cs[m] + py * sn[m] > r[m] + 1e-5) {  // the original absolute eps
          cache_valid = false;
          break;
        }
      }
    }
    st.samples++;
    if (!cache_valid) {
      continue;
    }
    st.criterion_accept++;
    const auto pr = ProductionPrism(1.0f, dist);
    if (pr.vtx_cnt == 0) {
      continue;
    }
    const int prod_corners = pr.vtx_cnt / 2;
    if (prod_corners == cached_corners) {
      continue;  // solver agrees with the cache: not scored either way
    }
    st.flagged_false_accept++;
    const int exact = ExactPrismCornerCount(dist);
    if (exact == cached_corners) {
      st.actually_solver_wrong++;
    } else {
      st.truly_criterion_wrong++;
    }
  }
  return st;
}

void BM_ReuseCriterionReadjudicated(benchmark::State& state) {
  const double sigma = static_cast<double>(state.range(0)) / 100.0;
  ReuseAdjudication st = RunReuseAdjudication(sigma, 4242u, 40000);
  for (auto _ : state) {
    benchmark::DoNotOptimize(st.flagged_false_accept);
  }
  state.counters["sigma"] = sigma;
  state.counters["samples"] = static_cast<double>(st.samples);
  state.counters["accepts"] = static_cast<double>(st.criterion_accept);
  state.counters["FLAGGED_false_accept"] = static_cast<double>(st.flagged_false_accept);
  state.counters["criterion_really_wrong"] = static_cast<double>(st.truly_criterion_wrong);
  state.counters["SOLVER_really_wrong"] = static_cast<double>(st.actually_solver_wrong);
}
BENCHMARK(BM_ReuseCriterionReadjudicated)->Arg(10)->Arg(20)->Arg(30)->Arg(50)->Arg(80);

}  // namespace
