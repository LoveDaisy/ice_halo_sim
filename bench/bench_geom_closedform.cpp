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
//      predicate runs in int64 with no tolerance at all (see the bit-width
//      analysis in test/support/exact_prism_oracle.hpp). NOTE: precision
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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "test/support/exact_prism_oracle.hpp"

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
// The full derivation, scaling constants and int64 implementation live in
// test/support/exact_prism_oracle.hpp — there is exactly one implementation.
// This one-line wrapper keeps the four call sites below signature-compatible.
int ExactPrismCornerCount(const float* dist) {
  return test_support::ExactPrism(dist).corner_count;
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
    // float dist[i] makes the whole predicate exact in int64 (see the bit-width
    // analysis in test/support/exact_prism_oracle.hpp for the budget).
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

// Cost of the production-quality closed form (src/core/geo3d_closedform.*).
// Distinct from BM_ClosedFormPrismCost above, which measures the bench-internal
// explore scaffolding — the two have different signatures (this one takes h
// and returns the packed POD; that one takes a tolerance and returns the
// double-templated struct) so they cannot share a benchmark body.
void BM_ComputeClosedFormPrism(benchmark::State& state) {
  float dist[kPrismFaces]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (auto _ : state) {
    auto r = ComputeClosedFormPrism(1.0f, dist);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_ComputeClosedFormPrism);

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

// ---- Experiment 6: is the pyramid family also 6-directional? ----------------
//
// The prism result rests on all six side planes having FIXED horizontal normal
// directions 60 deg apart. Algebra says the pyramidal planes share those same
// horizontal directions: an upper pyramidal face has xy-normal
// a1*0.5*(cos t_i, sin t_i), exactly parallel to prism face i's 0.5*(cos t_i,
// sin t_i); only the z component and the offset differ. If that holds, the whole
// hex crystal family is a "6-direction prismatoid": at every height z the cross
// section is an intersection of half-planes in the SAME six directions, with
// offsets affine in z. That would carry the constant-conditioning result from
// the prism to the pyramid, and reduce the 3D hull to a 1D problem per
// direction. Measured here rather than asserted.
void BM_PyramidSixDirections(benchmark::State& state) {
  std::mt19937 rng(31337u);
  std::uniform_real_distribution<double> alpha_dist(5.0, 85.0);
  std::normal_distribution<double> d_noise(1.0, 0.3);
  std::uniform_real_distribution<double> h_dist(0.2, 2.0);

  double worst_dir_err = 0.0;
  long samples = 0, planes_checked = 0, non_matching = 0;
  for (int s = 0; s < 4000; s++) {
    const auto au = static_cast<float>(alpha_dist(rng));
    const auto al = static_cast<float>(alpha_dist(rng));
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(d_noise(rng));
    }
    float coef[kMaxHexCrystalPlanes * 4];
    const int n =
        static_cast<int>(FillHexCrystalCoef(au, al, static_cast<float>(h_dist(rng)), static_cast<float>(h_dist(rng)),
                                            static_cast<float>(h_dist(rng)), dist, coef));
    if (n == 0) {
      continue;
    }
    samples++;
    for (int k = 2; k < n; k++) {  // skip the two basal planes
      const double nx = coef[k * 4 + 0], ny = coef[k * 4 + 1];
      const double h = std::sqrt(nx * nx + ny * ny);
      if (h < 1e-12) {
        non_matching++;  // a horizontal-normal-free non-basal plane would break the model
        continue;
      }
      planes_checked++;
      // best match against the six fixed directions
      double best = -2.0;
      for (int i = 0; i < kPrismFaces; i++) {
        const double t = i * M_PI / 3.0;
        best = std::max(best, (nx * std::cos(t) + ny * std::sin(t)) / h);
      }
      worst_dir_err = std::max(worst_dir_err, 1.0 - best);
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(worst_dir_err);
  }
  state.counters["samples"] = static_cast<double>(samples);
  state.counters["planes_checked"] = static_cast<double>(planes_checked);
  state.counters["non_matching"] = static_cast<double>(non_matching);
  state.counters["WORST_1_minus_cos"] = worst_dir_err;
}
BENCHMARK(BM_PyramidSixDirections);

// ---- Experiment 7: does the apex survive randomized face_distance? ----------
//
// "The pyramid apex is a vertex where more than three planes meet, which breaks
// the plane-triple cache model" was one of the two reasons topology reuse was
// abandoned. But the apex is a coincidence of the SYMMETRIC crystal: the six
// upper pyramidal planes meet at one point only when their offsets agree. Under
// randomized face_distance -- the entire point of the randomization work -- the
// offsets differ, so the apex should split into ordinary 3-plane vertices.
// Counts, per sample, the maximum number of planes passing through any solved
// vertex (degree), for the regular crystal and for randomized dist.
void BM_PyramidApexDegree(benchmark::State& state) {
  const bool randomize = state.range(0) != 0;
  const auto h1 = static_cast<float>(state.range(1)) / 100.0f;  // 1.0 => true apex
  std::mt19937 rng(9001u);
  std::normal_distribution<double> d_noise(1.0, 0.2);
  long samples = 0, high_degree_samples = 0, max_degree_seen = 0;
  long fam_count[4]{};  // family census of the highest-degree vertex found
  for (int s = 0; s < 400; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = randomize ? static_cast<float>(d_noise(rng)) : 1.0f;
    }
    float coef[kMaxHexCrystalPlanes * 4];
    const int n = static_cast<int>(FillHexCrystalCoef(28.0f, 28.0f, h1, 1.0f, h1, dist, coef));
    if (n == 0) {
      continue;
    }
    auto [vtx, cnt] = SolveConvexPolyhedronVtxD(n, coef);
    if (cnt == 0) {
      continue;
    }
    samples++;
    long worst = 0;
    for (int v = 0; v < cnt; v++) {
      const float* p = vtx.get() + v * 3;
      long deg = 0;
      long fam[4]{};
      double scale = 1.0;
      for (int k = 0; k < n; k++) {
        scale = std::max(scale, std::fabs(static_cast<double>(coef[k * 4 + 3])));
      }
      for (int k = 0; k < n; k++) {
        const double* cp = nullptr;
        const double sd = static_cast<double>(coef[k * 4 + 0]) * p[0] + static_cast<double>(coef[k * 4 + 1]) * p[1] +
                          static_cast<double>(coef[k * 4 + 2]) * p[2] + static_cast<double>(coef[k * 4 + 3]);
        const double nn = std::sqrt(static_cast<double>(coef[k * 4 + 0]) * coef[k * 4 + 0] +
                                    static_cast<double>(coef[k * 4 + 1]) * coef[k * 4 + 1] +
                                    static_cast<double>(coef[k * 4 + 2]) * coef[k * 4 + 2]);
        (void)cp;
        if (nn > 0 && std::fabs(sd) / nn < 1e-6 * scale) {
          deg++;
          fam[k < 2 ? 0 : (k < 8 ? 1 : (k < 14 ? 2 : 3))]++;
        }
      }
      if (deg > worst) {
        worst = deg;
        if (deg >= max_degree_seen) {
          for (int f = 0; f < 4; f++) {
            fam_count[f] = fam[f];
          }
        }
      }
    }
    max_degree_seen = std::max(max_degree_seen, worst);
    if (worst > 3) {
      high_degree_samples++;
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(max_degree_seen);
  }
  state.counters["randomized"] = randomize ? 1 : 0;
  state.counters["h1"] = h1;
  // Which plane FAMILIES meet at the worst vertex: 0=basal 1=prism 2=upper 3=lower.
  state.counters["fam_basal"] = static_cast<double>(fam_count[0]);
  state.counters["fam_prism"] = static_cast<double>(fam_count[1]);
  state.counters["fam_upper"] = static_cast<double>(fam_count[2]);
  state.counters["fam_lower"] = static_cast<double>(fam_count[3]);
  state.counters["samples"] = static_cast<double>(samples);
  state.counters["MAX_VERTEX_DEGREE"] = static_cast<double>(max_degree_seen);
  state.counters["samples_with_deg_gt3"] = static_cast<double>(high_degree_samples);
}
BENCHMARK(BM_PyramidApexDegree)->Args({ 0, 30 })->Args({ 1, 30 })->Args({ 0, 100 })->Args({ 1, 100 });

// ---- Experiment 10: dump plane sets for an INDEPENDENT exact oracle --------
//
// The exact oracle above shares this file's structural insight (six fixed
// horizontal directions). A shared misconception would pass both. This dumps
// raw plane coefficients as hex floats so a separate, structure-agnostic oracle
// (exact rationals, generic 3-plane enumeration, different language) can be the
// arbiter -- see scripts alongside this explore.
//
// Covers BOTH pyramid construction paths, because they are not interchangeable:
// Miller indices are the crystallographically rigorous route, while a directly
// specified wedge angle is what extreme-flat validation configs use (a Miller
// index is awkward there). The rationality of the substitution must not depend
// on which route produced the coefficients: after x = sqrt(3)*u the sqrt(3)
// cancels between the horizontal terms and the z coefficient (both carry the
// same det = sqrt(3)/8 factor), leaving 8*a1*M_i + z <= h/2 + a1*dist_i whose
// coefficients are just floats -- exact dyadic rationals whatever alpha is.
void BM_DumpPlaneSets(benchmark::State& state) {
  const char* path = std::getenv("LUMICE_PLANE_DUMP");
  long written = 0;
  if (path != nullptr) {
    FILE* f = std::fopen(path, "w");
    if (f != nullptr) {
      // sigma/N overridable so the same dump can populate BOTH arms of a
      // red/green test: sigma=0.3 barely produces a degenerate crystal at all
      // (the prism cross-tab saw 0 in 200k), so a criterion validated only
      // there would have no measured detection power.
      const char* sigma_env = std::getenv("LUMICE_PLANE_DUMP_SIGMA");
      const char* n_env = std::getenv("LUMICE_PLANE_DUMP_N");
      const double sigma = sigma_env != nullptr ? std::atof(sigma_env) : 0.3;
      const int n_per_arm = n_env != nullptr ? std::atoi(n_env) : 1500;
      std::mt19937 rng(20260720u);
      std::normal_distribution<double> d_noise(1.0, sigma);
      std::uniform_real_distribution<double> h_dist(0.2, 2.0);
      // arm: 0 = direct wedge angle (incl. extreme-flat), 1 = Miller index
      for (int arm = 0; arm < 2; arm++) {
        for (int s = 0; s < n_per_arm; s++) {
          float dist[kPrismFaces];
          for (float& d : dist) {
            d = static_cast<float>(d_noise(rng));
          }
          const auto h2 = static_cast<float>(h_dist(rng));
          const auto h1 = static_cast<float>(h_dist(rng) / 2.0);
          float au = 0, al = 0;
          if (arm == 0) {
            // sweep the whole legal wedge range, biased to the extreme-flat tail
            std::uniform_real_distribution<double> a_dist(1.0, 89.5);
            au = static_cast<float>(a_dist(rng));
            al = static_cast<float>(a_dist(rng));
          } else {
            std::uniform_int_distribution<int> idx(1, 4);
            const int i1 = idx(rng), i4 = idx(rng);
            au = static_cast<float>(std::atan(math::kSqrt3_2 * i4 / i1 / kIceCrystalC) * math::kRadToDegree);
            al = au;
          }
          float coef[kMaxHexCrystalPlanes * 4];
          const int n = static_cast<int>(FillHexCrystalCoef(au, al, h1, h2, h1, dist, coef));
          if (n == 0) {
            continue;
          }
          auto [vtx, cnt] = SolveConvexPolyhedronVtxD(n, coef);
          // The factory gate reads the *mesh* counts, not the solver's vertex
          // count, so dump those too -- otherwise the gate's verdict cannot be
          // reproduced offline. Appended after the coefficients so the existing
          // parser (which slices exactly n*4 coefficient fields) still works.
          auto mesh = CreateConvexPolyhedronMesh(n, coef);
          std::fprintf(f, "%d %d %d", arm, n, cnt);
          for (int k = 0; k < n * 4; k++) {
            std::fprintf(f, " %a", static_cast<double>(coef[k]));
          }
          std::fprintf(f, " %zu %zu\n", mesh.GetVtxCnt(), mesh.GetTriangleCnt());
          written++;
        }
      }
      std::fclose(f);
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(written);
  }
  state.counters["rows_written"] = static_cast<double>(written);
}
BENCHMARK(BM_DumpPlaneSets);

// ---- Experiment 11: apex degree is a DISTRIBUTION, not a yes/no ------------
//
// Correcting an earlier over-read: a single max-over-400-samples at one sigma
// was reported as "randomization splits the apex". Whether pyramidal face i
// reaches the topmost vertex is a per-sample random event, so the apex degree
// is a distribution that should tighten toward the symmetric value as sigma
// falls. Also tests the structural claim that a non-degenerate prism face i
// forces pyramidal face i to exist.
void BM_ApexDegreeDistribution(benchmark::State& state) {
  const double sigma = static_cast<double>(state.range(0)) / 1000.0;
  std::mt19937 rng(555u);
  std::normal_distribution<double> d_noise(1.0, sigma);
  long hist[9]{};  // degree 0..8+
  long samples = 0, prism_wo_pyr = 0, prism_present_total = 0;
  for (int s = 0; s < 3000; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = sigma > 0 ? static_cast<float>(d_noise(rng)) : 1.0f;
    }
    float coef[kMaxHexCrystalPlanes * 4];
    const int n = static_cast<int>(FillHexCrystalCoef(28.0f, 28.0f, 1.0f, 1.0f, 1.0f, dist, coef));
    if (n == 0) {
      continue;
    }
    auto [vtx, cnt] = SolveConvexPolyhedronVtxD(n, coef);
    if (cnt == 0) {
      continue;
    }
    samples++;
    double scale = 1.0;
    for (int k = 0; k < n; k++) {
      scale = std::max(scale, std::fabs(static_cast<double>(coef[k * 4 + 3])));
    }
    // degree of the topmost vertex
    int top = 0;
    for (int v = 1; v < cnt; v++) {
      if (vtx[v * 3 + 2] > vtx[top * 3 + 2]) {
        top = v;
      }
    }
    const float* p = vtx.get() + top * 3;
    int deg = 0;
    for (int k = 0; k < n; k++) {
      const double sd = static_cast<double>(coef[k * 4 + 0]) * p[0] + static_cast<double>(coef[k * 4 + 1]) * p[1] +
                        static_cast<double>(coef[k * 4 + 2]) * p[2] + static_cast<double>(coef[k * 4 + 3]);
      const double nn = std::sqrt(static_cast<double>(coef[k * 4 + 0]) * coef[k * 4 + 0] +
                                  static_cast<double>(coef[k * 4 + 1]) * coef[k * 4 + 1] +
                                  static_cast<double>(coef[k * 4 + 2]) * coef[k * 4 + 2]);
      if (nn > 0 && std::fabs(sd) / nn < 1e-6 * scale) {
        deg++;
      }
    }
    hist[std::min(deg, 8)]++;

    // structural claim: prism face i present => upper pyramidal face i present
    for (int i = 0; i < kPrismFaces; i++) {
      int on_prism = 0, on_pyr = 0;
      for (int v = 0; v < cnt; v++) {
        const float* q = vtx.get() + v * 3;
        for (int which = 0; which < 2; which++) {
          const int k = which == 0 ? (2 + i) : (8 + i);
          if (k >= n) {
            continue;
          }
          const double sd = static_cast<double>(coef[k * 4 + 0]) * q[0] + static_cast<double>(coef[k * 4 + 1]) * q[1] +
                            static_cast<double>(coef[k * 4 + 2]) * q[2] + static_cast<double>(coef[k * 4 + 3]);
          const double nn = std::sqrt(static_cast<double>(coef[k * 4 + 0]) * coef[k * 4 + 0] +
                                      static_cast<double>(coef[k * 4 + 1]) * coef[k * 4 + 1] +
                                      static_cast<double>(coef[k * 4 + 2]) * coef[k * 4 + 2]);
          if (nn > 0 && std::fabs(sd) / nn < 1e-6 * scale) {
            (which == 0 ? on_prism : on_pyr)++;
          }
        }
      }
      if (on_prism >= 3) {
        prism_present_total++;
        if (on_pyr < 3) {
          prism_wo_pyr++;
        }
      }
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(hist[0]);
  }
  state.counters["sigma"] = sigma;
  state.counters["samples"] = static_cast<double>(samples);
  for (int d = 3; d <= 8; d++) {
    char name[16];
    std::snprintf(name, sizeof(name), "deg%d", d);
    state.counters[name] = static_cast<double>(hist[d]);
  }
  state.counters["prism_present"] = static_cast<double>(prism_present_total);
  state.counters["PRISM_WITHOUT_PYR"] = static_cast<double>(prism_wo_pyr);
}
BENCHMARK(BM_ApexDegreeDistribution)->Arg(0)->Arg(5)->Arg(20)->Arg(50)->Arg(200);

// ---- Experiment 13: how sensitive is the merge threshold, really? ----------
//
// Owner's physics argument: whether two corners 3e-6 apart count as one vertex
// or two should not matter, because counting them as two only creates a face of
// vanishing area that Monte-Carlo entry sampling essentially never selects. The
// argument is about MEASURE, so it is testable: measure the area fraction of the
// sliver faces that the merge decision creates or destroys.
//
// The counter-consideration is that in the current representation the threshold
// is load-bearing for reasons that have nothing to do with physics: too wide
// over-merges into non-manifold fragments, too narrow lets the several candidate
// positions of one corner survive as distinct vertices. Both failures come from
// the triangulation + Euler machinery, not from the geometry. In a
// representation with no triangulation to destabilise, the only consequence of
// moving the threshold should be whether a sliver face is listed -- i.e. the
// physics insensitivity should become actual insensitivity.
//
// Reported per threshold: face count, and the smallest face's share of total
// surface area (the quantity the MC argument turns on).
void BM_MergeThresholdSensitivity(benchmark::State& state) {
  const double sigma = static_cast<double>(state.range(0)) / 100.0;
  const double thresh_rel = std::pow(10.0, -static_cast<double>(state.range(1)));

  std::mt19937 rng(24680u);
  std::normal_distribution<double> noise(1.0, sigma);
  const double k = std::sqrt(3.0) / 4.0;
  const double height = 1.0;

  long samples = 0, face_cnt_total = 0;
  double worst_sliver_frac = 1.0;  // smallest face area / total area, worst (largest) case
  double min_sliver_frac = 1.0;    // and the smallest observed
  long slivers_below_1e6 = 0;

  for (int s = 0; s < 20000; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(noise(rng));
    }
    auto cf = ClosedFormPrism<double>(dist, thresh_rel);
    if (cf.vtx_cnt < 3) {
      continue;
    }
    samples++;
    // order corners by angle to get the ring, then per-face edge lengths
    std::vector<std::pair<double, int>> ang;
    ang.reserve(cf.vtx_cnt);
    double cx = 0, cy = 0;
    for (int v = 0; v < cf.vtx_cnt; v++) {
      cx += cf.x[v];
      cy += cf.y[v];
    }
    cx /= cf.vtx_cnt;
    cy /= cf.vtx_cnt;
    for (int v = 0; v < cf.vtx_cnt; v++) {
      ang.emplace_back(std::atan2(cf.y[v] - cy, cf.x[v] - cx), v);
    }
    std::sort(ang.begin(), ang.end());
    double perim = 0, min_edge = std::numeric_limits<double>::max(), area2 = 0;
    for (size_t a = 0; a < ang.size(); a++) {
      const int i0 = ang[a].second;
      const int i1 = ang[(a + 1) % ang.size()].second;
      const double dx = cf.x[i1] - cf.x[i0], dy = cf.y[i1] - cf.y[i0];
      const double len = std::sqrt(dx * dx + dy * dy);
      perim += len;
      min_edge = std::min(min_edge, len);
      area2 += cf.x[i0] * cf.y[i1] - cf.x[i1] * cf.y[i0];
    }
    const double cap_area = std::fabs(area2);  // 2 caps of area/2 each
    const double total = perim * height + cap_area;
    const double sliver = (min_edge * height) / total;
    worst_sliver_frac = std::min(worst_sliver_frac, sliver);
    min_sliver_frac = std::min(min_sliver_frac, sliver);
    if (sliver < 1e-6) {
      slivers_below_1e6++;
    }
    face_cnt_total += static_cast<long>(ang.size());
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(face_cnt_total);
  }
  state.counters["sigma"] = sigma;
  state.counters["thresh_rel"] = thresh_rel;
  state.counters["samples"] = static_cast<double>(samples);
  state.counters["mean_face_cnt"] = samples ? static_cast<double>(face_cnt_total) / samples : 0.0;
  state.counters["SMALLEST_face_area_frac"] = min_sliver_frac;
  state.counters["n_sliver_lt_1e-6"] = static_cast<double>(slivers_below_1e6);
}
BENCHMARK(BM_MergeThresholdSensitivity)
    ->Args({ 30, 12 })
    ->Args({ 30, 10 })
    ->Args({ 30, 8 })
    ->Args({ 30, 6 })
    ->Args({ 30, 4 })
    ->Args({ 30, 3 });

// ---- Experiment 14: is the cross-section a UNIFORMLY eroded hexagon? -------
//
// Experiment 6 established that every non-basal plane's horizontal normal is one
// of six fixed directions, so at any height z the cross-section is a half-plane
// intersection in those six directions. The algebra suggests something stronger:
//
//   direction i's constraint at height z is
//       8*M_i(u,v) <= dist_i + min(0, (h/2 - z)/a1, (h/2 + z)/a2)
//                              \________ independent of i ________/
//
// i.e. the cross-section is the SAME base hexagon eroded inward by a uniform
// distance that depends only on z. If so, the entire 3D solid is determined by
// one 2D polygon plus one scalar erosion profile, and the remaining work is
// ordinary 2D engineering rather than an open question.
//
// Verified against the PRODUCTION plane coefficients (not the algebra): for each
// direction, the per-z offset is derived from the actual coef, and the spread of
// (offset_i - prism_offset_i) across the six directions is measured. Uniform
// erosion means that spread is zero.
void BM_UniformErosionModel(benchmark::State& state) {
  std::mt19937 rng(13579u);
  std::normal_distribution<double> d_noise(1.0, 0.3);
  std::uniform_real_distribution<double> a_dist(3.0, 87.0);
  std::uniform_real_distribution<double> h_dist(0.2, 2.0);

  double worst_spread = 0.0;
  double worst_spread_rel = 0.0;
  long samples = 0, z_probes = 0;

  for (int s = 0; s < 3000; s++) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(d_noise(rng));
    }
    const auto au = static_cast<float>(a_dist(rng));
    const auto al = static_cast<float>(a_dist(rng));
    const auto h2 = static_cast<float>(h_dist(rng));
    const auto h1 = static_cast<float>(h_dist(rng) / 2.0);
    float coef[kMaxHexCrystalPlanes * 4];
    const int n = static_cast<int>(FillHexCrystalCoef(au, al, h1, h2, h1, dist, coef));
    if (n < 14) {
      continue;  // want both pyramidal caps present
    }
    samples++;
    // classify each non-basal plane by its horizontal direction
    // offset_i,k(z) = (-d_k - c_k*z) / |n_h,k|
    const double z_hi = -static_cast<double>(coef[3]);  // plane (0,0,1,d):  z <= -d
    const double z_lo = static_cast<double>(coef[7]);   // plane (0,0,-1,d): z >= d
    for (int step = 0; step <= 8; step++) {
      const double z = z_lo + (z_hi - z_lo) * step / 8.0;
      double s_val[kPrismFaces];
      bool ok = true;
      for (int i = 0; i < kPrismFaces && ok; i++) {
        const double t = i * M_PI / 3.0;
        double g = std::numeric_limits<double>::max();
        double r_prism = std::numeric_limits<double>::max();
        for (int k = 2; k < n; k++) {
          const double nx = coef[k * 4 + 0], ny = coef[k * 4 + 1], nz = coef[k * 4 + 2];
          const double hn = std::sqrt(nx * nx + ny * ny);
          if (hn < 1e-12) {
            continue;
          }
          if ((nx * std::cos(t) + ny * std::sin(t)) / hn < 0.999999) {
            continue;  // not this direction
          }
          const double off = (-static_cast<double>(coef[k * 4 + 3]) - nz * z) / hn;
          g = std::min(g, off);
          if (std::fabs(nz) < 1e-12) {
            r_prism = off;  // the prism plane of this direction (no z component)
          }
        }
        if (g == std::numeric_limits<double>::max() || r_prism == std::numeric_limits<double>::max()) {
          ok = false;
          break;
        }
        s_val[i] = g - r_prism;
      }
      if (!ok) {
        continue;
      }
      z_probes++;
      double lo = s_val[0], hi = s_val[0], scale = 1.0;
      for (int i = 1; i < kPrismFaces; i++) {
        lo = std::min(lo, s_val[i]);
        hi = std::max(hi, s_val[i]);
      }
      for (int i = 0; i < kPrismFaces; i++) {
        scale = std::max(scale, std::fabs(s_val[i]));
      }
      worst_spread = std::max(worst_spread, hi - lo);
      worst_spread_rel = std::max(worst_spread_rel, (hi - lo) / scale);
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(worst_spread);
  }
  state.counters["samples"] = static_cast<double>(samples);
  state.counters["z_probes"] = static_cast<double>(z_probes);
  state.counters["WORST_SPREAD_abs"] = worst_spread;
  state.counters["WORST_SPREAD_rel"] = worst_spread_rel;
}
BENCHMARK(BM_UniformErosionModel);

// ---- Experiment 15: what does the malformed-mesh gate actually detect? ------
//
// `RejectMalformed` is the only validity gate at the Crystal factory boundary,
// and its whole content is `IsClosedTriMesh(V, F)`, i.e. V - F/2 == 2 with F
// even. It reads *nothing but two counts*. The replacement representation must
// be "equally strong", which is unmeasurable until the incumbent's strength is
// measured -- so cross-tabulate its verdict against the exact integer oracle on
// the same plane set:
//
//   oracle corner count n >= 3  <=>  the half-space intersection really is a
//   bounded 2D polygon, hence the extruded prism really is a solid.
//
// Four cells, and each one means something different:
//   (gate pass, n >= 3) agreement on healthy input
//   (gate FAIL, n >= 3) gate fires although the geometry is perfectly good
//                       -> it is detecting a *pipeline artifact*, not a defect
//   (gate pass, n <  3) gate blind spot: degenerate geometry admitted
//   (gate FAIL, n <  3) gate earns its keep
//
// The float dist[] handed to both arms is bit-identical, so the two verdicts
// are answering the same question about the same object.
void BM_MalformedGateCrossTab(benchmark::State& state) {
  const double sigma = static_cast<double>(state.range(0)) / 100.0;
  constexpr int kSamples = 200000;
  constexpr float kH = 1.0f;

  std::mt19937 rng(20260720u);
  std::normal_distribution<double> gauss(1.0, sigma);

  long n_pass_valid = 0, n_fail_valid = 0, n_pass_degen = 0, n_fail_degen = 0;
  long n_empty_planes = 0;
  // Worst (i.e. largest) oracle corner count seen among gate failures: a gate
  // failure on a 6-corner hexagon is a far stronger claim of "artifact" than
  // one on a 3-corner sliver.
  int max_corner_on_fail = 0;
  int min_corner_on_fail = 99;

  for (int s = 0; s < kSamples; s++) {
    float dist[kPrismFaces];
    for (int i = 0; i < kPrismFaces; i++) {
      dist[i] = static_cast<float>(gauss(rng));
    }

    float coef[kMaxHexCrystalPlanes * 4];
    const int plane_cnt = static_cast<int>(FillHexCrystalCoef(0, 0, 0, kH, 0, dist, coef));
    if (plane_cnt == 0) {
      n_empty_planes++;  // upstream zero-volume early-return; gate is not the actor
      continue;
    }
    auto mesh = CreateConvexPolyhedronMesh(plane_cnt, coef);
    const size_t v = mesh.GetVtxCnt();
    const size_t f = mesh.GetTriangleCnt();
    // IsClosedTriMesh inlined (crystal.hpp is not on the bench include path):
    // V - 3F/2 + F == 2, F even and nonzero.
    const bool gate_pass = (v != 0 && f != 0 && f % 2 == 0 &&
                            static_cast<int64_t>(v) - (3 * static_cast<int64_t>(f) / 2) + static_cast<int64_t>(f) == 2);

    const int corners = ExactPrismCornerCount(dist);
    const bool oracle_valid = corners >= 3;

    if (gate_pass && oracle_valid) {
      n_pass_valid++;
    } else if (!gate_pass && oracle_valid) {
      n_fail_valid++;
      max_corner_on_fail = std::max(max_corner_on_fail, corners);
      min_corner_on_fail = std::min(min_corner_on_fail, corners);
      // The decisive number, not the story: an n-corner prism must have
      // V = 2n and F = 4n - 4. Printing the actual (V, F) says *how* the
      // solver's answer differs -- one merged vertex, a whole missing face,
      // or something else entirely.
      if (n_fail_valid <= 12) {
        std::printf("  [FAIL_valid] corners=%d V=%zu F=%zu (expect V=%d F=%d) dist=%.6f %.6f %.6f %.6f %.6f %.6f\n",
                    corners, v, f, 2 * corners, 4 * corners - 4, static_cast<double>(dist[0]),
                    static_cast<double>(dist[1]), static_cast<double>(dist[2]), static_cast<double>(dist[3]),
                    static_cast<double>(dist[4]), static_cast<double>(dist[5]));
      }
    } else if (gate_pass && !oracle_valid) {
      n_pass_degen++;
    } else {
      n_fail_degen++;
      max_corner_on_fail = std::max(max_corner_on_fail, corners);
      min_corner_on_fail = std::min(min_corner_on_fail, corners);
    }
  }

  for (auto _ : state) {
    benchmark::DoNotOptimize(n_fail_valid);
  }
  state.counters["sigma"] = sigma;
  state.counters["samples"] = static_cast<double>(kSamples);
  state.counters["empty_planes"] = static_cast<double>(n_empty_planes);
  state.counters["PASS_valid"] = static_cast<double>(n_pass_valid);
  state.counters["FAIL_valid"] = static_cast<double>(n_fail_valid);
  state.counters["PASS_degen"] = static_cast<double>(n_pass_degen);
  state.counters["FAIL_degen"] = static_cast<double>(n_fail_degen);
  state.counters["fail_corner_min"] = static_cast<double>(min_corner_on_fail == 99 ? -1 : min_corner_on_fail);
  state.counters["fail_corner_max"] = static_cast<double>(max_corner_on_fail);
}
BENCHMARK(BM_MalformedGateCrossTab)->Arg(30)->Arg(50)->Arg(80);

// ============================================================================
// Pyramid oracle pre-flight diagnostic — is per-direction death time a purely-
// local event? DIAGNOSTIC, not production code.
// ============================================================================

// ---- Angular-death structure verification ---------------------------------
//
// Plan default assumption 3: "direction i's death event is determined only by
// dist[i-1], dist[i], dist[i+1]." If FALSE, the closed-form corner-death
// timetable derivation in Step 3 needs to look further. Verified empirically
// by scanning inset m from 0 to max(dist) on the closed-form 2D cross-section
// (uniform-erosion model), recording the m at which each side face i
// disappears from face_present, and comparing to what m*_i would be if we
// varied dist[i-1..i+1] alone (perturbing far directions and re-checking).
void BM_PyramidAngularDeathStructure(benchmark::State& state) {
  std::mt19937 rng(31415u);
  std::normal_distribution<double> d_noise(1.0, 0.3);
  long samples = 0;
  long deaths_local_confirmed = 0;
  long deaths_perturbed_by_far = 0;
  double worst_far_perturbation = 0.0;

  const int kSteps = 400;

  for (int s = 0; s < 400; s++) {
    float dist[6];
    for (int i = 0; i < 6; i++) {
      dist[i] = std::max(0.05f, static_cast<float>(d_noise(rng)));
    }
    // Find m*_i: for each direction i, the smallest m > 0 such that
    // face i is NOT present in ComputeClosedFormPrism(h=1, dist - m).
    // Scan in fine steps up to max(dist).
    double max_d = 0;
    for (int i = 0; i < 6; i++) {
      max_d = std::max(max_d, static_cast<double>(dist[i]));
    }
    auto find_death = [&](const float d[6]) {
      double death[6] = { -1, -1, -1, -1, -1, -1 };
      for (int step = 0; step <= kSteps; step++) {
        const double m = max_d * static_cast<double>(step) / kSteps;
        float d_eroded[6];
        for (int i = 0; i < 6; i++) {
          d_eroded[i] = d[i] - static_cast<float>(m);
        }
        auto r = ComputeClosedFormPrism(1.0f, d_eroded);
        for (int i = 0; i < 6; i++) {
          if (death[i] < 0 && !r.face_present[2 + i]) {
            death[i] = m;
          }
        }
      }
      return std::array<double, 6>{ death[0], death[1], death[2], death[3], death[4], death[5] };
    };
    auto base = find_death(dist);
    samples++;

    // Perturb ONLY far-away directions (i.e. those not in {i-1, i, i+1}) and
    // see whether m*_i shifts. If the structure assumption holds, base[i]
    // should stay the same.
    for (int i = 0; i < 6; i++) {
      if (base[i] < 0) {
        continue;  // face i never dies in [0, max_d] — nothing to check
      }
      float perturbed[6];
      for (int k = 0; k < 6; k++) {
        perturbed[k] = dist[k];
      }
      // Add +0.3 to two "far" faces (distance 2 and 3 in the hex).
      const int far_a = (i + 2) % 6;
      const int far_b = (i + 3) % 6;
      perturbed[far_a] += 0.3f;
      perturbed[far_b] += 0.3f;
      auto p = find_death(perturbed);
      if (p[i] < 0) {
        // Face i now lives forever (perturbation made the situation more
        // permissive) — not a local-vs-global collision, skip.
        continue;
      }
      const double delta = std::fabs(p[i] - base[i]);
      const double rel = delta / std::max(base[i], 1e-9);
      if (rel < 5e-3) {
        deaths_local_confirmed++;
      } else {
        deaths_perturbed_by_far++;
      }
      worst_far_perturbation = std::max(worst_far_perturbation, rel);
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(deaths_local_confirmed);
  }
  state.counters["samples"] = static_cast<double>(samples);
  state.counters["deaths_local_OK"] = static_cast<double>(deaths_local_confirmed);
  state.counters["deaths_FAR_MATTERED"] = static_cast<double>(deaths_perturbed_by_far);
  state.counters["worst_far_perturb_rel"] = worst_far_perturbation;
}
BENCHMARK(BM_PyramidAngularDeathStructure);

// ---- Step 7: closed-form pyramid construction cost -------------------------
//
// Same-binary, same-session comparison against bench_crystal.cpp's
// BM_MakeCrystal/pyramid_random (the geometry-construction clock's unit of
// work the closed form is meant to replace) and against the existing
// BM_ComputeClosedFormPrism (the sibling closed form; same file same session
// so measurement noise is shared). Two flavors:
//   _fixed:  regular pyramid α=28° h=(1,1,1) dist=[1]*6, mirrors
//            BM_ComputeClosedFormPrism (which is also a fixed dist=1 sample) —
//            isolates the algorithmic cost from random-draw overhead.
//   _random: matches RandomPyramid() in bench_crystal.cpp: h_prs=Gauss(1, 0.2),
//            h_pyr_{u,l}=Gauss(0.3, 0.05), dist=Gauss(1, 0.1), wedge=28°.
//            This is the apples-to-apples pair for BM_MakeCrystal/pyramid_random.
void BM_ComputeClosedFormPyramid_fixed(benchmark::State& state) {
  float dist[kPrismFaces]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (auto _ : state) {
    auto r = ComputeClosedFormPyramid(28.0f, 28.0f, 1.0f, 1.0f, 1.0f, dist);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_ComputeClosedFormPyramid_fixed);

void BM_ComputeClosedFormPyramid_random(benchmark::State& state) {
  std::mt19937 rng(0x5EEDu);
  std::normal_distribution<double> h_prs_dist(1.0, 0.2);
  std::normal_distribution<double> h_pyr_dist(0.3, 0.05);
  std::normal_distribution<double> d_noise(1.0, 0.1);
  for (auto _ : state) {
    float dist[kPrismFaces];
    for (float& d : dist) {
      d = static_cast<float>(d_noise(rng));
    }
    const auto h2 = static_cast<float>(h_prs_dist(rng));
    const auto h1 = static_cast<float>(h_pyr_dist(rng));
    const auto h3 = static_cast<float>(h_pyr_dist(rng));
    auto r = ComputeClosedFormPyramid(28.0f, 28.0f, h1, h2, h3, dist);
    benchmark::DoNotOptimize(r);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_ComputeClosedFormPyramid_random);

}  // namespace
