// Exact pyramid vertex-enumeration oracle — parameters-in, symbolic-a1 exact.
//
// Ground truth for the closed-form-pyramid work. This oracle takes the
// GEOMETRIC PARAMETERS (a1, a2, h1, h2, h3, dist[6]) — NOT the pre-rounded
// float plane coefficients — and constructs the 20 half-space planes
// internally.
//
// Two exact algebraic layers, sharing the same int64_t integer budget:
//
//   Layer 1 — Q(√3) (base ring).  Every horizontal direction constant sits in
//   {0, ±1/2, ±√3/2}; multiplying by float `dist[i]` and integer `h2` stays
//   in Q(√3) exactly. Represented as `QS3 = (a + b·√3) / 2^shift` with `a, b`
//   as int64_t. Equality is bit-exact integer comparison (`a == 0 && b == 0`
//   for zero; direct component comparison for equality). Sign of a QS3 uses
//   a double-Horner filter with a 128-ULP safety margin; ambiguous → refuse.
//
//   Layer 2 — PolyQS3(α, β) (extension by two formal symbols).  a1 and a2
//   are kept as symbolic α and β, NOT substituted numerically. Every plane
//   coefficient becomes a bivariate polynomial with QS3 coefficients:
//     upper cone plane's A, B, D are α-linear;
//     lower cone plane's A, B, D are β-linear;
//     upper basal plane's D is α-linear (via z_top);
//     lower basal plane's D is β-linear (via z_bot);
//     prism planes and C's of the cone planes are constant.
//   The 3×3 Cramer determinant grows joint degree at most (deg_α + deg_β ≤ 3);
//   IsIncident / IsFeasibleSided add one more plane factor, so joint degree
//   stays ≤ 4. Polynomial storage is a flat 5×5 triangular window.
//
//   The whole point of the α, β symbolic layer is that a1 and a2 are the sole
//   sources of ~53-bit-mantissa doubles in the pipeline; keeping them
//   symbolic collapses the intermediate integer bit-width from ~120 bits
//   (numeric substitution, hits __int128 walls) to ≤ ~30 bits (measured on
//   every fixed pool including a synthetic joint-89° worst case). int64_t
//   with a 60-bit guarded budget therefore has room to spare. See the
//   throwaway spike test/support/exact_pyramid_oracle_spike.cpp for the
//   measurement harness that produced these numbers.
//
// Equality vs. sign — two independent judgements:
//   • Polynomial equality (`PolyIsZero`) is used for incidence, dedup, and
//     "line is degenerate" checks. Under the algebraic-independence
//     assumption `α, β transcendental` (holds for direct-wedge a1 by
//     construction), `p(α, β) ≡ 0 ⇔ p(a1, a2) = 0`, so this is exact.
//     Under Miller a1 (rational i1·c/(2·i4)) equality is a strict
//     over-approximation of numeric equality — the oracle may split a
//     multi-way concurrence that a numeric engine merges; this shows up in
//     the cross-validation and is handled per-sample rather than by adding
//     a Miller special case up front.
//   • Sign / feasibility uses `PolySign`: double-Horner evaluation at the
//     actual (a1, a2), plus a `128 · Σ|c_ij|·|α|^i·|β|^j · ULP` margin.
//     |val| > margin resolves the sign; otherwise `ambiguous = true` and
//     the oracle refuses (same contract the __int128 predecessor already
//     used on the opposite-sign QS3 case).
//
// Independence & discipline
// -------------------------
// - Zero tolerance in the algebraic layer (Q(√3) and its polynomial
//   extension); the ONLY numerical filter is the sign predicate, and it
//   never returns a wrong sign — it either resolves or refuses.
// - No hardcoded shift, no noise-floor snap, no fuzzy dedup.
// - Runtime bit-width guard on every arithmetic operation
//   (`AddGuarded`/`MulGuarded`/`ShiftUpGuarded`); when a product or sum
//   would exceed the int64_t operand budget (kMaxOperandBits = 60), the
//   oracle sets `refused = true` and returns an empty verdict.
// - Structurally independent from the closed-form: enumerates every
//   C(n, 3) plane triple with generic 3×3 Cramer arithmetic; does NOT bake
//   in the "six fixed directions" insight, the "uniform erosion" model, or
//   the "adjacent-only death" assumption. A bug in the closed-form's
//   structural reasoning cannot also be a bug here.
// - Dedup uses INCIDENCE (not cross-multiply of coordinates): a candidate
//   point A is a duplicate of an existing point B iff A is exactly incident
//   (polynomially) to all three planes that defined B. This sidesteps the
//   cross-multiply bit-width explosion while remaining bit-exact under the
//   equality contract above.
//
// Consumers: the golden-analytic pyramid test and the bench self-verify.

#ifndef LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_
#define LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "core/math.hpp"

namespace lumice {
namespace test_support {

// 2 basal + 6 prism + 6 upper cone + 6 lower cone.
constexpr int kExactPyramidMaxPlanes = 20;
// Upper bound on distinct vertices for the pyramid family (well over the
// combinatorial maximum for any realistic dist[]).
constexpr int kExactPyramidMaxVtx = 64;

struct ExactPyramidVerdict {
  int vertex_count = 0;
  // Physical (double) coordinates for reporting / cross-checks. The exact
  // representation is not exposed on the public interface — internal
  // equality is bit-exact regardless of these values.
  double vertex_xyz[kExactPyramidMaxVtx][3]{};
  // Per-plane vertex count and presence flag (present iff >= 3 distinct
  // vertices lie on this plane, i.e. it bounds a 2-D face polygon).
  int face_vertex_count[kExactPyramidMaxPlanes]{};
  bool face_present[kExactPyramidMaxPlanes]{};
  // Refusal: some intermediate exceeded the int64_t arithmetic budget, or a
  // sign check was numerically ambiguous. When true, all other fields are
  // meaningless (empty). Downstream consumers must NOT interpret refused as
  // "no vertices found"; either route the sample to the Python `Fraction`
  // oracle (unlimited precision) or exclude it.
  bool refused = false;
  // Diagnostics — populated even when refused.
  int max_intermediate_bits = 0;
  const char* refuse_reason = "";
};

namespace exact_pyramid_detail {

// Bit-width budget for int64_t: signed magnitude ≤ 2^63-1. Cap operands at
// 60 bits so a subsequent product / sum stays inside int64 (worst 60+60 =
// 120 exceeds 63, so multiplies at the ceiling refuse — this is the design,
// see file header). The spike measured a peak of 30 bits across all fixed
// pools including synthetic joint-89° samples, so the ceiling is only
// approached under unexpectedly adversarial inputs.
constexpr int kMaxOperandBits = 60;

// Q(√3) rational: value = (a + b·√3) / 2^shift. Zero is (0, 0, any shift).
struct QS3 {
  int64_t a;
  int64_t b;
  int shift;
};

inline QS3 QS3Zero() {
  return { 0, 0, 0 };
}

// Bit width of a signed int64 magnitude. 0 → 0, ±1 → 1, ±2..3 → 2, …
inline int BitWidth64(int64_t x) {
  if (x == 0) {
    return 0;
  }
  // Unsigned magnitude avoids the INT64_MIN pitfall (|INT64_MIN| exceeds
  // int64 positive range).
  uint64_t u = (x < 0) ? static_cast<uint64_t>(-(x + 1)) + 1u : static_cast<uint64_t>(x);
  int w = 0;
  while (u != 0) {
    u >>= 1;
    w++;
  }
  return w;
}

inline void TrackBits(int w, int* max_bits) {
  if (w > *max_bits) {
    *max_bits = w;
  }
}

// Aligned add of two int64 values with overflow tracking. Refuses if the
// result bit width would exceed kMaxOperandBits (so the next multiply-add
// step still fits) or if the sum itself would wrap signed int64.
inline int64_t AddGuarded(int64_t x, int64_t y, bool* overflow, int* max_bits) {
  const int wx = BitWidth64(x);
  const int wy = BitWidth64(y);
  const int w_hint = std::max(wx, wy) + 1;
  if (w_hint > 62) {
    *overflow = true;
    TrackBits(w_hint, max_bits);
    return 0;
  }
  const int64_t s = x + y;
  const int w = BitWidth64(s);
  TrackBits(w, max_bits);
  if (w > kMaxOperandBits) {
    *overflow = true;
  }
  return s;
}

// Guarded multiply: refuse if the product bit width would exceed budget.
inline int64_t MulGuarded(int64_t x, int64_t y, bool* overflow, int* max_bits) {
  if (x == 0 || y == 0) {
    return 0;
  }
  const int wx = BitWidth64(x);
  const int wy = BitWidth64(y);
  if (wx + wy > kMaxOperandBits) {
    *overflow = true;
    TrackBits(wx + wy, max_bits);
    return 0;
  }
  const int64_t r = x * y;
  TrackBits(BitWidth64(r), max_bits);
  return r;
}

// Shift a QS3 up by delta bits (both a and b scaled by 2^delta).
inline void ShiftUpGuarded(QS3* x, int delta, bool* overflow, int* max_bits) {
  if (delta <= 0) {
    return;
  }
  const int wa = BitWidth64(x->a);
  const int wb = BitWidth64(x->b);
  if (wa + delta > kMaxOperandBits || wb + delta > kMaxOperandBits) {
    *overflow = true;
    TrackBits(std::max(wa, wb) + delta, max_bits);
    return;
  }
  // Multiply by 2^delta rather than left-shift: left shift of a negative
  // signed value is UB in C++17. The width guard above guarantees no overflow,
  // so signed multiply by (1<<delta) is well-defined and arithmetically exact.
  x->a = x->a * (INT64_C(1) << delta);
  x->b = x->b * (INT64_C(1) << delta);
  x->shift += delta;
  TrackBits(std::max(BitWidth64(x->a), BitWidth64(x->b)), max_bits);
}

// Align two QS3 to the same shift (raises the smaller-shift one).
inline void AlignShifts(QS3* x, QS3* y, bool* overflow, int* max_bits) {
  if (x->shift == y->shift) {
    return;
  }
  if (x->shift < y->shift) {
    ShiftUpGuarded(x, y->shift - x->shift, overflow, max_bits);
  } else {
    ShiftUpGuarded(y, x->shift - y->shift, overflow, max_bits);
  }
}

inline void StripTrailingZeros(QS3* x);

// QS3 addition: aligns shifts, adds component-wise, strips trailing zero bits.
inline QS3 QS3Add(QS3 x, QS3 y, bool* overflow, int* max_bits) {
  AlignShifts(&x, &y, overflow, max_bits);
  if (*overflow) {
    return QS3Zero();
  }
  QS3 out;
  out.a = AddGuarded(x.a, y.a, overflow, max_bits);
  out.b = AddGuarded(x.b, y.b, overflow, max_bits);
  out.shift = x.shift;
  StripTrailingZeros(&out);
  return out;
}

inline QS3 QS3Sub(QS3 x, QS3 y, bool* overflow, int* max_bits) {
  y.a = -y.a;
  y.b = -y.b;
  return QS3Add(x, y, overflow, max_bits);
}

// QS3 multiplication: (a1 + b1√3)(a2 + b2√3) = (a1·a2 + 3·b1·b2) + (a1·b2 + a2·b1)·√3.
inline QS3 QS3Mul(QS3 x, QS3 y, bool* overflow, int* max_bits) {
  const int64_t aa = MulGuarded(x.a, y.a, overflow, max_bits);
  const int64_t bb = MulGuarded(x.b, y.b, overflow, max_bits);
  const int64_t ab = MulGuarded(x.a, y.b, overflow, max_bits);
  const int64_t ba = MulGuarded(x.b, y.a, overflow, max_bits);
  const int64_t three_bb = MulGuarded(3, bb, overflow, max_bits);
  QS3 out;
  out.a = AddGuarded(aa, three_bb, overflow, max_bits);
  out.b = AddGuarded(ab, ba, overflow, max_bits);
  out.shift = x.shift + y.shift;
  StripTrailingZeros(&out);
  return out;
}

inline bool IsZero(QS3 x) {
  return x.a == 0 && x.b == 0;
}

inline QS3 QS3Neg(QS3 x) {
  x.a = -x.a;
  x.b = -x.b;
  return x;
}

// int64 GCD (magnitudes, Euclidean).
inline int64_t Gcd64(int64_t x, int64_t y) {
  if (x < 0) {
    x = -x;
  }
  if (y < 0) {
    y = -y;
  }
  while (y != 0) {
    const int64_t t = x % y;
    x = y;
    y = t;
  }
  return x;
}

// Strip trailing zero bits from a QS3 (canonical dyadic form).
inline void StripTrailingZeros(QS3* x) {
  if (x->a == 0 && x->b == 0) {
    x->shift = 0;
    return;
  }
  while (x->shift > 0) {
    const int64_t a_odd = x->a & 1;
    const int64_t b_odd = x->b & 1;
    if (a_odd != 0 || b_odd != 0) {
      break;
    }
    x->a >>= 1;
    x->b >>= 1;
    x->shift--;
  }
}

// Convert a float (dyadic rational) to a QS3 pure-rational value.
inline QS3 FloatToQS3(float f) {
  if (f == 0.0f) {
    return QS3Zero();
  }
  int e = 0;
  const double m = std::frexp(static_cast<double>(f), &e);
  const double m_int_d = std::ldexp(m, 24);
  int64_t mant = static_cast<int64_t>(std::llround(m_int_d));
  int shift = 24 - e;
  if (shift < 0) {
    mant = mant * (INT64_C(1) << (-shift));
    shift = 0;
  }
  QS3 out = { mant, 0, shift };
  StripTrailingZeros(&out);
  return out;
}

// Integer constant → QS3 pure-rational.
inline QS3 IntToQS3(int n) {
  return { static_cast<int64_t>(n), 0, 0 };
}

// Q(√3) constant √3 = (0, 1) at shift 0.
inline QS3 QS3Sqrt3() {
  return { 0, 1, 0 };
}

// Convert QS3 to double (for reporting / sign hints).
inline double QS3ToDouble(QS3 x) {
  const double kSqrt3D = 1.7320508075688772;
  return (static_cast<double>(x.a) + static_cast<double>(x.b) * kSqrt3D) * std::ldexp(1.0, -x.shift);
}

// Sign of a QS3 value using a double-Horner filter with 128-ULP margin.
// Ambiguous → sets *ambiguous = true; caller must refuse.
inline int SignQS3(QS3 x, bool* ambiguous) {
  if (x.a == 0 && x.b == 0) {
    return 0;
  }
  if (x.a == 0) {
    return x.b > 0 ? 1 : -1;
  }
  if (x.b == 0) {
    return x.a > 0 ? 1 : -1;
  }
  const bool a_pos = x.a > 0;
  const bool b_pos = x.b > 0;
  if (a_pos == b_pos) {
    return a_pos ? 1 : -1;
  }
  const double da = static_cast<double>(x.a);
  const double db = static_cast<double>(x.b);
  const double kSqrt3D = 1.7320508075688772;
  const double val = da + db * kSqrt3D;
  const double mag = std::fabs(da) + std::fabs(db) * kSqrt3D;
  const double kUlp = 2.220446049250313e-16;
  const double margin = 128.0 * mag * kUlp;
  if (std::fabs(val) > margin) {
    return val > 0 ? 1 : -1;
  }
  // TEMP ARM64-CI-DBG (387.10): dump the ambiguous QS3 so the CI ARM64 log shows
  // exactly which value diverged vs the passing x86/macOS runs.
  std::fprintf(stderr, "[DBG SignQS3-AMBIG] a=%lld b=%lld shift=%d val=%.17g margin=%.17g ratio=%.4g\n",
               static_cast<long long>(x.a), static_cast<long long>(x.b), x.shift, val, margin,
               margin > 0 ? std::fabs(val) / margin : -1.0);
  *ambiguous = true;
  return 0;
}

inline int CompareQS3(QS3 x, QS3 y, bool* overflow, int* max_bits, bool* ambiguous) {
  const QS3 diff = QS3Sub(x, y, overflow, max_bits);
  if (*overflow) {
    return 0;
  }
  return SignQS3(diff, ambiguous);
}

// ============================================================================
// PolyQS3(α, β) — bivariate polynomial with QS3 coefficients. Max joint degree
// i + j ≤ kMaxJointDeg = 4 (see file header). Storage: flat 5×5 array; slots
// outside the triangle are permanent zero. Extending the degree bound is a
// deliberate change (an unlisted higher term would drop silently), not a
// backwards-compatible growth axis.
// ============================================================================

constexpr int kMaxJointDeg = 4;

struct PolyQS3 {
  QS3 c[kMaxJointDeg + 1][kMaxJointDeg + 1];
};

inline PolyQS3 PolyZero() {
  PolyQS3 p{};
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg; j++) {
      p.c[i][j] = QS3Zero();
    }
  }
  return p;
}

inline PolyQS3 PolyFromQS3(QS3 x) {
  PolyQS3 p = PolyZero();
  p.c[0][0] = x;
  return p;
}

inline PolyQS3 PolyAlpha() {
  PolyQS3 p = PolyZero();
  p.c[1][0] = IntToQS3(1);
  return p;
}

inline PolyQS3 PolyBeta() {
  PolyQS3 p = PolyZero();
  p.c[0][1] = IntToQS3(1);
  return p;
}

inline PolyQS3 PolyNeg(PolyQS3 p) {
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg; j++) {
      p.c[i][j] = QS3Neg(p.c[i][j]);
    }
  }
  return p;
}

inline PolyQS3 PolyAdd(const PolyQS3& x, const PolyQS3& y, bool* overflow, int* max_bits) {
  PolyQS3 out{};
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg; j++) {
      out.c[i][j] = QS3Add(x.c[i][j], y.c[i][j], overflow, max_bits);
    }
  }
  return out;
}

inline PolyQS3 PolySub(const PolyQS3& x, const PolyQS3& y, bool* overflow, int* max_bits) {
  return PolyAdd(x, PolyNeg(y), overflow, max_bits);
}

// PolyMul with degree bound: enumerate the FULL triangular storage of both
// operands and refuse (set *overflow = true, skip the write) when the summed
// joint degree would exceed kMaxJointDeg. Any such product is a design
// violation of the "plane joint degree ≤ 4" analysis in the file header;
// silently dropping the term (as the previous loop-bound-tightened form did)
// would let a fresh degree-growing operation return numerically wrong
// polynomials with no signal. Refuse-not-truncate is the required contract.
inline PolyQS3 PolyMul(const PolyQS3& x, const PolyQS3& y, bool* overflow, int* max_bits) {
  PolyQS3 out = PolyZero();
  for (int i1 = 0; i1 <= kMaxJointDeg; i1++) {
    for (int j1 = 0; j1 <= kMaxJointDeg - i1; j1++) {
      if (IsZero(x.c[i1][j1])) {
        continue;
      }
      for (int i2 = 0; i2 <= kMaxJointDeg; i2++) {
        for (int j2 = 0; j2 <= kMaxJointDeg - i2; j2++) {
          if (IsZero(y.c[i2][j2])) {
            continue;
          }
          if (i1 + i2 + j1 + j2 > kMaxJointDeg) {
            *overflow = true;
            continue;
          }
          const QS3 term = QS3Mul(x.c[i1][j1], y.c[i2][j2], overflow, max_bits);
          out.c[i1 + i2][j1 + j2] = QS3Add(out.c[i1 + i2][j1 + j2], term, overflow, max_bits);
        }
      }
    }
  }
  return out;
}

inline bool PolyIsZero(const PolyQS3& p) {
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg; j++) {
      if (!IsZero(p.c[i][j])) {
        return false;
      }
    }
  }
  return true;
}

// Shared double-Horner evaluation of P(α, β). Extracted so PolySign (below)
// and the vertex-coordinate report in ExactPyramidFromParams evaluate polynomial
// values through a single implementation — avoids formula drift.
// NOTE: PolySign additionally accumulates a magnitude sum |c_ij|·|α|^i·|β|^j
// for its 128-ULP margin; that accumulation still lives inline in PolySign
// because it needs the same iteration structure but a different accumulator.
// If PolyQS3's coefficient shape changes, BOTH this function's loop and
// PolySign's magnitude loop must be updated together.
inline double PolyEvalDouble(const PolyQS3& p, double alpha_val, double beta_val) {
  const double kSqrt3D = 1.7320508075688772;
  double val = 0.0;
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg - i; j++) {
      const QS3& q = p.c[i][j];
      if (IsZero(q)) {
        continue;
      }
      const double coeff = (static_cast<double>(q.a) + static_cast<double>(q.b) * kSqrt3D) * std::ldexp(1.0, -q.shift);
      val += coeff * std::pow(alpha_val, i) * std::pow(beta_val, j);
    }
  }
  return val;
}

// Exact evaluation of P(α, β) at α = a1, β = a2 as pure QS3 (int64, no double).
// a1 / a2 are the exact dyadic values of the float pyramid parameters, so the
// substitution is exact; the overflow guard trips (sets *overflow) if any
// intermediate exceeds the int64 budget. Used only as the ambiguous-sign
// fallback (see IsFeasibleSided): the free-symbol double filter cannot certify
// a value that is zero only because of an α↔β relation (e.g. the regular
// pyramid's a1 == a2), but exact substitution resolves it with no wide
// arithmetic — the actual ambiguous incidences are low degree.
inline QS3 PolyEvalExact(const PolyQS3& p, QS3 a1, QS3 a2, bool* overflow, int* max_bits) {
  // Raise a1 / a2 only to the max degree actually present in p. Computing unused
  // high powers (a1^3 ≈ 72 bits, a1^4 ≈ 96 bits) would spuriously trip the int64
  // overflow guard and force a refuse even when the live monomials are low
  // degree (the regular pyramid's ambiguous incidence is k·(α−β), degree 1).
  int max_i = 0;
  int max_j = 0;
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg; j++) {
      if (!IsZero(p.c[i][j])) {
        if (i > max_i) {
          max_i = i;
        }
        if (j > max_j) {
          max_j = j;
        }
      }
    }
  }
  QS3 a1pow[kMaxJointDeg + 1];
  QS3 a2pow[kMaxJointDeg + 1];
  a1pow[0] = IntToQS3(1);
  a2pow[0] = IntToQS3(1);
  for (int i = 1; i <= max_i; i++) {
    a1pow[i] = QS3Mul(a1pow[i - 1], a1, overflow, max_bits);
  }
  for (int j = 1; j <= max_j; j++) {
    a2pow[j] = QS3Mul(a2pow[j - 1], a2, overflow, max_bits);
  }
  QS3 acc = QS3Zero();
  for (int i = 0; i <= max_i; i++) {
    for (int j = 0; j <= max_j; j++) {
      if (IsZero(p.c[i][j])) {
        continue;
      }
      const QS3 aij = QS3Mul(a1pow[i], a2pow[j], overflow, max_bits);
      const QS3 term = QS3Mul(p.c[i][j], aij, overflow, max_bits);
      acc = QS3Add(acc, term, overflow, max_bits);
    }
  }
  return acc;
}

// Double-Horner evaluation of P(α, β) with the 128-ULP error margin.
// Return: +1 / -1 / 0 (ambiguous). Ambiguous → *ambiguous = true.
inline int PolySign(const PolyQS3& p, double alpha_val, double beta_val, bool* ambiguous) {
  const double kSqrt3D = 1.7320508075688772;
  const double kUlp = 2.220446049250313e-16;
  const double val = PolyEvalDouble(p, alpha_val, beta_val);
  double mag = 0.0;
  const double abs_a = std::fabs(alpha_val);
  const double abs_b = std::fabs(beta_val);
  for (int i = 0; i <= kMaxJointDeg; i++) {
    for (int j = 0; j <= kMaxJointDeg - i; j++) {
      const QS3& q = p.c[i][j];
      if (IsZero(q)) {
        continue;
      }
      const double coeff = (static_cast<double>(q.a) + static_cast<double>(q.b) * kSqrt3D) * std::ldexp(1.0, -q.shift);
      mag += std::fabs(coeff) * std::pow(abs_a, i) * std::pow(abs_b, j);
    }
  }
  const double margin = 128.0 * mag * kUlp;
  if (std::fabs(val) > margin) {
    return val > 0 ? 1 : -1;
  }
  // TEMP ARM64-CI-DBG (387.10): dump the ambiguous PolySign eval.
  std::fprintf(stderr, "[DBG PolySign-AMBIG] alpha=%.17g beta=%.17g val=%.17g mag=%.17g margin=%.17g ratio=%.4g\n",
               alpha_val, beta_val, val, mag, margin, margin > 0 ? std::fabs(val) / margin : -1.0);
  *ambiguous = true;
  return 0;
}

// ============================================================================
// Six horizontal direction pairs + LP direction constants — pure QS3 (no
// α, β), same as the __int128 predecessor.
// ============================================================================

struct DirCorners {
  QS3 x1, x2, y1, y2;
};

inline DirCorners GetDirCorners(int i) {
  auto make = [](int a, int b) -> QS3 { return { static_cast<int64_t>(a), static_cast<int64_t>(b), 2 }; };
  switch (i) {
    case 0:
      return { make(0, 1), make(0, 1), make(-1, 0), make(1, 0) };
    case 1:
      return { make(0, 1), make(0, 0), make(1, 0), make(2, 0) };
    case 2:
      return { make(0, 0), make(0, -1), make(2, 0), make(1, 0) };
    case 3:
      return { make(0, -1), make(0, -1), make(1, 0), make(-1, 0) };
    case 4:
      return { make(0, -1), make(0, 0), make(-1, 0), make(-2, 0) };
    case 5:
      return { make(0, 0), make(0, 1), make(-2, 0), make(-1, 0) };
    default:
      return { QS3Zero(), QS3Zero(), QS3Zero(), QS3Zero() };
  }
}

// The constant √3/8 = (0, 1) at shift 3, shared by every prism/cone plane.
inline QS3 GetDirDet() {
  return { 0, 1, 3 };
}

// (cos, sin) at 60° multiples in Q(√3), used by the LP3 apex solver.
inline QS3 GetDirCos(int i) {
  auto make = [](int a, int b, int shift) -> QS3 {
    return { static_cast<int64_t>(a), static_cast<int64_t>(b), shift };
  };
  switch (i) {
    case 0:
      return make(1, 0, 0);
    case 1:
      return make(1, 0, 1);
    case 2:
      return make(-1, 0, 1);
    case 3:
      return make(-1, 0, 0);
    case 4:
      return make(-1, 0, 1);
    case 5:
      return make(1, 0, 1);
    default:
      return QS3Zero();
  }
}

inline QS3 GetDirSin(int i) {
  auto make = [](int a, int b, int shift) -> QS3 {
    return { static_cast<int64_t>(a), static_cast<int64_t>(b), shift };
  };
  switch (i) {
    case 0:
      return make(0, 0, 0);
    case 1:
      return make(0, 1, 1);
    case 2:
      return make(0, 1, 1);
    case 3:
      return make(0, 0, 0);
    case 4:
      return make(0, -1, 1);
    case 5:
      return make(0, -1, 1);
    default:
      return QS3Zero();
  }
}

// ============================================================================
// Plane construction — PolyPlane holds four PolyQS3 coefficients (A, B, C, D).
// Prism planes: all coefficients constant polynomials.
// Cone planes:  A, B, D depend on the cone symbol (α for upper, β for lower).
// Basal planes: A = B = 0; C is constant; D is α-linear (upper) or β-linear
//               (lower) via z_top / z_bot below.
// ============================================================================

struct PolyPlane {
  PolyQS3 A, B, C, D;
};

inline PolyPlane BuildPrismPlane(int i, QS3 dist_i, bool* overflow, int* max_bits) {
  const DirCorners dc = GetDirCorners(i);
  const QS3 det = GetDirDet();
  const QS3 y_diff = QS3Sub(dc.y2, dc.y1, overflow, max_bits);
  const QS3 x_diff = QS3Sub(dc.x1, dc.x2, overflow, max_bits);
  const QS3 neg_dist = QS3Neg(dist_i);
  const QS3 d_qs = QS3Mul(neg_dist, det, overflow, max_bits);
  PolyPlane p;
  p.A = PolyFromQS3(y_diff);
  p.B = PolyFromQS3(x_diff);
  p.C = PolyFromQS3(QS3Zero());
  p.D = PolyFromQS3(d_qs);
  return p;
}

// upper == true → symbolic α; upper == false → symbolic β.
inline PolyPlane BuildConePlane(int i, bool upper, QS3 h2_2, QS3 dist_i, bool* overflow, int* max_bits) {
  const DirCorners dc = GetDirCorners(i);
  const QS3 det = GetDirDet();
  const QS3 y_diff = QS3Sub(dc.y2, dc.y1, overflow, max_bits);
  const QS3 x_diff = QS3Sub(dc.x1, dc.x2, overflow, max_bits);
  const PolyQS3 sym = upper ? PolyAlpha() : PolyBeta();
  PolyPlane p;
  p.A = PolyMul(sym, PolyFromQS3(y_diff), overflow, max_bits);
  p.B = PolyMul(sym, PolyFromQS3(x_diff), overflow, max_bits);
  p.C = PolyFromQS3(upper ? det : QS3Neg(det));
  // D = -(h2/2 + sym·dist)·det
  const PolyQS3 sym_dist = PolyMul(sym, PolyFromQS3(dist_i), overflow, max_bits);
  const PolyQS3 inner = PolyAdd(PolyFromQS3(h2_2), sym_dist, overflow, max_bits);
  const PolyQS3 neg_inner = PolyNeg(inner);
  p.D = PolyMul(neg_inner, PolyFromQS3(det), overflow, max_bits);
  return p;
}

// ============================================================================
// LP3 apex — solves the 3-direction concurrence for the upward LP
//   max m subject to cos_i·u + sin_i·v + m ≤ √3/4·dist[i].
// The LP solution depends only on the 6 direction constants and dist[] —
// no α, β anywhere — so it stays in pure QS3 (int64), not PolyQS3.
// ============================================================================

struct LP3Solution {
  QS3 u, v, m, det;
  bool degenerate;
};

inline LP3Solution SolveLP3(int i, int j, int k, const QS3 d_scaled[6], bool* overflow, int* max_bits) {
  LP3Solution s{};
  s.degenerate = false;
  const QS3 ci = GetDirCos(i), si = GetDirSin(i);
  const QS3 cj = GetDirCos(j), sj = GetDirSin(j);
  const QS3 ck = GetDirCos(k), sk = GetDirSin(k);

  // det = ci·(sj - sk) - si·(cj - ck) + (cj·sk - ck·sj)
  const QS3 sj_sk = QS3Sub(sj, sk, overflow, max_bits);
  const QS3 cj_ck = QS3Sub(cj, ck, overflow, max_bits);
  const QS3 cjsk = QS3Mul(cj, sk, overflow, max_bits);
  const QS3 cksj = QS3Mul(ck, sj, overflow, max_bits);
  const QS3 t1 = QS3Mul(ci, sj_sk, overflow, max_bits);
  const QS3 t2 = QS3Mul(si, cj_ck, overflow, max_bits);
  const QS3 t3 = QS3Sub(cjsk, cksj, overflow, max_bits);
  const QS3 t12 = QS3Sub(t1, t2, overflow, max_bits);
  s.det = QS3Add(t12, t3, overflow, max_bits);
  if (*overflow) {
    return s;
  }
  if (IsZero(s.det)) {
    s.degenerate = true;
    return s;
  }

  const QS3 di = d_scaled[i], dj = d_scaled[j], dk = d_scaled[k];

  // u_num, v_num, m_num from Cramer.
  const QS3 dj_dk = QS3Sub(dj, dk, overflow, max_bits);
  const QS3 djsk = QS3Mul(dj, sk, overflow, max_bits);
  const QS3 dksj = QS3Mul(dk, sj, overflow, max_bits);
  const QS3 u1 = QS3Mul(di, sj_sk, overflow, max_bits);
  const QS3 u2 = QS3Mul(si, dj_dk, overflow, max_bits);
  const QS3 u3 = QS3Sub(djsk, dksj, overflow, max_bits);
  const QS3 u12 = QS3Sub(u1, u2, overflow, max_bits);
  s.u = QS3Add(u12, u3, overflow, max_bits);

  const QS3 cjdk = QS3Mul(cj, dk, overflow, max_bits);
  const QS3 ckdj = QS3Mul(ck, dj, overflow, max_bits);
  const QS3 v1 = QS3Mul(ci, dj_dk, overflow, max_bits);
  const QS3 v2 = QS3Mul(di, cj_ck, overflow, max_bits);
  const QS3 v3 = QS3Sub(cjdk, ckdj, overflow, max_bits);
  const QS3 v12 = QS3Sub(v1, v2, overflow, max_bits);
  s.v = QS3Add(v12, v3, overflow, max_bits);

  const QS3 sjdk = QS3Mul(sj, dk, overflow, max_bits);
  const QS3 skdj = QS3Mul(sk, dj, overflow, max_bits);
  const QS3 sub1 = QS3Sub(sjdk, skdj, overflow, max_bits);
  const QS3 m1 = QS3Mul(ci, sub1, overflow, max_bits);
  const QS3 sub2 = QS3Sub(cjdk, ckdj, overflow, max_bits);
  const QS3 m2 = QS3Mul(si, sub2, overflow, max_bits);
  const QS3 sub3 = QS3Sub(cjsk, cksj, overflow, max_bits);
  const QS3 m3 = QS3Mul(di, sub3, overflow, max_bits);
  const QS3 m12 = QS3Sub(m1, m2, overflow, max_bits);
  s.m = QS3Add(m12, m3, overflow, max_bits);
  return s;
}

// Canonicalize sign(det) > 0. May set *ambiguous if the sign is unresolvable.
inline void CanonicalizeSign(QS3* px, QS3* py, QS3* pz, QS3* det, bool* ambiguous) {
  const int s = SignQS3(*det, ambiguous);
  if (*ambiguous) {
    return;
  }
  if (s < 0) {
    *px = QS3Neg(*px);
    *py = QS3Neg(*py);
    *pz = QS3Neg(*pz);
    *det = QS3Neg(*det);
  }
}

inline void ReducePoint(QS3* px, QS3* py, QS3* pz, QS3* det) {
  int64_t g = 0;
  auto acc = [&](int64_t v) {
    if (v == 0) {
      return;
    }
    if (v < 0) {
      v = -v;
    }
    g = (g == 0) ? v : Gcd64(g, v);
  };
  acc(px->a);
  acc(px->b);
  acc(py->a);
  acc(py->b);
  acc(pz->a);
  acc(pz->b);
  acc(det->a);
  acc(det->b);
  if (g > 1) {
    px->a /= g;
    px->b /= g;
    py->a /= g;
    py->b /= g;
    pz->a /= g;
    pz->b /= g;
    det->a /= g;
    det->b /= g;
  }
}

struct ApexResult {
  QS3 m_num, det;
  QS3 u_num, v_num;
  bool valid;
  bool refused;
};

inline ApexResult ComputeApexLP(const QS3 d_scaled[6], int* max_bits, const char** refuse_reason) {
  ApexResult out{};
  out.valid = false;
  out.refused = false;
  bool best_set = false;
  QS3 best_m_num{}, best_det{};
  QS3 best_u_num{}, best_v_num{};

  for (int i = 0; i < 6; i++) {
    for (int j = i + 1; j < 6; j++) {
      for (int k = j + 1; k < 6; k++) {
        bool overflow = false;
        bool ambiguous = false;
        LP3Solution s = SolveLP3(i, j, k, d_scaled, &overflow, max_bits);
        if (overflow) {
          out.refused = true;
          *refuse_reason = "LP3 solve overflow";
          return out;
        }
        if (s.degenerate) {
          continue;
        }
        CanonicalizeSign(&s.u, &s.v, &s.m, &s.det, &ambiguous);
        if (ambiguous) {
          out.refused = true;
          *refuse_reason = "LP3 det sign ambiguous";
          return out;
        }
        ReducePoint(&s.u, &s.v, &s.m, &s.det);

        bool feasible = true;
        for (int r = 0; r < 6; r++) {
          if (r == i || r == j || r == k) {
            continue;
          }
          const QS3 cr = GetDirCos(r);
          const QS3 sr = GetDirSin(r);
          const QS3 lhs1 = QS3Mul(cr, s.u, &overflow, max_bits);
          const QS3 lhs2 = QS3Mul(sr, s.v, &overflow, max_bits);
          const QS3 lhs12 = QS3Add(lhs1, lhs2, &overflow, max_bits);
          const QS3 lhs = QS3Add(lhs12, s.m, &overflow, max_bits);
          const QS3 rhs = QS3Mul(d_scaled[r], s.det, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "LP3 feasibility overflow";
            return out;
          }
          const QS3 diff = QS3Sub(lhs, rhs, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "LP3 feasibility diff overflow";
            return out;
          }
          const int sign = SignQS3(diff, &ambiguous);
          if (ambiguous) {
            out.refused = true;
            *refuse_reason = "LP3 feasibility sign ambiguous";
            return out;
          }
          if (sign > 0) {
            feasible = false;
            break;
          }
        }
        if (!feasible) {
          continue;
        }

        if (!best_set) {
          best_m_num = s.m;
          best_det = s.det;
          best_u_num = s.u;
          best_v_num = s.v;
          best_set = true;
        } else {
          const QS3 lhs = QS3Mul(s.m, best_det, &overflow, max_bits);
          const QS3 rhs = QS3Mul(best_m_num, s.det, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "apex compare overflow";
            return out;
          }
          const QS3 diff = QS3Sub(lhs, rhs, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "apex compare diff overflow";
            return out;
          }
          const int sign = SignQS3(diff, &ambiguous);
          if (ambiguous) {
            out.refused = true;
            *refuse_reason = "apex compare sign ambiguous";
            return out;
          }
          if (sign > 0) {
            best_m_num = s.m;
            best_det = s.det;
            best_u_num = s.u;
            best_v_num = s.v;
          }
        }
      }
    }
  }
  if (!best_set) {
    out.valid = false;
    return out;
  }
  out.m_num = best_m_num;
  out.det = best_det;
  out.u_num = best_u_num;
  out.v_num = best_v_num;
  out.valid = true;
  return out;
}

// ============================================================================
// 3-plane intersection point in PolyQS3(α, β) — general Cramer.
//   px, py, pz, det are each PolyQS3; the physical point is (px/det, py/det,
//   pz/det) evaluated at (α = a1, β = a2). All equality tests use the
//   PolyIsZero / PolyMul primitives; strict feasibility uses PolySign at the
//   actual (a1, a2).
// ============================================================================

struct PolyTriplePoint {
  PolyQS3 px, py, pz, det;
  bool degenerate;
};

inline PolyTriplePoint SolveTriple(const PolyPlane& p0, const PolyPlane& p1, const PolyPlane& p2, bool* overflow,
                                   int* max_bits) {
  PolyTriplePoint out{};
  out.degenerate = false;
  // det = A0·(B1·C2 - B2·C1) - B0·(A1·C2 - A2·C1) + C0·(A1·B2 - A2·B1)
  const PolyQS3 b1c2 = PolyMul(p1.B, p2.C, overflow, max_bits);
  const PolyQS3 b2c1 = PolyMul(p2.B, p1.C, overflow, max_bits);
  const PolyQS3 minor0 = PolySub(b1c2, b2c1, overflow, max_bits);
  const PolyQS3 a1c2 = PolyMul(p1.A, p2.C, overflow, max_bits);
  const PolyQS3 a2c1 = PolyMul(p2.A, p1.C, overflow, max_bits);
  const PolyQS3 minor1 = PolySub(a1c2, a2c1, overflow, max_bits);
  const PolyQS3 a1b2 = PolyMul(p1.A, p2.B, overflow, max_bits);
  const PolyQS3 a2b1 = PolyMul(p2.A, p1.B, overflow, max_bits);
  const PolyQS3 minor2 = PolySub(a1b2, a2b1, overflow, max_bits);
  const PolyQS3 t0 = PolyMul(p0.A, minor0, overflow, max_bits);
  const PolyQS3 t1 = PolyMul(p0.B, minor1, overflow, max_bits);
  const PolyQS3 t2 = PolyMul(p0.C, minor2, overflow, max_bits);
  const PolyQS3 t01 = PolySub(t0, t1, overflow, max_bits);
  out.det = PolyAdd(t01, t2, overflow, max_bits);
  if (*overflow) {
    return out;
  }
  if (PolyIsZero(out.det)) {
    out.degenerate = true;
    return out;
  }

  const PolyQS3 nD0 = PolyNeg(p0.D);
  const PolyQS3 nD1 = PolyNeg(p1.D);
  const PolyQS3 nD2 = PolyNeg(p2.D);

  const PolyQS3 nD1c2 = PolyMul(nD1, p2.C, overflow, max_bits);
  const PolyQS3 nD2c1 = PolyMul(nD2, p1.C, overflow, max_bits);
  const PolyQS3 xm1 = PolySub(nD1c2, nD2c1, overflow, max_bits);
  const PolyQS3 nD1b2 = PolyMul(nD1, p2.B, overflow, max_bits);
  const PolyQS3 nD2b1 = PolyMul(nD2, p1.B, overflow, max_bits);
  const PolyQS3 xm2 = PolySub(nD1b2, nD2b1, overflow, max_bits);
  const PolyQS3 xt0 = PolyMul(nD0, minor0, overflow, max_bits);
  const PolyQS3 xt1 = PolyMul(p0.B, xm1, overflow, max_bits);
  const PolyQS3 xt2 = PolyMul(p0.C, xm2, overflow, max_bits);
  const PolyQS3 xt01 = PolySub(xt0, xt1, overflow, max_bits);
  out.px = PolyAdd(xt01, xt2, overflow, max_bits);

  const PolyQS3 a1nD2 = PolyMul(p1.A, nD2, overflow, max_bits);
  const PolyQS3 a2nD1 = PolyMul(p2.A, nD1, overflow, max_bits);
  const PolyQS3 ym2 = PolySub(a1nD2, a2nD1, overflow, max_bits);
  const PolyQS3 yt0 = PolyMul(p0.A, xm1, overflow, max_bits);
  const PolyQS3 yt1 = PolyMul(nD0, minor1, overflow, max_bits);
  const PolyQS3 yt2 = PolyMul(p0.C, ym2, overflow, max_bits);
  const PolyQS3 yt01 = PolySub(yt0, yt1, overflow, max_bits);
  out.py = PolyAdd(yt01, yt2, overflow, max_bits);

  const PolyQS3 b1nD2 = PolyMul(p1.B, nD2, overflow, max_bits);
  const PolyQS3 b2nD1 = PolyMul(p2.B, nD1, overflow, max_bits);
  const PolyQS3 zm1 = PolySub(b1nD2, b2nD1, overflow, max_bits);
  const PolyQS3 zt0 = PolyMul(p0.A, zm1, overflow, max_bits);
  const PolyQS3 zt1 = PolyMul(p0.B, ym2, overflow, max_bits);
  const PolyQS3 zt2 = PolyMul(nD0, minor2, overflow, max_bits);
  const PolyQS3 zt01 = PolySub(zt0, zt1, overflow, max_bits);
  out.pz = PolyAdd(zt01, zt2, overflow, max_bits);
  return out;
}

// Canonicalize sign(det) > 0 on a PolyTriplePoint by evaluating det at the
// physical (α = a1, β = a2). If det < 0 there, negate all four polynomials
// (px, py, pz, det) — the physical point (px/det, py/det, pz/det) is
// unchanged, PolyIsZero equality (used for incidence / dedup) is preserved
// (a polynomial is zero iff its negation is), but the linear-in-det
// feasibility predicate `A·px + B·py + C·pz + D·det ≤ 0` (which is
// `A·x + B·y + C·z + D ≤ 0` multiplied through by det and therefore only
// preserves the inequality direction when det > 0) is now interpretable in
// the intended direction.
//
// Parallels the pure-QS3 CanonicalizeSign (used by ComputeApexLP) but must
// evaluate the polynomial at (α, β) — the symbolic det is a PolyQS3 whose
// sign genuinely depends on the numeric (a1, a2), unlike the LP3 det which
// lives entirely in QS3 and is direction-agnostic.
//
// Two distinct outcomes when the sign cannot be read off as strictly + or -:
//   *numeric_degenerate — det evaluates to EXACTLY 0.0 (bit-for-bit, via
//     PolyEvalDouble) at (alpha_val, beta_val). This is a computational
//     fact for the two symmetric-input mechanisms this has actually been
//     verified against (not a tolerance judgment in those cases): det is
//     literally the 3x3 determinant of the three planes' normal
//     coefficients at this specific crystal shape, so an exact-zero
//     evaluation there means Cramer's rule is dividing 0/0 — the three
//     planes are linearly dependent AT THIS numeric point and this triple
//     structurally cannot define a unique intersection. Safe for the
//     caller to skip just this triple (mirrors tp.degenerate, which
//     catches the polynomial being identically zero for ALL (α, β); this
//     catches it being zero at THIS (α, β) only). Verified for both
//     symmetric-input regimes that trigger it in the fixed sample pools:
//     RegularPyramid feeds identical alpha_val/beta_val doubles by
//     construction (α=β), and Miller symmetric samples
//     (upper_i1==lower_i1 && upper_i4==lower_i4) compute a1/a2 via the
//     *same* arithmetic expression on identical integer inputs
//     (geo3d_closedform.cpp's Miller entry), which IEEE754 guarantees
//     reproduces bit-identical doubles — so for any det polynomial with a
//     (β−α) factor, the double-Horner evaluation cancels to exactly 0.0
//     (a−a=0 is exact in IEEE754, no rounding involved).
//     This does NOT prove the same for an arbitrary polynomial shape at an
//     arbitrary (α, β): a general multi-term PolyQS3 evaluated in floating
//     point could in principle land on exactly 0.0 through coincidental
//     rounding cancellation while the true algebraic determinant is a tiny
//     nonzero — in that hypothetical case this branch would skip a triple
//     it should have refused, with no direct signal. The residual risk is
//     bounded, not eliminated, by the §4.A golden test's independent
//     vertex_count check (any such misclassification changes vertex_count
//     and fails there) and by the fact that the only inputs actually
//     exercised today are the two proven-exact mechanisms above.
//   *ambiguous — det is within the 128-ULP margin but NOT exactly 0.0: a
//     genuine sign ambiguity where the true value could be a tiny nonzero
//     (the triple may still define a real, numerically fragile vertex).
//     Zero tolerance applies here: caller MUST refuse the whole point,
//     not silently drop the triple.
inline void CanonicalizePolyTripleSign(PolyTriplePoint* tp, double alpha_val, double beta_val, bool* ambiguous,
                                       bool* numeric_degenerate) {
  const double val = PolyEvalDouble(tp->det, alpha_val, beta_val);
  if (val == 0.0) {
    *numeric_degenerate = true;
    return;
  }
  const int s = PolySign(tp->det, alpha_val, beta_val, ambiguous);
  if (*ambiguous) {
    return;
  }
  if (s < 0) {
    tp->px = PolyNeg(tp->px);
    tp->py = PolyNeg(tp->py);
    tp->pz = PolyNeg(tp->pz);
    tp->det = PolyNeg(tp->det);
  }
}

// IncidenceExpr: A·px + B·py + C·pz + D·det as PolyQS3. Consumer decides
// equality (PolyIsZero) vs strict feasibility (PolySign at (a1, a2)).
inline PolyQS3 IncidenceExpr(const PolyPlane& plane, const PolyTriplePoint& tp, bool* overflow, int* max_bits) {
  const PolyQS3 apx = PolyMul(plane.A, tp.px, overflow, max_bits);
  const PolyQS3 bpy = PolyMul(plane.B, tp.py, overflow, max_bits);
  const PolyQS3 cpz = PolyMul(plane.C, tp.pz, overflow, max_bits);
  const PolyQS3 ddet = PolyMul(plane.D, tp.det, overflow, max_bits);
  const PolyQS3 s1 = PolyAdd(apx, bpy, overflow, max_bits);
  const PolyQS3 s2 = PolyAdd(s1, cpz, overflow, max_bits);
  return PolyAdd(s2, ddet, overflow, max_bits);
}

inline bool IsIncident(const PolyPlane& plane, const PolyTriplePoint& tp, double alpha_val, double beta_val,
                       bool* overflow, int* max_bits) {
  const PolyQS3 expr = IncidenceExpr(plane, tp, overflow, max_bits);
  if (*overflow) {
    return false;
  }
  if (PolyIsZero(expr)) {
    return true;
  }
  // Value-specific incidence: expr(α,β) can vanish at the actual (a1,a2) even
  // when it is not identically zero — the free-symbol form misses coincidences
  // that hold only because of an α↔β relation (the regular pyramid's a1 == a2
  // makes the belt incidence k·(α−β) exactly zero). The dedup / face-count
  // consumers need this value-specific equality to collapse coincident
  // vertices, mirroring what the __int128 predecessor saw by substituting a1's
  // value into the coefficients. Exact int64 QS3 substitution; if it overflows
  // the budget the coincidence cannot be certified, so report not-incident
  // (conservative — leaves the vertices separate rather than wrongly merging).
  bool of = false;
  const QS3 exact = PolyEvalExact(expr, FloatToQS3(static_cast<float>(alpha_val)),
                                  FloatToQS3(static_cast<float>(beta_val)), &of, max_bits);
  if (of) {
    return false;
  }
  return IsZero(exact);
}

// Returns +1 infeasible, 0 boundary/feasible, -1 refuse.
inline int IsFeasibleSided(const PolyPlane& plane, const PolyTriplePoint& tp, double alpha_val, double beta_val,
                           bool* overflow, int* max_bits, bool* ambiguous) {
  const PolyQS3 expr = IncidenceExpr(plane, tp, overflow, max_bits);
  if (*overflow) {
    return -1;
  }
  if (PolyIsZero(expr)) {
    return 0;
  }
  const int s = PolySign(expr, alpha_val, beta_val, ambiguous);
  if (*ambiguous) {
    // The double filter cannot certify this sign — the value is within the
    // 128-ULP margin of zero. This is not necessarily a genuine near-degeneracy:
    // it also fires when expr(α,β) is a nonzero polynomial that vanishes only
    // because of an α↔β relation the free-symbol form cannot see (the regular
    // pyramid's belt incidence k·(α−β) is exactly zero because a1 == a2).
    // Resolve it exactly by substituting the exact dyadic (a1,a2) via int64 QS3
    // arithmetic — no __int128, since the ambiguous incidences that actually
    // occur are low degree (the high-degree apex concurrence is already an
    // identity caught by PolyIsZero above). Only a genuine int64-budget overflow
    // (a true wide near-cancellation) falls through to refuse.
    *ambiguous = false;
    bool of = false;
    const QS3 a1q = FloatToQS3(static_cast<float>(alpha_val));
    const QS3 a2q = FloatToQS3(static_cast<float>(beta_val));
    const QS3 exact = PolyEvalExact(expr, a1q, a2q, &of, max_bits);
    if (of) {
      *ambiguous = true;
      return -1;
    }
    bool exact_ambiguous = false;
    const int es = SignQS3(exact, &exact_ambiguous);
    if (exact_ambiguous) {
      *ambiguous = true;
      return -1;
    }
    return es > 0 ? 1 : 0;
  }
  return s > 0 ? 1 : 0;
}

}  // namespace exact_pyramid_detail

// ============================================================================
// Public API: takes parameters, constructs planes, enumerates vertices.
// ============================================================================
//
// Usage:
//   auto v = ExactPyramidFromParams(a1, a2, h1, h2, h3, dist);
//   if (v.refused) { ... }
//   for (int i = 0; i < v.vertex_count; i++) { ... }
inline ExactPyramidVerdict ExactPyramidFromParams(double a1, double a2, float h1, float h2, float h3,
                                                  const float dist[6]) {
  namespace d = exact_pyramid_detail;
  ExactPyramidVerdict out;
  int max_bits = 0;

  // Legality gate — matches FillHexCrystalCoef (geo3d.cpp:381-382). Must use
  // the SAME threshold constant as production (math::kFloatEps); a mismatched
  // threshold creates a window where the oracle and the closed-form
  // implementation disagree on has_upper/has_lower purely from tolerance
  // drift, not from a genuine geometric divergence.
  const bool has_upper = (a1 > 0.0) && (h1 > math::kFloatEps);
  const bool has_lower = (a2 > 0.0) && (h3 > math::kFloatEps);

  // Convert scalar inputs to Q(√3) pure rationals; a1 / a2 stay as doubles —
  // the symbolic α, β representation absorbs them at PolySign eval time only.
  d::QS3 dist_qs[6];
  for (int i = 0; i < 6; i++) {
    dist_qs[i] = d::FloatToQS3(dist[i]);
  }
  const d::QS3 h2_2_qs = d::FloatToQS3(0.5f * h2);
  const d::QS3 h1_qs = has_upper ? d::FloatToQS3(h1) : d::QS3Zero();
  const d::QS3 h3_qs = has_lower ? d::FloatToQS3(h3) : d::QS3Zero();

  // Zero-volume short-circuit: no cones and prism section is zero. Same
  // threshold-consistency rationale as the has_upper/has_lower gate above.
  if (!has_upper && !has_lower && h2 < math::kFloatEps) {
    return out;
  }

  d::PolyPlane planes[kExactPyramidMaxPlanes] = {};
  bool plane_active[kExactPyramidMaxPlanes] = { false };
  bool overflow = false;

  // Prism planes (slots 2..7)
  for (int i = 0; i < 6; i++) {
    planes[2 + i] = d::BuildPrismPlane(i, dist_qs[i], &overflow, &max_bits);
    plane_active[2 + i] = true;
  }
  if (overflow) {
    out.refused = true;
    out.refuse_reason = "prism plane build overflow";
    out.max_intermediate_bits = max_bits;
    return out;
  }

  if (has_upper) {
    for (int i = 0; i < 6; i++) {
      planes[8 + i] = d::BuildConePlane(i, /*upper=*/true, h2_2_qs, dist_qs[i], &overflow, &max_bits);
      plane_active[8 + i] = true;
    }
    if (overflow) {
      out.refused = true;
      out.refuse_reason = "upper cone plane build overflow";
      out.max_intermediate_bits = max_bits;
      return out;
    }
  }
  if (has_lower) {
    for (int i = 0; i < 6; i++) {
      planes[14 + i] = d::BuildConePlane(i, /*upper=*/false, h2_2_qs, dist_qs[i], &overflow, &max_bits);
      plane_active[14 + i] = true;
    }
    if (overflow) {
      out.refused = true;
      out.refuse_reason = "lower cone plane build overflow";
      out.max_intermediate_bits = max_bits;
      return out;
    }
  }

  // Basal planes via LP3 apex — pure QS3 (no α, β), then wrap α or β into D.
  if (has_upper || has_lower) {
    const d::QS3 sqrt3_over_4 = { 0, 1, 2 };
    d::QS3 d_scaled[6];
    for (int i = 0; i < 6; i++) {
      d_scaled[i] = d::QS3Mul(sqrt3_over_4, dist_qs[i], &overflow, &max_bits);
    }
    if (overflow) {
      out.refused = true;
      out.refuse_reason = "d_scaled overflow";
      out.max_intermediate_bits = max_bits;
      return out;
    }
    const char* apex_reason = "";
    d::ApexResult apex = d::ComputeApexLP(d_scaled, &max_bits, &apex_reason);
    if (apex.refused) {
      out.refused = true;
      out.refuse_reason = apex_reason;
      out.max_intermediate_bits = max_bits;
      return out;
    }
    if (!apex.valid) {
      return out;
    }

    // z_top = h2/2 + h1·α·m_apex·(4/√3), keeping (num, denom) implicit.
    // Basal plane: (0, 0, C, D) with C = z_top_denom (pure QS3, α-free) and
    // D = -z_top_num (α-linear polynomial). Under the α, β decomposition:
    //   z_top_denom = 3 · m_top_det                              (QS3)
    //   z_top_num   = h2/2 · z_top_denom + α · h1·m_top_num·4√3  (PolyQS3)
    // The h1 (or h3) < 1 clamp reduces to m_top_num := h1·m_apex; else keep
    // apex m directly (h1 ≥ 1 saturates at apex).
    const d::QS3 four_sqrt3 = { 0, 4, 0 };
    const d::QS3 three = { 3, 0, 0 };
    if (has_upper) {
      bool ambiguous = false;
      const int cmp = d::CompareQS3(h1_qs, d::IntToQS3(1), &overflow, &max_bits, &ambiguous);
      if (overflow || ambiguous) {
        out.refused = true;
        out.refuse_reason = "h1 vs 1 compare failed";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      const d::QS3 m_top_num = (cmp >= 0) ? apex.m_num : d::QS3Mul(h1_qs, apex.m_num, &overflow, &max_bits);
      const d::QS3 m_top_det = apex.det;
      const d::QS3 z_top_denom = d::QS3Mul(three, m_top_det, &overflow, &max_bits);
      const d::QS3 m_num_times_4sqrt3 = d::QS3Mul(m_top_num, four_sqrt3, &overflow, &max_bits);
      const d::QS3 h2_2_zd = d::QS3Mul(h2_2_qs, z_top_denom, &overflow, &max_bits);
      if (overflow) {
        out.refused = true;
        out.refuse_reason = "z_top compute overflow";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      d::PolyQS3 z_top_num = d::PolyZero();
      z_top_num.c[0][0] = h2_2_zd;
      z_top_num.c[1][0] = m_num_times_4sqrt3;
      planes[0].A = d::PolyZero();
      planes[0].B = d::PolyZero();
      planes[0].C = d::PolyFromQS3(z_top_denom);
      planes[0].D = d::PolyNeg(z_top_num);
      plane_active[0] = true;
    }
    if (has_lower) {
      bool ambiguous = false;
      const int cmp = d::CompareQS3(h3_qs, d::IntToQS3(1), &overflow, &max_bits, &ambiguous);
      if (overflow || ambiguous) {
        out.refused = true;
        out.refuse_reason = "h3 vs 1 compare failed";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      const d::QS3 m_bot_num = (cmp >= 0) ? apex.m_num : d::QS3Mul(h3_qs, apex.m_num, &overflow, &max_bits);
      const d::QS3 m_bot_det = apex.det;
      const d::QS3 z_bot_denom = d::QS3Mul(three, m_bot_det, &overflow, &max_bits);
      const d::QS3 m_num_times_4sqrt3 = d::QS3Mul(m_bot_num, four_sqrt3, &overflow, &max_bits);
      const d::QS3 neg_h2_2 = d::QS3Neg(h2_2_qs);
      const d::QS3 neg_h2_2_zd = d::QS3Mul(neg_h2_2, z_bot_denom, &overflow, &max_bits);
      if (overflow) {
        out.refused = true;
        out.refuse_reason = "z_bot compute overflow";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      // Lower basal encodes z ≥ z_bot as -z ≤ -z_bot:
      //   plane = (0, 0, -z_bot_denom, +z_bot_num), where
      //   z_bot_num = -h2/2·z_bot_denom - β·(m_bot_num · 4√3)   (β-linear)
      d::PolyQS3 z_bot_num = d::PolyZero();
      z_bot_num.c[0][0] = neg_h2_2_zd;
      z_bot_num.c[0][1] = d::QS3Neg(m_num_times_4sqrt3);
      planes[1].A = d::PolyZero();
      planes[1].B = d::PolyZero();
      planes[1].C = d::PolyFromQS3(d::QS3Neg(z_bot_denom));
      planes[1].D = z_bot_num;
      plane_active[1] = true;
    }
  } else {
    // Pure prism: no cones. Basal planes are (0, 0, ±1, ∓h2/2).
    const d::QS3 neg_h2_2 = d::QS3Neg(h2_2_qs);
    planes[0].A = d::PolyZero();
    planes[0].B = d::PolyZero();
    planes[0].C = d::PolyFromQS3(d::IntToQS3(1));
    planes[0].D = d::PolyFromQS3(neg_h2_2);
    plane_active[0] = true;
    planes[1].A = d::PolyZero();
    planes[1].B = d::PolyZero();
    planes[1].C = d::PolyFromQS3(d::IntToQS3(-1));
    planes[1].D = d::PolyFromQS3(neg_h2_2);
    plane_active[1] = true;
  }

  int active_idx[kExactPyramidMaxPlanes];
  int active_n = 0;
  for (int i = 0; i < kExactPyramidMaxPlanes; i++) {
    if (plane_active[i]) {
      active_idx[active_n++] = i;
    }
  }

  struct Vertex {
    d::PolyTriplePoint tp;
    int def_plane[3];
  };
  Vertex verts[kExactPyramidMaxVtx] = {};
  int vtx_cnt = 0;

  const double alpha_val = has_upper ? a1 : 0.0;
  const double beta_val = has_lower ? a2 : 0.0;

  for (int a = 0; a < active_n; a++) {
    for (int b = a + 1; b < active_n; b++) {
      for (int c = b + 1; c < active_n; c++) {
        const int i = active_idx[a];
        const int j = active_idx[b];
        const int k = active_idx[c];
        bool overflow_t = false;
        d::PolyTriplePoint tp = d::SolveTriple(planes[i], planes[j], planes[k], &overflow_t, &max_bits);
        if (overflow_t) {
          // TEMP ARM64-CI-DBG (387.10): the regular pyramid must NOT overflow a
          // triple solve. Dump the offending triple + the raw QS3 coefficients
          // of all three planes so a garbage (uninitialised-at-O3) coefficient
          // is visible in the CI log.
          std::fprintf(stderr, "[DBG triple-overflow i=%d j=%d k=%d max_bits=%d]\n", i, j, k, max_bits);
          const d::PolyPlane* pp[3] = { &planes[i], &planes[j], &planes[k] };
          const char* cn[4] = { "A", "B", "C", "D" };
          for (int pi = 0; pi < 3; pi++) {
            const d::PolyQS3* coef[4] = { &pp[pi]->A, &pp[pi]->B, &pp[pi]->C, &pp[pi]->D };
            for (int ci = 0; ci < 4; ci++) {
              for (int ii = 0; ii <= d::kMaxJointDeg; ii++) {
                for (int jj = 0; jj <= d::kMaxJointDeg; jj++) {
                  const d::QS3& q = coef[ci]->c[ii][jj];
                  if (q.a != 0 || q.b != 0 || q.shift != 0) {
                    std::fprintf(stderr, "  plane[%d].%s c[%d][%d]={a=%lld b=%lld shift=%d}\n",
                                 (pi == 0 ? i : (pi == 1 ? j : k)), cn[ci], ii, jj, static_cast<long long>(q.a),
                                 static_cast<long long>(q.b), q.shift);
                  }
                }
              }
            }
          }
          out.refused = true;
          out.refuse_reason = "triple solve overflow";
          out.max_intermediate_bits = max_bits;
          return out;
        }
        if (tp.degenerate) {
          continue;
        }

        // Canonicalize sign(det) > 0 at (alpha_val, beta_val). Downstream
        // IsFeasibleSided reads A·px + B·py + C·pz + D·det ≤ 0, which is only
        // equivalent to A·x + B·y + C·z + D ≤ 0 when det > 0; without this
        // normalization every det<0 triple is judged with the inequality
        // flipped and its feasibility is systematically inverted.
        {
          bool sign_ambiguous = false;
          bool numeric_degenerate = false;
          d::CanonicalizePolyTripleSign(&tp, alpha_val, beta_val, &sign_ambiguous, &numeric_degenerate);
          if (numeric_degenerate) {
            // det evaluates to exactly 0.0 at (α, β) — see the function's
            // doc comment for why this is a computational fact (Cramer 0/0),
            // not a tolerance judgment, and safe to skip: no valid vertex is
            // produced by this specific triple at this specific (α, β). Other
            // triples cover any real vertex that shares this plane subset.
            continue;
          }
          if (sign_ambiguous) {
            // Genuine sign ambiguity (nonzero but within the 128-ULP margin)
            // — zero tolerance applies: refuse the whole point rather than
            // silently dropping a triple that may still define a real,
            // numerically fragile vertex.
            out.refused = true;
            out.refuse_reason = "triple det sign ambiguous";
            out.max_intermediate_bits = max_bits;
            return out;
          }
        }

        // Feasibility over all other active planes.
        bool feasible = true;
        for (int r = 0; r < active_n; r++) {
          const int p_idx = active_idx[r];
          if (p_idx == i || p_idx == j || p_idx == k) {
            continue;
          }
          bool ambiguous = false;
          const int fr = d::IsFeasibleSided(planes[p_idx], tp, alpha_val, beta_val, &overflow_t, &max_bits, &ambiguous);
          if (fr == -1) {
            // TEMP 387.10-DBG: report the exact plane configuration behind the
            // ambiguous feasibility. Slot map: 0=top basal, 1=bottom basal,
            // 2..7=prism, 8..13=upper cone, 14..19=lower cone.
            std::fprintf(stderr, "[DBG feas-ambig] vertex tri={%d,%d,%d} tested plane p_idx=%d overflow=%d\n", i, j, k,
                         p_idx, overflow_t ? 1 : 0);
            out.refused = true;
            out.refuse_reason = overflow_t ? "feasibility overflow" : "feasibility sign ambiguous";
            out.max_intermediate_bits = max_bits;
            return out;
          }
          if (fr > 0) {
            feasible = false;
            break;
          }
        }
        if (!feasible) {
          continue;
        }

        // Dedup by incidence: candidate matches an existing vertex iff it
        // sits on all three defining planes of that vertex.
        int existing = -1;
        for (int v = 0; v < vtx_cnt; v++) {
          bool all_incident = true;
          for (int q = 0; q < 3; q++) {
            const int p_idx = verts[v].def_plane[q];
            if (p_idx == i || p_idx == j || p_idx == k) {
              continue;
            }
            bool ov = false;
            if (!d::IsIncident(planes[p_idx], tp, alpha_val, beta_val, &ov, &max_bits)) {
              all_incident = false;
              break;
            }
            if (ov) {
              out.refused = true;
              out.refuse_reason = "incidence overflow";
              out.max_intermediate_bits = max_bits;
              return out;
            }
          }
          if (all_incident) {
            existing = v;
            break;
          }
        }
        if (existing < 0) {
          if (vtx_cnt >= kExactPyramidMaxVtx) {
            std::fprintf(stderr, "FATAL: ExactPyramid vertex pool overflow (%d)\n", vtx_cnt);
            std::abort();
          }
          verts[vtx_cnt].tp = tp;
          verts[vtx_cnt].def_plane[0] = i;
          verts[vtx_cnt].def_plane[1] = j;
          verts[vtx_cnt].def_plane[2] = k;
          vtx_cnt++;
        }
      }
    }
  }

  // Per-plane vertex counts via incidence.
  int face_vtx[kExactPyramidMaxPlanes] = { 0 };
  for (int v = 0; v < vtx_cnt; v++) {
    for (int p = 0; p < kExactPyramidMaxPlanes; p++) {
      if (!plane_active[p]) {
        continue;
      }
      if (verts[v].def_plane[0] == p || verts[v].def_plane[1] == p || verts[v].def_plane[2] == p) {
        face_vtx[p]++;
        continue;
      }
      bool ov = false;
      if (d::IsIncident(planes[p], verts[v].tp, alpha_val, beta_val, &ov, &max_bits)) {
        face_vtx[p]++;
      }
      if (ov) {
        out.refused = true;
        out.refuse_reason = "face incidence overflow";
        out.max_intermediate_bits = max_bits;
        return out;
      }
    }
  }

  out.vertex_count = vtx_cnt;
  for (int v = 0; v < vtx_cnt; v++) {
    const double det_d = d::PolyEvalDouble(verts[v].tp.det, alpha_val, beta_val);
    out.vertex_xyz[v][0] = d::PolyEvalDouble(verts[v].tp.px, alpha_val, beta_val) / det_d;
    out.vertex_xyz[v][1] = d::PolyEvalDouble(verts[v].tp.py, alpha_val, beta_val) / det_d;
    out.vertex_xyz[v][2] = d::PolyEvalDouble(verts[v].tp.pz, alpha_val, beta_val) / det_d;
  }
  for (int p = 0; p < kExactPyramidMaxPlanes; p++) {
    out.face_vertex_count[p] = face_vtx[p];
    out.face_present[p] = (face_vtx[p] >= 3);
  }
  out.max_intermediate_bits = max_bits;
  return out;
}

}  // namespace test_support
}  // namespace lumice

#endif  // LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_
