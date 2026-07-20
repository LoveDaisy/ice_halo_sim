// Exact pyramid vertex-enumeration oracle — parameters-in, Q(√3) exact.
//
// Ground truth for the closed-form-pyramid work. This oracle takes the
// GEOMETRIC PARAMETERS (a1, a2, h1, h2, h3, dist[6]) — NOT the pre-rounded
// float plane coefficients — and constructs the 20 half-space planes
// internally in the exact ring Q(√3). All six horizontal directions
// (cos, sin) at 30° multiples take values in {0, ±1/2, ±√3/2}, so every
// coefficient of the pyramid family lies in Q(√3) exactly. Algebraic
// identities like `2·(a1·√3/8) = a1·√3/4` therefore hold bit-for-bit,
// and the 4+ plane concurrences at the shoulder / apex become a SINGLE
// Q(√3) rational point instead of a cluster of nearby float artifacts.
//
// Independence & discipline
// -------------------------
// - Zero tolerance anywhere. Q(√3) is a 2-D field over Q, and (a + b·√3) == 0
//   iff a == 0 AND b == 0 (√3 irrational). Equality, incidence, and
//   feasibility are all bit-exact integer comparisons.
// - No hardcoded shift, no noise-floor snap, no fuzzy dedup. These were the
//   three failure modes owner identified in prior rounds.
// - Runtime bit-width guard on every arithmetic operation. When a product or
//   sum would exceed the __int128 budget (120 bits, leaves margin for the
//   next-step sum), the oracle sets `refused = true` and returns an empty
//   verdict. Silent downgrade to a lossy answer is what produced the
//   "regular pyramid → vtx=36" bug on the SHIFT=24 hardcode.
// - Structurally independent from the closed-form: enumerates every
//   C(n, 3) plane triple with generic 3×3 Cramer arithmetic; does NOT bake
//   in the "six fixed directions" insight, the "uniform erosion" model, or
//   the "adjacent-only death" assumption. A bug in the closed-form's
//   structural reasoning cannot also be a bug here.
// - Dedup uses INCIDENCE (not cross-multiply): a candidate point A is a
//   duplicate of an existing point B iff A is exactly incident to all three
//   planes that defined B. This sidesteps the cross-multiply bit-width
//   explosion (200+ bits, would blow past __int128) while remaining bit-
//   exact: two triples yielding the geometrically same point must both have
//   that point on ALL 6 planes involved.
//
// Consumers: the golden-analytic pyramid test and the bench self-verify.

#ifndef LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_
#define LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

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
  // Q(√3) representation is not exposed on the public interface — internal
  // equality is bit-exact regardless of these values.
  double vertex_xyz[kExactPyramidMaxVtx][3]{};
  // Per-plane vertex count and presence flag (present iff >= 3 distinct
  // vertices lie on this plane, i.e. it bounds a 2-D face polygon).
  int face_vertex_count[kExactPyramidMaxPlanes]{};
  bool face_present[kExactPyramidMaxPlanes]{};
  // Refusal: some intermediate exceeded the __int128 arithmetic budget, or a
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

// Bit-width budget for __int128: signed magnitude ≤ 2^127-1. Cap
// intermediate operands at 120 bits so a subsequent product (up to
// 120 + 6 = 126 bits) and sum still fit. Exceed → refuse.
constexpr int kMaxOperandBits = 120;

// Q(√3) rational: value = (a + b·√3) / 2^shift. Zero is (0, 0, any shift).
struct QS3 {
  __int128 a;
  __int128 b;
  int shift;
};

inline QS3 QS3Zero() {
  return { 0, 0, 0 };
}

// Bit width of a signed __int128 (magnitude). 0 → 0, ±1 → 1, ±2..3 → 2, …
inline int BitWidth(__int128 x) {
  if (x == 0) {
    return 0;
  }
  if (x < 0) {
    x = -x;
  }
  int w = 0;
  __int128 v = x;
  while (v != 0) {
    v >>= 1;
    w++;
  }
  return w;
}

inline void TrackBits(int w, int* max_bits) {
  if (w > *max_bits) {
    *max_bits = w;
  }
}

// Aligned add of two __int128 values with overflow tracking. Overflow of
// signed addition happens when both operands are same-signed and their sum's
// bit width would exceed __int128 signed range. We conservatively refuse if
// the resulting bit width would exceed kMaxOperandBits.
inline __int128 AddGuarded(__int128 x, __int128 y, bool* overflow, int* max_bits) {
  __int128 s = x + y;
  int w = BitWidth(s);
  TrackBits(w, max_bits);
  if (w > kMaxOperandBits) {
    // Even if signed __int128 didn't wrap here, the next op likely would.
    *overflow = true;
  }
  return s;
}

// Guarded multiply: refuse if the product bit width would exceed budget.
inline __int128 MulGuarded(__int128 x, __int128 y, bool* overflow, int* max_bits) {
  if (x == 0 || y == 0) {
    return 0;
  }
  int wx = BitWidth(x);
  int wy = BitWidth(y);
  if (wx + wy > kMaxOperandBits) {
    *overflow = true;
    TrackBits(wx + wy, max_bits);
    return 0;
  }
  __int128 r = x * y;
  TrackBits(BitWidth(r), max_bits);
  return r;
}

// Shift a QS3 up by delta bits so that both a and b are scaled by 2^delta.
// This is the "raise shift" operation used to align two QS3 values before
// addition. delta must be non-negative.
inline void ShiftUpGuarded(QS3* x, int delta, bool* overflow, int* max_bits) {
  if (delta <= 0) {
    return;
  }
  int wa = BitWidth(x->a);
  int wb = BitWidth(x->b);
  if (wa + delta > kMaxOperandBits || wb + delta > kMaxOperandBits) {
    *overflow = true;
    TrackBits(std::max(wa, wb) + delta, max_bits);
    return;
  }
  x->a <<= delta;
  x->b <<= delta;
  x->shift += delta;
  TrackBits(std::max(BitWidth(x->a), BitWidth(x->b)), max_bits);
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

// Forward decl for StripTrailingZeros — defined below input converters.
inline void StripTrailingZeros(QS3* x);

// QS3 addition: aligns shifts, adds component-wise, strips trailing zero bits
// from the result (keeps operand bit widths minimal for downstream ops).
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
// Strips trailing zero bits from the result (matches Add convention).
inline QS3 QS3Mul(QS3 x, QS3 y, bool* overflow, int* max_bits) {
  __int128 aa = MulGuarded(x.a, y.a, overflow, max_bits);
  __int128 bb = MulGuarded(x.b, y.b, overflow, max_bits);
  __int128 ab = MulGuarded(x.a, y.b, overflow, max_bits);
  __int128 ba = MulGuarded(x.b, y.a, overflow, max_bits);
  __int128 three_bb = MulGuarded(3, bb, overflow, max_bits);
  QS3 out;
  out.a = AddGuarded(aa, three_bb, overflow, max_bits);
  out.b = AddGuarded(ab, ba, overflow, max_bits);
  out.shift = x.shift + y.shift;
  StripTrailingZeros(&out);
  return out;
}

// QS3 == 0 iff both components are zero. Bit-exact.
inline bool IsZero(QS3 x) {
  return x.a == 0 && x.b == 0;
}

// Sign of a QS3 value.
//   Same-sign or one-zero cases: trivial, exact.
//   Opposite-sign case: use double approximation with a comfortable safety
//   margin. If ambiguous, set *sign_refuse = true; caller must refuse.
inline int SignQS3(QS3 x, bool* sign_refuse) {
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
  // Opposite-sign case: sign(a + b·√3). Use double with a 128-ULP safety
  // margin. The double approximation of a large __int128 loses precision
  // beyond 53 bits, but for the sign question the leading-bit ratio is what
  // matters. If magnitudes are grossly comparable but signs differ, the
  // double answer is reliable if the ratio a/b is not pathologically close
  // to -√3.
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
  // Ambiguous in double — refuse rather than risk a wrong sign. Bit-exact
  // resolution would require comparing a² vs 3·b² in __int256, which is
  // outside the current implementation's scope.
  *sign_refuse = true;
  return 0;
}

// Negate a QS3 in-place (used to canonicalize det > 0).
inline QS3 QS3Neg(QS3 x) {
  x.a = -x.a;
  x.b = -x.b;
  return x;
}

// __int128 GCD (magnitudes, Euclidean).
inline __int128 Gcd128(__int128 x, __int128 y) {
  if (x < 0) {
    x = -x;
  }
  if (y < 0) {
    y = -y;
  }
  while (y != 0) {
    __int128 t = x % y;
    x = y;
    y = t;
  }
  return x;
}

// Reduce a 4-QS3 tuple (px, py, pz, det) by dividing all 8 integer components
// by their common GCD. Also strips shift-common trailing zero bits.
inline void ReducePoint(QS3* px, QS3* py, QS3* pz, QS3* det) {
  __int128 g = 0;
  auto acc = [&](__int128 v) {
    if (v == 0) {
      return;
    }
    if (v < 0) {
      v = -v;
    }
    g = (g == 0) ? v : Gcd128(g, v);
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

// Strip trailing zero bits from a QS3 (canonical dyadic form: at least one of
// a/b must have a bottom-bit-set, else lower the shift). This is O(1) amortized
// and shrinks bit widths dramatically for inputs like h2=1 (which starts as
// mant=2^24, shift=24 → after strip: (1, 0, 0), bit width 1).
inline void StripTrailingZeros(QS3* x) {
  if (x->a == 0 && x->b == 0) {
    x->shift = 0;
    return;
  }
  while (x->shift > 0) {
    __int128 a_odd = x->a & 1;
    __int128 b_odd = x->b & 1;
    if (a_odd != 0 || b_odd != 0) {
      break;
    }
    x->a >>= 1;
    x->b >>= 1;
    x->shift--;
  }
}

// Convert a float (dyadic rational) to a QS3 pure-rational value.
// value = f = m·2^e (frexp), so `int_mantissa` at shift `24 − e` is exact.
inline QS3 FloatToQS3(float f) {
  if (f == 0.0f) {
    return QS3Zero();
  }
  int e = 0;
  double m = std::frexp(static_cast<double>(f), &e);  // m in [0.5, 1)
  double m_int_d = std::ldexp(m, 24);
  __int128 mant = static_cast<__int128>(std::llround(m_int_d));
  int shift = 24 - e;
  if (shift < 0) {
    mant <<= (-shift);
    shift = 0;
  }
  QS3 out = { mant, 0, shift };
  StripTrailingZeros(&out);
  return out;
}

inline QS3 DoubleToQS3(double d) {
  if (d == 0.0) {
    return QS3Zero();
  }
  int e = 0;
  double m = std::frexp(d, &e);
  double m_int_d = std::ldexp(m, 53);  // full double mantissa
  __int128 mant = static_cast<__int128>(std::llround(m_int_d));
  int shift = 53 - e;
  if (shift < 0) {
    mant <<= (-shift);
    shift = 0;
  }
  QS3 out = { mant, 0, shift };
  StripTrailingZeros(&out);
  return out;
}

// Integer constant → QS3 pure-rational.
inline QS3 IntToQS3(int n) {
  return { static_cast<__int128>(n), 0, 0 };
}

// Q(√3) constant √3 = (0, 1) at shift 0.
inline QS3 QS3Sqrt3() {
  return { 0, 1, 0 };
}

// Divide QS3 by 2^n (equivalently, add n to shift).
inline QS3 QS3DivPow2(QS3 x, int n) {
  x.shift += n;
  return x;
}

// Convert QS3 to double (for reporting / sign hints).
inline double QS3ToDouble(QS3 x) {
  const double kSqrt3D = 1.7320508075688772;
  return (static_cast<double>(x.a) + static_cast<double>(x.b) * kSqrt3D) * std::ldexp(1.0, -x.shift);
}

// ============================================================================
// Plane construction from parameters — mirrors FillHexCrystalCoef's symbolic
// form (geo3d.cpp:346-434). Every step in exact Q(√3).
// ============================================================================
//
// Six horizontal direction pairs (i=0..5). For each i, the two "corners" of
// the prism face bracket the direction at ±30° offsets from i·60°:
//   x1 = 0.5 · cos(-π/6 + i·π/3)   x2 = 0.5 · cos(π/6 + i·π/3)
//   y1 = 0.5 · sin(-π/6 + i·π/3)   y2 = 0.5 · sin(π/6 + i·π/3)
// The cos/sin values at 30° multiples take values in {0, ±1/2, ±√3/2}, so
// 0.5·cos and 0.5·sin take values in {0, ±1/4, ±√3/4}. Hardcoded as QS3
// constants at shift 2 (denominator 4).

struct DirCorners {
  QS3 x1, x2, y1, y2;
};

// (a, b) at shift 2 = (a + b·√3) / 4. Values in {0, ±1, ±√3}/4.
inline DirCorners GetDirCorners(int i) {
  auto make = [](int a, int b) -> QS3 { return { static_cast<__int128>(a), static_cast<__int128>(b), 2 }; };
  //  i    -π/6+iπ/3      π/6+iπ/3           x1=0.5·cos     y1=0.5·sin      x2=0.5·cos      y2=0.5·sin
  //  0    -30°           30°                √3/4           -1/4            √3/4            1/4
  //  1    30°            90°                √3/4           1/4             0               1/2
  //  2    90°            150°               0              1/2             -√3/4           1/4
  //  3    150°           210°               -√3/4          1/4             -√3/4           -1/4
  //  4    210°           270°               -√3/4          -1/4            0               -1/2
  //  5    270°           330°               0              -1/2            √3/4            -1/4
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

// The det = x1·y2 − x2·y1 is a constant √3/8 for all i (this is what makes
// dist[i] enter as `-dist[i]·√3/8` uniformly). Hardcoded to skip the QS3
// multiplication overhead and avoid any risk of arithmetic drift.
// Value: (0, 1) at shift 3.
inline QS3 GetDirDet() {
  return { 0, 1, 3 };
}

// Build the 20 planes in Q(√3), matching FillHexCrystalCoef's layout:
//   0: upper basal (0, 0, 1, -z_top)
//   1: lower basal (0, 0, -1, z_bot)
//   2..7: prism face i
//   8..13: upper cone face i (only if a1 > 0)
//   14..19: lower cone face i (only if a2 > 0)
struct PlaneQS3 {
  QS3 A, B, C, D;
};

inline void BuildPrismPlane(int i, QS3 dist_i, PlaneQS3* out, bool* overflow, int* max_bits) {
  DirCorners dc = GetDirCorners(i);
  QS3 det = GetDirDet();  // √3/8
  out->A = QS3Sub(dc.y2, dc.y1, overflow, max_bits);
  out->B = QS3Sub(dc.x1, dc.x2, overflow, max_bits);
  out->C = QS3Zero();
  QS3 neg_dist = QS3Neg(dist_i);
  out->D = QS3Mul(neg_dist, det, overflow, max_bits);
}

inline void BuildConePlane(int i, QS3 a_cone, QS3 h2_2, QS3 dist_i, bool upper, PlaneQS3* out, bool* overflow,
                           int* max_bits) {
  DirCorners dc = GetDirCorners(i);
  QS3 det = GetDirDet();  // √3/8; upper C = +det, lower C = -det
  QS3 y_diff = QS3Sub(dc.y2, dc.y1, overflow, max_bits);
  QS3 x_diff = QS3Sub(dc.x1, dc.x2, overflow, max_bits);
  out->A = QS3Mul(a_cone, y_diff, overflow, max_bits);
  out->B = QS3Mul(a_cone, x_diff, overflow, max_bits);
  out->C = upper ? det : QS3Neg(det);
  // D = -(h2/2 + a·dist[i]) · det
  QS3 a_dist = QS3Mul(a_cone, dist_i, overflow, max_bits);
  QS3 inner = QS3Add(h2_2, a_dist, overflow, max_bits);
  QS3 neg_inner = QS3Neg(inner);
  out->D = QS3Mul(neg_inner, det, overflow, max_bits);
}

// ============================================================================
// z_top / z_bot from the LP: max m subject to cos(θ_i)·u + sin(θ_i)·v + m ≤
// (√3/4)·dist[i]. The LP maximum lies at a 3-direction concurrence; enumerate
// C(6, 3) = 20 triples in Q(√3), pick the largest feasible m.
// ============================================================================
//
// (cos, sin) at 60° multiples in Q(√3):
//   i=0: (1, 0)   i=1: (1/2, √3/2)   i=2: (-1/2, √3/2)
//   i=3: (-1, 0)  i=4: (-1/2, -√3/2) i=5: (1/2, -√3/2)
inline QS3 GetDirCos(int i) {
  auto make = [](int a, int b, int shift) -> QS3 {
    return { static_cast<__int128>(a), static_cast<__int128>(b), shift };
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
    return { static_cast<__int128>(a), static_cast<__int128>(b), shift };
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

// Compare two QS3 values: returns sign(x - y). On refuse, sets *sign_refuse.
inline int CompareQS3(QS3 x, QS3 y, bool* overflow, int* max_bits, bool* sign_refuse) {
  QS3 diff = QS3Sub(x, y, overflow, max_bits);
  if (*overflow) {
    return 0;
  }
  return SignQS3(diff, sign_refuse);
}

// Solve the 3×3 system for the LP:
//   [cos_i  sin_i  1] [u]   [d_scaled_i]
//   [cos_j  sin_j  1] [v] = [d_scaled_j]
//   [cos_k  sin_k  1] [m]   [d_scaled_k]
// where d_scaled_i = √3/4 · dist[i]. Returns (u, v, m) as QS3, plus the
// system determinant (must be != 0 for a unique solution).
struct LP3Solution {
  QS3 u, v, m, det;
  bool degenerate;
};

inline LP3Solution SolveLP3(int i, int j, int k, const QS3 d_scaled[6], bool* overflow, int* max_bits) {
  LP3Solution s{};
  s.degenerate = false;
  QS3 ci = GetDirCos(i), si = GetDirSin(i);
  QS3 cj = GetDirCos(j), sj = GetDirSin(j);
  QS3 ck = GetDirCos(k), sk = GetDirSin(k);
  QS3 one = IntToQS3(1);

  // det = ci*(sj - sk) - si*(cj - ck) + (cj*sk - ck*sj)
  QS3 sj_sk = QS3Sub(sj, sk, overflow, max_bits);
  QS3 cj_ck = QS3Sub(cj, ck, overflow, max_bits);
  QS3 cjsk = QS3Mul(cj, sk, overflow, max_bits);
  QS3 cksj = QS3Mul(ck, sj, overflow, max_bits);
  QS3 t1 = QS3Mul(ci, sj_sk, overflow, max_bits);
  QS3 t2 = QS3Mul(si, cj_ck, overflow, max_bits);
  QS3 t3 = QS3Sub(cjsk, cksj, overflow, max_bits);
  QS3 t12 = QS3Sub(t1, t2, overflow, max_bits);
  s.det = QS3Add(t12, t3, overflow, max_bits);
  (void)one;

  if (*overflow) {
    return s;
  }
  if (IsZero(s.det)) {
    s.degenerate = true;
    return s;
  }

  QS3 di = d_scaled[i], dj = d_scaled[j], dk = d_scaled[k];

  // u_num (numerator; s.u = u_num / det): replace column 0 with d
  //   u_num = di*(sj - sk) - si*(dj - dk) + (dj*sk - dk*sj)
  QS3 dj_dk = QS3Sub(dj, dk, overflow, max_bits);
  QS3 djsk = QS3Mul(dj, sk, overflow, max_bits);
  QS3 dksj = QS3Mul(dk, sj, overflow, max_bits);
  QS3 u1 = QS3Mul(di, sj_sk, overflow, max_bits);
  QS3 u2 = QS3Mul(si, dj_dk, overflow, max_bits);
  QS3 u3 = QS3Sub(djsk, dksj, overflow, max_bits);
  QS3 u12 = QS3Sub(u1, u2, overflow, max_bits);
  s.u = QS3Add(u12, u3, overflow, max_bits);

  // v_num: replace column 1 with d
  //   v_num = ci*(dj - dk) - di*(cj - ck) + (cj*dk - ck*dj)
  QS3 cjdk = QS3Mul(cj, dk, overflow, max_bits);
  QS3 ckdj = QS3Mul(ck, dj, overflow, max_bits);
  QS3 v1 = QS3Mul(ci, dj_dk, overflow, max_bits);
  QS3 v2 = QS3Mul(di, cj_ck, overflow, max_bits);
  QS3 v3 = QS3Sub(cjdk, ckdj, overflow, max_bits);
  QS3 v12 = QS3Sub(v1, v2, overflow, max_bits);
  s.v = QS3Add(v12, v3, overflow, max_bits);

  // m_num: replace column 2 with d
  //   m_num = ci*(sj·dk - sk·dj) - si*(cj·dk - ck·dj) + di*(cj·sk - ck·sj)
  QS3 sjdk = QS3Mul(sj, dk, overflow, max_bits);
  QS3 skdj = QS3Mul(sk, dj, overflow, max_bits);
  QS3 sub1 = QS3Sub(sjdk, skdj, overflow, max_bits);
  QS3 m1 = QS3Mul(ci, sub1, overflow, max_bits);
  QS3 sub2 = QS3Sub(cjdk, ckdj, overflow, max_bits);
  QS3 m2 = QS3Mul(si, sub2, overflow, max_bits);
  QS3 sub3 = QS3Sub(cjsk, cksj, overflow, max_bits);
  QS3 m3 = QS3Mul(di, sub3, overflow, max_bits);
  QS3 m12 = QS3Sub(m1, m2, overflow, max_bits);
  s.m = QS3Add(m12, m3, overflow, max_bits);

  return s;
}

// Canonicalize a QS3 rational (u_num/det) sign so det > 0. If det == 0, returns
// as-is. Uses SignQS3 which may set sign_refuse for opposite-sign case.
inline void CanonicalizeSign(QS3* px, QS3* py, QS3* pz, QS3* det, bool* sign_refuse) {
  int s = SignQS3(*det, sign_refuse);
  if (*sign_refuse) {
    return;
  }
  if (s < 0) {
    *px = QS3Neg(*px);
    *py = QS3Neg(*py);
    *pz = QS3Neg(*pz);
    *det = QS3Neg(*det);
  }
}

// Compute the LP max m (as u/det, m/det tuple) over all feasible triples.
// Returns the m rational (num, det). If refused for any reason, sets refused.
// Also returns u, v of the winning triple.
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
        bool sign_refuse = false;
        LP3Solution s = SolveLP3(i, j, k, d_scaled, &overflow, max_bits);
        if (overflow) {
          out.refused = true;
          *refuse_reason = "LP3 solve overflow";
          return out;
        }
        if (s.degenerate) {
          continue;
        }
        // Canonicalize: sign(det) > 0.
        CanonicalizeSign(&s.u, &s.v, &s.m, &s.det, &sign_refuse);
        if (sign_refuse) {
          out.refused = true;
          *refuse_reason = "LP3 det sign ambiguous";
          return out;
        }
        // Reduce integer GCD so subsequent arithmetic doesn't grow unbounded.
        ReducePoint(&s.u, &s.v, &s.m, &s.det);

        // Feasibility: for every OTHER direction r, check
        //   cos_r · u + sin_r · v + m·1 ≤ d_scaled_r · det   (multiply through by det > 0)
        bool feasible = true;
        for (int r = 0; r < 6; r++) {
          if (r == i || r == j || r == k) {
            continue;
          }
          QS3 cr = GetDirCos(r);
          QS3 sr = GetDirSin(r);
          QS3 lhs1 = QS3Mul(cr, s.u, &overflow, max_bits);
          QS3 lhs2 = QS3Mul(sr, s.v, &overflow, max_bits);
          QS3 lhs12 = QS3Add(lhs1, lhs2, &overflow, max_bits);
          QS3 lhs = QS3Add(lhs12, s.m, &overflow, max_bits);
          QS3 rhs = QS3Mul(d_scaled[r], s.det, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "LP3 feasibility overflow";
            return out;
          }
          QS3 diff = QS3Sub(lhs, rhs, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "LP3 feasibility diff overflow";
            return out;
          }
          int sign = SignQS3(diff, &sign_refuse);
          if (sign_refuse) {
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
        // Compare m = s.m/s.det against best via cross-multiply of denominators
        // (both dets > 0 after canonicalize): m > best_m iff s.m · best_det > best_m · s.det.
        if (!best_set) {
          best_m_num = s.m;
          best_det = s.det;
          best_u_num = s.u;
          best_v_num = s.v;
          best_set = true;
        } else {
          QS3 lhs = QS3Mul(s.m, best_det, &overflow, max_bits);
          QS3 rhs = QS3Mul(best_m_num, s.det, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "apex compare overflow";
            return out;
          }
          QS3 diff = QS3Sub(lhs, rhs, &overflow, max_bits);
          if (overflow) {
            out.refused = true;
            *refuse_reason = "apex compare diff overflow";
            return out;
          }
          bool cmp_refuse = false;
          int sign = SignQS3(diff, &cmp_refuse);
          if (cmp_refuse) {
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
// 3-plane intersection point in Q(√3).
// ============================================================================
//
// For planes p0, p1, p2 with rows (A, B, C, D), solve
//   [A0 B0 C0] [x]   [-D0]
//   [A1 B1 C1] [y] = [-D1]
//   [A2 B2 C2] [z]   [-D2]
// Cramer: det = det3([A|B|C]); x_num = det with column 0 replaced by -D;
// similarly for y, z. Point = (x_num/det, y_num/det, z_num/det).
struct TriplePoint {
  QS3 px, py, pz, det;
  bool degenerate;  // det == 0
};

inline TriplePoint SolveTriple(const PlaneQS3& p0, const PlaneQS3& p1, const PlaneQS3& p2, bool* overflow,
                               int* max_bits) {
  TriplePoint out{};
  out.degenerate = false;
  // det via cofactor expansion along row 0.
  //   det = A0 * (B1·C2 - B2·C1) - B0 * (A1·C2 - A2·C1) + C0 * (A1·B2 - A2·B1)
  QS3 b1c2 = QS3Mul(p1.B, p2.C, overflow, max_bits);
  QS3 b2c1 = QS3Mul(p2.B, p1.C, overflow, max_bits);
  QS3 minor0 = QS3Sub(b1c2, b2c1, overflow, max_bits);
  QS3 a1c2 = QS3Mul(p1.A, p2.C, overflow, max_bits);
  QS3 a2c1 = QS3Mul(p2.A, p1.C, overflow, max_bits);
  QS3 minor1 = QS3Sub(a1c2, a2c1, overflow, max_bits);
  QS3 a1b2 = QS3Mul(p1.A, p2.B, overflow, max_bits);
  QS3 a2b1 = QS3Mul(p2.A, p1.B, overflow, max_bits);
  QS3 minor2 = QS3Sub(a1b2, a2b1, overflow, max_bits);
  QS3 t0 = QS3Mul(p0.A, minor0, overflow, max_bits);
  QS3 t1 = QS3Mul(p0.B, minor1, overflow, max_bits);
  QS3 t2 = QS3Mul(p0.C, minor2, overflow, max_bits);
  QS3 t01 = QS3Sub(t0, t1, overflow, max_bits);
  out.det = QS3Add(t01, t2, overflow, max_bits);

  if (*overflow) {
    return out;
  }
  if (IsZero(out.det)) {
    out.degenerate = true;
    return out;
  }

  QS3 nD0 = QS3Neg(p0.D);
  QS3 nD1 = QS3Neg(p1.D);
  QS3 nD2 = QS3Neg(p2.D);

  // x_num: replace column 0 with -D
  //   = nD0 * (B1·C2 - B2·C1) - B0 * (nD1·C2 - nD2·C1) + C0 * (nD1·B2 - nD2·B1)
  QS3 nD1c2 = QS3Mul(nD1, p2.C, overflow, max_bits);
  QS3 nD2c1 = QS3Mul(nD2, p1.C, overflow, max_bits);
  QS3 xm1 = QS3Sub(nD1c2, nD2c1, overflow, max_bits);
  QS3 nD1b2 = QS3Mul(nD1, p2.B, overflow, max_bits);
  QS3 nD2b1 = QS3Mul(nD2, p1.B, overflow, max_bits);
  QS3 xm2 = QS3Sub(nD1b2, nD2b1, overflow, max_bits);
  QS3 xt0 = QS3Mul(nD0, minor0, overflow, max_bits);
  QS3 xt1 = QS3Mul(p0.B, xm1, overflow, max_bits);
  QS3 xt2 = QS3Mul(p0.C, xm2, overflow, max_bits);
  QS3 xt01 = QS3Sub(xt0, xt1, overflow, max_bits);
  out.px = QS3Add(xt01, xt2, overflow, max_bits);

  // y_num: replace column 1 with -D
  //   = A0 * (nD1·C2 - nD2·C1) - nD0 * (A1·C2 - A2·C1) + C0 * (A1·nD2 - A2·nD1)
  QS3 a1nD2 = QS3Mul(p1.A, nD2, overflow, max_bits);
  QS3 a2nD1 = QS3Mul(p2.A, nD1, overflow, max_bits);
  QS3 ym2 = QS3Sub(a1nD2, a2nD1, overflow, max_bits);
  QS3 yt0 = QS3Mul(p0.A, xm1, overflow, max_bits);
  QS3 yt1 = QS3Mul(nD0, minor1, overflow, max_bits);
  QS3 yt2 = QS3Mul(p0.C, ym2, overflow, max_bits);
  QS3 yt01 = QS3Sub(yt0, yt1, overflow, max_bits);
  out.py = QS3Add(yt01, yt2, overflow, max_bits);

  // z_num: replace column 2 with -D
  //   = A0 * (B1·nD2 - B2·nD1) - B0 * (A1·nD2 - A2·nD1) + nD0 * (A1·B2 - A2·B1)
  QS3 b1nD2 = QS3Mul(p1.B, nD2, overflow, max_bits);
  QS3 b2nD1 = QS3Mul(p2.B, nD1, overflow, max_bits);
  QS3 zm1 = QS3Sub(b1nD2, b2nD1, overflow, max_bits);
  QS3 zt0 = QS3Mul(p0.A, zm1, overflow, max_bits);
  QS3 zt1 = QS3Mul(p0.B, ym2, overflow, max_bits);
  QS3 zt2 = QS3Mul(nD0, minor2, overflow, max_bits);
  QS3 zt01 = QS3Sub(zt0, zt1, overflow, max_bits);
  out.pz = QS3Add(zt01, zt2, overflow, max_bits);

  return out;
}

// IsIncident: plane · (px, py, pz)/det + D = 0 iff
//   A·px + B·py + C·pz + D·det == 0 (as QS3).
inline bool IsIncident(const PlaneQS3& plane, QS3 px, QS3 py, QS3 pz, QS3 det, bool* overflow, int* max_bits) {
  QS3 apx = QS3Mul(plane.A, px, overflow, max_bits);
  QS3 bpy = QS3Mul(plane.B, py, overflow, max_bits);
  QS3 cpz = QS3Mul(plane.C, pz, overflow, max_bits);
  QS3 ddet = QS3Mul(plane.D, det, overflow, max_bits);
  QS3 sum1 = QS3Add(apx, bpy, overflow, max_bits);
  QS3 sum2 = QS3Add(sum1, cpz, overflow, max_bits);
  QS3 sum3 = QS3Add(sum2, ddet, overflow, max_bits);
  return IsZero(sum3);
}

// IsFeasible: A·px + B·py + C·pz + D·det ≤ 0 (assuming det > 0).
// Returns +1 infeasible, 0 boundary/feasible, -1 refuse.
inline int IsFeasibleSided(const PlaneQS3& plane, QS3 px, QS3 py, QS3 pz, QS3 det, bool* overflow, int* max_bits,
                           bool* sign_refuse) {
  QS3 apx = QS3Mul(plane.A, px, overflow, max_bits);
  QS3 bpy = QS3Mul(plane.B, py, overflow, max_bits);
  QS3 cpz = QS3Mul(plane.C, pz, overflow, max_bits);
  QS3 ddet = QS3Mul(plane.D, det, overflow, max_bits);
  QS3 sum1 = QS3Add(apx, bpy, overflow, max_bits);
  QS3 sum2 = QS3Add(sum1, cpz, overflow, max_bits);
  QS3 sum3 = QS3Add(sum2, ddet, overflow, max_bits);
  if (*overflow) {
    return -1;
  }
  int s = SignQS3(sum3, sign_refuse);
  if (*sign_refuse) {
    return -1;
  }
  return s > 0 ? 1 : 0;
}

}  // namespace exact_pyramid_detail

// ============================================================================
// Public API: takes parameters, constructs planes in Q(√3), enumerates vertices.
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

  // Legality gate — matches FillHexCrystalCoef (geo3d.cpp:381-382).
  const bool has_upper = (a1 > 0.0) && (h1 > 1e-6f);
  const bool has_lower = (a2 > 0.0) && (h3 > 1e-6f);

  // Convert scalar inputs to Q(√3) pure rationals.
  d::QS3 dist_qs[6];
  for (int i = 0; i < 6; i++) {
    dist_qs[i] = d::FloatToQS3(dist[i]);
  }
  d::QS3 h2_2_qs = d::DoubleToQS3(0.5 * static_cast<double>(h2));
  d::QS3 a1_qs = has_upper ? d::DoubleToQS3(a1) : d::QS3Zero();
  d::QS3 a2_qs = has_lower ? d::DoubleToQS3(a2) : d::QS3Zero();
  d::QS3 h1_qs = has_upper ? d::FloatToQS3(h1) : d::QS3Zero();
  d::QS3 h3_qs = has_lower ? d::FloatToQS3(h3) : d::QS3Zero();

  // Zero-volume short-circuit: no cones and prism section is zero.
  if (!has_upper && !has_lower && h2 < 1e-6f) {
    return out;
  }

  // Build 20 planes (some may be zero/absent placeholders — we track presence).
  d::PlaneQS3 planes[kExactPyramidMaxPlanes];
  bool plane_active[kExactPyramidMaxPlanes] = { false };
  bool overflow = false;

  // Prism planes (slots 2..7)
  for (int i = 0; i < 6; i++) {
    d::BuildPrismPlane(i, dist_qs[i], &planes[2 + i], &overflow, &max_bits);
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
      d::BuildConePlane(i, a1_qs, h2_2_qs, dist_qs[i], /*upper=*/true, &planes[8 + i], &overflow, &max_bits);
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
      d::BuildConePlane(i, a2_qs, h2_2_qs, dist_qs[i], /*upper=*/false, &planes[14 + i], &overflow, &max_bits);
      plane_active[14 + i] = true;
    }
    if (overflow) {
      out.refused = true;
      out.refuse_reason = "lower cone plane build overflow";
      out.max_intermediate_bits = max_bits;
      return out;
    }
  }

  // Compute z_top / z_bot via the LP over Q(√3). z_top = h2/2 + h1·a1·m_apex_upper.
  // For h1 == 1 (apex reached), z_top passes through the apex. For h1 < 1, it truncates.
  d::QS3 z_top{}, z_bot{};
  if (has_upper || has_lower) {
    // d_scaled_i = √3/4 · dist[i] = √3 · dist[i] / 4
    d::QS3 sqrt3_over_4 = { 0, 1, 2 };  // (0 + 1·√3) / 4
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
      // Empty feasible region — nothing to build (matches production's early return).
      return out;
    }
    // m_apex (physical) = apex.m_num / apex.det · (4/√3) since LP m was in LP units.
    // Actually the LP was: cos·u + sin·v + m ≤ √3/4·dist[i]. So m_LP is at the same
    // scale as √3/4·dist. Physical m_phys satisfies √3/4·(dist - m_phys) = √3/4·dist - m_LP,
    // giving m_phys = m_LP · 4/√3 = m_LP · 4√3/3. To convert m_LP to physical, multiply
    // by (4·√3/3) — or equivalently divide by √3/4 = kInsetK.
    // Actually simpler: z_top = h2/2 + h1 · a1 · m_phys = h2/2 + h1 · a1 · (m_LP / (√3/4))
    //                        = h2/2 + h1 · a1 · m_LP · 4/√3
    //                        = h2/2 + h1 · a1 · m_LP · 4·√3/3.
    // Compute (4·√3/3) as QS3: numerator 4·√3, denom 3 (no dyadic). Since 3 is not a
    // power of 2, we cannot represent 1/3 as a pure dyadic QS3.
    //
    // BUT: the LP solution already tracks m_num and det. The rational m_phys is exactly
    //   m_phys = (m_num · 4·√3/3) / det = (4·√3·m_num) / (3·det).
    // For z_top: h2/2 + h1·a1·m_phys is a rational in Q(√3) (all inputs Q(√3)).
    //
    // To keep operands dyadic, express z_top with an aggregate denominator. Compute:
    //   inner = h1 · a1 · m_num · 4·√3    (numerator)
    //   denom = 3 · det                    (denominator)
    //   z_top = (h2/2 · denom + inner) / denom.
    // Both operands still in Q(√3), and 3·det is Q(√3) (not integer). We don't need
    // to normalize dyadically; QS3 arithmetic handles it.

    d::QS3 four_sqrt3 = { 0, 4, 0 };  // 4·√3 = (0 + 4·√3)/1
    d::QS3 three = { 3, 0, 0 };

    if (has_upper) {
      // clamp: m_top_LP = min(h1 · m_apex_LP, m_apex_LP)  — production behavior.
      // Since we represent m_apex as (m_num / det), we compute:
      //   candidate = h1 · m_num / det.
      //   if h1 > 1: cap at m_apex → m_top = m_num / det.
      //   else: m_top = h1·m_num / det.
      // Compare h1 vs 1 (both Q(√3)):
      d::QS3 one_qs = d::IntToQS3(1);
      bool cmp_refuse = false;
      int cmp = d::CompareQS3(h1_qs, one_qs, &overflow, &max_bits, &cmp_refuse);
      if (overflow || cmp_refuse) {
        out.refused = true;
        out.refuse_reason = "h1 vs 1 compare failed";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      d::QS3 m_top_num;
      if (cmp >= 0) {
        m_top_num = apex.m_num;
      } else {
        m_top_num = d::QS3Mul(h1_qs, apex.m_num, &overflow, &max_bits);
      }
      d::QS3 m_top_det = apex.det;
      // z_top = h2/2 + a1 · m_top_num · (4·√3) / (3 · m_top_det)
      // Represent as z_top_num / z_top_denom where z_top_denom = 3·m_top_det, and
      // z_top_num = h2/2 · z_top_denom + a1 · m_top_num · 4·√3.
      d::QS3 z_top_denom = d::QS3Mul(three, m_top_det, &overflow, &max_bits);
      d::QS3 a1_mtop_4s3 = d::QS3Mul(a1_qs, m_top_num, &overflow, &max_bits);
      a1_mtop_4s3 = d::QS3Mul(a1_mtop_4s3, four_sqrt3, &overflow, &max_bits);
      d::QS3 h2_2_zd = d::QS3Mul(h2_2_qs, z_top_denom, &overflow, &max_bits);
      d::QS3 z_top_num = d::QS3Add(h2_2_zd, a1_mtop_4s3, &overflow, &max_bits);
      if (overflow) {
        out.refused = true;
        out.refuse_reason = "z_top compute overflow";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      // Upper basal plane: (0, 0, 1, -z_top). To express -z_top in QS3 alone, we'd
      // need to divide z_top_num by z_top_denom. Instead, embed the fraction into
      // the plane: (0, 0, z_top_denom, -z_top_num). This scales the plane by
      // z_top_denom (which is positive by canonicalize). Half-space
      //   z_top_denom · z ≤ z_top_num
      // is equivalent to z ≤ z_top_num / z_top_denom = z_top.
      // For consistency with other planes (each has A, B, C in Q(√3)), we set C =
      // z_top_denom and D = -z_top_num.
      planes[0].A = d::QS3Zero();
      planes[0].B = d::QS3Zero();
      planes[0].C = z_top_denom;
      planes[0].D = d::QS3Neg(z_top_num);
      plane_active[0] = true;
      // Compute physical z_top for reporting.
      double zt = d::QS3ToDouble(z_top_num) / d::QS3ToDouble(z_top_denom);
      z_top = d::DoubleToQS3(zt);  // approximate for reporting only
      (void)z_top;
    }
    if (has_lower) {
      d::QS3 one_qs = d::IntToQS3(1);
      bool cmp_refuse = false;
      int cmp = d::CompareQS3(h3_qs, one_qs, &overflow, &max_bits, &cmp_refuse);
      if (overflow || cmp_refuse) {
        out.refused = true;
        out.refuse_reason = "h3 vs 1 compare failed";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      d::QS3 m_bot_num;
      if (cmp >= 0) {
        m_bot_num = apex.m_num;
      } else {
        m_bot_num = d::QS3Mul(h3_qs, apex.m_num, &overflow, &max_bits);
      }
      d::QS3 m_bot_det = apex.det;
      // z_bot = -h2/2 - a2 · m_bot_num · (4·√3) / (3 · m_bot_det)
      d::QS3 z_bot_denom = d::QS3Mul(three, m_bot_det, &overflow, &max_bits);
      d::QS3 a2_mbot_4s3 = d::QS3Mul(a2_qs, m_bot_num, &overflow, &max_bits);
      a2_mbot_4s3 = d::QS3Mul(a2_mbot_4s3, four_sqrt3, &overflow, &max_bits);
      d::QS3 neg_h2_2 = d::QS3Neg(h2_2_qs);
      d::QS3 neg_h2_2_zd = d::QS3Mul(neg_h2_2, z_bot_denom, &overflow, &max_bits);
      d::QS3 neg_a2 = d::QS3Neg(a2_mbot_4s3);
      d::QS3 z_bot_num = d::QS3Add(neg_h2_2_zd, neg_a2, &overflow, &max_bits);
      if (overflow) {
        out.refused = true;
        out.refuse_reason = "z_bot compute overflow";
        out.max_intermediate_bits = max_bits;
        return out;
      }
      // Lower basal plane: (0, 0, -1, z_bot) with our scaling.
      //   -z ≤ -z_bot ⇔ z ≥ z_bot. So (0, 0, -z_bot_denom, z_bot_num).
      planes[1].A = d::QS3Zero();
      planes[1].B = d::QS3Zero();
      planes[1].C = d::QS3Neg(z_bot_denom);
      planes[1].D = z_bot_num;
      plane_active[1] = true;
      z_bot = d::DoubleToQS3(d::QS3ToDouble(z_bot_num) / d::QS3ToDouble(z_bot_denom));
      (void)z_bot;
    }
  } else {
    // Pure prism: no cones. Basal planes are (0, 0, ±1, ∓h2/2).
    // Upper basal (0, 0, 1, -h2/2), lower basal (0, 0, -1, -h2/2). Both active.
    d::QS3 neg_h2_2 = d::QS3Neg(h2_2_qs);
    planes[0].A = d::QS3Zero();
    planes[0].B = d::QS3Zero();
    planes[0].C = d::IntToQS3(1);
    planes[0].D = neg_h2_2;
    plane_active[0] = true;
    planes[1].A = d::QS3Zero();
    planes[1].B = d::QS3Zero();
    planes[1].C = d::IntToQS3(-1);
    planes[1].D = neg_h2_2;
    plane_active[1] = true;
  }

  // Compact active plane indices.
  int active_idx[kExactPyramidMaxPlanes];
  int active_n = 0;
  for (int i = 0; i < kExactPyramidMaxPlanes; i++) {
    if (plane_active[i]) {
      active_idx[active_n++] = i;
    }
  }

  // Enumerate C(active_n, 3) triples.
  struct Vertex {
    d::QS3 px, py, pz, det;
    int def_plane[3];  // active-slot indices of the defining triple
  };
  Vertex verts[kExactPyramidMaxVtx];
  int vtx_cnt = 0;

  for (int a = 0; a < active_n; a++) {
    for (int b = a + 1; b < active_n; b++) {
      for (int c = b + 1; c < active_n; c++) {
        int i = active_idx[a];
        int j = active_idx[b];
        int k = active_idx[c];
        bool overflow_t = false;
        d::TriplePoint tp = d::SolveTriple(planes[i], planes[j], planes[k], &overflow_t, &max_bits);
        if (overflow_t) {
          out.refused = true;
          out.refuse_reason = "triple solve overflow";
          out.max_intermediate_bits = max_bits;
          return out;
        }
        if (tp.degenerate) {
          continue;
        }
        // Canonicalize det > 0.
        bool sign_refuse = false;
        d::CanonicalizeSign(&tp.px, &tp.py, &tp.pz, &tp.det, &sign_refuse);
        if (sign_refuse) {
          out.refused = true;
          out.refuse_reason = "triple det sign ambiguous";
          out.max_intermediate_bits = max_bits;
          return out;
        }
        // Reduce integer GCD to keep operands smaller for downstream ops.
        d::ReducePoint(&tp.px, &tp.py, &tp.pz, &tp.det);

        // Feasibility: check against all active planes (not just the defining 3).
        bool feasible = true;
        for (int r = 0; r < active_n; r++) {
          int p_idx = active_idx[r];
          if (p_idx == i || p_idx == j || p_idx == k) {
            continue;
          }
          int fr = d::IsFeasibleSided(planes[p_idx], tp.px, tp.py, tp.pz, tp.det, &overflow_t, &max_bits, &sign_refuse);
          if (fr == -1) {
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

        // Dedup: is this point already in the pool? A candidate matches an
        // existing vertex iff it is incident to ALL 3 of that vertex's defining
        // planes (equivalently, they share 3 planes' concurrence).
        int existing = -1;
        for (int v = 0; v < vtx_cnt; v++) {
          bool all_incident = true;
          for (int q = 0; q < 3; q++) {
            int p_idx = verts[v].def_plane[q];
            if (p_idx == i || p_idx == j || p_idx == k) {
              continue;  // trivially incident by construction
            }
            bool ov = false;
            if (!d::IsIncident(planes[p_idx], tp.px, tp.py, tp.pz, tp.det, &ov, &max_bits)) {
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
          verts[vtx_cnt].px = tp.px;
          verts[vtx_cnt].py = tp.py;
          verts[vtx_cnt].pz = tp.pz;
          verts[vtx_cnt].det = tp.det;
          verts[vtx_cnt].def_plane[0] = i;
          verts[vtx_cnt].def_plane[1] = j;
          verts[vtx_cnt].def_plane[2] = k;
          vtx_cnt++;
        }
      }
    }
  }

  // Compute per-plane vertex counts via incidence tests over all planes.
  int face_vtx[kExactPyramidMaxPlanes] = { 0 };
  for (int v = 0; v < vtx_cnt; v++) {
    for (int p = 0; p < kExactPyramidMaxPlanes; p++) {
      if (!plane_active[p]) {
        continue;
      }
      // Fast path: if p is one of the defining triple, it's incident.
      if (verts[v].def_plane[0] == p || verts[v].def_plane[1] == p || verts[v].def_plane[2] == p) {
        face_vtx[p]++;
        continue;
      }
      bool ov = false;
      if (d::IsIncident(planes[p], verts[v].px, verts[v].py, verts[v].pz, verts[v].det, &ov, &max_bits)) {
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

  // Fill output.
  out.vertex_count = vtx_cnt;
  for (int v = 0; v < vtx_cnt; v++) {
    double det_d = d::QS3ToDouble(verts[v].det);
    out.vertex_xyz[v][0] = d::QS3ToDouble(verts[v].px) / det_d;
    out.vertex_xyz[v][1] = d::QS3ToDouble(verts[v].py) / det_d;
    out.vertex_xyz[v][2] = d::QS3ToDouble(verts[v].pz) / det_d;
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
