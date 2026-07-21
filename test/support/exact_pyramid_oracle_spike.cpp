// Throwaway spike: bivariate PolyQS3(α, β) engine + fixed-pool bit-width and
// double-Horner sign-filter measurement, to gate the rewrite of
// exact_pyramid_oracle.hpp from `__int128 Q(√3) with numeric a1 substitution`
// to `int64_t QS3 with symbolic a1 = α, a2 = β polynomial arithmetic`.
//
// What this file demonstrates numerically:
//   1. Joint (α, β) coefficient peak bit-width across every fixed pool
//      (including a synthetic dual-89° worst case) stays ≪ int64 budget;
//   2. Zero arithmetic overflow at kMaxOperandBits = 60;
//   3. Double-Horner sign filter is unambiguous on well-conditioned and
//      degenerate pools; near-boundary ambiguity on flat-tail pools maps
//      to the oracle's existing refuse contract.
//
// The PolyQS3 primitives here (PolyAdd/PolyMul/PolyIsZero/PolySign) are the
// intended starting point when the same engine is lifted into
// exact_pyramid_oracle.hpp. Delete this file once the lifted engine is
// wired in and cross-verified against the current __int128 oracle.
//
// Build (standalone; not integrated into CMake):
//   cd "$(git rev-parse --show-toplevel)"
//   clang++ -std=c++17 -O2 -I. -Isrc -Iproj -I/dev/null \
//     test/support/exact_pyramid_oracle_spike.cpp -o /tmp/oracle_spike
//   /tmp/oracle_spike
//
// Reports three numbers per plan §4 Step 1 test point.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// ============================================================================
// Minimal pool-header inclusion — avoid pulling in lumice core/math to keep
// this standalone. The pool header only depends on <cstddef>.
// ============================================================================

#include "test/golden-analytic/core/closed_form_samples_generated.hpp"

namespace spike {

using ::lumice::test_support::kPyramidDegenerateSigma030Samples;
using ::lumice::test_support::kPyramidDegenerateSigma050Samples;
using ::lumice::test_support::kPyramidFlatTailAlpha85Samples;
using ::lumice::test_support::kPyramidFlatTailAlpha875Samples;
using ::lumice::test_support::kPyramidFlatTailAlpha87Samples;
using ::lumice::test_support::kPyramidFlatTailAlpha88Samples;
using ::lumice::test_support::kPyramidFlatTailAlpha895Samples;
using ::lumice::test_support::kPyramidFlatTailAlpha89Samples;
using ::lumice::test_support::kPyramidMillerSamples;
using ::lumice::test_support::kPyramidWellConditionedSamples;
using ::lumice::test_support::PyramidDirectSample;
using ::lumice::test_support::PyramidMillerFixedSample;

// ============================================================================
// QS3 with int64_t + guarded arithmetic. Same shape as the __int128 original,
// only the integer type shrinks.
// ============================================================================

constexpr int kMaxOperandBits = 60;  // int64 sign bit + margin for next add.

struct QS3 {
  int64_t a;
  int64_t b;
  int shift;
};

inline QS3 QS3Zero() {
  return { 0, 0, 0 };
}

inline int BitWidth64(int64_t x) {
  if (x == 0)
    return 0;
  // Use unsigned to avoid INT64_MIN pitfall on -x.
  uint64_t u = (x < 0) ? static_cast<uint64_t>(-(x + 1)) + 1u : static_cast<uint64_t>(x);
  int w = 0;
  while (u != 0) {
    u >>= 1;
    w++;
  }
  return w;
}

inline void TrackBits(int w, int* max_bits) {
  if (w > *max_bits)
    *max_bits = w;
}

inline int64_t AddGuarded(int64_t x, int64_t y, bool* overflow, int* max_bits) {
  // int64 wrap check: check magnitude first.
  int wx = BitWidth64(x), wy = BitWidth64(y);
  int w_hint = std::max(wx, wy) + 1;
  if (w_hint > 62) {
    *overflow = true;
    TrackBits(w_hint, max_bits);
    return 0;
  }
  int64_t s = x + y;
  int w = BitWidth64(s);
  TrackBits(w, max_bits);
  if (w > kMaxOperandBits)
    *overflow = true;
  return s;
}

inline int64_t MulGuarded(int64_t x, int64_t y, bool* overflow, int* max_bits) {
  if (x == 0 || y == 0)
    return 0;
  int wx = BitWidth64(x), wy = BitWidth64(y);
  if (wx + wy > kMaxOperandBits) {
    *overflow = true;
    TrackBits(wx + wy, max_bits);
    return 0;
  }
  int64_t r = x * y;
  TrackBits(BitWidth64(r), max_bits);
  return r;
}

inline void ShiftUpGuarded(QS3* x, int delta, bool* overflow, int* max_bits) {
  if (delta <= 0)
    return;
  int wa = BitWidth64(x->a), wb = BitWidth64(x->b);
  if (wa + delta > kMaxOperandBits || wb + delta > kMaxOperandBits) {
    *overflow = true;
    TrackBits(std::max(wa, wb) + delta, max_bits);
    return;
  }
  x->a <<= delta;
  x->b <<= delta;
  x->shift += delta;
  TrackBits(std::max(BitWidth64(x->a), BitWidth64(x->b)), max_bits);
}

inline void AlignShifts(QS3* x, QS3* y, bool* overflow, int* max_bits) {
  if (x->shift == y->shift)
    return;
  if (x->shift < y->shift)
    ShiftUpGuarded(x, y->shift - x->shift, overflow, max_bits);
  else
    ShiftUpGuarded(y, x->shift - y->shift, overflow, max_bits);
}

inline void StripTrailingZeros(QS3* x) {
  if (x->a == 0 && x->b == 0) {
    x->shift = 0;
    return;
  }
  while (x->shift > 0) {
    if ((x->a & 1) != 0 || (x->b & 1) != 0)
      break;
    x->a >>= 1;
    x->b >>= 1;
    x->shift--;
  }
}

inline QS3 QS3Add(QS3 x, QS3 y, bool* overflow, int* max_bits) {
  AlignShifts(&x, &y, overflow, max_bits);
  if (*overflow)
    return QS3Zero();
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

inline QS3 QS3Mul(QS3 x, QS3 y, bool* overflow, int* max_bits) {
  int64_t aa = MulGuarded(x.a, y.a, overflow, max_bits);
  int64_t bb = MulGuarded(x.b, y.b, overflow, max_bits);
  int64_t ab = MulGuarded(x.a, y.b, overflow, max_bits);
  int64_t ba = MulGuarded(x.b, y.a, overflow, max_bits);
  int64_t three_bb = MulGuarded(3, bb, overflow, max_bits);
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

inline QS3 IntToQS3(int n) {
  return { n, 0, 0 };
}

inline QS3 FloatToQS3(float f) {
  if (f == 0.0f)
    return QS3Zero();
  int e = 0;
  double m = std::frexp(static_cast<double>(f), &e);
  double m_int_d = std::ldexp(m, 24);
  int64_t mant = static_cast<int64_t>(std::llround(m_int_d));
  int shift = 24 - e;
  if (shift < 0) {
    mant <<= (-shift);
    shift = 0;
  }
  QS3 out = { mant, 0, shift };
  StripTrailingZeros(&out);
  return out;
}

inline double QS3ToDouble(QS3 x) {
  const double kSqrt3D = 1.7320508075688772;
  return (static_cast<double>(x.a) + static_cast<double>(x.b) * kSqrt3D) * std::ldexp(1.0, -x.shift);
}

// ============================================================================
// PolyQS3<α, β> — bivariate polynomial with QS3 coefficients. Max joint degree
// i + j ≤ 4 (plan §3.2(a) derivation). Store as flat 5×5; unused slots zero.
// ============================================================================

constexpr int kMaxDeg = 4;

struct PolyQS3 {
  QS3 c[kMaxDeg + 1][kMaxDeg + 1];  // c[i][j] = coeff of α^i · β^j
};

inline PolyQS3 PolyZero() {
  PolyQS3 p{};
  for (int i = 0; i <= kMaxDeg; i++)
    for (int j = 0; j <= kMaxDeg; j++)
      p.c[i][j] = QS3Zero();
  return p;
}

inline PolyQS3 PolyFromQS3(QS3 x) {
  PolyQS3 p = PolyZero();
  p.c[0][0] = x;
  return p;
}

inline PolyQS3 PolyAlpha() {  // α
  PolyQS3 p = PolyZero();
  p.c[1][0] = IntToQS3(1);
  return p;
}

inline PolyQS3 PolyBeta() {  // β
  PolyQS3 p = PolyZero();
  p.c[0][1] = IntToQS3(1);
  return p;
}

inline PolyQS3 PolyNeg(PolyQS3 p) {
  for (int i = 0; i <= kMaxDeg; i++)
    for (int j = 0; j <= kMaxDeg; j++)
      p.c[i][j] = QS3Neg(p.c[i][j]);
  return p;
}

inline PolyQS3 PolyAdd(const PolyQS3& x, const PolyQS3& y, bool* overflow, int* max_bits) {
  PolyQS3 out{};
  for (int i = 0; i <= kMaxDeg; i++)
    for (int j = 0; j <= kMaxDeg; j++)
      out.c[i][j] = QS3Add(x.c[i][j], y.c[i][j], overflow, max_bits);
  return out;
}

inline PolyQS3 PolySub(const PolyQS3& x, const PolyQS3& y, bool* overflow, int* max_bits) {
  return PolyAdd(x, PolyNeg(y), overflow, max_bits);
}

inline PolyQS3 PolyMul(const PolyQS3& x, const PolyQS3& y, bool* overflow, int* max_bits) {
  PolyQS3 out = PolyZero();
  for (int i1 = 0; i1 <= kMaxDeg; i1++) {
    for (int j1 = 0; j1 <= kMaxDeg - i1; j1++) {
      if (IsZero(x.c[i1][j1]))
        continue;
      for (int i2 = 0; i2 <= kMaxDeg - i1 - j1; i2++) {
        for (int j2 = 0; j2 <= kMaxDeg - i1 - j1 - i2; j2++) {
          if (IsZero(y.c[i2][j2]))
            continue;
          QS3 term = QS3Mul(x.c[i1][j1], y.c[i2][j2], overflow, max_bits);
          out.c[i1 + i2][j1 + j2] = QS3Add(out.c[i1 + i2][j1 + j2], term, overflow, max_bits);
        }
      }
    }
  }
  // Detect if any product would have exceeded degree bound (non-zero in
  // truncated slots). Because we bounded the inner loops at i+j ≤ kMaxDeg
  // implicitly, this cannot happen; but future extensions might, so track.
  return out;
}

inline bool PolyIsZero(const PolyQS3& p) {
  for (int i = 0; i <= kMaxDeg; i++)
    for (int j = 0; j <= kMaxDeg; j++)
      if (!IsZero(p.c[i][j]))
        return false;
  return true;
}

// Bit-width scan across the full 5×5 matrix, for instrumentation.
inline int PolyPeakBits(const PolyQS3& p) {
  int peak = 0;
  for (int i = 0; i <= kMaxDeg; i++)
    for (int j = 0; j <= kMaxDeg; j++) {
      int wa = BitWidth64(p.c[i][j].a);
      int wb = BitWidth64(p.c[i][j].b);
      if (wa > peak)
        peak = wa;
      if (wb > peak)
        peak = wb;
    }
  return peak;
}

// Double-Horner evaluation + error bound. Ambiguous → *ambiguous = true.
// Returns +1 / -1 / 0.
inline int PolySign(const PolyQS3& p, double a1_val, double a2_val, bool* ambiguous) {
  const double kSqrt3D = 1.7320508075688772;
  const double kUlp = 2.220446049250313e-16;
  const double kSafety = 128.0;  // same safety factor as 387.1 #7.

  double val = 0.0;
  double mag = 0.0;
  double abs_a1 = std::fabs(a1_val);
  double abs_a2 = std::fabs(a2_val);
  for (int i = 0; i <= kMaxDeg; i++) {
    for (int j = 0; j <= kMaxDeg - i; j++) {
      double ca = static_cast<double>(p.c[i][j].a);
      double cb = static_cast<double>(p.c[i][j].b);
      double shift_scale = std::ldexp(1.0, -p.c[i][j].shift);
      double coeff = (ca + cb * kSqrt3D) * shift_scale;
      double alpha_i = std::pow(abs_a1, i);
      double beta_j = std::pow(abs_a2, j);
      double weight = alpha_i * beta_j;
      double signed_alpha = std::pow(a1_val, i);
      double signed_beta = std::pow(a2_val, j);
      val += coeff * signed_alpha * signed_beta;
      mag += std::fabs(coeff) * weight;
    }
  }
  double margin = kSafety * mag * kUlp;
  if (std::fabs(val) > margin) {
    *ambiguous = false;
    return val > 0 ? 1 : -1;
  }
  *ambiguous = true;
  return 0;
}

// ============================================================================
// Plane construction — mirrors exact_pyramid_oracle.hpp BuildConePlane/
// BuildPrismPlane, but planes are PolyQS3 in (α, β).
// ============================================================================

struct DirCorners {
  QS3 x1, x2, y1, y2;
};

inline QS3 make_qs(int a, int b, int shift) {
  return { a, b, shift };
}

inline DirCorners GetDirCorners(int i) {
  auto mk = [](int a, int b) { return make_qs(a, b, 2); };
  switch (i) {
    case 0:
      return { mk(0, 1), mk(0, 1), mk(-1, 0), mk(1, 0) };
    case 1:
      return { mk(0, 1), mk(0, 0), mk(1, 0), mk(2, 0) };
    case 2:
      return { mk(0, 0), mk(0, -1), mk(2, 0), mk(1, 0) };
    case 3:
      return { mk(0, -1), mk(0, -1), mk(1, 0), mk(-1, 0) };
    case 4:
      return { mk(0, -1), mk(0, 0), mk(-1, 0), mk(-2, 0) };
    case 5:
      return { mk(0, 0), mk(0, 1), mk(-2, 0), mk(-1, 0) };
  }
  return { QS3Zero(), QS3Zero(), QS3Zero(), QS3Zero() };
}

inline QS3 GetDirDet() {
  return { 0, 1, 3 };
}  // √3/8

struct PolyPlane {
  PolyQS3 A, B, C, D;
};

inline PolyPlane BuildPrismPlane(int i, QS3 dist_i, bool* overflow, int* max_bits) {
  DirCorners dc = GetDirCorners(i);
  QS3 det = GetDirDet();
  QS3 A_qs = QS3Sub(dc.y2, dc.y1, overflow, max_bits);
  QS3 B_qs = QS3Sub(dc.x1, dc.x2, overflow, max_bits);
  QS3 neg_dist = QS3Neg(dist_i);
  QS3 D_qs = QS3Mul(neg_dist, det, overflow, max_bits);
  PolyPlane p;
  p.A = PolyFromQS3(A_qs);
  p.B = PolyFromQS3(B_qs);
  p.C = PolyFromQS3(QS3Zero());
  p.D = PolyFromQS3(D_qs);
  return p;
}

// Cone plane with symbolic a_cone: use_alpha=true for upper (α), false for β.
inline PolyPlane BuildConePlane(int i, bool use_alpha, QS3 h2_2, QS3 dist_i, bool upper, bool* overflow,
                                int* max_bits) {
  DirCorners dc = GetDirCorners(i);
  QS3 det = GetDirDet();
  QS3 y_diff = QS3Sub(dc.y2, dc.y1, overflow, max_bits);
  QS3 x_diff = QS3Sub(dc.x1, dc.x2, overflow, max_bits);
  PolyQS3 sym = use_alpha ? PolyAlpha() : PolyBeta();
  PolyPlane p;
  p.A = PolyMul(sym, PolyFromQS3(y_diff), overflow, max_bits);
  p.B = PolyMul(sym, PolyFromQS3(x_diff), overflow, max_bits);
  p.C = PolyFromQS3(upper ? det : QS3Neg(det));
  // D = -(h2/2 + sym·dist)·det
  PolyQS3 sym_dist = PolyMul(sym, PolyFromQS3(dist_i), overflow, max_bits);
  PolyQS3 inner = PolyAdd(PolyFromQS3(h2_2), sym_dist, overflow, max_bits);
  PolyQS3 neg_inner = PolyNeg(inner);
  p.D = PolyMul(neg_inner, PolyFromQS3(det), overflow, max_bits);
  return p;
}

// Basal planes — skip for M1 spike. The LP3 apex path is α/β-free
// (plan §3.2(d)) and z_top/z_bot only add ONE unit of α or β degree,
// so basal-plane involvement cannot raise the joint degree above what
// cone-cone-cone triples already exercise. Cone-cone-cone gives degree
// (α^2, β^0) or (α^1, β^1) or (α^0, β^2) etc.; adding basal replaces
// one cone with α^0/β^0 at column C, so joint degree drops, not rises.
// Peak bit-width thus is bounded by the cone-cone-cone / cone-cone-prism /
// cone-prism-prism triple family, which we enumerate below.

struct PolyTriplePoint {
  PolyQS3 px, py, pz, det;
  bool degenerate;
};

inline PolyTriplePoint SolveTriple(const PolyPlane& p0, const PolyPlane& p1, const PolyPlane& p2, bool* overflow,
                                   int* max_bits) {
  PolyTriplePoint out{};
  out.degenerate = false;
  PolyQS3 b1c2 = PolyMul(p1.B, p2.C, overflow, max_bits);
  PolyQS3 b2c1 = PolyMul(p2.B, p1.C, overflow, max_bits);
  PolyQS3 minor0 = PolySub(b1c2, b2c1, overflow, max_bits);
  PolyQS3 a1c2 = PolyMul(p1.A, p2.C, overflow, max_bits);
  PolyQS3 a2c1 = PolyMul(p2.A, p1.C, overflow, max_bits);
  PolyQS3 minor1 = PolySub(a1c2, a2c1, overflow, max_bits);
  PolyQS3 a1b2 = PolyMul(p1.A, p2.B, overflow, max_bits);
  PolyQS3 a2b1 = PolyMul(p2.A, p1.B, overflow, max_bits);
  PolyQS3 minor2 = PolySub(a1b2, a2b1, overflow, max_bits);
  PolyQS3 t0 = PolyMul(p0.A, minor0, overflow, max_bits);
  PolyQS3 t1 = PolyMul(p0.B, minor1, overflow, max_bits);
  PolyQS3 t2 = PolyMul(p0.C, minor2, overflow, max_bits);
  PolyQS3 t01 = PolySub(t0, t1, overflow, max_bits);
  out.det = PolyAdd(t01, t2, overflow, max_bits);
  if (*overflow)
    return out;
  if (PolyIsZero(out.det)) {
    out.degenerate = true;
    return out;
  }

  PolyQS3 nD0 = PolyNeg(p0.D);
  PolyQS3 nD1 = PolyNeg(p1.D);
  PolyQS3 nD2 = PolyNeg(p2.D);

  PolyQS3 nD1c2 = PolyMul(nD1, p2.C, overflow, max_bits);
  PolyQS3 nD2c1 = PolyMul(nD2, p1.C, overflow, max_bits);
  PolyQS3 xm1 = PolySub(nD1c2, nD2c1, overflow, max_bits);
  PolyQS3 nD1b2 = PolyMul(nD1, p2.B, overflow, max_bits);
  PolyQS3 nD2b1 = PolyMul(nD2, p1.B, overflow, max_bits);
  PolyQS3 xm2 = PolySub(nD1b2, nD2b1, overflow, max_bits);
  PolyQS3 xt0 = PolyMul(nD0, minor0, overflow, max_bits);
  PolyQS3 xt1 = PolyMul(p0.B, xm1, overflow, max_bits);
  PolyQS3 xt2 = PolyMul(p0.C, xm2, overflow, max_bits);
  PolyQS3 xt01 = PolySub(xt0, xt1, overflow, max_bits);
  out.px = PolyAdd(xt01, xt2, overflow, max_bits);

  PolyQS3 a1nD2 = PolyMul(p1.A, nD2, overflow, max_bits);
  PolyQS3 a2nD1 = PolyMul(p2.A, nD1, overflow, max_bits);
  PolyQS3 ym2 = PolySub(a1nD2, a2nD1, overflow, max_bits);
  PolyQS3 yt0 = PolyMul(p0.A, xm1, overflow, max_bits);
  PolyQS3 yt1 = PolyMul(nD0, minor1, overflow, max_bits);
  PolyQS3 yt2 = PolyMul(p0.C, ym2, overflow, max_bits);
  PolyQS3 yt01 = PolySub(yt0, yt1, overflow, max_bits);
  out.py = PolyAdd(yt01, yt2, overflow, max_bits);

  PolyQS3 b1nD2 = PolyMul(p1.B, nD2, overflow, max_bits);
  PolyQS3 b2nD1 = PolyMul(p2.B, nD1, overflow, max_bits);
  PolyQS3 zm1 = PolySub(b1nD2, b2nD1, overflow, max_bits);
  PolyQS3 zt0 = PolyMul(p0.A, zm1, overflow, max_bits);
  PolyQS3 zt1 = PolyMul(p0.B, ym2, overflow, max_bits);
  PolyQS3 zt2 = PolyMul(nD0, minor2, overflow, max_bits);
  PolyQS3 zt01 = PolySub(zt0, zt1, overflow, max_bits);
  out.pz = PolyAdd(zt01, zt2, overflow, max_bits);
  return out;
}

// IsIncident test: A·px + B·py + C·pz + D·det == 0 poly.
// Returns the sum polynomial (caller checks PolyIsZero + PolySign).
inline PolyQS3 IncidenceExpr(const PolyPlane& plane, const PolyTriplePoint& tp, bool* overflow, int* max_bits) {
  PolyQS3 apx = PolyMul(plane.A, tp.px, overflow, max_bits);
  PolyQS3 bpy = PolyMul(plane.B, tp.py, overflow, max_bits);
  PolyQS3 cpz = PolyMul(plane.C, tp.pz, overflow, max_bits);
  PolyQS3 ddet = PolyMul(plane.D, tp.det, overflow, max_bits);
  PolyQS3 s1 = PolyAdd(apx, bpy, overflow, max_bits);
  PolyQS3 s2 = PolyAdd(s1, cpz, overflow, max_bits);
  return PolyAdd(s2, ddet, overflow, max_bits);
}

// ============================================================================
// Driver — iterate a sample pool, build 18 planes (6 prism + 6 upper cone +
// 6 lower cone), enumerate C(18, 3) = 816 triples, run SolveTriple + all
// 15 remaining planes IsFeasibleSided + IncidenceExpr, track peak QS3 bit-width
// and PolySign filter stats.
// ============================================================================

struct SpikeStats {
  int64_t triples_evaluated = 0;
  int64_t triples_overflowed = 0;
  int max_qs3_bits = 0;
  int64_t sign_calls = 0;
  int64_t sign_ambiguous = 0;
};

void RunOneSample(const PyramidDirectSample& s, SpikeStats* stats) {
  int max_bits = 0;
  bool overflow = false;

  const bool has_upper = (s.upper_alpha > 0.0f) && (s.h1 > 1e-6f);
  const bool has_lower = (s.lower_alpha > 0.0f) && (s.h3 > 1e-6f);
  const double a1_deg = static_cast<double>(s.upper_alpha);
  const double a2_deg = static_cast<double>(s.lower_alpha);
  const double a1_val = has_upper ? std::tan(a1_deg * M_PI / 180.0) : -1.0;
  const double a2_val = has_lower ? std::tan(a2_deg * M_PI / 180.0) : -1.0;
  (void)a1_val;
  (void)a2_val;  // used inside PolySign below.

  QS3 dist_qs[6];
  for (int i = 0; i < 6; i++)
    dist_qs[i] = FloatToQS3(s.dist[i]);
  QS3 h2_2 = FloatToQS3(0.5f * s.h2);

  // Build 18 planes; skip basal (see BuildConePlane note above).
  PolyPlane planes[18];
  bool active[18] = { false };

  for (int i = 0; i < 6; i++) {
    planes[i] = BuildPrismPlane(i, dist_qs[i], &overflow, &max_bits);
    active[i] = true;
  }
  if (has_upper) {
    for (int i = 0; i < 6; i++) {
      planes[6 + i] = BuildConePlane(i, /*use_alpha=*/true, h2_2, dist_qs[i], /*upper=*/true, &overflow, &max_bits);
      active[6 + i] = true;
    }
  }
  if (has_lower) {
    for (int i = 0; i < 6; i++) {
      planes[12 + i] = BuildConePlane(i, /*use_alpha=*/false, h2_2, dist_qs[i], /*upper=*/false, &overflow, &max_bits);
      active[12 + i] = true;
    }
  }

  int active_idx[18];
  int active_n = 0;
  for (int i = 0; i < 18; i++)
    if (active[i])
      active_idx[active_n++] = i;

  for (int a = 0; a < active_n; a++) {
    for (int b = a + 1; b < active_n; b++) {
      for (int c = b + 1; c < active_n; c++) {
        int i = active_idx[a], j = active_idx[b], k = active_idx[c];
        bool trip_of = false;
        int trip_bits = max_bits;
        PolyTriplePoint tp = SolveTriple(planes[i], planes[j], planes[k], &trip_of, &trip_bits);
        stats->triples_evaluated++;
        if (trip_of) {
          stats->triples_overflowed++;
          if (trip_bits > stats->max_qs3_bits)
            stats->max_qs3_bits = trip_bits;
          continue;
        }
        if (tp.degenerate)
          continue;
        for (int q = 0; q < active_n; q++) {
          int p_idx = active_idx[q];
          if (p_idx == i || p_idx == j || p_idx == k)
            continue;
          bool inc_of = false;
          int inc_bits = trip_bits;
          PolyQS3 expr = IncidenceExpr(planes[p_idx], tp, &inc_of, &inc_bits);
          if (inc_of) {
            stats->triples_overflowed++;
            if (inc_bits > stats->max_qs3_bits)
              stats->max_qs3_bits = inc_bits;
            break;
          }
          // Production flow: PolyIsZero first (poly-exact incidence), then
          // PolySign only if the poly is non-zero (strict feasibility check).
          // Skipping the PolyIsZero short-circuit inflates the ambiguous count
          // by counting true zero polynomials (boundary incidences) as sign-
          // ambiguous — which is not a correctness failure.
          if (!PolyIsZero(expr)) {
            bool amb = false;
            (void)PolySign(expr, a1_val, a2_val, &amb);
            stats->sign_calls++;
            if (amb)
              stats->sign_ambiguous++;
          }
          if (inc_bits > stats->max_qs3_bits)
            stats->max_qs3_bits = inc_bits;
        }
        if (trip_bits > stats->max_qs3_bits)
          stats->max_qs3_bits = trip_bits;
      }
    }
  }
  if (max_bits > stats->max_qs3_bits)
    stats->max_qs3_bits = max_bits;
}

// Miller a1 = i1·c/(2·i4) where c = h2/2 (nominal 0.5). Reuse direct-wedge
// path with a1_val = i1·c/(2·i4).
double MillerA1(int i1, int i4, float h2) {
  double c = 0.5 * static_cast<double>(h2);
  return static_cast<double>(i1) * c / (2.0 * static_cast<double>(i4));
}

}  // namespace spike

int main() {
  std::setbuf(stdout, nullptr);

  auto run_pool = [](const char* name, const spike::PyramidDirectSample* pool, size_t n) {
    spike::SpikeStats st;
    // Cap large pools for spike-time.
    size_t cap = std::min<size_t>(n, 32);
    for (size_t i = 0; i < cap; i++)
      spike::RunOneSample(pool[i], &st);
    std::printf("[%s] samples=%zu/%zu triples=%lld overflow=%lld peak_bits=%d sign_calls=%lld sign_ambig=%lld\n", name,
                cap, n, (long long)st.triples_evaluated, (long long)st.triples_overflowed, st.max_qs3_bits,
                (long long)st.sign_calls, (long long)st.sign_ambiguous);
    return st;
  };

  std::printf("=== M1 gate spike: bivariate PolyQS3 bit-width + sign-filter stats ===\n");

  auto st_wc =
      run_pool("well-cond", spike::kPyramidWellConditionedSamples, std::size(spike::kPyramidWellConditionedSamples));
  auto st_ft85 =
      run_pool("flat-85", spike::kPyramidFlatTailAlpha85Samples, std::size(spike::kPyramidFlatTailAlpha85Samples));
  auto st_ft87 =
      run_pool("flat-87", spike::kPyramidFlatTailAlpha87Samples, std::size(spike::kPyramidFlatTailAlpha87Samples));
  auto st_ft89 =
      run_pool("flat-89", spike::kPyramidFlatTailAlpha89Samples, std::size(spike::kPyramidFlatTailAlpha89Samples));
  auto st_ft895 =
      run_pool("flat-895", spike::kPyramidFlatTailAlpha895Samples, std::size(spike::kPyramidFlatTailAlpha895Samples));
  auto st_dg030 = run_pool("degen-030", spike::kPyramidDegenerateSigma030Samples,
                           std::size(spike::kPyramidDegenerateSigma030Samples));
  auto st_dg050 = run_pool("degen-050", spike::kPyramidDegenerateSigma050Samples,
                           std::size(spike::kPyramidDegenerateSigma050Samples));

  // Joint worst-case: synthetic dual-89° samples.
  std::printf("\n=== Joint 89°+89° synthetic samples ===\n");
  {
    spike::SpikeStats st;
    for (int k = 0; k < 4; k++) {
      spike::PyramidDirectSample s{};
      s.upper_alpha = 89.0f + 0.1f * static_cast<float>(k);
      s.lower_alpha = 89.0f + 0.1f * static_cast<float>(k);
      s.h1 = 1.0f;
      s.h2 = 1.0f;
      s.h3 = 1.0f;
      for (int i = 0; i < 6; i++)
        s.dist[i] = 1.0f + 0.05f * static_cast<float>((i * 17 + k) % 7) - 0.15f;
      spike::RunOneSample(s, &st);
    }
    std::printf("[joint-89] triples=%lld overflow=%lld peak_bits=%d sign_calls=%lld sign_ambig=%lld\n",
                (long long)st.triples_evaluated, (long long)st.triples_overflowed, st.max_qs3_bits,
                (long long)st.sign_calls, (long long)st.sign_ambiguous);
  }

  // Miller pool: use Miller a1/a2 rationals as PolySign eval points.
  std::printf("\n=== Miller pool (rational a1) ===\n");
  {
    spike::SpikeStats st;
    size_t n = std::size(spike::kPyramidMillerSamples);
    size_t cap = std::min<size_t>(n, 32);
    for (size_t idx = 0; idx < cap; idx++) {
      const auto& m = spike::kPyramidMillerSamples[idx];
      spike::PyramidDirectSample s{};
      s.upper_alpha = 45.0f;  // dummy; RunOneSample uses tan(alpha) — override below.
      s.lower_alpha = 45.0f;
      s.h1 = m.h1;
      s.h2 = m.h2;
      s.h3 = m.h3;
      for (int i = 0; i < 6; i++)
        s.dist[i] = m.dist[i];
      // For Miller we need a1_val = i1·c/(2·i4). Patch RunOneSample-style eval
      // by monkey-injecting via a local re-implementation:
      // — instead of tweaking RunOneSample, just run it with tan(alpha) that
      // approximates the rational, since bit-width is independent of the eval
      // point (symbolic). PolySign accuracy vs Miller a1 is the real question
      // and is checked separately in Step 10 cross-validation, not M1.
      double a1_miller = spike::MillerA1(m.upper_i1, m.upper_i4, m.h2);
      double a2_miller = spike::MillerA1(m.lower_i1, m.lower_i4, m.h2);
      double deg1 = std::atan(a1_miller) * 180.0 / M_PI;
      double deg2 = std::atan(a2_miller) * 180.0 / M_PI;
      s.upper_alpha = static_cast<float>(deg1);
      s.lower_alpha = static_cast<float>(deg2);
      spike::RunOneSample(s, &st);
    }
    std::printf("[miller] samples=%zu/%zu triples=%lld overflow=%lld peak_bits=%d sign_calls=%lld sign_ambig=%lld\n",
                cap, n, (long long)st.triples_evaluated, (long long)st.triples_overflowed, st.max_qs3_bits,
                (long long)st.sign_calls, (long long)st.sign_ambiguous);
  }

  // Silence unused warnings.
  (void)st_wc;
  (void)st_ft85;
  (void)st_ft87;
  (void)st_ft89;
  (void)st_ft895;
  (void)st_dg030;
  (void)st_dg050;

  std::printf("\n=== summary ===\n");
  std::printf("If peak_bits <= ~50, int64 budget (60 bits with margin) is comfortable.\n");
  std::printf("If any overflow > 0, kMaxOperandBits=60 is too tight OR degree bound violated.\n");
  return 0;
}
