#ifndef UTIL_ILLUMINANT_H_
#define UTIL_ILLUMINANT_H_

#include "util/illuminant_data.hpp"

namespace lumice {

// Returns the relative SPD value for the given CIE standard illuminant at the
// specified wavelength (in nm). Returns 0.0 for out-of-range wavelengths.
//
// D-series: reconstructed from S0/S1/S2 basis vectors (300–830 nm range)
// A: Planck formula at 2856 K (380–830 nm range, practical)
// E: constant 1.0 for any wavelength in [300, 830]
float GetIlluminantSpd(IlluminantType type, float wavelength);

// Returns E[GetIlluminantSpd(type, wl)] for wl ~ Uniform[380, 780] nm — the
// expected SPD weight a batch would draw if its wavelength were sampled
// uniformly over the simulated band.
//
// CONTRACT: this is a normalization-audit quantity only. It must never drive
// the physical weight of a traced ray — that is `GetIlluminantSpd` at the
// wavelength actually sampled for the batch. Using this in a shading path
// would replace a per-batch spectral weight with a band average and flatten
// the spectrum. It exists so the emitted-energy denominator stays
// deterministic: accumulating the sampled weights instead would make the same
// config at a different seed differ in brightness by the sampling noise of the
// draw (~0.2% at typical batch counts), which is exactly the content-dependence
// the absolute scale is meant to remove.
//
// Deterministic (no RNG): a fixed-grid uniform average over the band. Memoized
// per IlluminantType; the first call for a type does the quadrature.
float MeanIlluminantWeight(IlluminantType type);

}  // namespace lumice

#endif  // UTIL_ILLUMINANT_H_
