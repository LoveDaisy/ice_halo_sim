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

}  // namespace lumice

#endif  // UTIL_ILLUMINANT_H_
