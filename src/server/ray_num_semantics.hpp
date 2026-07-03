#ifndef LUMICE_SERVER_RAY_NUM_SEMANTICS_HPP
#define LUMICE_SERVER_RAY_NUM_SEMANTICS_HPP

#include <cstddef>

namespace lumice {

// Per-wavelength ray budget from the total-across-wavelengths ray_num (task-323).
// n_wl <= 1 (illuminant / single wavelength) is the identity. For discrete spectra the
// per-wavelength count is ceil(total / n_wl) so at least `total` rays are traced overall.
// Caller must handle the infinite sentinel separately (never divides).
inline std::size_t PerWavelengthRayNum(std::size_t total, std::size_t n_wl) {
  if (n_wl <= 1)
    return total;
  return (total + n_wl - 1) / n_wl;  // ceil
}

}  // namespace lumice

#endif  // LUMICE_SERVER_RAY_NUM_SEMANTICS_HPP
