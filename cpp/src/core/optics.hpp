#ifndef SRC_CORE_OPTICS_H_
#define SRC_CORE_OPTICS_H_

#include <cstddef>

#include "core/buffer.hpp"
#include "core/crystal.hpp"


namespace icehalo {

class IceRefractiveIndex {
 public:
  static constexpr float kMinWaveLength = 350;
  static constexpr float kMaxWaveLength = 900;

  static double Get(double wave_length);

 private:
  /* Shellmeier's equation:
   *
   *   n^2 = 1 + B1 * lambda^2 / (lambda^2 - C1^2)
   *           + B2 * lambda^2 / (lambda^2 - C2^2)
   *   lambda in micrometer, C1 * 1e-2, C2 * 1e2
   *
   * Data from https://refractiveindex.info/?shelf=3d&book=crystals&page=ice
   * Fitted formula by myself.
   */
  static constexpr float kCoefAvr[] = { 0.701777f, 1.091144f, 0.884400f, 0.796950f };  // B1, B2, C1, C2
  static constexpr float kCoefO[] = { 0.696364f, 0.719271f, 0.957220f, 1.096889f };
  static constexpr float kCoefE[] = { 0.699934f, 0.640071f, 0.960906f, 0.964654f };
};

namespace v3 {

void HitSurface(const Crystal& crystal, float n, size_t num,                          // input
                const float_bf_t d_in, const float_bf_t w_in, const int_bf_t fid_in,  // input
                float_bf_t d_out, float_bf_t w_out);                                  // output

void Propagate(const Crystal& crystal, size_t num, size_t step,                      // input
               const float_bf_t d_in, const float_bf_t p_in, const float_bf_t w_in,  // input
               float_bf_t p_out, int_bf_t fid_out);                                  // output

}  // namespace v3
}  // namespace icehalo


#endif  // SRC_CORE_OPTICS_H_
