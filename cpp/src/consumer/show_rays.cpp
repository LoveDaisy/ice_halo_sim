#include "consumer/show_rays.hpp"

#include "util/log.hpp"

namespace icehalo {
namespace v3 {

void ShowRaysInfo::Consume(const SimData& data) {
  LOG_DEBUG("p  d  w fid prev_id");
  for (const auto& r : data.rays_) {
    LOG_DEBUG("%.6f,%.6f,%.6f  %.6f,%.6f,%.6f  %.6f  %d  %d",  //
              r.p_[0], r.p_[1], r.p_[2],                       // p
              r.d_[0], r.d_[1], r.d_[2],                       // d
              r.w_,                                            // w
              r.fid_,                                          // fid
              r.prev_ray_idx_);                                // prev_ray_id
  }
}

}  // namespace v3
}  // namespace icehalo
