#include "server/show_rays.hpp"

#include "include/log.hpp"

namespace icehalo {
namespace v3 {

void ShowRayInfoConsumer::Consume(const SimData& data) {
  LOG_INFO("p  d  w fid prev_id");
  for (const auto& r : data.rays_) {
    LOG_INFO("%.6f,%.6f,%.6f  %.6f,%.6f,%.6f  %.6f  %d  %d",  //
             r.p_[0], r.p_[1], r.p_[2],                       // p
             r.d_[0], r.d_[1], r.d_[2],                       // d
             r.w_,                                            // w
             r.fid_,                                          // fid
             r.prev_ray_idx_);                                // prev_ray_id
  }
}

}  // namespace v3
}  // namespace icehalo
