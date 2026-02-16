#include "server/show_rays.hpp"

#include <cstddef>

#include "util/logger.hpp"

namespace lumice {

void ShowRayInfoConsumer::Consume(const SimData& data) {
  LOG_INFO("(id) p  d  w fid prev_id");
  for (size_t i = 0; i < data.rays_.size_; i++) {
    const auto& r = data.rays_[i];
    LOG_INFO("({:02}) {:+9.6f},{:+9.6f},{:+9.6f}  {:+9.6f},{:+9.6f},{:+9.6f}  {:.6f}  {}  {}",  //
             i,                                                                                 // id
             r.p_[0], r.p_[1], r.p_[2],                                                         // p
             r.d_[0], r.d_[1], r.d_[2],                                                         // d
             r.w_,                                                                              // w
             r.fid_,                                                                            // fid
             r.prev_ray_idx_);                                                                  // prev_ray_id
  }
}

}  // namespace lumice
