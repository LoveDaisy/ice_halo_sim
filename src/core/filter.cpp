#include "core/filter.hpp"

#include <cmath>
#include <set>
#include <variant>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "util/logger.hpp"

namespace lumice {

class NoneFilter : public Filter {
 protected:
  bool InternalCheck(const RaySeg& /**/) const override { return true; }
};


class RaypathFilter : public Filter {
 public:
  explicit RaypathFilter(const std::vector<IdType>& rp) : rp_(rp) {}

  void InitCrystalSymmetry(const Crystal& crystal, uint8_t symmetry) override {
    candidate_hash_.clear();
    auto expand_rp = crystal.ExpandRaypath(rp_, symmetry);
    RaypathHash h;
    for (const auto& rp : expand_rp) {
      candidate_hash_.emplace(h(rp));
    }
  }

 protected:
  bool InternalCheck(const RaySeg& ray) const override {
    RaypathHash h;
    auto curr_hash = h(ray.rp_);
    return candidate_hash_.count(curr_hash) > 0;
  }

 private:
  std::set<size_t> candidate_hash_;
  std::vector<IdType> rp_;
};


class EntryExitFilter : public Filter {
 public:
  EntryExitFilter(IdType entry, IdType exit) : entry_(entry), exit_(exit) {}

  void InitCrystalSymmetry(const Crystal& crystal, uint8_t symmetry) override {
    std::vector<IdType> ee_rp{ entry_, exit_ };
    auto expand_ee_rp = crystal.ExpandRaypath(ee_rp, symmetry);
    RaypathHash h;
    for (const auto& rp : expand_ee_rp) {
      candidate_hash_.emplace(h(rp));
    }
  }

 protected:
  bool InternalCheck(const RaySeg& ray) const override {
    if (ray.rp_.size_ == 0) {
      return false;
    }

    RaypathHash h;
    RaypathRecorder rp;
    rp << ray.rp_[0] << ray.rp_[ray.rp_.size_ - 1];
    return candidate_hash_.count(h(rp)) > 0;
  }

 private:
  std::set<size_t> candidate_hash_;
  IdType entry_;
  IdType exit_;
};


class DirectionFilter : public Filter {
 public:
  DirectionFilter(float lon, float lat, float radii)
      : d_{ std::cos(lat * math::kDegreeToRad) * std::cos(lon * math::kDegreeToRad),
            std::cos(lat * math::kDegreeToRad) * std::sin(lon * math::kDegreeToRad),
            std::sin(lat * math::kDegreeToRad) },
        radii_c_(std::cos(radii * math::kDegreeToRad)) {}

 protected:
  bool InternalCheck(const RaySeg& ray) const override {
    if (ray.state_ != RaySeg::kOutgoing) {
      return false;
    }

    auto c = Dot3(d_, ray.d_);
    return c > radii_c_;
  }

 private:
  float d_[3];
  float radii_c_;  // cos(radii)
};


class CrystalFilter : public Filter {
 public:
  explicit CrystalFilter(IdType crystal_id) : crystal_id_(crystal_id){};

 protected:
  bool InternalCheck(const RaySeg& ray) const override { return ray.crystal_config_id_ == crystal_id_; }

 private:
  IdType crystal_id_;
};


// =============== Filter ===============
struct SimpleFilterCreator {
  FilterPtrU operator()(const NoneFilterParam& /* p */) {
    // NoneFilter
    return std::make_unique<NoneFilter>();
  }

  FilterPtrU operator()(const RaypathFilterParam& p) {
    // RaypathFilter
    return std::make_unique<RaypathFilter>(p.raypath_);
  }

  FilterPtrU operator()(const EntryExitFilterParam& p) {
    // EntryExitFilter
    return std::make_unique<EntryExitFilter>(p.entry_, p.exit_);
  }

  FilterPtrU operator()(const DirectionFilterParam& p) {
    // DirectionFilter
    return std::make_unique<DirectionFilter>(p.lon_, p.lat_, p.radii_);
  }

  FilterPtrU operator()(const CrystalFilterParam& p) {
    // CrystalFilter
    return std::make_unique<CrystalFilter>(p.crystal_id_);
  }
};


class ComplexFilter : public Filter {
 public:
  explicit ComplexFilter(const std::vector<std::vector<std::pair<IdType, SimpleFilterParam>>>& all_param) {
    for (const auto& p : all_param) {
      std::vector<FilterPtrU> f;
      for (const auto& pp : p) {
        f.emplace_back(std::visit(SimpleFilterCreator{}, pp.second));
      }
      filters_.emplace_back(std::move(f));
    }
  }

  void InitCrystalSymmetry(const Crystal& crystal, uint8_t symmetry) override {
    for (auto& or_f : filters_) {
      for (auto& and_f : or_f) {
        and_f->InitCrystalSymmetry(crystal, symmetry);
      }
    }
  }

 protected:
  bool InternalCheck(const RaySeg& ray) const override {
    for (const auto& or_f : filters_) {
      bool and_check = true;
      for (const auto& and_f : or_f) {
        if (!and_f->Check(ray)) {
          and_check = false;
          break;
        }
      }
      if (and_check) {
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<std::vector<FilterPtrU>> filters_;
};


struct FilterCreator {
  FilterPtrU operator()(const SimpleFilterParam& p) {
    // SimpleFilters
    return std::visit(SimpleFilterCreator{}, p);
  }

  FilterPtrU operator()(const ComplexFilterParam& p) {
    // ComplexFilters
    return std::make_unique<ComplexFilter>(p.filters_);
  }
};


// =============== Filter ===============
FilterPtrU Filter::Create(const FilterConfig& config) {
  auto filter = std::visit(FilterCreator{}, config.param_);
  filter->action_ = config.action_;
  return filter;
}

bool Filter::Check(const RaySeg& ray) const {
  bool checked = InternalCheck(ray);
  if (action_ == FilterConfig::kFilterIn) {
    return checked;
  } else if (action_ == FilterConfig::kFilterOut) {
    return !checked;
  } else {
    LOG_ERROR("Filter action not recognize!");
    return false;
  }
}

}  // namespace lumice
