#include "core/filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <set>
#include <variant>
#include <vector>

#include "config/filter_config.hpp"
#include "context/crystal_context.hpp"
#include "context/filter_context.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "include/log.hpp"

namespace icehalo {

AbstractRayPathFilter::AbstractRayPathFilter()
    : symmetry_flag_(kSymmetryNone), complementary_(false), remove_homodromous_(false) {}


AbstractRayPathFilter::AbstractRayPathFilter(const AbstractRayPathFilter& other)
    : symmetry_flag_(other.symmetry_flag_), complementary_(other.complementary_),
      remove_homodromous_(other.remove_homodromous_) {}


AbstractRayPathFilter& AbstractRayPathFilter::operator=(const AbstractRayPathFilter& other) {
  if (&other != this) {
    symmetry_flag_ = other.symmetry_flag_;
    complementary_ = other.complementary_;
    remove_homodromous_ = other.remove_homodromous_;
  }
  return *this;
}


bool AbstractRayPathFilter::Filter(const Crystal* crystal, RaySegment* last_r) const {
  if (remove_homodromous_ &&
      Dot3(last_r->dir.val(), last_r->root_ctx->first_ray_segment->dir.val()) > 1.0 - 5 * math::kFloatEps) {
    return false;
  }

  bool result = FilterPath(crystal, last_r);
  return result ^ complementary_;
}


void AbstractRayPathFilter::SetSymmetryFlag(uint8_t symmetry_flag) {
  symmetry_flag_ = symmetry_flag;
}


void AbstractRayPathFilter::AddSymmetry(Symmetry symmetry) {
  symmetry_flag_ |= symmetry;
}


uint8_t AbstractRayPathFilter::GetSymmetryFlag() const {
  return symmetry_flag_;
}


void AbstractRayPathFilter::ApplySymmetry(const CrystalContext* /* crystal */) {}


void AbstractRayPathFilter::EnableComplementary(bool enable) {
  complementary_ = enable;
}


bool AbstractRayPathFilter::GetComplementary() const {
  return complementary_;
}


void AbstractRayPathFilter::EnableRemoveHomodromous(bool enable) {
  remove_homodromous_ = enable;
}


bool AbstractRayPathFilter::GetRemoveHomodromous() const {
  return remove_homodromous_;
}


void to_json(nlohmann::json& obj, const AbstractRayPathFilter& filter) {
  obj["complementary"] = filter.complementary_;
  obj["remove_homodromous"] = filter.remove_homodromous_;
  if (filter.symmetry_flag_ != kSymmetryNone) {
    char sym_buf[16]{};
    size_t buf_idx = 0;
    if (filter.symmetry_flag_ & kSymmetryBasal) {
      sym_buf[buf_idx++] = 'B';
    }
    if (filter.symmetry_flag_ & kSymmetryDirection) {
      sym_buf[buf_idx++] = 'D';
    }
    if (filter.symmetry_flag_ & kSymmetryPrism) {
      sym_buf[buf_idx] = 'P';
    }
    obj["symmetry"] = sym_buf;
  }

  filter.SaveToJson(obj);
}


void from_json(const nlohmann::json& obj, AbstractRayPathFilter& filter) {
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "complementary", filter.complementary_)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "remove_homodromous", filter.remove_homodromous_)

  filter.symmetry_flag_ = kSymmetryNone;
  try {
    const auto sym = obj.at("symmetry").get<std::string>();
    for (const auto c : sym) {
      switch (c) {
        case 'P':
        case 'p':
          filter.AddSymmetry(kSymmetryPrism);
          break;
        case 'B':
        case 'b':
          filter.AddSymmetry(kSymmetryBasal);
          break;
        case 'D':
        case 'd':
          filter.AddSymmetry(kSymmetryDirection);
          break;
        default:
          throw std::invalid_argument("<ray_path_filter[%d].symmetry> cannot recognize!");
      }
    }
  } catch (const std::exception& e) {
    LOG_VERBOSE(e.what());
  }

  filter.LoadFromJson(obj);
}


size_t RayPathReverseHash(const Crystal* crystal,                    // used for get face number
                          const RaySegment* last_ray, int length) {  // ray path and length
  constexpr size_t kStep = 7;
  constexpr size_t kTotalBits = sizeof(size_t) * CHAR_BIT;

  if (length < 0 && length != kAutoDetectLength) {
    return 0;
  }

  if (length == kAutoDetectLength) {
    length = 0;
    const auto* p = last_ray;
    while (p->prev) {
      p = p->prev;
      length++;
    }
  }

  size_t result = 0;
  size_t curr_offset = kStep * (length - 1) % kTotalBits;
  const auto* p = last_ray;
  while (p->prev) {
    size_t fn = crystal->FaceNumber(p->face_id);
    size_t tmp_hash = (fn << curr_offset) | (fn >> (kTotalBits - curr_offset));
    result ^= tmp_hash;
    curr_offset -= kStep;
    curr_offset %= kTotalBits;
    p = p->prev;
  }

  return result;
}


bool NoneRayPathFilter::FilterPath(const Crystal* /* crystal */, RaySegment* /* r */) const {
  return true;
}


RayPathFilterPtrU NoneRayPathFilter::MakeCopy() const {
  return std::unique_ptr<AbstractRayPathFilter>{ new NoneRayPathFilter(*this) };
}


void NoneRayPathFilter::SaveToJson(nlohmann::json& obj) const {
  obj["type"] = "none";
}


void NoneRayPathFilter::LoadFromJson(const nlohmann::json& /* obj */) {}


void SpecificRayPathFilter::AddPath(const RayPath& path) {
  ray_paths_.emplace_back(path.MakePermanentCopy());
}


void SpecificRayPathFilter::ClearPaths() {
  ray_paths_.clear();
}


std::vector<RayPath> SpecificRayPathFilter::GetRayPaths() const {
  return ray_paths_;
}


RayPathFilterPtrU SpecificRayPathFilter::MakeCopy() const {
  return std::unique_ptr<AbstractRayPathFilter>{ new SpecificRayPathFilter(*this) };
}


void SpecificRayPathFilter::ApplySymmetry(const CrystalContext* crystal_ctx) {
  std::vector<RayPath> augmented_ray_paths{};

  // Add the original path.
  for (auto rp : ray_paths_) {
    rp.PrependId(crystal_ctx->GetId());
    rp << kInvalidId;
    for (auto&& p : MakeSymmetryExtension(rp, crystal_ctx, symmetry_flag_)) {
      augmented_ray_paths.emplace_back(p);
    }
  }

  // Calculate hash.
  ray_path_hashes_.clear();
  for (const auto& rp : augmented_ray_paths) {
    ray_path_hashes_.emplace(RayPathRecorder::Hash(rp));
  }
}


bool SpecificRayPathFilter::FilterPath(const Crystal* crystal, RaySegment* last_r) const {
  if (ray_path_hashes_.empty()) {
    return true;
  }

  int curr_fn0 = crystal->FaceNumber(last_r->root_ctx->first_ray_segment->face_id);
  if (curr_fn0 == kInvalidId || crystal->GetFaceNumberPeriod() < 0) {  // If do not have face number mapping.
    return true;
  }

  auto current_ray_path_hash = last_r->recorder.Hash();
  return ray_path_hashes_.count(current_ray_path_hash) != 0;
}


void SpecificRayPathFilter::SaveToJson(nlohmann::json& obj) const {
  obj["type"] = "specific";

  if (ray_paths_.size() == 1) {
    for (const auto& f : ray_paths_[0]) {
      obj["path"].emplace_back(f);
    }
  } else if (ray_paths_.size() > 1) {
    for (const auto& rp : ray_paths_) {
      obj["path"].emplace_back(nlohmann::json::array());
      for (const auto& f : rp) {
        obj["path"].back().emplace_back(f);
      }
    }
  }
}


void SpecificRayPathFilter::LoadFromJson(const nlohmann::json& obj) {
  ClearPaths();
  if (!obj["path"].is_array()) {
    throw nlohmann::detail::other_error::create(-1, "filter path is not an array!", obj);
  }
  if (obj["path"][0].is_array()) {
    for (const auto& p : obj["path"]) {
      RayPath curr_path;
      for (const auto& f : p) {
        curr_path << f.get<int>();
      }
      AddPath(curr_path);
    }
  } else {
    RayPath curr_path;
    for (const auto& f : obj["path"]) {
      curr_path << f.get<int>();
    }
    AddPath(curr_path);
  }
}


void GeneralRayPathFilter::AddEntryExitFace(ShortIdType entry_face, ShortIdType exit_face) {
  entry_exit_faces_.emplace_back(EntryExitFace{ entry_face, exit_face });
}


void GeneralRayPathFilter::AddHitNumber(int hit_num) {
  hit_nums_.emplace(hit_num);
}


void GeneralRayPathFilter::ClearFaces() {
  entry_exit_faces_.clear();
}


void GeneralRayPathFilter::ClearHitNumbers() {
  hit_nums_.clear();
}


bool GeneralRayPathFilter::FilterPath(const Crystal* crystal, RaySegment* last_r) const {
  if (entry_exit_faces_.empty()) {
    return true;
  }

  if (!hit_nums_.empty()) {  // Check hit number.
    auto* p = last_r;
    int n = 0;
    while (p) {
      p = p->prev;
      n++;
    }
    if (hit_nums_.count(n) == 0) {
      return false;
    }
  }

  auto curr_entry_fn = crystal->FaceNumber(last_r->root_ctx->first_ray_segment->face_id);
  auto curr_exit_fn = crystal->FaceNumber(last_r->face_id);
  if (curr_entry_fn == kInvalidId || curr_exit_fn == kInvalidId ||
      crystal->GetFaceNumberPeriod() < 0) {  // If do not have a face number mapping
    return true;
  }

  auto iter = std::find_if(entry_exit_faces_.begin(), entry_exit_faces_.end(),
                           [=](const EntryExitFace& f) { return f.entry == curr_entry_fn && f.exit == curr_exit_fn; });
  return iter != entry_exit_faces_.end();
}


RayPathFilterPtrU GeneralRayPathFilter::MakeCopy() const {
  return std::unique_ptr<AbstractRayPathFilter>{ new GeneralRayPathFilter(*this) };
}


void GeneralRayPathFilter::ApplySymmetry(const CrystalContext* crystal_ctx) {
  std::vector<EntryExitFace> tmp_faces;
  tmp_faces.emplace_back(entry_exit_faces_[0]);

  RayPath rp;
  rp << crystal_ctx->GetId() << tmp_faces[0].entry << tmp_faces[0].exit << kInvalidId;
  for (auto&& p : MakeSymmetryExtension(rp, crystal_ctx, symmetry_flag_)) {
    tmp_faces.emplace_back(EntryExitFace{ p.ids[1], p.ids[p.len - 2] });
  }

  entry_exit_faces_.swap(tmp_faces);
}


void GeneralRayPathFilter::SaveToJson(nlohmann::json& obj) const {
  obj["type"] = "general";

  if (!entry_exit_faces_.empty()) {
    auto entry = entry_exit_faces_[0].entry;
    auto exit = entry_exit_faces_[0].exit;
    obj["entry"] = entry;
    obj["exit"] = exit;
  }

  if (!hit_nums_.empty()) {
    for (const auto& n : hit_nums_) {
      obj["hit"].emplace_back(n);
    }
  }
}


void GeneralRayPathFilter::LoadFromJson(const nlohmann::json& obj) {
  ClearHitNumbers();
  ClearFaces();

  auto entry = obj.at("entry").get<ShortIdType>();
  auto exit = obj.at("exit").get<ShortIdType>();
  AddEntryExitFace(entry, exit);

  if (!obj.at("hit").is_array()) {
    throw nlohmann::detail::other_error::create(-1, "<hit> must be an array!", obj);
  }
  for (const auto& n : obj.at("hit")) {
    AddHitNumber(n.get<int>());
  }
}

namespace v3 {

class NoneFilter : public Filter {
 protected:
  bool InternalCheck(const RaySeg& /**/) const override { return true; }
};


class RaypathFilter : public Filter {
 public:
  RaypathFilter(const std::vector<IdType>& rp) : rp_(rp) {}

  void InitCrystalSymmetry(const Crystal& crystal) override {
    auto expand_rp = crystal.ExpandRaypath(rp_, symmetry_);
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

  void InitCrystalSymmetry(const Crystal& crystal) override {
    std::vector<IdType> ee_rp{ entry_, exit_ };
    auto expand_ee_rp = crystal.ExpandRaypath(ee_rp, symmetry_);
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
  CrystalFilter(IdType crystal_id) : crystal_id_(crystal_id){};

 protected:
  bool InternalCheck(const RaySeg& ray) const override { return ray.crystal_id_ == crystal_id_; }

 private:
  IdType crystal_id_;
};


// =============== Filter ===============
struct SimpleFilterCreator {
  FilterPtrU operator()(const NoneFilterParam& /* p */) {
    // NoneFilter
    return FilterPtrU{ new NoneFilter };
  }

  FilterPtrU operator()(const RaypathFilterParam& p) {
    // RaypathFilter
    return FilterPtrU{ new RaypathFilter{ p.raypath_ } };
  }

  FilterPtrU operator()(const EntryExitFilterParam& p) {
    // EntryExitFilter
    return FilterPtrU{ new EntryExitFilter{ p.entry_, p.exit_ } };
  }

  FilterPtrU operator()(const DirectionFilterParam& p) {
    // DirectionFilter
    return FilterPtrU{ new DirectionFilter{ p.lon_, p.lat_, p.radii_ } };
  }

  FilterPtrU operator()(const CrystalFilterParam& p) {
    // CrystalFilter
    return FilterPtrU{ new CrystalFilter{ p.crystal_id_ } };
  }
};


class ComplexFilter : public Filter {
 public:
  ComplexFilter(const std::vector<std::vector<std::pair<IdType, SimpleFilterParam>>>& all_param) {
    for (const auto& p : all_param) {
      std::vector<FilterPtrU> f;
      for (const auto& pp : p) {
        f.emplace_back(std::visit(SimpleFilterCreator{}, pp.second));
      }
      filters_.emplace_back(std::move(f));
    }
  }

 protected:
  bool InternalCheck(const RaySeg& ray) const override {
    for (const auto& and_f : filters_) {
      bool or_check = false;
      for (const auto& or_f : and_f) {
        if (or_f->Check(ray)) {
          or_check = true;
          break;
        }
      }
      if (!or_check) {
        return false;
      }
    }
    return true;
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
    return FilterPtrU{ new ComplexFilter{ p.filters_ } };
  }
};


// =============== Filter ===============
FilterPtrU Filter::Create(const FilterConfig& config) {
  auto filter = std::visit(FilterCreator{}, config.param_);
  filter->action_ = config.action_;
  filter->symmetry_ = config.symmetry_;
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

}  // namespace v3

}  // namespace icehalo
