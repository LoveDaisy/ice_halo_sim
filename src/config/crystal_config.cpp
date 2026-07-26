#include "config/crystal_config.hpp"

#include <iterator>
#include <nlohmann/json.hpp>
#include <numeric>
#include <utility>
#include <variant>
#include <vector>

#include "core/math.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

// Which ShapeScalar slots physically exist on each crystal type. A prism has one
// height and six faces; a pyramid has three stacked heights and six faces.
constexpr bool kApplicablePrism[kShapeScalarCount] = {
  true,   // kShapeScalarHeight
  false,  // kShapeScalarUpperH
  false,  // kShapeScalarPrismH
  false,  // kShapeScalarLowerH
  true,  true, true, true, true, true,
};
constexpr bool kApplicablePyramid[kShapeScalarCount] = {
  false,  // kShapeScalarHeight
  true,   // kShapeScalarUpperH
  true,   // kShapeScalarPrismH
  true,   // kShapeScalarLowerH
  true,  true, true, true, true, true,
};

void CanonicalizeSyncGroupsImpl(int sync_group[kShapeScalarCount], const bool applicable[kShapeScalarCount]) {
  // Rule 1: a group declared on a slot this crystal type does not have is not a
  // membership at all. Zeroing first is what makes rules 2 and 3 see the real
  // member set.
  for (int i = 0; i < kShapeScalarCount; i++) {
    if (!applicable[i]) {
      sync_group[i] = 0;
    }
  }

  // Rule 2: a one-member group is independence spelled differently.
  for (int i = 0; i < kShapeScalarCount; i++) {
    if (sync_group[i] == 0) {
      continue;
    }
    int members = 0;
    for (int k = 0; k < kShapeScalarCount; k++) {
      if (sync_group[k] == sync_group[i]) {
        members++;
      }
    }
    if (members < 2) {
      sync_group[i] = 0;
    }
  }

  // Rule 3: renumber 1..N by first appearance in ShapeScalar order, so the same
  // partition always has the same integers. Linear scans over <= 10 slots — a map
  // would cost more than it saves.
  int old_id[kShapeScalarCount]{};
  int new_id[kShapeScalarCount]{};
  int assigned = 0;
  for (int i = 0; i < kShapeScalarCount; i++) {
    if (sync_group[i] == 0) {
      continue;
    }
    int mapped = 0;
    for (int k = 0; k < assigned; k++) {
      if (old_id[k] == sync_group[i]) {
        mapped = new_id[k];
        break;
      }
    }
    if (mapped == 0) {
      old_id[assigned] = sync_group[i];
      mapped = assigned + 1;
      new_id[assigned] = mapped;
      assigned++;
    }
    sync_group[i] = mapped;
  }
}

// Leader-normalize one param's groups. `slots` maps a ShapeScalar index to the
// Distribution living there, or nullptr for a slot this crystal type lacks.
void NormalizeSyncGroupsImpl(const int sync_group[kShapeScalarCount], Distribution* const slots[kShapeScalarCount]) {
  for (int i = 0; i < kShapeScalarCount; i++) {
    if (sync_group[i] == 0 || slots[i] == nullptr) {
      continue;
    }
    // The leader is this group's first occupied slot in ShapeScalar order, which
    // — because that order is the RNG draw order — is also the member that
    // actually consumes the draw.
    int leader = -1;
    for (int k = 0; k < i; k++) {
      if (sync_group[k] == sync_group[i] && slots[k] != nullptr) {
        leader = k;
        break;
      }
    }
    if (leader < 0) {
      continue;  // i is itself the leader.
    }
    if (!DistributionValueEqual(*slots[i], *slots[leader])) {
      LOG_WARNING(
          "Crystal shape sync group {}: member at shape-scalar index {} declared a different distribution than its "
          "group leader at index {}; overriding the member to match the leader.",
          sync_group[i], i, leader);
    }
    *slots[i] = *slots[leader];
  }
}

}  // namespace


void CanonicalizeSyncGroups(PrismCrystalParam& p) {
  CanonicalizeSyncGroupsImpl(p.sync_group_, kApplicablePrism);
}

void CanonicalizeSyncGroups(PyramidCrystalParam& p) {
  CanonicalizeSyncGroupsImpl(p.sync_group_, kApplicablePyramid);
}

void NormalizeSyncGroups(PrismCrystalParam& p) {
  Distribution* slots[kShapeScalarCount] = {
    &p.h_, nullptr, nullptr, nullptr, &p.d_[0], &p.d_[1], &p.d_[2], &p.d_[3], &p.d_[4], &p.d_[5],
  };
  NormalizeSyncGroupsImpl(p.sync_group_, slots);
}

void NormalizeSyncGroups(PyramidCrystalParam& p) {
  Distribution* slots[kShapeScalarCount] = {
    nullptr, &p.h_pyr_u_, &p.h_prs_, &p.h_pyr_l_, &p.d_[0], &p.d_[1], &p.d_[2], &p.d_[3], &p.d_[4], &p.d_[5],
  };
  NormalizeSyncGroupsImpl(p.sync_group_, slots);
}

void PrepareSyncGroups(PrismCrystalParam& p) {
  CanonicalizeSyncGroups(p);
  NormalizeSyncGroups(p);
}

void PrepareSyncGroups(PyramidCrystalParam& p) {
  CanonicalizeSyncGroups(p);
  NormalizeSyncGroups(p);
}


namespace {

// Read the optional "sync_group" sub-map. Keys name shape scalars the same way
// the surrounding shape JSON names their distributions ("height", "prism_h",
// "upper_h", "lower_h", "face_distance"), so a reader never has to know the
// ShapeScalar integers. Absent key = every scalar independent, which is why an
// older config file needs no edit at all.
void ReadSyncGroupJson(const nlohmann::json& j, int sync_group[kShapeScalarCount],
                       const std::pair<const char*, int>* scalar_keys, size_t scalar_key_cnt) {
  for (int i = 0; i < kShapeScalarCount; i++) {
    sync_group[i] = 0;
  }
  if (!j.contains("sync_group")) {
    return;
  }
  const auto& sg = j.at("sync_group");
  for (size_t k = 0; k < scalar_key_cnt; k++) {
    if (sg.contains(scalar_keys[k].first)) {
      sync_group[scalar_keys[k].second] = sg.at(scalar_keys[k].first).get<int>();
    }
  }
  if (sg.contains("face_distance")) {
    size_t i = 0;
    for (const auto& elem : sg.at("face_distance")) {
      if (i >= 6) {
        break;
      }
      sync_group[kShapeScalarFace0 + i] = elem.get<int>();
      i++;
    }
  }
}

// Write "sync_group" only when something is actually synced, so the serialized
// form of every existing config stays byte-identical.
void WriteSyncGroupJson(nlohmann::json& j, const int sync_group[kShapeScalarCount],
                        const std::pair<const char*, int>* scalar_keys, size_t scalar_key_cnt) {
  nlohmann::json sg = nlohmann::json::object();
  for (size_t k = 0; k < scalar_key_cnt; k++) {
    if (sync_group[scalar_keys[k].second] != 0) {
      sg[scalar_keys[k].first] = sync_group[scalar_keys[k].second];
    }
  }
  bool any_face = false;
  for (int i = 0; i < 6; i++) {
    any_face = any_face || sync_group[kShapeScalarFace0 + i] != 0;
  }
  if (any_face) {
    // All six, zeros included — matching how face_distance itself serializes.
    sg["face_distance"] = std::vector<int>(sync_group + kShapeScalarFace0, sync_group + kShapeScalarFace0 + 6);
  }
  if (!sg.empty()) {
    j["sync_group"] = sg;
  }
}

constexpr std::pair<const char*, int> kPrismSyncGroupKeys[] = {
  { "height", kShapeScalarHeight },
};
constexpr std::pair<const char*, int> kPyramidSyncGroupKeys[] = {
  { "prism_h", kShapeScalarPrismH },
  { "upper_h", kShapeScalarUpperH },
  { "lower_h", kShapeScalarLowerH },
};

}  // namespace


// convert to & from json object
// ========== PrismCrystalParam ==========
void to_json(nlohmann::json& j, const PrismCrystalParam& p) {
  j["height"] = p.h_;
  j["face_distance"] = p.d_;
  // Canonicalize a local copy: serialization must not depend on the caller having
  // normalized, and must not mutate what it was handed either.
  PrismCrystalParam canon = p;
  CanonicalizeSyncGroups(canon);
  WriteSyncGroupJson(j, canon.sync_group_, kPrismSyncGroupKeys, std::size(kPrismSyncGroupKeys));
}

void from_json(const nlohmann::json& j, PrismCrystalParam& p) {
  if (j.contains("height")) {
    j.at("height").get_to(p.h_);
  }

  // Face distance: default value 1.0 (1.0 = regular hexagon in FillHexCrystalCoef)
  for (auto& x : p.d_) {
    x.type = DistributionType::kNoRandom;
    x.center = 1.0f;
  }
  if (j.contains("face_distance")) {
    size_t i = 0;
    for (const auto& elem : j.at("face_distance")) {
      if (i >= 6) {
        break;
      }
      elem.get_to(p.d_[i]);
      i++;
    }
  }

  ReadSyncGroupJson(j, p.sync_group_, kPrismSyncGroupKeys, std::size(kPrismSyncGroupKeys));
  PrepareSyncGroups(p);
}


// Convert Miller index (i1, i4) to wedge angle in degrees. Returns 28.0 (default) if i1 == 0.
static float MillerToAlpha(int i1, int i4) {
  constexpr float kSqrt3_2 = 0.866025403784f;
  constexpr float kIceCrystalC = 1.629f;
  constexpr float kRadToDeg = 57.2957795131f;
  if (i1 == 0) {
    return 28.0f;
  }
  return std::atan(kSqrt3_2 * i4 / i1 / kIceCrystalC) * kRadToDeg;
}

// ========== PyramidCrystalParam ==========
void to_json(nlohmann::json& j, const PyramidCrystalParam& p) {
  j["prism_h"] = p.h_prs_;
  j["upper_h"] = p.h_pyr_u_;
  j["lower_h"] = p.h_pyr_l_;
  j["upper_wedge_angle"] = p.wedge_angle_u_;
  j["lower_wedge_angle"] = p.wedge_angle_l_;
  j["face_distance"] = p.d_;
  PyramidCrystalParam canon = p;
  CanonicalizeSyncGroups(canon);
  WriteSyncGroupJson(j, canon.sync_group_, kPyramidSyncGroupKeys, std::size(kPyramidSyncGroupKeys));
}

void from_json(const nlohmann::json& j, PyramidCrystalParam& p) {
  // Heights
  j.at("prism_h").get_to(p.h_prs_);
  if (j.contains("upper_h")) {
    j.at("upper_h").get_to(p.h_pyr_u_);
  }
  if (j.contains("lower_h")) {
    j.at("lower_h").get_to(p.h_pyr_l_);
  }

  // Wedge angle: prefer "upper_wedge_angle", fallback to "upper_indices" conversion
  if (j.contains("upper_wedge_angle")) {
    p.wedge_angle_u_ = j.at("upper_wedge_angle").get<float>();
  } else if (j.contains("upper_indices") && j.at("upper_indices").is_array() && j.at("upper_indices").size() == 3) {
    auto& ui = j.at("upper_indices");
    p.wedge_angle_u_ = MillerToAlpha(ui[0].get<int>(), ui[2].get<int>());
  }
  if (j.contains("lower_wedge_angle")) {
    p.wedge_angle_l_ = j.at("lower_wedge_angle").get<float>();
  } else if (j.contains("lower_indices") && j.at("lower_indices").is_array() && j.at("lower_indices").size() == 3) {
    auto& li = j.at("lower_indices");
    p.wedge_angle_l_ = MillerToAlpha(li[0].get<int>(), li[2].get<int>());
  }

  // Face distance: default value 1.0 (1.0 = regular hexagon in FillHexCrystalCoef)
  for (auto& x : p.d_) {
    x.type = DistributionType::kNoRandom;
    x.center = 1.0f;
  }
  if (j.contains("face_distance")) {
    size_t i = 0;
    for (const auto& elem : j.at("face_distance")) {
      if (i >= 6) {
        break;
      }
      elem.get_to(p.d_[i]);
      i++;
    }
  }

  ReadSyncGroupJson(j, p.sync_group_, kPyramidSyncGroupKeys, std::size(kPyramidSyncGroupKeys));
  PrepareSyncGroups(p);
}


// ========== CrystalConfig ==========
void to_json(nlohmann::json& j, const CrystalConfig& c) {
  j["id"] = c.id_;
  j["axis"] = c.axis_;
  std::visit([&j](auto&& p) { j["shape"] = p; }, c.param_);
}

void from_json(const nlohmann::json& j, CrystalConfig& c) {
  j.at("id").get_to(c.id_);

  const auto& j_type = j.at("type");
  if (j_type == "prism") {
    c.param_ = j.at("shape").get<PrismCrystalParam>();
  } else if (j_type == "pyramid") {
    c.param_ = j.at("shape").get<PyramidCrystalParam>();
  } else {
    LOG_ERROR("Unknown crystal type!");
  }

  if (j.contains("axis")) {
    j.at("axis").get_to(c.axis_);
  }
}

}  // namespace lumice
