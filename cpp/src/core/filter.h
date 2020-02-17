#ifndef SRC_CORE_FILTER_H_
#define SRC_CORE_FILTER_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "core/crystal.h"
#include "core/optics.h"
#include "io/serialize.h"


namespace icehalo {

enum Symmetry : uint8_t {
  kSymmetryNone = 0u,
  kSymmetryPrism = 1u,
  kSymmetryBasal = 2u,
  kSymmetryDirection = 4u,
  kSymmetryRepeatedReflection = 8u,
};

size_t RayPathHash(const std::vector<uint16_t>& ray_path, bool reverse = false);
size_t RayPathReverseHash(const Crystal* crystal, const RaySegment* last_ray, int length);


class AbstractRayPathFilter : public IJsonizable {
 public:
  AbstractRayPathFilter();

  bool Filter(const Crystal* crystal, RaySegment* last_r) const;

  void SetSymmetryFlag(uint8_t symmetry_flag);
  void AddSymmetry(Symmetry symmetry);
  uint8_t GetSymmetryFlag() const;
  virtual void ApplySymmetry(const Crystal* crystal);

  void EnableComplementary(bool enable);
  bool GetComplementary() const;

  void EnableRemoveHomodromous(bool enable);
  bool GetRemoveHomodromous() const;

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

 protected:
  virtual bool FilterPath(const Crystal* crystal, RaySegment* last_r) const = 0;

  uint8_t symmetry_flag_;
  bool complementary_;
  bool remove_homodromous_;
};

using RayPathFilterPtrU = std::unique_ptr<AbstractRayPathFilter>;


class NoneRayPathFilter : public AbstractRayPathFilter {
 public:
  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;
};


class SpecificRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddPath(const std::vector<uint16_t>& path);
  void ClearPaths();

  void ApplySymmetry(const Crystal* crystal) override;
  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<size_t> ray_path_hashes_;
  std::vector<std::vector<uint16_t>> ray_paths_;
};


class GeneralRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddEntryFace(uint16_t face_number);
  void AddExitFace(uint16_t face_number);
  void AddHitNumber(int hit_num);
  void ClearFaces();
  void ClearHitNumbers();

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<uint16_t> entry_faces_;
  std::unordered_set<uint16_t> exit_faces_;
  std::unordered_set<int> hit_nums_;
};


}  // namespace icehalo


#endif  // SRC_CORE_FILTER_H_
