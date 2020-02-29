#ifndef SRC_CORE_FILTER_H_
#define SRC_CORE_FILTER_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "context/crystal_context.hpp"
#include "core/core_def.hpp"
#include "io/serialize.hpp"


namespace icehalo {

struct RaySegment;
enum class RaySegmentState : uint8_t;

class AbstractRayPathFilter : public IJsonizable {
 public:
  AbstractRayPathFilter();
  AbstractRayPathFilter(const AbstractRayPathFilter& other);

  virtual std::unique_ptr<AbstractRayPathFilter> MakeCopy() const = 0;

  bool Filter(const Crystal* crystal, RaySegment* last_r) const;

  void SetSymmetryFlag(uint8_t symmetry_flag);
  void AddSymmetry(Symmetry symmetry);
  uint8_t GetSymmetryFlag() const;
  virtual void ApplySymmetry(const CrystalContext* crystal_ctx);

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
  RayPathFilterPtrU MakeCopy() const override;

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;
};


class SpecificRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddPath(const RayPath& path);
  void ClearPaths();
  std::vector<RayPath> GetRayPaths() const;

  RayPathFilterPtrU MakeCopy() const override;
  void ApplySymmetry(const CrystalContext* crystal_ctx) override;
  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<size_t> ray_path_hashes_;
  std::vector<RayPath> ray_paths_;  //!< The original ray path. Not include crystal ID and kInvalidFaceNumber
};


class GeneralRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddEntryFace(FaceNumberType face_number);
  void AddExitFace(FaceNumberType face_number);
  void AddHitNumber(int hit_num);
  void ClearFaces();
  void ClearHitNumbers();

  RayPathFilterPtrU MakeCopy() const override;
  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

 private:
  std::unordered_set<FaceNumberType> entry_faces_;
  std::unordered_set<FaceNumberType> exit_faces_;
  std::unordered_set<int> hit_nums_;
};


}  // namespace icehalo


#endif  // SRC_CORE_FILTER_H_
