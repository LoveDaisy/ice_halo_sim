#ifndef SRC_CORE_FILTER_H_
#define SRC_CORE_FILTER_H_

#include <cstddef>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/raypath.hpp"
#include "io/json_util.hpp"
#include "io/serialize.hpp"
#include "protocol/filter_config.hpp"

namespace icehalo {

struct RaySegment;
enum class RaySegmentState : uint8_t;

class AbstractRayPathFilter {
 public:
  AbstractRayPathFilter();
  AbstractRayPathFilter(const AbstractRayPathFilter& other);
  virtual ~AbstractRayPathFilter() = default;

  AbstractRayPathFilter& operator=(const AbstractRayPathFilter& other);

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

  friend void to_json(nlohmann::json& obj, const AbstractRayPathFilter& filter);
  friend void from_json(const nlohmann::json& obj, AbstractRayPathFilter& filter);

 protected:
  virtual bool FilterPath(const Crystal* crystal, RaySegment* last_r) const = 0;

  virtual void SaveToJson(nlohmann::json& obj) const = 0;
  virtual void LoadFromJson(const nlohmann::json& obj) = 0;

  uint8_t symmetry_flag_;
  bool complementary_;
  bool remove_homodromous_;
};

using RayPathFilterPtrU = std::unique_ptr<AbstractRayPathFilter>;


class NoneRayPathFilter : public AbstractRayPathFilter {
 public:
  RayPathFilterPtrU MakeCopy() const override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

  void SaveToJson(nlohmann::json& obj) const override;
  void LoadFromJson(const nlohmann::json& obj) override;
};


class SpecificRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddPath(const RayPath& path);
  void ClearPaths();
  std::vector<RayPath> GetRayPaths() const;

  RayPathFilterPtrU MakeCopy() const override;
  void ApplySymmetry(const CrystalContext* crystal_ctx) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

  void SaveToJson(nlohmann::json& obj) const override;
  void LoadFromJson(const nlohmann::json& obj) override;

 private:
  std::unordered_set<size_t> ray_path_hashes_;
  std::vector<RayPath> ray_paths_;  //!< The original ray path. Not include crystal ID and kInvalidFaceNumber
};


class GeneralRayPathFilter : public AbstractRayPathFilter {
 public:
  void AddEntryExitFace(ShortIdType entry_face, ShortIdType exit_face);
  void AddHitNumber(int hit_num);
  void ClearFaces();
  void ClearHitNumbers();

  RayPathFilterPtrU MakeCopy() const override;
  void ApplySymmetry(const CrystalContext* crystal_ctx) override;

 protected:
  bool FilterPath(const Crystal* crystal, RaySegment* last_r) const override;

  void SaveToJson(nlohmann::json& obj) const override;
  void LoadFromJson(const nlohmann::json& obj) override;

 private:
  struct EntryExitFace {
    ShortIdType entry;
    ShortIdType exit;
  };
  std::vector<EntryExitFace> entry_exit_faces_;
  std::unordered_set<int> hit_nums_;
};


namespace v3 {

class Filter;
using FilterPtrU = std::unique_ptr<Filter>;
using FilterPtrS = std::shared_ptr<Filter>;

class Filter {
 public:
  static FilterPtrU Create(const FilterConfig& config);

  Filter() = default;
  Filter(const Filter& other) = default;
  virtual ~Filter() = default;

  Filter& operator=(const Filter& other) = default;

  bool Check(const RaySeg& ray) const;  // Put action logic here, and detail logic in InternalCheck()
  virtual void InitCrystalSymmetry(const Crystal& /* crystal */){};

 protected:
  virtual bool InternalCheck(const RaySeg& ray) const = 0;

  uint8_t symmetry_;
  FilterConfig::Action action_;
};

}  // namespace v3

}  // namespace icehalo


#endif  // SRC_CORE_FILTER_H_
