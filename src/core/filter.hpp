#ifndef SRC_CORE_FILTER_H_
#define SRC_CORE_FILTER_H_

#include <cstdint>
#include <memory>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/raypath.hpp"

namespace lumice {

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
  virtual void InitCrystalSymmetry(const Crystal& /* crystal */) {};

 protected:
  virtual bool InternalCheck(const RaySeg& ray) const = 0;

  uint8_t symmetry_;
  FilterConfig::Action action_;
};

}  // namespace lumice


#endif  // SRC_CORE_FILTER_H_
