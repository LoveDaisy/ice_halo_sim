#ifndef SRC_CONTEXT_CRYSTAL_CONTEXT_H_
#define SRC_CONTEXT_CRYSTAL_CONTEXT_H_

#include <memory>

#include "core/crystal.h"
#include "io/serialize.h"


namespace icehalo {

struct CrystalContext;
using CrystalContextPtrU = std::unique_ptr<CrystalContext>;

struct CrystalContext {
  CrystalContext(CrystalPtrU g, AxisDistribution axis);
  CrystalContext(const CrystalContext& other) = delete;

  int RandomSampleFace(const float* ray_dir) const;

  const CrystalPtrU crystal;
  const AxisDistribution axis;
};


}  // namespace icehalo


#endif  // SRC_CONTEXT_CRYSTAL_CONTEXT_H_
