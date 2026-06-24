#ifndef CORE_BACKEND_BACKEND_KIND_H_
#define CORE_BACKEND_BACKEND_KIND_H_

namespace lumice {

enum class BackendKind : int {
  kCpu = 0,
  kMetal = 1,
  kCuda = 2,
};

}  // namespace lumice

#endif  // CORE_BACKEND_BACKEND_KIND_H_
