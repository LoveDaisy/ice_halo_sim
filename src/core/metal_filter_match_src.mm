// MSL device filter match — string definition retired (task-#283).
//
// Previously this file defined `kFilterMatchHelperSrc` as a C++ raw string
// literal containing the device filter match MSL helpers. That MSL body has
// moved into `src/core/metal/lumice_trace.metal`, which CMake precompiles to
// a .metallib at build time. The header is now compile-time only (just the
// kDevFilterMatchRecCap constant); this translation unit remains in the
// build so the .o slot stays stable and CMake's APPLE source list does not
// need to be edited downstream — but it carries no runtime state.

#include "core/metal_filter_match_src.hpp"
