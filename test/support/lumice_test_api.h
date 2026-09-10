#ifndef LUMICE_TEST_API_H_
#define LUMICE_TEST_API_H_

// The TEST-ONLY export surface of liblumice_testapi: lumice.h plus a handful of LUMICE_TEST_*
// hooks the pytest (ctypes) harness needs and the product ABI must never carry. Everything a test
// can do through the product C API it still does through the product C API — the hooks here
// exist only for what has no product-facing reason to be exported at all.
//
// liblumice_testapi is a SUPERSET STAND-IN for liblumice, not a companion library: it is built
// from the same lumice_obj objects (so every LUMICE_* call behaves identically) and additionally
// exports the symbols declared below. A test process loads this ONE library and never liblumice
// beside it — two copies would be two engines with two sets of statics. Only the shared flavor
// builds it (root CMakeLists.txt, target `lumice_testapi`); the file names are
// liblumice_testapi.dylib / liblumice_testapi.so / lumice_testapi.dll.
//
// The header lives under test/ on purpose, and scripts/check_policies.py's no-test-symbol-in-src
// rule keeps the LUMICE_TEST_ prefix out of src/, so a hook cannot migrate into the product
// surface by accident.

#include "lumice.h"

#ifdef __cplusplus
extern "C" {
#endif

// Same export mechanism as lumice.h: with -fvisibility=hidden on the target, only this block is
// exported from the shared library. MSVC has no equivalent pragma and the target is built with
// WINDOWS_EXPORT_ALL_SYMBOLS instead.
#if !defined(_MSC_VER)
#pragma GCC visibility push(default)
#endif

// The render-domain mask of one view: row-major width*height, indexed py * width + px, 1 where the
// lens images a piece of sky the view is allowed to draw — imaged, inside `visible`, and inside
// the front hemisphere when `front` is set. Byte-identical to lumice::annotation::Overlay::drawable
// for the same view — the mask the CLI renderer composites against — since both come from one
// lumice::annotation::ComputeOverlay sweep and the same mask_detail::PixelToWorld + VisibleByRange
// + FrontVisible predicate the renderer bakes with. The product API exports no mask at all (see the
// v4.28 note at LUMICE_API_VERSION); this hook is the only door onto it from outside the process. `imaged` is NULL only
// for a degenerate view (width or height <= 0).
typedef struct LUMICE_TEST_RenderDomainMask_ {
  int width;
  int height;
  const unsigned char* imaged;
  // Opaque handle to the storage `imaged` points into. Do not read, write, copy or free it; pass
  // this struct to LUMICE_TEST_ReleaseRenderDomainMask exactly once instead — the same
  // acquire/release discipline LUMICE_Scene / LUMICE_ResultFrame / LUMICE_AnnotationAnchors use.
  // Copying the struct copies the handle, so only ONE copy may be released.
  void* storage;
} LUMICE_TEST_RenderDomainMask;

// Compute ONLY the render-domain mask of one view — no curve masks, no label anchors, no markers.
// Exists so a test fixture that needs the mask can have it without the product ABI exporting one
// for a test's sake. Pure, deterministic and thread-safe like LUMICE_ComputeAnnotationAnchors. `*out` is fully
// overwritten on success and left untouched on failure. A degenerate view (width or height <= 0) is not an error: it
// yields width = height = 0 and imaged = NULL, and still must be Released.
//
// Returns LUMICE_ERR_NULL_ARG if `view` or `out` is NULL; LUMICE_ERR_INVALID_VALUE for an unknown
// lens_type / visible; LUMICE_ERR_UNKNOWN on allocation failure.
LUMICE_ErrorCode LUMICE_TEST_ComputeRenderDomainMask(const LUMICE_AnnotationView* view,
                                                     LUMICE_TEST_RenderDomainMask* out);

// Release the storage a successful LUMICE_TEST_ComputeRenderDomainMask allocated, and NULL out the
// pointer so a double release is a no-op rather than a double free. NULL-safe, and safe on a
// zero-initialized or already-released struct.
void LUMICE_TEST_ReleaseRenderDomainMask(LUMICE_TEST_RenderDomainMask* mask);

#if !defined(_MSC_VER)
#pragma GCC visibility pop
#endif

#ifdef __cplusplus
}
#endif

#endif  // LUMICE_TEST_API_H_
