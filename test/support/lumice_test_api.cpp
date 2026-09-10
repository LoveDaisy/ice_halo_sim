// Implementation of the test-only export surface (lumice_test_api.h). Compiled twice, into two
// binaries that never link each other: the `lumice_testapi` shared library (root CMakeLists.txt,
// what pytest dlopens) and `unit_correctness_test` (test/CMakeLists.txt, where the parity test
// against the product API lives). If a THIRD consumer of these symbols ever appears, that is the
// point to split this file into an OBJECT library rather than list it a third time.
#include "lumice_test_api.h"

#include <memory>

#include "core/annotation_overlay.hpp"
#include "server/c_api_internal.hpp"  // ToAnnotationViewSnapshot (a56: single translation owner)

namespace {

// Everything `imaged` points into, owned through the struct's opaque `storage` handle so a single
// Release frees it and the released state (NULL pointer) is expressible.
struct RenderDomainMaskStorage {
  lumice::annotation::Overlay overlay;
};

}  // namespace


LUMICE_ErrorCode LUMICE_TEST_ComputeRenderDomainMask(const LUMICE_AnnotationView* view,
                                                     LUMICE_TEST_RenderDomainMask* out) {
  if (!view || !out) {
    return LUMICE_ERR_NULL_ARG;
  }
  if (view->lens_type < 0 || view->lens_type > LUMICE_LENS_TYPE_GLOBE) {
    return LUMICE_ERR_INVALID_VALUE;
  }
  if (view->visible != LUMICE_VISIBLE_UPPER && view->visible != LUMICE_VISIBLE_LOWER &&
      view->visible != LUMICE_VISIBLE_FULL) {
    return LUMICE_ERR_INVALID_VALUE;
  }

  // Same view translation LUMICE_ComputeAnnotationOverlay performs (ToAnnotationViewSnapshot,
  // server/c_api_internal.hpp — a56: single owner), feeding the same core sweep. No line list, no
  // markers, and no label walk: every other Request field keeps its default (empty lists,
  // zenith_nadir = false), and `labels` is switched off explicitly because its default is on. The
  // drawable sweep is independent of all of them — the parity test in
  // test/unit-correctness/server/ is what holds that claim to the product API's output.
  lumice::annotation::Request req;
  req.view = ToAnnotationViewSnapshot(*view);
  req.labels = false;

  std::unique_ptr<RenderDomainMaskStorage> storage;
  try {
    storage = std::make_unique<RenderDomainMaskStorage>();
    storage->overlay = lumice::annotation::ComputeOverlay(req);
  } catch (...) {
    return LUMICE_ERR_UNKNOWN;
  }

  const lumice::annotation::Overlay& o = storage->overlay;
  out->width = o.width;
  out->height = o.height;
  out->imaged = o.drawable.empty() ? nullptr : o.drawable.data();
  out->storage = storage.release();
  return LUMICE_OK;
}


void LUMICE_TEST_ReleaseRenderDomainMask(LUMICE_TEST_RenderDomainMask* mask) {
  if (!mask || !mask->storage) {
    return;  // NULL-safe, and idempotent on an already-released or zero-initialized struct
  }
  const std::unique_ptr<RenderDomainMaskStorage> owned(static_cast<RenderDomainMaskStorage*>(mask->storage));
  mask->storage = nullptr;
  // Leave no dangling view of freed memory behind, so a caller that keeps reading the struct after
  // Release sees "nothing here" rather than a use-after-free.
  mask->imaged = nullptr;
  mask->width = 0;
  mask->height = 0;
}
