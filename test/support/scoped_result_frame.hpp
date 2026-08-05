#pragma once

// Header-only test support: holds a LUMICE_ResultFrame for the duration of a scope.
//
// The C API's contract is a plain acquire/release pair (lumice.h, LUMICE_ReleaseResultFrame) —
// the same shape as LUMICE_SceneDestroy and every other handle there. That contract is not what
// this header exists for. It exists because a gtest ASSERT_* / IM_CHECK_* returns from the middle
// of a case, so a hand-placed LUMICE_ReleaseResultFrame() at the bottom of a test body is skipped
// exactly on the runs that already failed — leaking a frame at the moment the output is hardest to
// read. A destructor cannot be skipped that way. This is a caller-side convenience for C++ test
// code, not an obligation the API places on its consumers; C consumers pair the two calls by hand.
//
// It sits in test/support/ rather than in one test's anonymous namespace because its consumers
// span three CMake targets (unit_correctness_test, gui_unit_test, gui_test), all of which carry
// ${PROJ_TEST_DIR} on their include path, so `#include "support/scoped_result_frame.hpp"` is one
// spelling valid in all of them.

#include "lumice.h"

namespace lumice::test {

class ScopedResultFrame {
 public:
  explicit ScopedResultFrame(LUMICE_Server* server) { err_ = LUMICE_AcquireResultFrame(server, &frame_); }

  ~ScopedResultFrame() { LUMICE_ReleaseResultFrame(frame_); }

  ScopedResultFrame(const ScopedResultFrame&) = delete;
  ScopedResultFrame& operator=(const ScopedResultFrame&) = delete;
  ScopedResultFrame(ScopedResultFrame&&) = delete;
  ScopedResultFrame& operator=(ScopedResultFrame&&) = delete;

  // The acquire's own status, so a case can assert on it before reading the frame.
  LUMICE_ErrorCode err() const { return err_; }

  // Null if the acquire failed; the FrameGet* functions reject that with LUMICE_ERR_NULL_ARG
  // rather than crashing, so a case that forgets to check err() still fails loudly.
  LUMICE_ResultFrame* get() const { return frame_; }

 private:
  LUMICE_ResultFrame* frame_ = nullptr;
  LUMICE_ErrorCode err_ = LUMICE_OK;
};

}  // namespace lumice::test
