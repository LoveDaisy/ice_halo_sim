#pragma once

// Header-only RAII wrapper for a LUMICE_ResultFrame handle, shared by every C++ consumer of
// the result-frame API in this repo's production code (the CLI in src/main.cpp, and the GUI's
// src/gui/app.cpp / src/gui/server_poller.cpp).
//
// The C API's contract is a plain acquire/release pair (lumice.h, LUMICE_ReleaseResultFrame) —
// that is what a C caller writes, and this header does not change it. It exists because a C++
// caller has early returns: three of these call sites bail out mid-function when a FrameGet*
// rejects the frame, and a hand-placed release at the bottom is skipped on exactly those paths.
// A destructor cannot be skipped that way. So this is a caller-side convenience, not an
// obligation the API places on its consumers.
//
// It lives under util/ because its consumers span the CLI and the GUI, and util/ is the only
// place both already include from (src/gui/ may not include core/ or config/ — see the API
// boundary rule in AGENTS.md, enforced by scripts/check_policies.py). Note the layering is
// unusual for this directory: everything else in util/ sits BELOW the C API, whereas this sits
// above it, consuming lumice.h like any external caller would. It depends on nothing else in
// util/, so it does not drag the rest of the directory across that line.
//
// test/support/scoped_result_frame.hpp is the test-side counterpart. The two are deliberately
// NOT shared: src/ must not depend on test/, and the test helper answers a different question
// (a gtest ASSERT_* returning from the middle of a case, plus an err() accessor a test asserts
// on). Keeping them separate costs a few lines and keeps the dependency arrow pointing one way.

#include <memory>

#include "lumice.h"

namespace lumice {

// Stateless deleter, so ResultFramePtr is pointer-sized — unlike a
// unique_ptr<LUMICE_ResultFrame, void(*)(LUMICE_ResultFrame*)>, which stores the function
// pointer alongside the handle.
struct ResultFrameDeleter {
  void operator()(LUMICE_ResultFrame* frame) const { LUMICE_ReleaseResultFrame(frame); }
};

// The default shape: sole ownership, released at end of scope.
using ResultFramePtr = std::unique_ptr<LUMICE_ResultFrame, ResultFrameDeleter>;

// The shared shape, for the one consumer that needs the frame to outlive the scope that
// acquired it: server_poller.cpp hands a share to the texture payload it materializes, which
// is what lets the payload point straight at the frame's buffers instead of copying them.
//
// shared_ptr type-erases its deleter instead of carrying it in the type, so unlike
// ResultFramePtr this alias cannot make the right deleter automatic. Build one with
// MakeSharedResultFrame below rather than constructing the alias directly — that turns
// "remember to pass ResultFrameDeleter{}" from an instruction into a mechanism.
using SharedResultFramePtr = std::shared_ptr<LUMICE_ResultFrame>;

inline SharedResultFramePtr MakeSharedResultFrame(LUMICE_ResultFrame* frame) {
  return SharedResultFramePtr(frame, ResultFrameDeleter{});
}

}  // namespace lumice
