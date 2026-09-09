// The wedge dropdown's custom Miller-index row must say what core says — and say nothing of its own.
//
// The row exists so a user can reach a cone face the four built-in presets do not offer, by typing
// indices instead of degrees. That only works if the verdict it shows is the same verdict the mesh
// builder will act on, and the way that guarantee is normally lost is not a wrong formula: it is a
// second copy of the rules. config, server and GUI each transcribed the bare Miller formula once,
// and the copies then disagreed with core about what h == 0 means. So EvaluateCustomWedgeInput
// holds no rules — it forwards to LUMICE_ConvertMillerIndexToWedgeAngle and looks up text on the
// (state, invalid_index) pair that comes back.
//
// This test is that claim's standing check. Every row below puts the same indices through both the
// C API directly and the GUI function, and demands they agree on state and angle bit for bit. A
// local shortcut added "so the popup can answer faster" goes red here even when it happens to be
// correct today, which is the point: agreement by construction, not by coincidence.
//
// One unit's pure logic, no frame and no input event, so it lives here rather than in gui_test —
// which matters twice over, because CI runs gui_test's `modal_layout` / `defaults_panel_layout`
// groups and nothing else, and would never have evaluated it there.

#include <gtest/gtest.h>

#include <cctype>
#include <set>
#include <string>

#include "gui/edit_modals.hpp"
#include "include/lumice.h"

namespace gui = lumice::gui;

namespace {

// The slots LUMICE_ConvertMillerIndexToWedgeAngle's out_invalid_index names (0 = h, 1 = k, 2 = l,
// -1 = no single slot). Spelled out here for the same reason edit_modals.cpp spells them out: a
// bare 1 in an expectation is a magic number whose meaning lives in another file's comment.
constexpr int kSlotH = 0;
constexpr int kSlotK = 1;
constexpr int kSlotL = 2;
constexpr int kSlotNone = -1;

struct Case {
  const char* what;  // Which branch of the mapping this row is here to exercise.
  int h;
  int k;
  int l;
  LUMICE_MillerConversionState expect_state;
  int expect_slot;
};

// h = 1, l = 0 is the {1,0,-1,0} row that kWedgePresetIndices (edit_modals.cpp) deliberately does
// not carry: a PRISM face, whose ratio converts to 0 degrees and so falls below the buildable floor. h = 1, l = 2000 is
// the same rejection from the other end. Both are ratio verdicts, hence kSlotNone — neither integer is wrong on its
// own. The expectations here are asserted against the API's live answer below, not merely trusted; a row whose premise
// stops holding fails rather than silently testing nothing.
constexpr Case kCases[] = {
  { "a buildable face the built-in presets do not offer ({3,0,-3,1})", 3, 0, 1, LUMICE_MILLER_VALID, kSlotNone },
  { "the shallowest built-in face ({1,0,-1,1})", 1, 0, 1, LUMICE_MILLER_VALID, kSlotNone },
  { "h = 0: legal, but it means 'no cap on this side'", 0, 0, 1, LUMICE_MILLER_NO_CONE, kSlotNone },
  { "k != 0: a second-order face this crystal model cannot express", 1, 1, 2, LUMICE_MILLER_INVALID, kSlotK },
  { "a negative h", -1, 0, 1, LUMICE_MILLER_INVALID, kSlotH },
  { "a negative l", 1, 0, -1, LUMICE_MILLER_INVALID, kSlotL },
  { "a ratio below the buildable floor (a prism face)", 1, 0, 0, LUMICE_MILLER_INVALID, kSlotNone },
  { "a ratio above the buildable ceiling", 1, 0, 2000, LUMICE_MILLER_INVALID, kSlotNone },
};

// The C API's own answer for one triple, so a row can be compared against the owner rather than
// against a number somebody typed into this file.
struct ApiAnswer {
  LUMICE_ErrorCode err;
  LUMICE_MillerConversionState state;
  float angle;
  int slot;
};

ApiAnswer AskOwner(int h, int k, int l) {
  ApiAnswer a{ LUMICE_OK, LUMICE_MILLER_INVALID, 0.0f, kSlotNone };
  a.err = LUMICE_ConvertMillerIndexToWedgeAngle(h, k, l, 3, &a.state, &a.angle, &a.slot);
  return a;
}

// Whether `message` opens by naming the box it blames, e.g. "l must not be negative.".
bool NamesSlot(const std::string& message, char slot_letter) {
  return message.size() >= 2 && message[0] == slot_letter && std::isspace(static_cast<unsigned char>(message[1]));
}

std::string Lowered(const std::string& s) {
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return out;
}

// The load-bearing one: state and angle must be the owner's, to the bit. Everything the row shows
// is derived from these two, so a divergence here is a second implementation no matter how it got
// there.
TEST(CustomWedgeInput, ForwardsTheOwnersVerdictWithoutRestatingIt) {
  for (const Case& c : kCases) {
    SCOPED_TRACE(testing::Message() << c.what << " -- h=" << c.h << " k=" << c.k << " l=" << c.l);

    const ApiAnswer api = AskOwner(c.h, c.k, c.l);
    // Non-fatal, then `continue`: a fatal assert would return out of the loop and hide every case
    // after the first failure, which is the census this test exists to take.
    if (api.err != LUMICE_OK) {
      ADD_FAILURE() << "conversion call failed with error code " << api.err;
      continue;
    }

    // The row's premise. If the owner's verdict for these indices ever changes, this is what says
    // so — rather than the case quietly exercising a branch it was not written for.
    EXPECT_EQ(api.state, c.expect_state);
    EXPECT_EQ(api.slot, c.expect_slot);

    const gui::CustomWedgeInputFeedback fb = gui::EvaluateCustomWedgeInput(c.h, c.k, c.l);
    EXPECT_EQ(fb.state, api.state);
    EXPECT_FLOAT_EQ(fb.angle_deg, api.angle);
  }
}

// Apply writes fb.angle_deg straight into the wedge field, so it must be live for exactly the one
// verdict that carries a buildable angle. NO_CONE is the interesting exclusion: its angle, 0, is a
// truthful answer that this field cannot hold — the slider's domain starts at 0.1 and would clamp
// it up into a visibly different crystal without telling anyone.
TEST(CustomWedgeInput, ApplyIsLiveOnlyForAnAngleTheFieldCanActuallyHold) {
  for (const Case& c : kCases) {
    SCOPED_TRACE(testing::Message() << c.what);
    const gui::CustomWedgeInputFeedback fb = gui::EvaluateCustomWedgeInput(c.h, c.k, c.l);
    EXPECT_EQ(fb.can_apply, c.expect_state == LUMICE_MILLER_VALID);
  }
}

// AC3: every refusal has to be visible and has to say which number to change. A blank message is
// the silent fallback this row exists to rule out.
TEST(CustomWedgeInput, EveryRefusalCarriesAMessageAndEveryAcceptanceCarriesNone) {
  for (const Case& c : kCases) {
    SCOPED_TRACE(testing::Message() << c.what);
    const gui::CustomWedgeInputFeedback fb = gui::EvaluateCustomWedgeInput(c.h, c.k, c.l);
    if (c.expect_state == LUMICE_MILLER_VALID) {
      EXPECT_TRUE(fb.message.empty()) << "an accepted triple should need no explanation: " << fb.message;
    } else {
      EXPECT_FALSE(fb.message.empty());
    }
  }
}

// A message that blames a slot must name that slot, and the ratio verdict must name none of them.
// The owner returns -1 there on purpose: both integers are well formed and it is their ratio that
// fails, so pointing at either would send the user to fix a number that is not wrong.
TEST(CustomWedgeInput, AMessageNamesTheSlotTheOwnerBlamedAndNoOther) {
  for (const Case& c : kCases) {
    SCOPED_TRACE(testing::Message() << c.what);
    const gui::CustomWedgeInputFeedback fb = gui::EvaluateCustomWedgeInput(c.h, c.k, c.l);
    if (c.expect_state != LUMICE_MILLER_INVALID) {
      continue;
    }
    EXPECT_EQ(NamesSlot(fb.message, 'h'), c.expect_slot == kSlotH) << fb.message;
    EXPECT_EQ(NamesSlot(fb.message, 'k'), c.expect_slot == kSlotK) << fb.message;
    EXPECT_EQ(NamesSlot(fb.message, 'l'), c.expect_slot == kSlotL) << fb.message;
  }
}

// h = 0 is a correct thing to type. It says "this side has no pyramidal cap", which the owner
// reports as its own state rather than as an error — so the row must not scold the user for it.
// The wording is the only place that distinction reaches them.
TEST(CustomWedgeInput, NoConeReadsAsAnAnswerNotAsAnError) {
  const gui::CustomWedgeInputFeedback fb = gui::EvaluateCustomWedgeInput(0, 0, 1);
  ASSERT_EQ(fb.state, LUMICE_MILLER_NO_CONE);
  EXPECT_FALSE(fb.can_apply);
  const std::string lowered = Lowered(fb.message);
  EXPECT_EQ(lowered.find("invalid"), std::string::npos) << fb.message;
  EXPECT_EQ(lowered.find("error"), std::string::npos) << fb.message;
  EXPECT_EQ(lowered.find("illegal"), std::string::npos) << fb.message;
}

// The mapping is keyed on the (state, invalid_index) pair, so each distinguishable verdict has to
// come out as its own sentence. Collapsing two of them to one string would still pass every check
// above while telling the user less than the owner knows.
TEST(CustomWedgeInput, DistinctVerdictsProduceDistinctMessages) {
  std::set<std::string> messages;
  for (const Case& c : kCases) {
    const gui::CustomWedgeInputFeedback fb = gui::EvaluateCustomWedgeInput(c.h, c.k, c.l);
    if (c.expect_state == LUMICE_MILLER_VALID) {
      continue;  // All of these are empty by the contract asserted above.
    }
    messages.insert(fb.message);
  }
  // NO_CONE, k != 0, h < 0, l < 0, and the ratio verdict (both ratio rows share one message).
  EXPECT_EQ(messages.size(), 5u);
}

}  // namespace
