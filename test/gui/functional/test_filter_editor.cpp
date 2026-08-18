// The document inspector's Filter tab: the sum-of-products row editor and the controls beside it.
//
// What this suite is for: the row editor is a buffer, not the document. Rows are authored into a
// scratch list, validated per row, and materialised into the entry's FilterConfig by the page's
// per-frame commit — which every case below relies on, since there is no longer an OK to press.
// Falsifying any of it needs the real editor on screen and real typing: the
// buffers are file-scope statics inside edit_modals.cpp with no seam to write them from a test,
// which is exactly the point (a test that could set them directly would not be testing the
// editor). Where these cases used to open a modal from a card's Edit button and dismiss it with
// OK or Cancel, they now select the entry and let the page commit — see OpenFilterTab. What the grammar means once
// committed is settled elsewhere and deliberately not re-asserted here — ValidateSummandText in
// unit-correctness/gui/test_filter_sop_grammar.cpp, and the reconstruction and clause arithmetic in
// composition-correctness/gui/test_filter_reconstruct_chain.cpp.
//
// What a user sees when these break: a row that cannot commit silently reaching the document
// anyway; deleting one OR row silently editing a different one; Remove Filter appearing to work
// and the filter coming back on the next save; a forgotten blank row turning a filter into a
// match-all, which reads as "my filter does nothing" under filter_in and as an all-black render
// under filter_out.
//
// Five propositions from the catalogue are NOT covered here and the reason is mechanical, the
// same one recorded for the Colors window: the per-row validation hints, the static syntax hint,
// the live SoP preview and the "Clauses: N / M" line are all drawn with TextColored /
// TextWrapped / TextDisabled / TextUnformatted, which reach ItemAdd() with id == 0 and are never
// registered with the test engine; and the row's validation tint is a style colour, not an item
// flag, so ItemInfo cannot see it either. Their user-visible consequence IS covered, by the
// commit: the same per-row validation that drives the hints also decides whether a row reaches
// the document, so a validation regression surfaces here as a wrong FilterConfig even though the
// text does not. That surface used to be the OK button's disabled flag, which is gone with it.
// The clause count's own arithmetic is covered a layer down.

#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/edit_modal_rules.hpp"  // kMaxSummandRows — the cap the add-row button enforces
#include "gui/edit_modals.hpp"       // IsCurrentModalDApplicable
#include "gui/file_io.hpp"           // SaveLmcFile / LoadLmcFile
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

const char* const kAddRow = "**/+ Add OR row##summand_add";

// Put the inspector on the entry's Filter tab. The extra frames let the tab's selection settle
// before the rows below it are addressed.
void OpenFilterModal(ImGuiTestContext* ctx) {
  OpenFilterTab(ctx);
  ctx->Yield(4);
}

// Whether the editor is still on screen and rendering its Filter tab.
//
// This replaces OkIsDisabled, which asked whether the modal's OK button was greyed out. There is no
// OK: the page commits every frame, so "the commit is not blocked" is no longer a property of a
// button but of whether the row's text reached the document — which each case checks directly, on
// the document. What is left for this predicate is the other half those assertions carried: that
// the editor SURVIVED rendering whatever was just typed into it. The over-cap preview below is the
// case that needs it, since its warning branch pushes a style colour it must also pop.
bool EditorStillUp(ImGuiTestContext* ctx) {
  return InspectorItemExists(ctx, "**/###filter_tab");
}

// Give the entry a committed filter to edit, so a case can start from "there is something here"
// rather than from the blank editor.
void SeedRaypathFilter(const char* text) {
  gui::FilterConfig f;
  f.SetRaypath(gui::RaypathParams{ text });
  gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
}

const gui::FilterConfig* CommittedFilter() {
  const auto& entry = gui::g_state.layers[0].entries[0];
  return entry.filter_id.has_value() ? &gui::g_state.filters[static_cast<size_t>(*entry.filter_id)] : nullptr;
}

// Type `text` into row `uid`, adding a row first when the caller is past row 0.
void AuthorRow(ImGuiTestContext* ctx, int uid, const char* text) {
  if (uid > 0) {
    ctx->ItemClick(kAddRow);
    ctx->Yield(2);
  }
  ctx->ItemInputValue(("**/##row_text_" + std::to_string(uid)).c_str(), text);
  ctx->Yield(2);
}

}  // namespace

void RegisterFilterEditorTests(ImGuiTestEngine* engine) {
  // ---------------------------------------------------------------------------------------------
  // Validation: what a row that cannot commit does, and does not do.
  // ---------------------------------------------------------------------------------------------

  // The clause-count line is a diagnostic, not a gate — deliberately so. An over-cap row must not
  // block the edit: the ABI limit is enforced at commit, and turning the preview into a second
  // enforcement point would strand the user with an edit that will not take and an unreadable
  // reason. What is addressable here is that the editor kept rendering; the line itself is text
  // the engine never sees.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "filter_editor", "an_over_cap_clause_count_warns_without_blocking_the_edit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);

      // Four factors of nine alternatives each: 6561 clauses, well past LUMICE_MAX_CONFIG_CLAUSES.
      ctx->ItemInputValue("**/##row_text_0",
                          "1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4 & 1;2;3;4;5;6;7;8;3-4");
      ctx->Yield(3);
      // The editor survived rendering that preview — the over-cap branch pushes a style colour it
      // must also pop, and an unbalanced pair takes the whole page down with it.
      IM_CHECK(EditorStillUp(ctx));

      ctx->Yield(2);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // The row list.
  // ---------------------------------------------------------------------------------------------

  // The editor keeps at least one row, so the last row's delete button is greyed rather than
  // removed — removing it would reflow the row on every add and delete.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "the_last_row_cannot_be_deleted");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_0"));
      IM_CHECK(IsDisabled(InspectorItemInfo(ctx, "**/" ICON_FA_XMARK "##row_delete_0")));

      ctx->ItemClick(kAddRow);
      ctx->Yield(2);
      IM_CHECK(!IsDisabled(InspectorItemInfo(ctx, "**/" ICON_FA_XMARK "##row_delete_0")));
      IM_CHECK(!IsDisabled(InspectorItemInfo(ctx, "**/" ICON_FA_XMARK "##row_delete_1")));

      ctx->Yield(2);
    };
  }

  // Row widgets are keyed by a per-row uid rather than by position, which is what stops a
  // middle-row delete from re-colliding the ImGui id stack and pouring one row's buffer into
  // another's widget. Authoring distinct text into each row is what makes that observable: if the
  // ids shifted, the survivors' text would follow the wrong widget.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "deleting_a_middle_row_leaves_the_survivors_alone");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      AuthorRow(ctx, 0, "1-2");
      AuthorRow(ctx, 1, "3-4");
      AuthorRow(ctx, 2, "5-6");

      ctx->ItemClick("**/" ICON_FA_XMARK "##row_delete_1");  // the middle one
      ctx->Yield(2);
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_0"));
      IM_CHECK(!InspectorItemExists(ctx, "**/##row_text_1"));
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_2"));  // uid 2 did NOT slide down into slot 1

      ctx->Yield(2);
      const auto* f = CommittedFilter();
      IM_CHECK(f != nullptr);
      IM_CHECK_EQ(f->param.size(), static_cast<size_t>(2));
      IM_CHECK_STR_EQ(f->param[0].text.c_str(), "1-2");
      IM_CHECK_STR_EQ(f->param[1].text.c_str(), "5-6");
    };
  }

  // The soft cap was raised from 16 to 256, and the button must stay live across the old boundary
  // — that boundary is where a stale constant would still be hiding.
  //
  // Honest scope: this walks to 20 rows, not to the 256-row cap, so it falsifies "the old limit is
  // still in force" but not "the new limit is enforced". AtSummandRowCap is a pure predicate on
  // the row count and is the single owner of both answers; clicking 256 times to see the button
  // grey out would buy the second half at roughly ten times the runtime of every other case here.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "add_row_stays_live_past_the_old_sixteen_row_limit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      IM_CHECK(gui::kMaxSummandRows > 20u);  // the walk below must stay below the real cap

      std::string disabled_at;
      for (int i = 1; i < 20; ++i) {
        if (IsDisabled(InspectorItemInfo(ctx, kAddRow))) {
          disabled_at += " " + std::to_string(i);
        }
        ctx->ItemClick(kAddRow);
        ctx->Yield(2);
      }
      IM_CHECK_STR_EQ(disabled_at.c_str(), "");
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_16"));  // the 17th row exists at all
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_19"));
      IM_CHECK(!IsDisabled(InspectorItemInfo(ctx, kAddRow)));

      ctx->Yield(2);
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Committing what was authored.
  // ---------------------------------------------------------------------------------------------

  // Each row is one OR alternative and its text is stored verbatim — the editor does not
  // normalise, reorder or merge. Three rows of deliberately different grammar shapes, so a
  // regression that only survives the simplest one shows up.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "rows_commit_verbatim_one_summand_each");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      AuthorRow(ctx, 0, "3-5");
      AuthorRow(ctx, 1, "entry:2 & exit:4");
      AuthorRow(ctx, 2, "3-5 & entry:2");
      IM_CHECK(EditorStillUp(ctx));
      ctx->Yield(2);

      const auto* f = CommittedFilter();
      IM_CHECK(f != nullptr);
      IM_CHECK_EQ(f->param.size(), static_cast<size_t>(3));
      IM_CHECK_STR_EQ(f->param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(f->param[1].text.c_str(), "entry:2 & exit:4");
      IM_CHECK_STR_EQ(f->param[2].text.c_str(), "3-5 & entry:2");
    };
  }

  // A ';' inside one row is an alternative within that row's token, not a row separator. The
  // distinction is invisible until commit: fanning it out here would change the on-disk row count
  // and, on the next open, hand the user back a different editor state than they typed.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "a_semicolon_row_commits_as_a_single_row");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      ctx->ItemInputValue("**/##row_text_0", "1-3;3-5");
      ctx->Yield(2);
      IM_CHECK(EditorStillUp(ctx));
      ctx->Yield(2);

      const auto* f = CommittedFilter();
      IM_CHECK(f != nullptr);
      IM_CHECK_EQ(f->param.size(), static_cast<size_t>(1));
      IM_CHECK_STR_EQ(f->param[0].text.c_str(), "1-3;3-5");
      IM_CHECK_EQ(f->param[0].factors.size(), static_cast<size_t>(1));
    };
  }

  // Regression sentinel. A forgotten blank row used to materialise as a factor-less clause, which
  // the expansion turns into match-all: OR(3-5, everything). Under filter_in that is a filter that
  // silently does nothing; under filter_out it is an all-black render. Blank rows are stripped
  // before materialising, and if none survive there is no filter at all rather than a match-all one.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "a_blank_row_is_dropped_not_lowered_to_match_all");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      AuthorRow(ctx, 0, "3-5");
      ctx->ItemClick(kAddRow);
      ctx->Yield(2);
      ctx->Yield(2);

      const auto* f = CommittedFilter();
      IM_CHECK(f != nullptr);
      IM_CHECK_EQ(f->param.size(), static_cast<size_t>(1));
      IM_CHECK_STR_EQ(f->param[0].text.c_str(), "3-5");

      // And the degenerate case: blank out the only real row too. All rows blank means no filter,
      // not a filter that matches everything.
      OpenFilterModal(ctx);
      ctx->ItemInputValue("**/##row_text_0", "");
      ctx->Yield(2);
      ctx->Yield(2);
      IM_CHECK(CommittedFilter() == nullptr);
    };
  }

  // Reopening has to rebuild the row list from what was committed, or the second edit starts from
  // a stale buffer and silently discards whatever the first one wrote.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "reopening_rebuilds_the_rows_from_the_committed_filter");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      AuthorRow(ctx, 0, "3-5");
      AuthorRow(ctx, 1, "1-2");
      ctx->Yield(2);
      IM_CHECK(CommittedFilter() != nullptr);

      OpenFilterModal(ctx);
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_0"));
      IM_CHECK(InspectorItemExists(ctx, "**/##row_text_1"));
      IM_CHECK(!InspectorItemExists(ctx, "**/##row_text_2"));  // two rows back, not three or one

      // Edit the second row and commit again: the rebuilt buffer is the one being edited.
      ctx->ItemInputValue("**/##row_text_1", "4-6");
      ctx->Yield(2);
      ctx->Yield(2);
      const auto* f = CommittedFilter();
      IM_CHECK(f != nullptr);
      IM_CHECK_EQ(f->param.size(), static_cast<size_t>(2));
      IM_CHECK_STR_EQ(f->param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(f->param[1].text.c_str(), "4-6");
    };
  }

  // What the editor produced has to survive the document it goes into. The authoring half is why
  // this lives here rather than beside the other round-trip chains: only a real modal and real
  // typing produce the SoP being saved, and a serialisation that silently normalised the rows
  // would hand the user back a different editor on the next open.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "authored_rows_survive_a_save_and_reload");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      AuthorRow(ctx, 0, "3-5");
      AuthorRow(ctx, 1, "entry:2 & exit:4");
      AuthorRow(ctx, 2, "3-5 & entry:2");
      ctx->Yield(2);
      const auto* committed = CommittedFilter();
      IM_CHECK(committed != nullptr);
      const gui::FilterConfig original = *committed;

      const std::string tmp_path = GuiTestTempPath("lumice_filter_editor_roundtrip.lmc").string();
      IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*save_texture=*/false));

      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      IM_CHECK(gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h));

      const auto* reloaded = CommittedFilter();
      IM_CHECK(reloaded != nullptr);
      IM_CHECK(*reloaded == original);
      IM_CHECK_EQ(reloaded->param.size(), static_cast<size_t>(3));
      IM_CHECK_STR_EQ(reloaded->param[2].text.c_str(), "3-5 & entry:2");
      std::remove(tmp_path.c_str());
    };
  }

  // ---------------------------------------------------------------------------------------------
  // The controls beside the row list.
  // ---------------------------------------------------------------------------------------------

  // Two radios over one integer, so exclusivity is structural and not worth asserting — what is
  // worth asserting is that each radio writes ITS OWN value. Two call sites that both wrote 0
  // would look correct on the default and leave Filter Out permanently inert, which is a real
  // shape of copy-paste bug and the only one a frame can catch here.
  //
  // Asserted through the committed action rather than the widget's checked state: ImGui's
  // RadioButton never reports ImGuiItemStatusFlags_Checked to the test engine (unlike Checkbox),
  // so ctx->ItemIsChecked() cannot read one — the same registration gap that hides this modal's
  // hint text.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "each_action_radio_commits_its_own_value");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      OpenFilterModal(ctx);
      IM_CHECK(InspectorItemExists(ctx, "**/Filter In##filter_action"));
      IM_CHECK(InspectorItemExists(ctx, "**/Filter Out##filter_action"));

      ctx->ItemClick("**/Filter Out##filter_action");
      ctx->Yield(2);
      ctx->ItemInputValue("**/##row_text_0", "3-1-5");
      ctx->Yield(2);
      ctx->Yield(2);
      const auto* out = CommittedFilter();
      IM_CHECK(out != nullptr);
      IM_CHECK_EQ(out->action, 1);

      // And back: the In radio is not a no-op that merely leaves the default in place.
      OpenFilterModal(ctx);
      ctx->ItemClick("**/Filter In##filter_action");
      ctx->Yield(2);
      ctx->Yield(2);
      const auto* in = CommittedFilter();
      IM_CHECK(in != nullptr);
      IM_CHECK_EQ(in->action, 0);
    };
  }

  // D only means something for axis configurations that have the symmetry it names. When they do
  // not, the checkbox stays live but grows an explanation beside it — a hover target that exists
  // only in the inapplicable case, which is what makes the rule observable at all (the text it
  // carries is a tooltip, and tooltips are not addressable).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "filter_editor", "d_is_explained_when_the_axis_does_not_support_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct AxisCase {
        const char* label;
        gui::AxisDistType az_type;
        float az_std;
        float roll_mean;
        bool expect_applicable;
      };
      // Uniform-360 azimuth with a roll on the 30-degree lattice is the applicable case; break
      // either half and it stops being applicable. Both breakages are asserted because they are
      // two independent conditions in one predicate.
      const AxisCase kCases[] = {
        { "uniform_az_roll_0", gui::AxisDistType::kUniform, 360.0f, 0.0f, true },
        { "roll_off_lattice", gui::AxisDistType::kUniform, 360.0f, 15.0f, false },
        { "az_not_uniform", gui::AxisDistType::kGauss, 30.0f, 0.0f, false },
      };

      // Collected, then asserted once: a fatal assert in the loop would leave the later axis
      // shapes unevaluated, and each row here is an independent condition.
      std::string wrong_predicate;
      std::string wrong_icon;
      for (const auto& c : kCases) {
        ResetTestState();
        ctx->Yield(2);
        auto& crystal = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
        crystal.azimuth = { c.az_type, 0.0f, c.az_std };
        crystal.roll = { gui::AxisDistType::kUniform, c.roll_mean, 360.0f };
        ctx->Yield();
        OpenFilterModal(ctx);

        if (gui::IsCurrentModalDApplicable() != c.expect_applicable) {
          wrong_predicate += std::string(" ") + c.label;
        }
        // The user-visible half: the info icon appears exactly when D does not apply.
        const bool icon = InspectorItemExists(ctx, "**/" ICON_FA_CIRCLE_INFO "##d_tooltip_icon_filter_modal");
        if (icon == c.expect_applicable) {
          wrong_icon += std::string(" ") + c.label;
        }
        // D itself stays editable either way — it is explained, not taken away.
        if (IsDisabled(InspectorItemInfo(ctx, "**/D##filter_modal"))) {
          wrong_icon += std::string(" ") + c.label + ":disabled";
        }
        ctx->Yield(2);
      }
      IM_CHECK_STR_EQ(wrong_predicate.c_str(), "");
      IM_CHECK_STR_EQ(wrong_icon.c_str(), "");
    };
  }

  // ---------------------------------------------------------------------------------------------
  // Remove Filter — an action, where it used to be an intent.
  // ---------------------------------------------------------------------------------------------

  // Under the modal, Remove armed an intent that OK honoured regardless of what the rows said
  // afterwards, and Cancel was how you aborted it. With the per-frame commit there is no interval
  // between arming and honouring, so Remove is simply an action: it clears the filter on the spot,
  // and what the rows say next is a new authoring act rather than a contradiction of it. This case
  // pins both halves, because the first without the second would also pass if typing had been
  // wrongly swallowed.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "filter_editor", "remove_filter_applies_at_once_and_typing_authors_a_new_one");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      SeedRaypathFilter("3-1-5");
      ctx->Yield();
      IM_CHECK(CommittedFilter() != nullptr);
      OpenFilterModal(ctx);

      // Always enabled: it is an intent flag, not something derived from row emptiness.
      IM_CHECK(!IsDisabled(InspectorItemInfo(ctx, "**/Remove Filter##filter")));
      // InspectorItemInfo puts the scroll back where it found it, so the button it just located
      // may well be off screen again by the time the click runs — and a clipped item is invisible
      // to ItemClick. The scroll is part of the action, not a nicety.
      ScrollInspectorTo(ctx, "**/Remove Filter##filter");
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      // The row stays editable and the editor stays up — the intent bypasses row validation
      // entirely.
      IM_CHECK(!IsDisabled(InspectorItemInfo(ctx, "**/##row_text_0")));
      IM_CHECK(EditorStillUp(ctx));

      // Remove took effect on the spot; there is no pending removal for a later keystroke to
      // contradict.
      IM_CHECK(CommittedFilter() == nullptr);

      // ...and typing afterwards authors a NEW filter rather than being swallowed. This is the
      // opposite of what the modal did, and deliberately so: there, Remove was an INTENT held
      // until OK, so a keystroke arriving before OK had to lose to it or the user's "remove this"
      // would have been silently undone by their own next click. Here the removal already
      // happened, so the keystroke is not competing with anything — it is a new filter being
      // written, and swallowing it would leave the user typing into a row that never takes.
      ctx->ItemInputValue("**/##row_text_0", "1-2");
      ctx->Yield(3);
      IM_CHECK(CommittedFilter() != nullptr);
    };
  }
}
