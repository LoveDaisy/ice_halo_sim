// The preview's GPU texture and the CPU mirror the document is written from.
//
// What this suite is for. PreviewRenderer holds the rendered sky twice: once in a GL texture, which
// is what the user sees, and once in a CPU-side byte buffer, which is what SaveLmcFile writes into
// the document. Nothing keeps those two in step except the setters that write them, and the
// failure they invite is silent in both directions — the screen keeps showing the right picture
// while the file gets an older one, or the file gets the right bytes under the wrong dimensions.
//
// Why here and not in the windowless target. Every case below has to seed the renderer through
// UploadTexture, which is a glTexImage2D call and returns without doing anything at all when there
// is no texture object (preview_renderer.cpp's first guard). The dimension case in particular
// cannot be written any other way: the defect it guards is UpdateCpuTextureData leaving behind the
// width and height that a PREVIOUS UploadTexture set, and UpdateCpuTextureData is the only other
// writer of those fields, so without GL there is no way to put a stale value there to begin with.
//
// Deliberately NOT here. Which document fields survive a save/load is
// composition-correctness/gui/test_document_roundtrip_chain.cpp — that layer works on GuiState and
// never touches the texture. What a document switch is required to clear from the preview is
// test/gui/functional/test_file_ops.cpp. Whether the saved frame LOOKS like the live one is
// test/gui/visual/test_preview_pixels.cpp.
//
// What a user sees when these break: they run a simulation, save, reopen, and get the halo from
// two runs ago — or a file that will not open at all because its declared size does not match its
// payload.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// GL uploads have to happen on the main thread; a TestFunc runs on the test engine's coroutine.
void UploadOnMainThreadGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_capture.capture_requested && !g_capture.capture_done) {
    gui::g_preview.UploadTexture(g_capture.pixels.data(), g_capture.width, g_capture.height);
    g_capture.capture_done = true;
  }
}

// Hand `pixels` to the renderer and return once the upload has landed.
void UploadAndWait(ImGuiTestContext* ctx, const std::vector<unsigned char>& pixels, int w, int h) {
  g_capture.Reset();
  g_capture.pixels = pixels;
  g_capture.width = w;
  g_capture.height = h;
  g_capture.capture_requested = true;
  ctx->Yield(2);
  IM_CHECK(g_capture.capture_done);
}

// A synthetic RGB frame whose value at every pixel is a function of its coordinates. The point is
// that no two pixels share a value pattern: a mirror that dropped a row, transposed the image, or
// wrote the first row everywhere would still be the right SIZE, and only per-pixel content says so.
std::vector<unsigned char> CoordinatePattern(int w, int h, int seed) {
  std::vector<unsigned char> tex(static_cast<size_t>(w) * h * 3);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const size_t idx = (static_cast<size_t>(y) * w + x) * 3;
      tex[idx + 0] = static_cast<unsigned char>((x * 7 + y * 3 + seed) % 256);
      tex[idx + 1] = static_cast<unsigned char>((x * 11 + y * 5 + seed) % 256);
      tex[idx + 2] = static_cast<unsigned char>(((x ^ y) * 13 + seed) % 256);
    }
  }
  return tex;
}

// Byte equality with a diagnostic that says how far off it was, since "not equal" on a quarter of a
// megabyte is not an answer anyone can act on. PSNR separates "one channel is swapped" (finite,
// low) from "the buffer is a different image entirely".
void ExpectSamePixels(const char* tag, const unsigned char* actual, const std::vector<unsigned char>& expected, int w,
                      int h) {
  const bool match = std::memcmp(actual, expected.data(), expected.size()) == 0;
  if (!match) {
    fprintf(stderr, "[preview_texture] %s: mismatch, PSNR = %.2f dB\n", tag,
            lumice::test::ComputePsnr(actual, expected.data(), w, h, 3));
  }
  IM_CHECK(match);
}

}  // namespace

void RegisterPreviewTextureTests(ImGuiTestEngine* engine) {
  // The mirror is a copy, not a re-read. UploadTexture stores the caller's bytes directly, and this
  // is the case that says the stored copy is the same bytes rather than, say, the GL texture read
  // back through a format conversion.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_texture", "an_uploaded_frame_is_mirrored_byte_for_byte");
    t->GuiFunc = UploadOnMainThreadGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      constexpr int kW = 256;
      constexpr int kH = 256;
      const std::vector<unsigned char> frame = CoordinatePattern(kW, kH, /*seed=*/0);

      UploadAndWait(ctx, frame, kW, kH);

      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), kW);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), kH);
      const unsigned char* mirror = gui::g_preview.GetTextureData();
      IM_CHECK(mirror != nullptr);
      ExpectSamePixels("upload_mirror", mirror, frame, kW, kH);
    };
  }

  // The full trip the mirror exists for: upload, save with the frame baked in, clear, load, and
  // upload what came back. The re-upload at the end is not decoration — it is what makes this
  // assert that the loaded bytes reach the RENDERER, rather than only that LoadLmcFile handed back
  // a vector. PNG is lossless, so anything short of byte equality is a defect and not a tolerance
  // question.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_texture", "the_saved_document_brings_the_frame_back_unchanged");
    t->GuiFunc = UploadOnMainThreadGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      constexpr int kW = 256;
      constexpr int kH = 256;
      const std::vector<unsigned char> original = CoordinatePattern(kW, kH, /*seed=*/17);

      UploadAndWait(ctx, original, kW, kH);
      IM_CHECK(gui::g_preview.HasTexture());

      const std::string tmp_path = GuiTestTempPath("lumice_preview_texture_roundtrip.lmc").string();
      IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*include_texture=*/true));

      gui::DoNew();
      IM_CHECK(!gui::g_preview.HasTexture());

      std::vector<unsigned char> loaded;
      int loaded_w = 0;
      int loaded_h = 0;
      bool loaded_radiance_only = false;
      IM_CHECK(gui::LoadLmcFile(tmp_path, gui::g_state, loaded, loaded_w, loaded_h, loaded_radiance_only));
      IM_CHECK_EQ(loaded_w, kW);
      IM_CHECK_EQ(loaded_h, kH);
      IM_CHECK(!loaded.empty());

      UploadAndWait(ctx, loaded, loaded_w, loaded_h);
      IM_CHECK(gui::g_preview.HasTexture());
      const unsigned char* mirror = gui::g_preview.GetTextureData();
      IM_CHECK(mirror != nullptr);
      ExpectSamePixels("lmc_roundtrip", mirror, original, kW, kH);

      std::remove(tmp_path.c_str());
    };
  }

  // The dimensions belong to whichever writer wrote the bytes. RefreshCpuTextureForSave() calls
  // UpdateCpuTextureData just before a save, at whatever resolution the server's snapshot happens
  // to be — which is not always the resolution of the last UploadTexture. When those fields were
  // left behind, SaveLmcFile declared the OLD size over the NEW payload, and the document was
  // written malformed with nothing complaining at save time.
  //
  // The two sizes differ AND the payloads differ, deliberately: a check on size alone would pass on
  // a writer that carried the dimensions across but kept the stale pixels.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_texture", "a_cpu_side_refresh_carries_its_own_dimensions_into_the_document");
    t->GuiFunc = UploadOnMainThreadGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Step 1: an ordinary upload, which is what puts a size in the renderer to go stale.
      constexpr int kOldW = 256;
      constexpr int kOldH = 256;
      UploadAndWait(ctx, CoordinatePattern(kOldW, kOldH, /*seed=*/3), kOldW, kOldH);
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), kOldW);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), kOldH);

      // Step 2: the pre-save refresh, at a different size.
      constexpr int kNewW = 128;
      constexpr int kNewH = 128;
      const std::vector<unsigned char> refreshed = CoordinatePattern(kNewW, kNewH, /*seed=*/61);
      gui::g_preview.UpdateCpuTextureData(refreshed.data(), kNewW, kNewH);
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), kNewW);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), kNewH);

      // Step 3: what actually lands in the document.
      const std::string tmp_path = GuiTestTempPath("lumice_preview_texture_dims.lmc").string();
      IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*include_texture=*/true));

      gui::GuiState loaded_state;
      std::vector<unsigned char> loaded;
      int loaded_w = 0;
      int loaded_h = 0;
      bool loaded_radiance_only = false;
      IM_CHECK(gui::LoadLmcFile(tmp_path, loaded_state, loaded, loaded_w, loaded_h, loaded_radiance_only));
      IM_CHECK_EQ(loaded_w, kNewW);
      IM_CHECK_EQ(loaded_h, kNewH);
      IM_CHECK_EQ(loaded.size(), static_cast<size_t>(kNewW) * kNewH * 3);
      ExpectSamePixels("refresh_dimensions", loaded.data(), refreshed, kNewW, kNewH);

      std::remove(tmp_path.c_str());
    };
  }
}
