// Composition chain: what a FAILED .lmc load leaves behind.
//
// Units in the chain: file_io's binary container reader × gui_state's JSON deserializer × the
// document the caller handed in.
//
// What the collaboration produces that is observable: after a load that reports failure, the
// document on screen is still the document that was on screen before. That is not what the two
// units produced together before this suite existed — the JSON section is deserialized straight
// into the caller's GuiState (and DeserializeGuiStateJson opens with `state = GuiState{}`), while
// the texture section is read afterwards and has three ways to fail. Between those two facts sits
// a window where the loader returns false having already replaced the whole document. The caller
// (DoOpen) reads that false and skips current_file_path / dirty / run_intent / ResetFrontendState,
// so the end state is the new file's contents wearing the old file's name, with an error toast on
// top. Nothing about that looks like a failed open.
//
// The three texture failures are pinned one by one rather than as a representative sample: they
// are three separate `return false` statements, and a fix that only moves one of them out of the
// window would pass a single-case suite.
//
// The fourth failure case (a corrupt JSON section) is a PIN, not a regression probe — see its
// comment. It was already atomic before this suite; what it stops is a future change that makes
// it not be.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {
namespace {

// Byte offsets into the .lmc header. The single authority for this layout is the comment block
// above kLmcMagic in src/gui/file_io.cpp ("Header: 44 bytes, little-endian / magic[4] / version:
// uint32 / flags: uint32 / json_offset: uint64 / json_size: uint64 / tex_offset: uint64 /
// tex_size: uint64"); the constants themselves are file-static there, so a test that needs to
// corrupt one section without disturbing the others has to restate the offsets. If the layout ever
// changes, kLmcVersion is bumped in the same edit (file_io.cpp calls that bump mandatory) and
// these three constants have to follow — otherwise the corruptions below land on the wrong bytes
// and these cases quietly stop testing what they name.
constexpr size_t kJsonOffsetField = 12;
constexpr size_t kJsonSizeField = 20;
constexpr size_t kTexOffsetField = 28;
constexpr size_t kTexSizeField = 36;

// The 4x4 RGB pattern that goes into the texture section. Values are arbitrary but non-uniform, so
// a successful load that returns the wrong pixels does not look like a successful load.
std::vector<unsigned char> TexturePixels() {
  std::vector<unsigned char> px(4 * 4 * 3);
  for (size_t i = 0; i < px.size(); ++i) {
    px[i] = static_cast<unsigned char>((i * 7 + 11) & 0xFF);
  }
  return px;
}

// A document carrying one crystal, reachable from the serializer.
//
// The crystal has to hang off a layer entry: the .lmc JSON has no top-level crystal array — each
// entry embeds its own crystal inline, and GuiState::crystals is the runtime ID pool the entries
// index into. A crystal with no entry pointing at it serializes to nothing at all, which was
// measured here the direct way: a first draft set only crystals[0].height and the two documents
// below came out byte-identical, so the whole suite failed its own premise assertion.
//
// sun.altitude is set alongside it so the two documents differ in a scalar as well as inside the
// layer tree — a rollback that restored the vectors but not the scalars (or the reverse) is then
// not a pass.
GuiState DocumentWith(float crystal_height, float sun_altitude) {
  GuiState s{};
  s.sun.altitude = sun_altitude;
  s.crystals.emplace_back();
  s.crystals[0].height = ShapeDist{ ShapeDistType::kUniform, crystal_height, 0.5f };
  Layer layer;
  layer.entries.emplace_back();  // crystal_id 0
  s.layers.push_back(std::move(layer));
  return s;
}

// "The document currently open in the app." current_file_path and dirty are the two fields that
// carry the defect most directly: neither is written to the .lmc JSON at all, so the ONLY thing in
// a load that can change them is `state = GuiState{}` running. If they come back changed, the
// document was overwritten — there is no other path.
GuiState SentinelDocument() {
  GuiState s = DocumentWith(/*crystal_height=*/9.5f, /*sun_altitude=*/42.0f);
  s.current_file_path = "/sentinel/current-document.lmc";
  s.dirty = true;
  return s;
}

// "The contents of the file being opened." Differs from the sentinel in fields that ARE
// serialized, so the JSON comparison in ExpectLoadFailsAndDocumentUnchanged can tell the two
// documents apart.
GuiState FileDocument() {
  return DocumentWith(/*crystal_height=*/2.5f, /*sun_altitude=*/11.0f);
}

struct TempFile {
  std::filesystem::path path;
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path, ec);  // best-effort: a teardown failure must not fail the case
  }
};

std::filesystem::path TempPath(const char* name) {
  return std::filesystem::temp_directory_path() / name;
}

std::vector<unsigned char> ReadAllBytes(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  return std::vector<unsigned char>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

bool WriteAllBytes(const std::filesystem::path& path, const std::vector<unsigned char>& bytes) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }
  out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  return out.good();
}

// Native-endian memcpy, matching how the production writer emits these fields (WriteU64 memcpy-s
// the raw uint64). A cross-endian .lmc is not a case the format supports on either side.
uint64_t ReadU64Field(const std::vector<unsigned char>& bytes, size_t offset) {
  uint64_t v = 0;
  std::memcpy(&v, bytes.data() + offset, sizeof(v));
  return v;
}

void WriteU64Field(std::vector<unsigned char>& bytes, size_t offset, uint64_t value) {
  std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

// Produce a real, valid .lmc through the production writer rather than hand-assembling a header.
// The corruptions below then edit one field of a file that the shipping code wrote, so a change to
// the writer cannot leave these cases testing a format the loader no longer reads.
bool WriteValidLmc(const std::filesystem::path& path, bool with_texture) {
  PreviewRenderer preview;  // no Init(): UpdateCpuTextureData and the getters touch no GL
  if (with_texture) {
    const std::vector<unsigned char> px = TexturePixels();
    preview.UpdateCpuTextureData(px.data(), 4, 4);
  }
  return SaveLmcFile(path, FileDocument(), preview, with_texture);
}

// The shared assertion. Every failure case ends here, so "the document is untouched" is stated
// once and means the same thing in all four.
void ExpectLoadFailsAndDocumentUnchanged(const std::filesystem::path& path, const char* which) {
  SCOPED_TRACE(which);

  // Premise: the two documents must actually be distinguishable, or the JSON comparison below
  // passes for the wrong reason.
  ASSERT_NE(SerializeGuiStateJson(SentinelDocument()), SerializeGuiStateJson(FileDocument()))
      << "premise broken: the sentinel and the file serialize identically, so this case cannot "
         "observe an overwrite at all";

  GuiState current = SentinelDocument();
  std::vector<unsigned char> tex_data;
  int tex_w = 0;
  int tex_h = 0;

  EXPECT_FALSE(LoadLmcFile(path, current, tex_data, tex_w, tex_h))
      << "premise broken: this file was supposed to be unloadable, so the corruption did not take";

  EXPECT_EQ(current.current_file_path, SentinelDocument().current_file_path)
      << "a failed load cleared the path of the document that is still open";
  EXPECT_TRUE(current.dirty) << "a failed load cleared the dirty flag of the document that is still open";

  // Whole-document comparison, not a handful of probes: any serialized field that moved shows up
  // here, and gtest prints both strings, so a red run names the field instead of saying "false is
  // not true". The sentinel's 9.5 against the file's 2.5 is what the diff reads as.
  EXPECT_EQ(SerializeGuiStateJson(current), SerializeGuiStateJson(SentinelDocument()))
      << "a failed load replaced the open document with the contents of the file it could not load";
}

// --- The three texture-section failures ------------------------------------------------------
//
// All three keep the JSON section intact and valid, so the loader gets all the way past
// DeserializeGuiStateJson before it fails. That is the window this task exists to close.

TEST(LmcLoadAtomicityChain, ZeroTexSizeWithFlagSetRollsBackWithoutTouchingCurrentDocument) {
  TempFile f{ TempPath("lumice_lmc_atomicity_zero_tex_size.lmc") };
  ASSERT_TRUE(WriteValidLmc(f.path, /*with_texture=*/true));

  std::vector<unsigned char> bytes = ReadAllBytes(f.path);
  ASSERT_GT(bytes.size(), kTexSizeField + 8u);
  ASSERT_NE(ReadU64Field(bytes, kTexSizeField), 0u) << "premise: the file was written with a texture";
  WriteU64Field(bytes, kTexSizeField, 0);  // has_texture flag left set — that is the inconsistency
  ASSERT_TRUE(WriteAllBytes(f.path, bytes));

  ExpectLoadFailsAndDocumentUnchanged(f.path, "tex_size == 0 with the has_texture flag set");
}

TEST(LmcLoadAtomicityChain, TruncatedTextureSectionRollsBackWithoutTouchingCurrentDocument) {
  TempFile f{ TempPath("lumice_lmc_atomicity_truncated_tex.lmc") };
  ASSERT_TRUE(WriteValidLmc(f.path, /*with_texture=*/true));

  std::vector<unsigned char> bytes = ReadAllBytes(f.path);
  ASSERT_GT(bytes.size(), kTexSizeField + 8u);
  const uint64_t tex_offset = ReadU64Field(bytes, kTexOffsetField);
  const uint64_t tex_size = ReadU64Field(bytes, kTexSizeField);
  ASSERT_GT(tex_size, 0u);
  ASSERT_EQ(bytes.size(), tex_offset + tex_size) << "premise: the texture section is the tail of the file";
  bytes.resize(bytes.size() - 1);  // one byte short — header still claims the full section
  ASSERT_TRUE(WriteAllBytes(f.path, bytes));

  ExpectLoadFailsAndDocumentUnchanged(f.path, "texture section truncated by one byte");
}

TEST(LmcLoadAtomicityChain, CorruptTexturePngRollsBackWithoutTouchingCurrentDocument) {
  TempFile f{ TempPath("lumice_lmc_atomicity_corrupt_png.lmc") };
  ASSERT_TRUE(WriteValidLmc(f.path, /*with_texture=*/true));

  std::vector<unsigned char> bytes = ReadAllBytes(f.path);
  const uint64_t tex_offset = ReadU64Field(bytes, kTexOffsetField);
  const uint64_t tex_size = ReadU64Field(bytes, kTexSizeField);
  ASSERT_GT(tex_size, 0u);
  ASSERT_GE(bytes.size(), tex_offset + tex_size);
  // Same length, so the section still READS in full; only the decode fails. This isolates the
  // third `return false` from the second one.
  for (uint64_t i = 0; i < tex_size; ++i) {
    bytes[static_cast<size_t>(tex_offset + i)] = 0xAB;
  }
  ASSERT_TRUE(WriteAllBytes(f.path, bytes));

  ExpectLoadFailsAndDocumentUnchanged(f.path, "texture section is not a decodable PNG");
}

// --- The JSON-section failure: a PIN, not a regression probe ----------------------------------
//
// This one was already atomic before the fix, and deliberately so is worth stating: within
// DeserializeGuiStateJson the ONLY `return false` is the catch around json::parse, and it sits
// ABOVE the `state = GuiState{}` line. So a JSON section that will not parse never reaches the
// overwrite. (The plan for this task assumed otherwise; reading the function settled it.)
//
// What this case buys is the ordering itself. "Return before you reset" is a property of two
// adjacent statements with nothing enforcing their order — move the reset above the try/catch, or
// add a second `return false` below it, and the atomicity this task just established for the
// texture section silently reopens on the JSON section. Green here today, red the day that
// happens.
//
// The payload is non-JSON bytes rather than well-formed-but-empty JSON on purpose: `{}` parses
// fine, so DeserializeGuiStateJson would return TRUE on it and the load would SUCCEED (into an
// empty document). That is a different proposition and not a failure path at all.
TEST(LmcLoadAtomicityChain, CorruptJsonPayloadRollsBackWithoutTouchingCurrentDocument) {
  TempFile f{ TempPath("lumice_lmc_atomicity_corrupt_json.lmc") };
  ASSERT_TRUE(WriteValidLmc(f.path, /*with_texture=*/true));

  std::vector<unsigned char> bytes = ReadAllBytes(f.path);
  const uint64_t json_offset = ReadU64Field(bytes, kJsonOffsetField);
  const uint64_t json_size = ReadU64Field(bytes, kJsonSizeField);
  ASSERT_GT(json_size, 0u);
  ASSERT_GE(bytes.size(), json_offset + json_size);
  // Same length, and the texture section stays valid, so the only thing wrong with this file is
  // that its JSON does not parse.
  for (uint64_t i = 0; i < json_size; ++i) {
    bytes[static_cast<size_t>(json_offset + i)] = '~';
  }
  ASSERT_TRUE(WriteAllBytes(f.path, bytes));

  ExpectLoadFailsAndDocumentUnchanged(f.path, "JSON section does not parse");
}

// --- The success path -------------------------------------------------------------------------
//
// Both texture modes, because "never touch the document" is satisfiable by a loader that never
// loads anything. These assert the replacement still happens, including the two fields the
// failure cases assert do NOT move.
//
// Field-level assertions rather than a whole-document JSON comparison against FileDocument(): the
// round trip's field-by-field fidelity is test_document_roundtrip_chain's subject, and restating
// it here would make this suite red for that suite's reasons.

TEST(LmcLoadAtomicityChain, SuccessfulLoadWithTextureFullyReplacesTheDocument) {
  TempFile f{ TempPath("lumice_lmc_atomicity_success_tex.lmc") };
  ASSERT_TRUE(WriteValidLmc(f.path, /*with_texture=*/true));

  GuiState current = SentinelDocument();
  std::vector<unsigned char> tex_data;
  int tex_w = 0;
  int tex_h = 0;
  ASSERT_TRUE(LoadLmcFile(f.path, current, tex_data, tex_w, tex_h));

  ASSERT_EQ(current.crystals.size(), 1u);
  EXPECT_FLOAT_EQ(current.crystals[0].height.center, 2.5f) << "the file's document did not land";
  EXPECT_FLOAT_EQ(current.sun.altitude, 11.0f) << "the file's document did not land";
  EXPECT_TRUE(current.current_file_path.empty()) << "the sentinel's path survived a successful load";
  EXPECT_FALSE(current.dirty) << "the sentinel's dirty flag survived a successful load";

  EXPECT_EQ(tex_w, 4);
  EXPECT_EQ(tex_h, 4);
  EXPECT_EQ(tex_data, TexturePixels());
}

TEST(LmcLoadAtomicityChain, SuccessfulLoadWithoutTextureFullyReplacesTheDocument) {
  TempFile f{ TempPath("lumice_lmc_atomicity_success_no_tex.lmc") };
  ASSERT_TRUE(WriteValidLmc(f.path, /*with_texture=*/false));

  GuiState current = SentinelDocument();
  std::vector<unsigned char> tex_data;
  int tex_w = 0;
  int tex_h = 0;
  ASSERT_TRUE(LoadLmcFile(f.path, current, tex_data, tex_w, tex_h));

  ASSERT_EQ(current.crystals.size(), 1u);
  EXPECT_FLOAT_EQ(current.crystals[0].height.center, 2.5f) << "the file's document did not land";
  EXPECT_FLOAT_EQ(current.sun.altitude, 11.0f) << "the file's document did not land";
  EXPECT_TRUE(current.current_file_path.empty()) << "the sentinel's path survived a successful load";
  EXPECT_FALSE(current.dirty) << "the sentinel's dirty flag survived a successful load";
  EXPECT_TRUE(tex_data.empty());
}

}  // namespace
}  // namespace lumice::gui
