#ifndef LUMICE_GUI_FILE_IO_HPP
#define LUMICE_GUI_FILE_IO_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

struct GuiState;
struct PreviewViewport;
class PreviewRenderer;

// Fill LUMICE_Config C struct from GuiState (for LUMICE_CommitConfigStruct, bypasses JSON serialization)
void FillLumiceConfig(const GuiState& state, LUMICE_Config* out);

// Serialize GuiState to Core JSON string (for .lmc save and CLI compatibility)
std::string SerializeCoreConfig(const GuiState& state);

// Deserialize Core JSON string to GuiState, returns true on success
bool DeserializeFromJson(const std::string& json_str, GuiState& state);

// Serialize GuiState to full JSON (for .lmc file, preserves all frontend params)
std::string SerializeGuiStateJson(const GuiState& state);

// Deserialize full GuiState JSON, returns true on success
bool DeserializeGuiStateJson(const std::string& json_str, GuiState& state);

// Save .lmc binary file
bool SaveLmcFile(const std::filesystem::path& path, const GuiState& state, const PreviewRenderer& preview,
                 bool save_texture);

// Load .lmc binary file. If texture data is present, returns it via tex_data/tex_w/tex_h.
bool LoadLmcFile(const std::filesystem::path& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h);

// Export preview as PNG (renders via FBO, must be called on GL thread)
bool ExportPreviewPng(const std::filesystem::path& path, PreviewRenderer& renderer, const PreviewViewport& vp);

// Export equirect panorama as PNG (pure I/O, accepts pre-converted RGB data)
bool ExportEquirectPng(const std::filesystem::path& path, const unsigned char* data, int width, int height);

// Export configuration as JSON (CLI-compatible format)
bool ExportConfigJson(const std::filesystem::path& path, const std::string& json_str);

// File dialog wrappers (return empty path on cancel)
std::filesystem::path ShowOpenDialog();
std::filesystem::path ShowSaveDialog();
std::filesystem::path ShowExportPngDialog();
std::filesystem::path ShowExportEquirectDialog();
std::filesystem::path ShowExportJsonDialog();
std::filesystem::path ShowOpenImageDialog();


// Parse dash/comma-separated raypath text into face indices (tolerant: skips invalid tokens).
std::vector<int> ParseRaypathText(const std::string& text);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FILE_IO_HPP
