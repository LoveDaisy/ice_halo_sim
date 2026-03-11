#ifndef LUMICE_GUI_FILE_IO_HPP
#define LUMICE_GUI_FILE_IO_HPP

#include <string>
#include <vector>

namespace lumice::gui {

struct GuiState;
class PreviewRenderer;

// Serialize GuiState to Core JSON format (for LUMICE_CommitConfig)
std::string SerializeCoreConfig(const GuiState& state);

// Deserialize Core JSON string to GuiState, returns true on success
bool DeserializeFromJson(const std::string& json_str, GuiState& state);

// Serialize GuiState to full JSON (for .lmc file, preserves all frontend params)
std::string SerializeGuiStateJson(const GuiState& state);

// Deserialize full GuiState JSON, returns true on success
bool DeserializeGuiStateJson(const std::string& json_str, GuiState& state);

// Save .lmc binary file
bool SaveLmcFile(const std::string& path, const GuiState& state, const PreviewRenderer& preview, bool save_texture);

// Load .lmc binary file. If texture data is present, returns it via tex_data/tex_w/tex_h.
bool LoadLmcFile(const std::string& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h);

// File dialog wrappers (return empty string on cancel)
std::string ShowOpenDialog();
std::string ShowSaveDialog();

// Read entire file to string
bool ReadFileToString(const std::string& path, std::string& out);

// Write string to file
bool WriteStringToFile(const std::string& path, const std::string& content);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FILE_IO_HPP
