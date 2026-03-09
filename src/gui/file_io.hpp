#ifndef LUMICE_GUI_FILE_IO_HPP
#define LUMICE_GUI_FILE_IO_HPP

#include <string>

namespace lumice::gui {

struct GuiState;

// Serialize GuiState to JSON string (Lumice config format)
std::string SerializeToJson(const GuiState& state);

// Deserialize JSON string to GuiState, returns true on success
bool DeserializeFromJson(const std::string& json_str, GuiState& state);

// File dialog wrappers (return empty string on cancel)
std::string ShowOpenDialog();
std::string ShowSaveDialog();

// Read entire file to string
bool ReadFileToString(const std::string& path, std::string& out);

// Write string to file
bool WriteStringToFile(const std::string& path, const std::string& content);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FILE_IO_HPP
