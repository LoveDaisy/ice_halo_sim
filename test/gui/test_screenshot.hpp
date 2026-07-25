#ifndef LUMICE_TEST_GUI_SCREENSHOT_HPP
#define LUMICE_TEST_GUI_SCREENSHOT_HPP

#include <string>
#include <vector>

// Forward declare GL types to avoid GL header dependency in the header
using GLuint = unsigned int;

namespace lumice::test {

// Read RGBA pixels from a GL_TEXTURE_2D texture.
// Returns pixel data with Y-axis flipped (top-to-bottom row order for image files).
// Saves/restores GL_TEXTURE_BINDING_2D state.
// Note: glGetTexImage is deprecated on macOS but still functional.
// Fallback: bind FBO + glReadPixels if needed.
std::vector<unsigned char> ReadTexturePixels(GLuint tex_id, int w, int h);

// Save RGBA/RGB pixel data to a PNG file.
// Returns true on success.
bool SavePng(const char* path, const unsigned char* data, int w, int h, int channels);

// Load a PNG file into pixel data.
// Returns true on success; sets w, h, channels.
bool LoadPng(const char* path, std::vector<unsigned char>& data, int& w, int& h, int& channels);

// Compute PSNR between two images of the same dimensions.
// Returns PSNR in dB. Returns infinity if images are identical.
// Returns -1.0 if dimensions don't match or data is null.
double ComputePsnr(const unsigned char* img1, const unsigned char* img2, int w, int h, int channels);

// Strip alpha channel from RGBA pixel data, returning RGB.
std::vector<unsigned char> StripAlpha(const unsigned char* rgba, int width, int height);

// Compare the capture saved at tmp_path against the reference image at ref_path.
// Returns true when the capture matches within threshold; false (with a stderr diagnostic)
// when the reference is missing, the capture cannot be read, dimensions/channels disagree,
// or PSNR is below threshold.
//
// group/tag are the "[<group>] <tag>:" stderr prefix; scripts/regen_gui_test_refs.py parses
// PSNR lines by that prefix, so a reference group's <group> must match its registry key there.
// keep_capture_png=false deletes tmp_path on success (pass the binary's --keep-export-png
// flag through so the regen driver can collect per-run PNGs).
bool CheckAgainstReference(const char* group, const char* tag, const std::string& tmp_path, const std::string& ref_path,
                           double threshold, bool keep_capture_png);

}  // namespace lumice::test

#endif  // LUMICE_TEST_GUI_SCREENSHOT_HPP
