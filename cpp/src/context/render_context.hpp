#ifndef SRC_CONTEXT_RENDER_CONTEXT_H_
#define SRC_CONTEXT_RENDER_CONTEXT_H_

#include <memory>

#include "core/core_def.hpp"
#include "core/crystal.hpp"
#include "io/json_util.hpp"
#include "io/serialize.hpp"

namespace icehalo {

enum class VisibleRange;

enum class ColorCompactLevel : int {
  kTrueColor = 24,  //!< True color, use all 24-bit
  kMonochrome = 8,  //!< Gray scale, 8-bit per channel
  kLowQuality = 4,  //!< 4-bit per channel
};

NLOHMANN_JSON_SERIALIZE_ENUM(ColorCompactLevel, {
                                                    { ColorCompactLevel::kTrueColor, "true_color" },
                                                    { ColorCompactLevel::kMonochrome, "monochrome" },
                                                    { ColorCompactLevel::kLowQuality, "low_quality" },
                                                })


enum class RenderSplitterType {
  kNone = 0,
  kTopHalo,
  kFilter,
};

NLOHMANN_JSON_SERIALIZE_ENUM(RenderSplitterType, {
                                                     { RenderSplitterType::kNone, "none" },
                                                     { RenderSplitterType::kTopHalo, "top_halo" },
                                                     { RenderSplitterType::kFilter, "filter" },
                                                 })


struct RenderSplitter {
  RenderSplitterType type;
  int top_halo_num;
  std::vector<std::vector<ShortIdType>> crystal_filters;

  static constexpr uint8_t kDefaultSymmetry = kSymmetryPrism | kSymmetryDirection | kSymmetryBasal;

  RenderSplitter();
};

void to_json(nlohmann::json& obj, const RenderSplitter& splitter);
void from_json(const nlohmann::json& obj, RenderSplitter& splitter);


enum class LineType {
  kSolid = 0,
  kDashed,
};

NLOHMANN_JSON_SERIALIZE_ENUM(LineType, {
                                           { LineType::kSolid, "solid" },
                                           { LineType::kDashed, "dashed" },
                                       })


struct LineSpecifier {
  LineType type;
  float width;
  float color[3];
  float alpha;

  static constexpr float kDefaultWidth = 0.8f;
  static constexpr float kMinWidth = 0.1f;
  static constexpr float kDefaultAlpha = 1.0f;
  static constexpr float kDefaultColor[3]{ 0.5f, 0.5f, 0.5f };
  static constexpr float kDefaultDashSize = 6.0f;  // Measured by line width

  LineSpecifier();
  LineSpecifier(LineType type, float width, const float color[3], float alpha);
};

void to_json(nlohmann::json& obj, const LineSpecifier& line);
void from_json(const nlohmann::json& obj, LineSpecifier& line);


struct GridLine {
  float value;
  LineSpecifier line_specifier;
};

void to_json(nlohmann::json& obj, const GridLine& line);
void from_json(const nlohmann::json& obj, GridLine& line);


/**
 * @brief Context for render
 *
 * @note There are some properties prior to others when determine ray color.
 *       The proper steps are as follows:
 * 1. Get `color_compact_level` by calling GetColorCompactLevel()
 * 2. If `color_compact_level` is ColorCompactLevel::kTrueColor
 *     2.1 Get `ray_color` by Calling GetRayColor()
 *     2.1 If `ray_color[0]` is less than 0, then use true color
 *     2.3 Else use `ray_color`
 * 3. Else ignore other ray color settings and background color settings
 */
class RenderContext {
 public:
  const float* GetRayColor() const;
  void SetRayColor(float r, float g, float b);
  void ResetRayColor();
  void UseRealRayColor();

  const float* GetBackgroundColor() const;
  void SetBackgroundColor(float r, float g, float b);
  void ResetBackgroundColor();
  void UseSkyBackground();

  float GetIntensity() const;
  void SetIntensity(float intensity);

  int GetImageWidth() const;
  int GetImageHeight() const;
  void SetImageWidth(int w);
  void SetImageHeight(int h);

  int GetImageOffsetX() const;
  int GetImageOffsetY() const;
  void SetImageOffsetX(int offset_x);
  void SetImageOffsetY(int offset_y);

  const RenderSplitter& GetSplitter() const;
  int GetSplitNumber() const;
  int GetSplitNumberPerImage() const;
  int GetSplitImageNumber() const;

  ColorCompactLevel GetColorCompactLevel() const;
  void SetColorCompactLevel(ColorCompactLevel level);

  VisibleRange GetVisibleRange() const;
  void SetVisibleRange(VisibleRange r);

  const std::vector<GridLine>& GetElevationGrids() const;
  const std::vector<GridLine>& GetRadiusGrids() const;

  static RenderContextPtrU CreateDefault();

  static constexpr float kDefaultIntensity = 1.0f;
  static constexpr float kMinIntensity = 0.01f;
  static constexpr float kMaxIntensity = 5.0e6f;
  static constexpr int kDefaultImageSize = 800;
  static constexpr int kMaxImageSize = 65535;
  static constexpr int kMaxTopHaloNumber = 300;

  friend void to_json(nlohmann::json& obj, const RenderContext& ctx);
  friend void from_json(const nlohmann::json& obj, RenderContext& ctx);

 private:
  RenderContext();

  void LoadColorConfig(const nlohmann::json& obj);
  void LoadIntensity(const nlohmann::json& obj);
  void LoadImageSize(const nlohmann::json& obj);
  void LoadImageOffset(const nlohmann::json& obj);
  void LoadVisibleRange(const nlohmann::json& obj);
  void LoadRenderSplitter(const nlohmann::json& obj);
  void LoadGridLines(const nlohmann::json& obj);

  void SaveColorConfig(nlohmann::json& obj) const;
  void SaveIntensity(nlohmann::json& obj) const;
  void SaveImageSize(nlohmann::json& obj) const;
  void SaveImageOffset(nlohmann::json& obj) const;
  void SaveVisibleRange(nlohmann::json& obj) const;
  void SaveRenderSplitter(nlohmann::json& obj) const;
  void SaveGridLines(nlohmann::json& obj) const;

  float ray_color_[3];
  float background_color_[3];
  float intensity_;
  int image_width_;
  int image_height_;
  int offset_x_;
  int offset_y_;
  VisibleRange visible_range_;
  ColorCompactLevel color_compact_level_;
  RenderSplitter splitter_;
  std::vector<GridLine> elevation_grid_;
  std::vector<GridLine> radius_rid_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_RENDER_CONTEXT_H_
