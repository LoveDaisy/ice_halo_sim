#ifndef SRC_CONTEXT_RENDER_CONTEXT_H_
#define SRC_CONTEXT_RENDER_CONTEXT_H_

#include <memory>

#include "core/core_def.hpp"
#include "core/crystal.hpp"
#include "io/serialize.hpp"
#include "rapidjson/document.h"


namespace icehalo {

enum class VisibleRange;

enum class ColorCompactLevel : int {
  kTrueColor = 24,  //!< True color, use all 24-bit
  kMonochrome = 8,  //!< Gray scale, 8-bit per channel
  kLowQuality = 4,  //!< 4-bit per channel
};


enum class RenderSplitterType {
  kNone = 0,
  kTopHalo,
  kFilter,
};


struct RenderSplitter : public IJsonizable {
  RenderSplitterType type;
  int top_halo_num;
  std::vector<std::vector<ShortIdType>> crystal_filters;

  static constexpr uint8_t kDefaultSymmetry = kSymmetryPrism | kSymmetryDirection | kSymmetryBasal;

  RenderSplitter();

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;
};


enum class LineType {
  kSolid = 0,
  kDashed,
};


struct LineSpecifier : public IJsonizable {
  LineType type;
  float width;
  float color[3];
  float alpha;

  static constexpr float kDefaultWidth = 0.8f;
  static constexpr float kMinWidth = 0.1f;
  static constexpr float kDefaultAlpha = 1.0f;
  static constexpr float kDefaultColor[3]{ 0.5f, 0.5f, 0.5f };
  static constexpr float kDefaultDashSize = 7.0f;  // Measured by line width

  LineSpecifier();
  LineSpecifier(LineType type, float width, const float color[3], float alpha);

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;
};


struct GridLine {
  float value;
  LineSpecifier line_specifier;
};


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
class RenderContext : public IJsonizable {
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

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static RenderContextPtrU CreateDefault();

  static constexpr float kDefaultIntensity = 1.0f;
  static constexpr float kMinIntensity = 0.01f;
  static constexpr float kMaxIntensity = 100.0f;
  static constexpr int kDefaultImageSize = 800;
  static constexpr int kMaxImageSize = 4096;
  static constexpr int kMaxTopHaloNumber = 300;

 private:
  RenderContext();

  void LoadColorConfig(const rapidjson::Value& root);
  void LoadIntensity(const rapidjson::Value& root);
  void LoadImageSize(const rapidjson::Value& root);
  void LoadImageOffset(const rapidjson::Value& root);
  void LoadVisibleRange(const rapidjson::Value& root);
  void LoadRenderSplitter(const rapidjson::Value& root);
  void LoadGridLines(const rapidjson::Value& root);

  void SaveColorConfig(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveIntensity(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveImageSize(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveImageOffset(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveVisibleRange(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveRenderSplitter(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);
  void SaveGridLines(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator);

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
