#ifndef SRC_CONTEXT_RENDER_CONTEXT_H_
#define SRC_CONTEXT_RENDER_CONTEXT_H_

#include <memory>

#include "io/serialize.h"
#include "rapidjson/document.h"


namespace icehalo {

enum class VisibleRange;
class RenderContext;
using RenderContextPtrU = std::unique_ptr<RenderContext>;
using RenderContextPtr = std::shared_ptr<RenderContext>;


enum class ColorCompactLevel : int {
  kTrueColor = 24,  //!< True color, use all 24-bit
  kMonoChrome = 8,  //!< Gray scale, 8-bit per channel
  kLowQuality = 4,  //!< 4-bit per channel
};


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

  int GetTopHaloNumber() const;
  void SetTopHaloNumber(int n);

  ColorCompactLevel GetColorCompactLevel() const;
  void SetColorCompactLevel(ColorCompactLevel level);

  VisibleRange GetVisibleRange() const;
  void SetVisibleRange(VisibleRange r);

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static RenderContextPtrU CreateDefault();

  static constexpr float kMinIntensity = 0.01f;
  static constexpr float kMaxIntensity = 100.0f;
  static constexpr int kMaxImageSize = 4096;
  static constexpr int kMaxTopHaloNumber = 150;

 private:
  RenderContext();

  float ray_color_[3];
  float background_color_[3];
  float intensity_;
  int image_width_;
  int image_height_;
  int offset_x_;
  int offset_y_;
  int top_halo_num_;
  VisibleRange visible_range_;
  ColorCompactLevel color_compact_level_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_RENDER_CONTEXT_H_
