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

  VisibleRange GetVisibleRange() const;
  void SetVisibleRange(VisibleRange r);

  void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) override;
  void LoadFromJson(const rapidjson::Value& root) override;

  static RenderContextPtrU CreateFromJson(rapidjson::Document& d);

  static constexpr float kMinIntensity = 0.01f;
  static constexpr float kMaxIntensity = 100.0f;

  static constexpr int kMaxImageSize = 4096;

 private:
  RenderContext();

  float ray_color_[3];
  float background_color_[3];
  float intensity_;
  int image_width_;
  int image_height_;
  int offset_x_;
  int offset_y_;
  VisibleRange visible_range_;
};

}  // namespace icehalo


#endif  // SRC_CONTEXT_RENDER_CONTEXT_H_
