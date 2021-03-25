#ifndef SRC_CORE_RENDER_H_
#define SRC_CORE_RENDER_H_

#include <functional>

#include "context/context.hpp"
#include "core/core_def.hpp"
#include "util/enum_map.hpp"


namespace icehalo {

enum class VisibleRange { kUpper, kLower, kFront, kFull };


enum class LensType {
  kLinear,
  kEqualArea,
  kEquidistant,
  kDualEqualArea,
  kDualEquidistant,
};


using ProjectionFunction = std::function<void(const Pose3f& cam_pose,    // Camera rotation (lon, lat, roll), in degree.
                                              float hov,                 // Half field of view, in degree
                                              size_t data_number,        // Data number
                                              const float* dir,          // Ray directions, [x, y, z]
                                              int img_wid, int img_hei,  // Image size
                                              int* img_xy,               // Image coordinates
                                              VisibleRange visible_semi_sphere)>;  // Which semi-sphere can be visible


EnumMap<LensType, ProjectionFunction>& GetProjectionFunctions();


using ImageSpectrumData = std::pair<size_t, std::unique_ptr<float[]>>;

constexpr int kMinWavelength = 360;
constexpr int kMaxWaveLength = 830;


struct RayCollectionInfo;
struct SimpleRayData;

class SpectrumRenderer {
 public:
  SpectrumRenderer();
  SpectrumRenderer(const SpectrumRenderer& other) = delete;
  SpectrumRenderer(SpectrumRenderer&& other) noexcept;

  SpectrumRenderer& operator=(SpectrumRenderer&& other) noexcept;

  void SetCameraContext(CameraContextPtr cam_ctx);
  void SetRenderContext(RenderContextPtr render_ctx);

  void LoadRayData(size_t identifier, const RayCollectionInfo& collection_info, const SimpleRayData& final_ray_data);

  /**
   * @brief Render ray data to image buffer.
   *
   * 1. Get `color_compact_level` by calling RenderContext::GetColorCompactLevel()
   * 2. If `color_compact_level` is ColorCompactLevel::kTrueColor
   *     2.1 Get `ray_color` by Calling RenderContext::GetRayColor()
   *     2.1 If `ray_color[0]` is less than 0, then use true color
   *     2.3 Else use `ray_color`
   * 3. Else ignore other ray color settings and background color settings
   *
   * @param channel_index If `color_compact_level` is ColorCompactLevel::kTrueColor, this parameter
   *        will be ignored.
   */
  void RenderToImage();
  uint8_t* GetImageBuffer() const;

  static constexpr int kImageBits = 24;

 private:
  CameraContextPtr cam_ctx_;
  RenderContextPtr render_ctx_;
  std::unique_ptr<uint8_t[]> output_image_buffer_;
  std::vector<ImageSpectrumData> spectrum_data_;
  std::vector<ImageSpectrumData> spectrum_data_compensation_;
  float total_w_;
};

}  // namespace icehalo


#endif  // SRC_CORE_RENDER_H_
