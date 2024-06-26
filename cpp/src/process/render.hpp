#ifndef SRC_CORE_RENDER_H_
#define SRC_CORE_RENDER_H_

#include <functional>

#include "context/context.hpp"
#include "core/core_def.hpp"
#include "util/enum_map.hpp"
#include "util/threading_pool.hpp"


namespace icehalo {

enum class VisibleRange { kUpper, kLower, kFront, kFull };

NLOHMANN_JSON_SERIALIZE_ENUM(VisibleRange, {
                                               { VisibleRange::kFull, "full" },
                                               { VisibleRange::kUpper, "upper" },
                                               { VisibleRange::kLower, "lower" },
                                               { VisibleRange::kFront, "front" },
                                           })

using ProjectionFunction = std::function<void(Pose3f cam_pose,           // Camera rotation (lon, lat, roll), in degree.
                                              float hov,                 // Half field of view, in degree
                                              size_t data_number,        // Data number
                                              const float* dir,          // Ray directions, [x, y, z]
                                              int img_wid, int img_hei,  // Image size
                                              float* img_xy,             // Image coordinates
                                              VisibleRange visible_semi_sphere)>;  // Which semi-sphere can be visible


ProjectionFunction GetProjectionFunction(LensType lens_type);


using ImageSpectrumData = std::pair<int, std::unique_ptr<float[]>>;

constexpr int kMinWavelength = 360;
constexpr int kMaxWaveLength = 830;


struct RayCollectionInfo;
struct SimpleRayData;

class Renderer {
 public:
  Renderer();
  Renderer(const Renderer& other) = delete;
  Renderer(Renderer&& other) noexcept;
  ~Renderer() = default;

  Renderer& operator=(Renderer&& other) noexcept;
  Renderer& operator=(const Renderer& other) = delete;

  void SetCameraContext(CameraContextPtr cam_ctx);
  void SetRenderContext(RenderContextPtr render_ctx);
  void SetSunContext(SunContextPtr sun_ctx);
  void SetThreadingPool(ThreadingPoolPtr threading_pool);

  void LoadRayData(int identifier, const RayCollectionInfo& collection_info, const SimpleRayData& final_ray_data);

  /**
   * @brief Render ray data to image buffer.
   *
   * 1. Get `color_compact_level` by calling RenderContext::GetColorCompactLevel()
   * 2. If `color_compact_level` is ColorCompactLevel::kTrueColor
   *     2.1 Get `ray_color` by Calling RenderContext::GetRayColor()
   *     2.1 If `ray_color[0]` is less than 0, then use true color
   *     2.3 Else use `ray_color`
   * 3. Else ignore other ray color settings and background color settings
   */
  void Render();
  uint8_t* GetImageBuffer() const;

  static constexpr int kImageBits = 24;
  static constexpr float kLineD2ExclusionLimitRatio = 0.1;

 private:
  void LoadPartialRayData(const std::vector<size_t>& idx, const SimpleRayData& final_ray_data, float* current_data,
                          float* current_data_compensation);
  void LoadFullRayData(const SimpleRayData& final_ray_data, float* current_data, float* current_data_compensation);

  void RenderHaloImage();
  void DrawGrids();
  void DrawElevationGrids();
  void DrawRadiusGrids();

  static constexpr int kLineD2Lower = 2;
  static constexpr int kLineD2Upper = 20;
  static constexpr float kDefaultLineStep = 1.0f;
  static constexpr float kMinLineStep = 0.05f;
  static constexpr float kMaxLineStep = 5.0f;

  CameraContextPtr cam_ctx_;
  RenderContextPtr render_ctx_;
  SunContextPtr sun_ctx_;
  std::unique_ptr<uint8_t[]> output_image_buffer_;
  std::vector<ImageSpectrumData> spectrum_data_;
  std::vector<ImageSpectrumData> spectrum_data_compensation_;
  float total_w_;
  ThreadingPoolPtr threading_pool_;
};

}  // namespace icehalo


#endif  // SRC_CORE_RENDER_H_
