#ifndef SRC_CORE_RENDER_H_
#define SRC_CORE_RENDER_H_

#include <functional>
#include <unordered_map>

#include "context/context.h"
#include "core/enum_map.h"
#include "core/simulation.h"
#include "io/file.h"


namespace icehalo {

enum class VisibleRange { kUpper, kLower, kFront, kFull };


enum class LensType {
  kLinear,
  kEqualArea,
  kEquidistant,
  kDualEqualArea,
  kDualEquidistant,
};


void EqualAreaFishEye(const float* cam_rot,                                // Camera rotation. [lon, lat, roll]
                      float hov,                                           // Half field of view.
                      size_t data_number,                                  // Data number
                      const float* dir,                                    // Ray directions, [x, y, z]
                      int img_wid, int img_hei,                            // Image size
                      int* img_xy,                                         // Image coordinates
                      VisibleRange visible_range = VisibleRange::kUpper);  // Visible range


void EquidistantFishEye(const float* cam_rot,                                // Camera rotation. [lon, lat, roll]
                        float hov,                                           // Half field of view.
                        size_t data_number,                                  // Data number
                        const float* dir,                                    // Ray directions, [x, y, z]
                        int img_wid, int img_hei,                            // Image size
                        int* img_xy,                                         // Image coordinates
                        VisibleRange visible_range = VisibleRange::kUpper);  // Visible range


void DualEqualAreaFishEye(const float* cam_rot,                                // Not used
                          float hov,                                           // Not used
                          size_t data_number,                                  // Data number
                          const float* dir,                                    // Ray directions, [x, y, z]
                          int img_wid, int img_hei,                            // Image size
                          int* img_xy,                                         // Image coordinates
                          VisibleRange visible_range = VisibleRange::kUpper);  // Not used


void DualEquidistantFishEye(const float* cam_rot,                                // Not used
                            float hov,                                           // Not used
                            size_t data_number,                                  // Data number
                            const float* dir,                                    // Ray directions, [x, y, z]
                            int img_wid, int img_hei,                            // Image size
                            int* img_xy,                                         // Image coordinates
                            VisibleRange visible_range = VisibleRange::kUpper);  // Not used


void RectLinear(const float* cam_rot,                                // Camera rotation. [lon, lat, roll]
                float hov,                                           // Half field of view.
                size_t data_number,                                  // Data number
                const float* dir,                                    // Ray directions, [x, y, z]
                int img_wid, int img_hei,                            // Image size
                int* img_xy,                                         // Image coordinates
                VisibleRange visible_range = VisibleRange::kUpper);  // Visible range


using ProjectionFunction = std::function<void(const float* cam_rot,      // Camera rotation (lon, lat, roll), in degree.
                                              float hov,                 // Half field of view, in degree
                                              size_t data_number,        // Data number
                                              const float* dir,          // Ray directions, [x, y, z]
                                              int img_wid, int img_hei,  // Image size
                                              int* img_xy,               // Image coordinates
                                              VisibleRange visible_semi_sphere)>;  // Which semi-sphere can be visible


EnumMap<LensType, ProjectionFunction>& GetProjectionFunctions();


using ImageSpectrumData = std::pair<int, std::unique_ptr<float[]>>;


void SrgbGamma(float* linear_rgb, size_t num = 3);
void RenderSpecToRgb(const std::vector<ImageSpectrumData>& spec_data,   // spec_data: wavelength_number * data_number
                     size_t data_number, float factor,                  //
                     uint8_t* rgb_data);                                // rgb data, data_number * 3
void RenderSpecToGray(const std::vector<ImageSpectrumData>& spec_data,  // spec_data: wavelength_number * data_number
                      size_t data_number, float factor,                 //
                      RenderColorCompactLevel level, int index,         // color compact level and channel index
                      uint8_t* rgb_data);                               // rgb data, data_number * 3

constexpr int kMinWavelength = 360;
constexpr int kMaxWaveLength = 830;


class SpectrumRenderer {
 public:
  SpectrumRenderer();

  void SetCameraContext(CameraContextPtr cam_ctx);
  void SetRenderContext(RenderContextPtr render_ctx);

  void LoadRayData(const SimpleRayData& final_ray_data);
  void ClearRayData();

  void RenderToImage();
  void RenderToImage(RenderColorCompactLevel level, int index);
  uint8_t* GetImageBuffer() const;

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
