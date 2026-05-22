#include "iray_detect.hpp"
#include "esp_log.h"
#include <filesystem>

#if CONFIG_IRAY_DETECT_MODEL_IN_FLASH_RODATA
extern const uint8_t iray_model_espdl[] asm("_binary_iray_model_espdl_start");
static const char *path = (const char *)iray_model_espdl;
#elif CONFIG_IRAY_DETECT_MODEL_IN_FLASH_PARTITION
static const char *path = "iray_det";
#else
#if !defined(CONFIG_BSP_SD_MOUNT_POINT)
#define CONFIG_BSP_SD_MOUNT_POINT "/sdcard"
#endif
#endif

namespace iray_detect {
IRayDet::IRayDet(const char *model_name, float score_thr, float nms_thr) {
#if !CONFIG_IRAY_DETECT_MODEL_IN_SDCARD
  m_model = new dl::Model(path, model_name,
                          static_cast<fbs::model_location_type_t>(
                              CONFIG_IRAY_DETECT_MODEL_LOCATION));
#else
  auto sd_path = std::filesystem::path(CONFIG_BSP_SD_MOUNT_POINT) /
                 CONFIG_IRAY_DETECT_MODEL_SDCARD_DIR / model_name;
  m_model = new dl::Model(sd_path.c_str(), fbs::MODEL_LOCATION_IN_SDCARD);
#endif
  m_model->minimize();
#if CONFIG_IDF_TARGET_ESP32P4
  m_image_preprocessor =
      new dl::image::ImagePreprocessor(m_model, {0, 0, 0}, {255, 255, 255});
#else
  m_image_preprocessor = new dl::image::ImagePreprocessor(
      m_model, {0, 0, 0}, {255, 255, 255},
      dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN);
#endif
  m_image_preprocessor->enable_letterbox({114, 114, 114});
  m_postprocessor = new dl::detect::ESPDetPostProcessor(
      m_model, m_image_preprocessor, score_thr, nms_thr, 10,
      {{8, 8, 4, 4}, {16, 16, 8, 8}, {32, 32, 16, 16}});
}
} // namespace iray_detect

IRayDetect::IRayDetect(model_type_t model_type, bool lazy_load)
    : m_model_type(model_type) {
  switch (model_type) {
  case IRAY_PICO_120_160:
    m_score_thr[0] = iray_detect::IRayDet::default_score_thr;
    m_nms_thr[0] = iray_detect::IRayDet::default_nms_thr;
    break;
  }
  if (lazy_load) {
    m_model = nullptr;
  } else {
    load_model();
  }
}

void IRayDetect::load_model() {
  switch (m_model_type) {
  case IRAY_PICO_120_160:
#if CONFIG_FLASH_IRAY_PICO_120_160 || CONFIG_IRAY_DETECT_MODEL_IN_SDCARD
    m_model = new iray_detect::IRayDet("espdet_pico_120_160_inface.espdl",
                                       m_score_thr[0], m_nms_thr[0]);
#else
    ESP_LOGE("iray_detect",
             "espdet_pico_120_160_inface is not selected in menuconfig.");
#endif
    break;
  }
}
