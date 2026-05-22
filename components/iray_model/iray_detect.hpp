#pragma once
#include "dl_detect_base.hpp"
#include "dl_detect_espdet_postprocessor.hpp"

namespace iray_detect {
class IRayDet : public dl::detect::DetectImpl {
public:
  static inline constexpr float default_score_thr = 0.50f;
  static inline constexpr float default_nms_thr = 0.4f;
  IRayDet(const char *model_name, float score_thr, float nms_thr);
};
} // namespace iray_detect

class IRayDetect : public dl::detect::DetectWrapper {
public:
  typedef enum {
    IRAY_PICO_120_160 = 0,
  } model_type_t;
  explicit IRayDetect(model_type_t model_type = IRAY_PICO_120_160,
                      bool lazy_load = true);

private:
  void load_model() override;
  model_type_t m_model_type;
};
