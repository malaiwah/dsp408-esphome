#include "dsp408_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dsp408 {

static const char *const TAG = "dsp408.number";

void DSP408Number::dump_config() {
  if (this->is_master_) {
    ESP_LOGCONFIG(TAG, "DSP-408 Number (master volume): '%s'", this->get_name().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "DSP-408 Number (channel %u volume): '%s'",
                  this->channel_, this->get_name().c_str());
  }
}

void DSP408Number::control(float value) {
  if (this->parent_ == nullptr)
    return;
  this->publish_state(value);  // optimistic; the device-side ack will reaffirm
  if (this->is_master_) {
    this->parent_->request_master_volume(value);
  } else {
    this->parent_->request_channel_volume(this->channel_, value);
  }
}

}  // namespace dsp408
}  // namespace esphome
