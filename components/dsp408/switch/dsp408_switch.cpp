#include "dsp408_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dsp408 {

static const char *const TAG = "dsp408.switch";

void DSP408Switch::dump_config() {
  if (this->is_master_) {
    ESP_LOGCONFIG(TAG, "DSP-408 Switch (master mute): '%s'", this->get_name().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "DSP-408 Switch (channel %u mute): '%s'",
                  this->channel_, this->get_name().c_str());
  }
}

void DSP408Switch::write_state(bool state) {
  if (this->parent_ == nullptr)
    return;
  this->publish_state(state);  // optimistic
  if (this->is_master_) {
    this->parent_->request_master_mute(state);
  } else {
    this->parent_->request_channel_mute(this->channel_, state);
  }
}

}  // namespace dsp408
}  // namespace esphome
