#pragma once

#include "esphome/components/switch/switch.h"
#include "../dsp408.h"

namespace esphome {
namespace dsp408 {

class DSP408Switch : public switch_::Switch, public Component {
 public:
  void set_parent(DSP408 *parent) { this->parent_ = parent; }
  void set_master() { this->is_master_ = true; }
  void set_channel(uint8_t ch) {
    this->is_master_ = false;
    this->channel_ = ch;
  }

  void setup() override {}
  void dump_config() override;

 protected:
  void write_state(bool state) override;

  DSP408 *parent_{nullptr};
  bool is_master_{false};
  uint8_t channel_{0};
};

}  // namespace dsp408
}  // namespace esphome
