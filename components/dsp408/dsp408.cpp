#if defined(USE_ESP32_VARIANT_ESP32P4) || defined(USE_ESP32_VARIANT_ESP32S2) || defined(USE_ESP32_VARIANT_ESP32S3)

#include "dsp408.h"
#include "protocol.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"

#include <algorithm>

namespace esphome {
namespace dsp408 {

static const char *const TAG = "dsp408";

static const char *cmd_name(uint32_t cmd) {
  switch (cmd) {
    case CMD_CONNECT:    return "CONNECT";
    case CMD_GET_INFO:   return "GET_INFO";
    case CMD_MASTER:     return "MASTER";
    case CMD_IDLE_POLL:  return "IDLE_POLL";
    case CMD_PRESET_NAME: return "PRESET_NAME";
    case CMD_STATUS:     return "STATUS";
    default:             return "?";
  }
}

void DSP408::setup() {
  USBClient::setup();
  ESP_LOGCONFIG(TAG, "DSP-408 USB host client initialised (VID 0x%04X PID 0x%04X)",
                DSP408_VID, DSP408_PID);
}

void DSP408::dump_config() {
  USBClient::dump_config();
  ESP_LOGCONFIG(TAG,
                "  DSP-408\n"
                "    Interface claimed: %s\n"
                "    Phase: %d\n"
                "    Master known: %s (%+d dB, %s)",
                YESNO(this->interface_claimed_), static_cast<int>(this->phase_),
                YESNO(this->master_known_), this->master_db_,
                this->master_muted_ ? "muted" : "audible");
}

void DSP408::on_connected() {
  ESP_LOGI(TAG, "USB device opened — searching for HID interface...");
  if (!this->find_hid_endpoints_()) {
    ESP_LOGE(TAG, "No HID interface with EP-IN + EP-OUT found");
    this->status_set_error("No HID interface");
    this->disconnect();
    return;
  }

  auto err = usb_host_interface_claim(this->handle_, this->device_handle_,
                                      this->intf_number_, /*alt=*/0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_interface_claim(intf=%u) failed: %s",
             this->intf_number_, esp_err_to_name(err));
    this->status_set_error("interface_claim failed");
    this->disconnect();
    return;
  }
  this->interface_claimed_ = true;
  this->status_clear_error();
  ESP_LOGI(TAG, "Claimed HID interface %u, EP-IN=0x%02X EP-OUT=0x%02X",
           this->intf_number_, this->ep_in_->bEndpointAddress,
           this->ep_out_->bEndpointAddress);

  // Kick off the IN pump.
  this->input_pending_.store(false);
  this->start_input_();

  // Defer the first OUT command by a short interval so the device
  // settles after enumeration. dsp408-py does this implicitly via the
  // hidapi open_path round-trip.
  this->phase_ = StartupPhase::WAIT_AFTER_CONNECT;
  this->phase_started_ms_ = millis();
  this->connected_at_ms_ = this->phase_started_ms_;
  this->cmd_in_flight_ = 0;
  this->enable_loop();
}

void DSP408::on_disconnected() {
  ESP_LOGI(TAG, "USB device disconnected");
  if (this->interface_claimed_ && this->ep_in_ != nullptr) {
    usb_host_endpoint_halt(this->device_handle_, this->ep_in_->bEndpointAddress);
    usb_host_endpoint_flush(this->device_handle_, this->ep_in_->bEndpointAddress);
  }
  if (this->interface_claimed_ && this->ep_out_ != nullptr) {
    usb_host_endpoint_halt(this->device_handle_, this->ep_out_->bEndpointAddress);
    usb_host_endpoint_flush(this->device_handle_, this->ep_out_->bEndpointAddress);
  }
  if (this->interface_claimed_) {
    usb_host_interface_release(this->handle_, this->device_handle_, this->intf_number_);
    this->interface_claimed_ = false;
  }
  this->ep_in_ = nullptr;
  this->ep_out_ = nullptr;
  this->intf_number_ = 0xFF;
  this->phase_ = StartupPhase::IDLE;
  this->cmd_in_flight_ = 0;
  this->master_known_ = false;
  USBClient::on_disconnected();
}

bool DSP408::find_hid_endpoints_() {
  const usb_config_desc_t *config_desc;
  if (usb_host_get_active_config_descriptor(this->device_handle_, &config_desc) != ESP_OK) {
    ESP_LOGE(TAG, "get_active_config_descriptor failed");
    return false;
  }
  int conf_offset = 0;
  for (uint8_t intf_idx = 0; intf_idx < config_desc->bNumInterfaces; intf_idx++) {
    const auto *intf =
        usb_parse_interface_descriptor(config_desc, intf_idx, /*alt=*/0, &conf_offset);
    if (!intf)
      continue;
    ESP_LOGD(TAG, "intf %u: class=0x%02X subclass=0x%02X proto=0x%02X eps=%u",
             intf->bInterfaceNumber, intf->bInterfaceClass, intf->bInterfaceSubClass,
             intf->bInterfaceProtocol, intf->bNumEndpoints);
    if (intf->bInterfaceClass != 0x03) {  // not HID — skip
      continue;
    }
    const usb_ep_desc_t *ep_in = nullptr;
    const usb_ep_desc_t *ep_out = nullptr;
    for (uint8_t i = 0; i < intf->bNumEndpoints; i++) {
      int ep_offset = conf_offset;
      const auto *ep =
          usb_parse_endpoint_descriptor_by_index(intf, i, config_desc->wTotalLength, &ep_offset);
      if (!ep)
        break;
      ESP_LOGD(TAG, "  ep %u: addr=0x%02X attr=0x%02X mps=%u",
               i, ep->bEndpointAddress, ep->bmAttributes, ep->wMaxPacketSize);
      if ((ep->bmAttributes & 0x03) != USB_BM_ATTRIBUTES_XFER_INT)
        continue;
      if (ep->bEndpointAddress & usb_host::USB_DIR_IN) {
        if (ep_in == nullptr) ep_in = ep;
      } else {
        if (ep_out == nullptr) ep_out = ep;
      }
    }
    if (ep_in != nullptr && ep_out != nullptr) {
      this->intf_number_ = intf->bInterfaceNumber;
      this->ep_in_ = ep_in;
      this->ep_out_ = ep_out;
      return true;
    }
  }
  return false;
}

void DSP408::start_input_() {
  if (!this->interface_claimed_ || this->ep_in_ == nullptr)
    return;
  // Compare-exchange to ensure single in-flight IN at a time.
  bool expected = false;
  if (!this->input_pending_.compare_exchange_strong(expected, true))
    return;
  auto cb = [this](const usb_host::TransferStatus &status) {
    this->on_in_report_usb_task_(status);
  };
  if (!this->transfer_in(this->ep_in_->bEndpointAddress, cb, 64)) {
    ESP_LOGE(TAG, "transfer_in submit failed");
    this->input_pending_.store(false);
  }
}

// USB-task context. Keep it short: copy data into a queued slot,
// wake main loop, restart IN.
void DSP408::on_in_report_usb_task_(const usb_host::TransferStatus &status) {
  if (status.success && status.data_len > 0 && status.data_len <= 64) {
    InReport *rep = this->in_pool_.allocate();
    if (rep != nullptr) {
      memcpy(rep->data, status.data, status.data_len);
      rep->length = static_cast<uint16_t>(status.data_len);
      this->in_queue_.push(rep);
      this->enable_loop_soon_any_context();
      App.wake_loop_threadsafe();
    } else {
      // Pool empty — drop the report; we'll catch up next iteration.
      ESP_LOGW(TAG, "IN pool exhausted, dropping report");
    }
  } else if (!status.success) {
    ESP_LOGW(TAG, "IN transfer failed status=0x%X", status.error_code);
  }
  this->input_pending_.store(false);
  // Restart the IN pump unconditionally — the device may still be alive.
  this->start_input_();
}

void DSP408::loop() {
  // Always drain the USB-event queue inherited from USBClient.
  bool had_work = this->process_usb_events_();

  // Drain inbound HID reports.
  this->process_in_queue_();

  // Drive the startup state machine.
  uint32_t now = millis();
  switch (this->phase_) {
    case StartupPhase::WAIT_AFTER_CONNECT:
      // 50ms grace post-claim before sending CMD_CONNECT
      if (now - this->phase_started_ms_ >= 50) {
        if (this->send_connect_()) {
          this->phase_ = StartupPhase::SENT_CONNECT;
          this->phase_started_ms_ = now;
          had_work = true;
        }
      }
      break;
    case StartupPhase::SENT_CONNECT:
    case StartupPhase::SENT_GET_INFO:
    case StartupPhase::SENT_GET_MASTER:
      // Watchdog timeout — if no reply in REQUEST_TIMEOUT_MS, retry the
      // whole startup from CMD_CONNECT.
      if (this->cmd_in_flight_ != 0 &&
          now - this->cmd_in_flight_started_ms_ > REQUEST_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Timeout waiting for reply to %s — restarting startup",
                 cmd_name(this->cmd_in_flight_));
        this->cmd_in_flight_ = 0;
        this->phase_ = StartupPhase::WAIT_AFTER_CONNECT;
        this->phase_started_ms_ = now;
      }
      break;
    case StartupPhase::RUNNING:
      // Periodic master state poll if no command in flight.
      if (this->cmd_in_flight_ == 0 &&
          now - this->last_master_poll_ms_ > MASTER_POLL_INTERVAL_MS) {
        this->send_get_master_();
        this->last_master_poll_ms_ = now;
        had_work = true;
      }
      break;
    case StartupPhase::IDLE:
    default:
      break;
  }

  // ESPHome will keep loop() running while we have work; otherwise we
  // can ride along with the next inbound report or scheduled poll.
  if (!had_work) {
    // Don't disable_loop() here — USBClient's loop() handles event-pump
    // and we may still be polling. Keep it cheap and let ESPHome's
    // scheduler run us at its normal cadence.
  }
}

void DSP408::process_in_queue_() {
  InReport *rep;
  while ((rep = this->in_queue_.pop()) != nullptr) {
    this->dispatch_frame_(*rep);
    this->in_pool_.release(rep);
  }
}

void DSP408::dispatch_frame_(const InReport &report) {
  ParsedFrame f = parse_frame(report.data, report.length);
  if (!f.valid) {
    ESP_LOGV(TAG, "Inbound report not a DSP-408 frame (len=%u)", report.length);
    return;
  }
  // Only accept replies (RESP / WRITE_ACK).
  if (f.direction != DIR_RESP && f.direction != DIR_WRITE_ACK) {
    ESP_LOGV(TAG, "Inbound frame with unexpected direction 0x%02X", f.direction);
    return;
  }
  ESP_LOGV(TAG, "<- dir=0x%02X cat=0x%02X cmd=0x%X seq=%u len=%u",
           f.direction, f.category, static_cast<unsigned>(f.cmd), f.seq, f.payload_len);

  // Match against in-flight cmd. Lenient on seq because firmware sometimes
  // returns seq=0 even for reads.
  bool was_in_flight = (this->cmd_in_flight_ == f.cmd);
  if (was_in_flight)
    this->cmd_in_flight_ = 0;

  switch (f.cmd) {
    case CMD_CONNECT:
      this->handle_connect_reply_(f.payload, f.payload_bytes_in_frame);
      break;
    case CMD_GET_INFO:
      this->handle_get_info_reply_(f.payload, f.payload_bytes_in_frame);
      break;
    case CMD_MASTER:
      this->handle_master_reply_(f.payload, f.payload_bytes_in_frame,
                                 f.direction == DIR_WRITE_ACK);
      break;
    default:
      // Per-channel write ack? CMD = 0x1F00..0x1F07
      if (f.cmd >= CMD_WRITE_CHANNEL_BASE && f.cmd < CMD_WRITE_CHANNEL_BASE + 8) {
        this->handle_channel_write_ack_(f.cmd, f.payload, f.payload_bytes_in_frame);
      } else {
        ESP_LOGV(TAG, "Unhandled cmd 0x%X", static_cast<unsigned>(f.cmd));
      }
      break;
  }
  (void) was_in_flight;
}

bool DSP408::send_cmd_(uint8_t direction, uint32_t cmd, uint8_t category,
                       const uint8_t *payload, size_t payload_len, uint8_t seq) {
  if (!this->interface_claimed_ || this->ep_out_ == nullptr) {
    ESP_LOGW(TAG, "send_cmd: interface not ready");
    return false;
  }
  uint8_t frame[FRAME_SIZE];
  if (!build_frame(frame, direction, seq, cmd, category, payload, payload_len)) {
    ESP_LOGE(TAG, "build_frame failed (payload too large?)");
    return false;
  }
  ESP_LOGV(TAG, "-> dir=0x%02X cat=0x%02X cmd=0x%X seq=%u len=%u",
           direction, category, static_cast<unsigned>(cmd), seq, static_cast<unsigned>(payload_len));
  auto cb = [this, cmd](const usb_host::TransferStatus &status) {
    if (!status.success) {
      ESP_LOGW(TAG, "OUT submit for cmd=0x%X failed status=0x%X",
               static_cast<unsigned>(cmd), status.error_code);
    }
  };
  bool ok = this->transfer_out(this->ep_out_->bEndpointAddress, cb, frame, FRAME_SIZE);
  if (ok) {
    this->cmd_in_flight_ = cmd;
    this->cmd_in_flight_started_ms_ = millis();
    this->cmd_in_flight_seq_ = seq;
    this->cmd_in_flight_dir_ = direction;
  }
  return ok;
}

bool DSP408::send_connect_() {
  return this->send_cmd_(DIR_CMD, CMD_CONNECT, CAT_STATE, nullptr, 0, this->next_read_seq_++);
}

bool DSP408::send_get_info_() {
  return this->send_cmd_(DIR_CMD, CMD_GET_INFO, CAT_STATE, nullptr, 0, this->next_read_seq_++);
}

bool DSP408::send_get_master_() {
  return this->send_cmd_(DIR_CMD, CMD_MASTER, CAT_STATE, nullptr, 0, this->next_read_seq_++);
}

bool DSP408::send_set_master_(uint8_t lvl_raw, bool muted) {
  // Per dsp408-py: payload = [lvl, 00, 00, 32, 00, 32, mute_bit, 00]
  // mute_bit: 1 = unmuted/audible, 0 = muted.
  uint8_t payload[8] = {lvl_raw, 0x00, 0x00, 0x32, 0x00, 0x32,
                        static_cast<uint8_t>(muted ? 0 : 1), 0x00};
  return this->send_cmd_(DIR_WRITE, CMD_MASTER, CAT_STATE, payload, sizeof(payload), 0);
}

bool DSP408::send_set_channel_(uint8_t ch) {
  if (ch >= 8) return false;
  ChannelState &c = this->ch_state_[ch];
  uint8_t subidx = c.subidx != 0 ? c.subidx : CHANNEL_SUBIDX_DEFAULT[ch];

  // gain raw = (dB * 10) + 600, clamped to [0..600]
  int gain = static_cast<int>(c.db) * 10 + CHANNEL_VOL_OFFSET;
  if (gain < CHANNEL_VOL_MIN) gain = CHANNEL_VOL_MIN;
  if (gain > CHANNEL_VOL_MAX) gain = CHANNEL_VOL_MAX;
  uint16_t gain_u16 = static_cast<uint16_t>(gain);

  uint16_t delay = c.delay_samples;
  if (delay > CHANNEL_DELAY_MAX) delay = CHANNEL_DELAY_MAX;

  uint8_t payload[8] = {
      static_cast<uint8_t>(c.muted ? 0 : 1),         // [0] enable
      0x00,                                          // [1] polar
      static_cast<uint8_t>(gain_u16 & 0xFF),         // [2..3] gain LE
      static_cast<uint8_t>((gain_u16 >> 8) & 0xFF),
      static_cast<uint8_t>(delay & 0xFF),            // [4..5] delay LE
      static_cast<uint8_t>((delay >> 8) & 0xFF),
      0x00,                                          // [6] reserved
      subidx,                                        // [7] DSP channel-type
  };
  uint32_t cmd = CMD_WRITE_CHANNEL_BASE + ch;
  return this->send_cmd_(DIR_WRITE, cmd, CAT_STATE, payload, sizeof(payload), 0);
}

void DSP408::handle_connect_reply_(const uint8_t *payload, size_t len) {
  if (len == 0) {
    ESP_LOGW(TAG, "CONNECT reply with empty payload");
    return;
  }
  ESP_LOGI(TAG, "CONNECT ok (status=0x%02X)", payload[0]);
  // Move to GET_INFO
  if (this->send_get_info_()) {
    this->phase_ = StartupPhase::SENT_GET_INFO;
    this->phase_started_ms_ = millis();
  }
}

void DSP408::handle_get_info_reply_(const uint8_t *payload, size_t len) {
  // ASCII identity, e.g. "MYDW-AV1.06"
  std::string s;
  for (size_t i = 0; i < len; i++) {
    if (payload[i] == 0) break;
    s.push_back(static_cast<char>(payload[i]));
  }
  ESP_LOGI(TAG, "GET_INFO: '%s'", s.c_str());
  if (this->model_ts_ != nullptr)
    this->model_ts_->publish_state(s);
  // Move on to read master state
  if (this->send_get_master_()) {
    this->phase_ = StartupPhase::SENT_GET_MASTER;
    this->phase_started_ms_ = millis();
  }
}

void DSP408::handle_master_reply_(const uint8_t *payload, size_t len, bool is_ack) {
  if (len < 8) {
    ESP_LOGW(TAG, "MASTER reply too short (%u bytes)", static_cast<unsigned>(len));
    return;
  }
  uint8_t lvl = payload[0];
  bool muted = payload[6] == 0;  // 1 = audible, 0 = muted
  this->master_known_ = true;
  this->master_db_ = static_cast<int8_t>(static_cast<int>(lvl) - MASTER_LEVEL_OFFSET);
  this->master_muted_ = muted;
  ESP_LOGI(TAG, "MASTER %s: %+d dB %s",
           is_ack ? "ack" : "read", this->master_db_, muted ? "(muted)" : "(audible)");

  if (this->master_vol_num_ != nullptr)
    this->master_vol_num_->publish_state(static_cast<float>(this->master_db_));
  if (this->master_mute_sw_ != nullptr)
    this->master_mute_sw_->publish_state(this->master_muted_);

  if (this->phase_ != StartupPhase::RUNNING) {
    this->phase_ = StartupPhase::RUNNING;
    this->phase_started_ms_ = millis();
    this->last_master_poll_ms_ = this->phase_started_ms_;
    ESP_LOGI(TAG, "DSP-408 ready (startup took %u ms)",
             static_cast<unsigned>(this->phase_started_ms_ - this->connected_at_ms_));
  }
}

void DSP408::handle_channel_write_ack_(uint32_t cmd, const uint8_t *payload, size_t len) {
  uint8_t ch = static_cast<uint8_t>(cmd - CMD_WRITE_CHANNEL_BASE);
  ESP_LOGD(TAG, "Channel %u write ack (len=%u)", ch, static_cast<unsigned>(len));
  if (ch < 8) {
    if (this->ch_vol_num_[ch] != nullptr)
      this->ch_vol_num_[ch]->publish_state(static_cast<float>(this->ch_state_[ch].db));
    if (this->ch_mute_sw_[ch] != nullptr)
      this->ch_mute_sw_[ch]->publish_state(this->ch_state_[ch].muted);
  }
  (void) payload;
}

// ── Public command entry points ─────────────────────────────────────────

void DSP408::request_master_volume(float db) {
  int lvl = static_cast<int>(std::round(db + MASTER_LEVEL_OFFSET));
  if (lvl < MASTER_LEVEL_MIN) lvl = MASTER_LEVEL_MIN;
  if (lvl > MASTER_LEVEL_MAX) lvl = MASTER_LEVEL_MAX;
  bool muted = this->master_known_ ? this->master_muted_ : false;
  this->master_db_ = static_cast<int8_t>(lvl - MASTER_LEVEL_OFFSET);
  this->master_known_ = true;
  this->send_set_master_(static_cast<uint8_t>(lvl), muted);
}

void DSP408::request_master_mute(bool muted) {
  uint8_t lvl_raw;
  if (this->master_known_) {
    lvl_raw = static_cast<uint8_t>(this->master_db_ + MASTER_LEVEL_OFFSET);
  } else {
    lvl_raw = MASTER_LEVEL_OFFSET;  // default 0 dB
  }
  this->master_muted_ = muted;
  this->master_known_ = true;
  this->send_set_master_(lvl_raw, muted);
}

void DSP408::request_channel_volume(uint8_t ch, float db) {
  if (ch >= 8) return;
  int v = static_cast<int>(std::round(db));
  if (v < -60) v = -60;
  if (v > 0) v = 0;
  this->ch_state_[ch].db = static_cast<int16_t>(v);
  this->ch_state_[ch].primed = true;
  this->send_set_channel_(ch);
}

void DSP408::request_channel_mute(uint8_t ch, bool muted) {
  if (ch >= 8) return;
  this->ch_state_[ch].muted = muted;
  this->ch_state_[ch].primed = true;
  this->send_set_channel_(ch);
}

}  // namespace dsp408
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32S2/S3/P4
