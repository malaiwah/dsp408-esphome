#pragma once
//
// DSP-408 ESPHome external_component — main parent class.
//
// Subclasses esphome::usb_host::USBClient so it inherits the USB host
// task, transfer-request pool, and lock-free event queue. We add a HID
// interface claim (bInterfaceClass==0x03, EP 0x01 OUT + EP 0x82 IN),
// the DSP-408 wire protocol on top of 64-byte HID reports, and a small
// command state machine that drives connect/identify/master-state probes.
//
// Threading: transfer callbacks run in the USB task. They copy raw IN
// reports into a lock-free queue and wake the main loop via
// enable_loop_soon_any_context() + App.wake_loop_threadsafe(). All
// protocol parsing and entity updates happen in loop() on the main task.
// Outbound transfer_out() is non-blocking and safe from main loop.

#if defined(USE_ESP32_VARIANT_ESP32P4) || defined(USE_ESP32_VARIANT_ESP32S2) || defined(USE_ESP32_VARIANT_ESP32S3)

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/usb_host/usb_host.h"
#include "esphome/core/event_pool.h"
#include "esphome/core/lock_free_queue.h"

#include "usb/usb_host.h"

#include "protocol.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

namespace esphome {

// Forward decls so the .h can mention pointers without dragging in
// the full sub-platform headers.
namespace text_sensor { class TextSensor; }
namespace text { class Text; }
namespace number { class Number; }
namespace switch_ { class Switch; }
namespace select { class Select; }

namespace dsp408 {

// One inbound USB report (64 bytes) carried from USB task to main loop
// via a lock-free SPSC queue. Sized for HID; matches the IDF transfer
// buffer configured by usb_host_client.cpp.
struct InReport {
  uint8_t data[64];
  uint16_t length;
  // Required by EventPool — POD with no resources to release.
  void release() {}
};

static constexpr size_t IN_QUEUE_SIZE = 16;

// State machine of the connect/identify/probe sequence.
// Driven by loop() reacting to inbound reports + by per-tick timeouts.
enum class StartupPhase : uint8_t {
  IDLE,                      // not yet usb-connected
  WAIT_AFTER_CONNECT,        // usb-connected, waiting before sending CMD_CONNECT
  SENT_CONNECT,              // CMD_CONNECT in flight
  SENT_GET_INFO,             // CMD_GET_INFO in flight
  SENT_GET_MASTER,           // CMD_MASTER read in flight
  WARMUP_CHANNELS,           // sequentially reading channel state 0..7
  RUNNING,                   // steady-state
};

class DSP408 : public usb_host::USBClient {
 public:
  DSP408(uint16_t vid, uint16_t pid) : USBClient(vid, pid) {}

  void setup() override;
  void loop() override;
  void on_connected() override;
  void on_disconnected() override;
  void dump_config() override;

  // Setters for sub-platform entities — wired by codegen in
  // {number,switch,text_sensor}/__init__.py.
  void set_model_text_sensor(text_sensor::TextSensor *ts) { this->model_ts_ = ts; }
  void set_master_volume_number(number::Number *n) { this->master_vol_num_ = n; }
  void set_master_mute_switch(switch_::Switch *s) { this->master_mute_sw_ = s; }
  void set_channel_volume_number(uint8_t ch, number::Number *n) {
    if (ch < 8) this->ch_vol_num_[ch] = n;
  }
  void set_channel_mute_switch(uint8_t ch, switch_::Switch *s) {
    if (ch < 8) this->ch_mute_sw_[ch] = s;
  }
  void set_channel_delay_number(uint8_t ch, number::Number *n) {
    if (ch < 8) this->ch_delay_num_[ch] = n;
  }
  void set_channel_polar_switch(uint8_t ch, switch_::Switch *s) {
    if (ch < 8) this->ch_polar_sw_[ch] = s;
  }
  void set_channel_hpf_freq_number(uint8_t ch, number::Number *n) {
    if (ch < 8) this->ch_hpf_freq_num_[ch] = n;
  }
  void set_channel_lpf_freq_number(uint8_t ch, number::Number *n) {
    if (ch < 8) this->ch_lpf_freq_num_[ch] = n;
  }
  void set_channel_hpf_filter_select(uint8_t ch, select::Select *s) {
    if (ch < 8) this->ch_hpf_filter_sel_[ch] = s;
  }
  void set_channel_hpf_slope_select(uint8_t ch, select::Select *s) {
    if (ch < 8) this->ch_hpf_slope_sel_[ch] = s;
  }
  void set_channel_lpf_filter_select(uint8_t ch, select::Select *s) {
    if (ch < 8) this->ch_lpf_filter_sel_[ch] = s;
  }
  void set_channel_lpf_slope_select(uint8_t ch, select::Select *s) {
    if (ch < 8) this->ch_lpf_slope_sel_[ch] = s;
  }
  void set_preset_name_text(text::Text *t) { this->preset_name_text_ = t; }
  void set_channel_name_text(uint8_t ch, text::Text *t) {
    if (ch < 8) this->ch_name_text_[ch] = t;
  }

  // Public command entry points (called from main loop / entity control()).
  // These post an OUT transfer; replies arrive asynchronously and update
  // entity state.
  void request_master_volume(float db);
  void request_master_mute(bool muted);
  void request_channel_volume(uint8_t ch, float db);
  void request_channel_mute(uint8_t ch, bool muted);
  void request_channel_delay(uint8_t ch, uint16_t samples);
  void request_channel_polar(uint8_t ch, bool inverted);
  void request_channel_hpf_freq(uint8_t ch, uint16_t hz);
  void request_channel_lpf_freq(uint8_t ch, uint16_t hz);
  void request_channel_hpf_filter(uint8_t ch, uint8_t filter);
  void request_channel_hpf_slope(uint8_t ch, uint8_t slope);
  void request_channel_lpf_filter(uint8_t ch, uint8_t filter);
  void request_channel_lpf_slope(uint8_t ch, uint8_t slope);

  // EQ band write — cmd = 0x10000 + (band << 8) + channel.
  // Q is converted to b4_byte via 256/Q with clamping to [1..255].
  void request_eq_band(uint8_t channel, uint8_t band, uint16_t freq_hz,
                       float gain_db, float q);

  // Routing matrix cell — sets the mixer level for one (channel, input)
  // pair. `level` is mapped: 0 = OFF, anything else = ON (0x64). Channel
  // is the OUTPUT (0..7), input_idx is 0..7 (low bank IN1..IN8).
  // For IN9..IN16 use the high bank API (DSP-408 hw only has 4 inputs;
  // most users won't need this).
  void request_routing(uint8_t channel, uint8_t input_idx, bool on);

  // Preset-name read/write (15-byte ASCII in cat=CAT_STATE).
  void request_preset_name(const std::string &name);

  // Channel name write (cmd=0x2400+ch, CAT_PARAM, 8-byte ASCII NUL-pad).
  void request_channel_name(uint8_t channel, const std::string &name);

 protected:
  // ───────── USB descriptor walk + interface claim ─────────────────────
  bool find_hid_endpoints_();

  // ───────── IN-pump (continuous interrupt-IN reads) ────────────────────
  void start_input_();
  // Static-style transfer callback wrapper (USB task context) — pushes
  // a copy of the report into the lock-free queue and wakes the loop.
  void on_in_report_usb_task_(const usb_host::TransferStatus &status);

  // Main-loop side: drain queue, parse frame, dispatch.
  void process_in_queue_();
  void dispatch_frame_(const InReport &report);

  // ───────── OUT submit ────────────────────────────────────────────────
  // Build + submit a single-frame command. Returns false on submission
  // failure (transfer pool exhausted etc.). Reply is asynchronous.
  bool send_cmd_(uint8_t direction, uint32_t cmd, uint8_t category,
                 const uint8_t *payload, size_t payload_len, uint8_t seq);

  // Convenience wrappers used by the startup state machine + entity setters.
  bool send_connect_();
  bool send_get_info_();
  bool send_get_master_();
  bool send_set_master_(uint8_t lvl_raw, bool muted);
  bool send_set_channel_(uint8_t ch);  // builds payload from ch_state_
  bool send_read_channel_(uint8_t ch);  // multi-frame 296-byte read
  bool send_set_crossover_(uint8_t ch);
  bool send_set_routing_(uint8_t ch);   // builds payload from ch_state_
  bool send_set_eq_band_(uint8_t ch, uint8_t band, uint16_t freq_hz,
                         int16_t gain_raw, uint8_t b4_byte);
  bool send_set_preset_name_(const std::string &name);
  bool send_get_preset_name_();
  bool send_set_channel_name_(uint8_t ch, const std::string &name);

  // ───────── Frame handlers ────────────────────────────────────────────
  void handle_connect_reply_(const uint8_t *payload, size_t len);
  void handle_get_info_reply_(const uint8_t *payload, size_t len);
  void handle_master_reply_(const uint8_t *payload, size_t len, bool is_ack);
  void handle_channel_write_ack_(uint32_t cmd, const uint8_t *payload, size_t len);
  void handle_channel_state_blob_(uint8_t ch, const uint8_t *blob, size_t len);
  void handle_crossover_write_ack_(uint32_t cmd, const uint8_t *payload, size_t len);
  void handle_preset_name_reply_(const uint8_t *payload, size_t len, bool is_ack);
  void handle_routing_write_ack_(uint32_t cmd, const uint8_t *payload, size_t len);
  void handle_eq_band_write_ack_(uint32_t cmd, const uint8_t *payload, size_t len);
  void handle_channel_name_write_ack_(uint32_t cmd, const uint8_t *payload, size_t len);

  // Publish the full set of cached state for one channel to all
  // attached entities (volume, mute, delay, polar, hpf/lpf).
  void publish_channel_state_(uint8_t ch);

  // ───────── Per-channel cached desired state ──────────────────────────
  // Mirror of dsp408-py's _channel_cache. We only have the WRITE payload
  // semantics (read replies are 296-byte multi-frame and not implemented
  // in v0.1) so we drive the device entirely from this cache and the
  // user's HA inputs.
  struct ChannelState {
    bool primed = false;       // true once we've read or written this channel
    int16_t db = 0;            // -60..0 dB
    bool muted = false;
    bool polar = false;        // phase invert
    uint16_t delay_samples = 0;
    uint8_t byte_254 = 0;      // semantic unknown; preserve verbatim
    uint8_t subidx = 0;        // speaker-role byte (blob[255])

    // Crossover (HPF + LPF)
    uint16_t hpf_freq_hz = 20;     // default lower bound
    uint8_t hpf_filter = 0;        // 0=BW 1=Bessel 2=LR (3=LR alias)
    uint8_t hpf_slope = 8;         // 8 = Off (default)
    uint16_t lpf_freq_hz = 20000;
    uint8_t lpf_filter = 0;
    uint8_t lpf_slope = 8;

    // Output routing matrix — IN1..IN8 levels (low bank). DSP-408 hw
    // exposes 4 physical inputs; cells 4..7 read 0 in practice.
    uint8_t routing_lo[8] = {};

    // 8-byte ASCII channel name (NUL-padded), from blob[288..295].
    char name[9] = {};  // 8 chars + NUL terminator for safe printing
  };
  ChannelState ch_state_[8];

  // Preset name (15-byte ASCII, NUL-terminated for safe printing).
  char preset_name_[16] = {};
  bool preset_name_known_ = false;

  // ───────── USB endpoint state ────────────────────────────────────────
  uint8_t intf_number_ = 0xFF;
  const usb_ep_desc_t *ep_in_ = nullptr;   // 0x82 typically
  const usb_ep_desc_t *ep_out_ = nullptr;  // 0x01 typically
  bool interface_claimed_ = false;
  std::atomic<bool> input_pending_{false};

  // ───────── Lock-free queue for USB-task -> main-loop reports ─────────
  EventPool<InReport, IN_QUEUE_SIZE - 1> in_pool_;
  LockFreeQueue<InReport, IN_QUEUE_SIZE> in_queue_;

  // ───────── Startup state machine ─────────────────────────────────────
  StartupPhase phase_ = StartupPhase::IDLE;
  uint32_t phase_started_ms_ = 0;
  uint32_t connected_at_ms_ = 0;

  // Sequence counter — 0 for writes (firmware drops non-zero seq on
  // category-04 writes per dsp408-py), incrementing per read.
  uint8_t next_read_seq_ = 1;

  // Pending request tracking — for v0.1 we keep it simple: at most one
  // request in flight at a time. cmd_in_flight_ == 0 means idle.
  uint32_t cmd_in_flight_ = 0;
  uint32_t cmd_in_flight_started_ms_ = 0;
  uint8_t cmd_in_flight_seq_ = 0;
  uint8_t cmd_in_flight_dir_ = 0;  // DIR_CMD or DIR_WRITE
  static constexpr uint32_t REQUEST_TIMEOUT_MS = 1500;

  // ───────── Entity pointers ───────────────────────────────────────────
  text_sensor::TextSensor *model_ts_ = nullptr;
  number::Number *master_vol_num_ = nullptr;
  switch_::Switch *master_mute_sw_ = nullptr;
  number::Number *ch_vol_num_[8] = {};
  switch_::Switch *ch_mute_sw_[8] = {};
  number::Number *ch_delay_num_[8] = {};
  switch_::Switch *ch_polar_sw_[8] = {};
  number::Number *ch_hpf_freq_num_[8] = {};
  number::Number *ch_lpf_freq_num_[8] = {};
  select::Select *ch_hpf_filter_sel_[8] = {};
  select::Select *ch_hpf_slope_sel_[8] = {};
  select::Select *ch_lpf_filter_sel_[8] = {};
  select::Select *ch_lpf_slope_sel_[8] = {};
  text::Text *preset_name_text_ = nullptr;
  text::Text *ch_name_text_[8] = {};

  // ───────── Cached master state (from last reply) ─────────────────────
  bool master_known_ = false;
  int8_t master_db_ = 0;       // dB integer
  bool master_muted_ = false;

  // Periodic re-poll of master state (catch external knob changes if any
  // future firmware has them).
  uint32_t last_master_poll_ms_ = 0;
  static constexpr uint32_t MASTER_POLL_INTERVAL_MS = 5000;

  // Round-robin which channel's cached state we re-publish each tick.
  // Burst-publishing all 8 × 6 entity states at once during the master
  // poll overruns the ESPHome native API send buffer when the entity
  // surface gets large (~60+ entities), causing HA's connection to drop.
  // Spreading it across MASTER_POLL_INTERVAL_MS / 8 = ~625 ms per
  // channel keeps each tick's message volume manageable.
  uint8_t republish_channel_rr_ = 0;
  uint32_t last_republish_ms_ = 0;
  static constexpr uint32_t REPUBLISH_STAGGER_MS = 625;

  // ───────── Multi-frame reassembly state ──────────────────────────────
  // The DSP-408 returns 296-byte payloads (cmd=0x77NN channel-state read)
  // as a sequence of 64-byte HID reports: the first carries the standard
  // header + 50 payload bytes (no chk/end), then 4 raw 64-byte
  // continuation reports until the declared length is satisfied. The
  // last continuation also carries chk + end + zero-pad after the final
  // payload byte. We mirror dsp408-py.transport.read_response semantics:
  // once we see a multi-frame first frame, we treat every subsequent
  // inbound report as raw payload until 296 bytes have been collected,
  // then re-dispatch the assembled blob.
  bool mf_in_progress_ = false;
  uint32_t mf_cmd_ = 0;
  uint8_t mf_direction_ = 0;
  uint8_t mf_category_ = 0;
  uint16_t mf_declared_len_ = 0;
  uint16_t mf_collected_len_ = 0;
  uint8_t mf_header_[10] = {};   // raw[4..14] from first frame for chk validation
  uint8_t mf_buffer_[CHANNEL_BLOB_SIZE] = {};

  // Warmup channel-read state (inside StartupPhase::WARMUP_CHANNELS).
  uint8_t warmup_channel_ = 0;   // next channel index to read (0..7)

  // Per-channel read convergence — mirrors dsp408-py's
  // ``read_channel_state(retry_on_divergence=True)``. The firmware
  // occasionally emits a 2-byte-shifted variant of the EQ region (and
  // sometimes worse — bench test Ch6 returned gain=+324 dB / subidx=0x55
  // i.e. 'U' which is clearly leaked from elsewhere in the blob). We
  // keep the previous blob's bytes and re-read until two consecutive
  // reads agree, up to MAX_BLOB_RETRY attempts.
  static constexpr uint8_t MAX_BLOB_RETRY = 4;
  uint8_t warmup_attempt_ = 0;
  bool warmup_have_prev_ = false;
  uint8_t warmup_prev_blob_[CHANNEL_BLOB_SIZE] = {};
};

}  // namespace dsp408
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32S2/S3/P4
