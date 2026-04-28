// Host-side unit tests for components/dsp408/protocol.h
//
// Doesn't depend on ESPHome or ESP-IDF — protocol.h is a pure header
// of compute-only frame builders/parsers. Build:
//
//     c++ -std=c++17 -Wall -Wextra -I../components/dsp408 \
//         -o test_protocol test_protocol.cpp && ./test_protocol
//
// All tests use plain assert() — exit non-zero on first failure.

#define ESPHOME_LOG_HAS_TRACE 0
#include "protocol.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace esphome::dsp408;

#define RUN(name) do { printf("RUN %s\n", #name); name(); printf("OK  %s\n", #name); } while (0)

// ────────────────────────────────────────────────────────────────────────
// Helpers
// ────────────────────────────────────────────────────────────────────────

static void hexdump(const uint8_t *buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    printf("%02x ", buf[i]);
    if ((i & 0x0F) == 0x0F) printf("\n");
  }
  printf("\n");
}

static bool eq_bytes(const uint8_t *a, const uint8_t *b, size_t n,
                     const char *label) {
  for (size_t i = 0; i < n; i++) {
    if (a[i] != b[i]) {
      printf("Mismatch in %s at byte %zu: got 0x%02X want 0x%02X\n",
             label, i, a[i], b[i]);
      printf("got:\n"); hexdump(a, n);
      printf("want:\n"); hexdump(b, n);
      return false;
    }
  }
  return true;
}

// ────────────────────────────────────────────────────────────────────────
// XOR checksum sanity
// ────────────────────────────────────────────────────────────────────────

static void test_xor_checksum_basic() {
  // XOR of 0..0xFF is 0
  uint8_t buf[256];
  for (int i = 0; i < 256; i++) buf[i] = static_cast<uint8_t>(i);
  assert(xor_checksum(buf, 256) == 0);

  // XOR of single byte == that byte
  uint8_t one[1] = {0x42};
  assert(xor_checksum(one, 1) == 0x42);

  // XOR over zero bytes == 0
  assert(xor_checksum(buf, 0) == 0);
}

// ────────────────────────────────────────────────────────────────────────
// CMD_GET_INFO frame: dir=0xA2 cat=0x09 cmd=0x04 seq=2 len=0
// Captured byte-exact from dsp408-py output.
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_get_info() {
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_CMD, 2, CMD_GET_INFO, CAT_STATE,
                        nullptr, 0);
  assert(ok);

  // Expected bytes:
  //   80 80 80 ee  (magic)
  //   a2 01 02 09  (dir=a2 ver=01 seq=02 cat=09)
  //   04 00 00 00  (cmd=0x04 LE)
  //   00 00        (len=0 LE)
  //   chk=XOR(a2,01,02,09,04,00,00,00,00,00)
  //     = a2^01^02^09^04 = 0xAC
  //   end=aa
  //   pad zeroes to 64
  uint8_t want[FRAME_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA2, 0x01, 0x02, 0x09,
      0x04, 0x00, 0x00, 0x00,
      0x00, 0x00,
      0xAC, 0xAA,
  };
  // Bytes 16..63 are zero
  assert(eq_bytes(out, want, 16, "get_info_frame_first_16"));
  for (size_t i = 16; i < FRAME_SIZE; i++) assert(out[i] == 0);
}

// ────────────────────────────────────────────────────────────────────────
// Master volume write: dir=0xA1 cat=0x09 cmd=0x05 seq=0
// payload = [lvl, 00, 00, 32, 00, 32, mute, 00]
// for db=-20, muted=false:  lvl_raw = -20 + 60 = 40
//   payload = [28 00 00 32 00 32 01 00]
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_master_write() {
  uint8_t payload[8] = {40, 0x00, 0x00, 0x32, 0x00, 0x32, 0x01, 0x00};
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_WRITE, 0, CMD_MASTER, CAT_STATE,
                        payload, sizeof(payload));
  assert(ok);
  // header[0..14)
  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x09,
      0x05, 0x00, 0x00, 0x00,
      0x08, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "master_write_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 8, "master_write_payload"));
  // chk over [direction .. last payload byte] = bytes 4..21 inclusive
  uint8_t expected_chk = xor_checksum(out + 4, HEADER_SIZE - 4 + 8);
  assert(out[HEADER_SIZE + 8] == expected_chk);
  assert(out[HEADER_SIZE + 8 + 1] == END_MARKER);
}

// ────────────────────────────────────────────────────────────────────────
// Channel write: dir=0xA1 cat=0x04 cmd=0x1F00..0x1F07 seq=0
// payload = [enable, polar, gain_le16, delay_le16, byte254, subidx]
// for ch=0, db=-15, muted=false, polar=false, delay=100:
//   enable=1, polar=0, gain=(-15*10+600)=450 -> 0xC2 0x01,
//   delay=100=0x64 0x00, byte254=0, subidx=0x01
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_channel_write() {
  uint8_t payload[8] = {0x01, 0x00, 0xC2, 0x01, 0x64, 0x00, 0x00, 0x01};
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_WRITE, 0, CMD_WRITE_CHANNEL_BASE + 0,
                        CAT_PARAM, payload, sizeof(payload));
  assert(ok);

  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x04,
      0x00, 0x1F, 0x00, 0x00,
      0x08, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "ch0_write_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 8, "ch0_write_payload"));
  uint8_t expected_chk = xor_checksum(out + 4, HEADER_SIZE - 4 + 8);
  assert(out[HEADER_SIZE + 8] == expected_chk);
  assert(out[HEADER_SIZE + 8 + 1] == END_MARKER);
}

// ────────────────────────────────────────────────────────────────────────
// Crossover write: dir=0xA1 cat=0x04 cmd=0x12000..0x12007
// payload = [hpf_freq_le16, hpf_filter, hpf_slope, lpf_freq_le16, lpf_filter, lpf_slope]
// for ch=2, hpf=80 BW @ 24 dB/oct, lpf=4000 BW @ 24 dB/oct:
//   hpf_freq=80 -> 0x50 0x00, hpf_filter=0, hpf_slope=3
//   lpf_freq=4000 -> 0xA0 0x0F, lpf_filter=0, lpf_slope=3
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_crossover_write() {
  uint8_t payload[8] = {0x50, 0x00, 0x00, 0x03, 0xA0, 0x0F, 0x00, 0x03};
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_WRITE, 0, /*cmd=*/0x12002,
                        CAT_PARAM, payload, sizeof(payload));
  assert(ok);

  // cmd=0x12002 LE = 02 20 01 00
  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x04,
      0x02, 0x20, 0x01, 0x00,
      0x08, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "xover_ch2_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 8, "xover_ch2_payload"));
}

// ────────────────────────────────────────────────────────────────────────
// Round-trip: build a frame, parse it, verify fields agree.
// ────────────────────────────────────────────────────────────────────────

static void test_round_trip_master_read() {
  uint8_t out[FRAME_SIZE];
  build_frame(out, DIR_CMD, 7, CMD_MASTER, CAT_STATE, nullptr, 0);
  ParsedFrame f = parse_frame(out, FRAME_SIZE);
  assert(f.valid);
  assert(f.direction == DIR_CMD);
  assert(f.seq == 7);
  assert(f.category == CAT_STATE);
  assert(f.cmd == CMD_MASTER);
  assert(f.payload_len == 0);
  assert(!f.is_multi_frame_first);
  assert(f.checksum_ok);
  assert(f.payload_bytes_in_frame == 0);
}

static void test_round_trip_master_write() {
  uint8_t payload[8] = {60, 0x00, 0x00, 0x32, 0x00, 0x32, 0x01, 0x00};
  uint8_t out[FRAME_SIZE];
  build_frame(out, DIR_WRITE, 0, CMD_MASTER, CAT_STATE,
              payload, sizeof(payload));
  ParsedFrame f = parse_frame(out, FRAME_SIZE);
  assert(f.valid);
  assert(f.direction == DIR_WRITE);
  assert(f.cmd == CMD_MASTER);
  assert(f.payload_len == 8);
  assert(f.payload_bytes_in_frame == 8);
  assert(f.checksum_ok);
  for (size_t i = 0; i < 8; i++) assert(f.payload[i] == payload[i]);
}

// Synthesize a multi-frame FIRST report (declared len=296). Verify the
// parser flags it as multi-frame-first and returns 50 payload bytes
// (no chk validation since chk lives in the last continuation).

static void test_parse_multi_frame_first() {
  uint8_t raw[FRAME_SIZE];
  // Manually craft: header with payload_len=296 (0x0128).
  std::memset(raw, 0, FRAME_SIZE);
  raw[0] = 0x80; raw[1] = 0x80; raw[2] = 0x80; raw[3] = 0xEE;
  raw[4] = DIR_RESP; raw[5] = PROTO_VERSION; raw[6] = 0; raw[7] = CAT_PARAM;
  raw[8] = 0x00; raw[9] = 0x77; raw[10] = 0x00; raw[11] = 0x00;  // cmd=0x7700
  raw[12] = 0x28; raw[13] = 0x01;  // len=0x0128 = 296
  // 50 bytes of payload (the rest of the 64-byte report)
  for (int i = 0; i < 50; i++) raw[HEADER_SIZE + i] = static_cast<uint8_t>(i);

  ParsedFrame f = parse_frame(raw, FRAME_SIZE);
  assert(f.valid);
  assert(f.is_multi_frame_first);
  assert(f.payload_len == 296);
  assert(f.payload_bytes_in_frame == 50);
  for (int i = 0; i < 50; i++)
    assert(f.payload[i] == static_cast<uint8_t>(i));
}

// Verify rejection of garbled magic / wrong version / too-short raw.

static void test_parse_invalid_frames() {
  uint8_t raw[FRAME_SIZE] = {};
  // Wrong magic
  raw[0] = 0x12;
  ParsedFrame f = parse_frame(raw, FRAME_SIZE);
  assert(!f.valid);

  // Too short
  raw[0] = 0x80; raw[1] = 0x80; raw[2] = 0x80; raw[3] = 0xEE;
  f = parse_frame(raw, 8);  // less than HEADER_SIZE+2
  assert(!f.valid);

  // Wrong version byte
  raw[5] = 0x99;
  f = parse_frame(raw, FRAME_SIZE);
  assert(!f.valid);
}

// ────────────────────────────────────────────────────────────────────────
// Build_frame error paths
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_payload_too_large() {
  uint8_t out[FRAME_SIZE];
  uint8_t big[FRAME_SIZE]; std::memset(big, 0xFF, sizeof(big));
  // 50 bytes is too big for single-frame (max 48); should reject.
  bool ok = build_frame(out, DIR_WRITE, 0, CMD_MASTER, CAT_STATE, big, 50);
  assert(!ok);
}

// ────────────────────────────────────────────────────────────────────────
// EQ band write — cmd = 0x10000 + (band << 8) + channel
// payload = [freq_le16, gain_raw_le16, b4_byte, 0, 0, 0]
// for ch=2, band=4, freq=500 Hz, gain=-3 dB (raw = -30 + 600 = 570),
//          Q=4.0 (b4 = round(256/4) = 64):
//   payload = [F4 01 3A 02 40 00 00 00]
//   cmd     = 0x10000 + (4 << 8) + 2 = 0x10402
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_eq_band_write() {
  uint8_t payload[8] = {0xF4, 0x01, 0x3A, 0x02, 0x40, 0x00, 0x00, 0x00};
  uint8_t out[FRAME_SIZE];
  uint32_t cmd = CMD_WRITE_EQ_BAND_BASE + (4u << 8) + 2u;  // 0x10402
  bool ok = build_frame(out, DIR_WRITE, 0, cmd, CAT_PARAM, payload, sizeof(payload));
  assert(ok);

  // cmd=0x10402 LE = 02 04 01 00
  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x04,
      0x02, 0x04, 0x01, 0x00,
      0x08, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "eq_band_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 8, "eq_band_payload"));
}

// ────────────────────────────────────────────────────────────────────────
// Routing write — cmd = 0x2100 + ch, 8-byte payload (IN1..IN8 levels).
// for ch=0 with IN1 ON, others OFF:
//   payload = [64 00 00 00 00 00 00 00]
//   cmd     = 0x2100
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_routing_write() {
  uint8_t payload[8] = {ROUTING_ON, 0, 0, 0, 0, 0, 0, 0};
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_WRITE, 0, CMD_ROUTING_BASE + 0, CAT_PARAM,
                        payload, sizeof(payload));
  assert(ok);

  // cmd=0x2100 LE = 00 21 00 00
  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x04,
      0x00, 0x21, 0x00, 0x00,
      0x08, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "routing_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 8, "routing_payload"));
}

// ────────────────────────────────────────────────────────────────────────
// Per-channel name write — cmd = 0x2400 + ch, 8-byte ASCII NUL-pad.
// for ch=0, name="Tweet":
//   payload = ['T' 'w' 'e' 'e' 't' 00 00 00]
//   cmd     = 0x2400
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_channel_name_write() {
  uint8_t payload[8] = {'T', 'w', 'e', 'e', 't', 0, 0, 0};
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_WRITE, 0, CMD_WRITE_CHANNEL_NAME_BASE + 0,
                        CAT_PARAM, payload, sizeof(payload));
  assert(ok);

  // cmd=0x2400 LE = 00 24 00 00
  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x04,
      0x00, 0x24, 0x00, 0x00,
      0x08, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "channel_name_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 8, "channel_name_payload"));
}

// ────────────────────────────────────────────────────────────────────────
// Preset-name write — cmd = 0x00, cat=0x09, 15-byte ASCII NUL-pad.
// ────────────────────────────────────────────────────────────────────────

static void test_build_frame_preset_name_write() {
  uint8_t payload[15] = {'B','e','n','c','h',0,0,0,0,0,0,0,0,0,0};
  uint8_t out[FRAME_SIZE];
  bool ok = build_frame(out, DIR_WRITE, 0, CMD_PRESET_NAME, CAT_STATE,
                        payload, sizeof(payload));
  assert(ok);
  uint8_t want_hdr[HEADER_SIZE] = {
      0x80, 0x80, 0x80, 0xEE,
      0xA1, 0x01, 0x00, 0x09,
      0x00, 0x00, 0x00, 0x00,
      0x0F, 0x00,
  };
  assert(eq_bytes(out, want_hdr, HEADER_SIZE, "preset_name_header"));
  assert(eq_bytes(out + HEADER_SIZE, payload, 15, "preset_name_payload"));
}

// ────────────────────────────────────────────────────────────────────────
// EQ Q ↔ b4_byte encoding: b4 ≈ 256 / Q, clamped to [1..255].
// ────────────────────────────────────────────────────────────────────────

static void test_eq_q_encoding() {
  // Spot-check the encoding constants.
  assert(EQ_Q_BW_CONSTANT == 256.0f);
  assert(EQ_GAIN_RAW_OFFSET == 600);
  assert(EQ_GAIN_RAW_MIN == 0);
  assert(EQ_GAIN_RAW_MAX == 1200);

  // Default firmware Q=4.9 → b4 ≈ 256/4.9 ≈ 52 (matches dsp408-py docstring)
  int b4 = static_cast<int>(EQ_Q_BW_CONSTANT / 4.9f + 0.5f);
  assert(b4 == 52);
  // Q=2 → b4=128
  b4 = static_cast<int>(EQ_Q_BW_CONSTANT / 2.0f + 0.5f);
  assert(b4 == 128);
  // Q=8 → b4=32
  b4 = static_cast<int>(EQ_Q_BW_CONSTANT / 8.0f + 0.5f);
  assert(b4 == 32);
}

// ────────────────────────────────────────────────────────────────────────
// Constant sanity
// ────────────────────────────────────────────────────────────────────────

static void test_constants() {
  assert(FRAME_SIZE == 64);
  assert(HEADER_SIZE == 14);
  assert(DIR_CMD == 0xA2);
  assert(DIR_WRITE == 0xA1);
  assert(DIR_RESP == 0x53);
  assert(DIR_WRITE_ACK == 0x51);
  assert(CAT_STATE == 0x09);
  assert(CAT_PARAM == 0x04);
  assert(CAT_INPUT == 0x03);
  assert(CMD_CONNECT == 0xCC);
  assert(CMD_GET_INFO == 0x04);
  assert(CMD_MASTER == 0x05);
  assert(CMD_WRITE_CHANNEL_BASE == 0x1F00);
  assert(CMD_READ_CHANNEL_BASE == 0x7700);
  assert(CHANNEL_BLOB_SIZE == 296);
  assert(OFF_GAIN == 250);
  assert(OFF_HPF_FREQ == 256);
  assert(OFF_LPF_FREQ == 260);
  assert(OFF_NAME == 288);
  assert(MASTER_LEVEL_OFFSET == 60);
  assert(CHANNEL_VOL_OFFSET == 600);
  assert(CHANNEL_DELAY_MAX == 359);
  // Default subidx table
  assert(CHANNEL_SUBIDX_DEFAULT[0] == 0x01);
  assert(CHANNEL_SUBIDX_DEFAULT[7] == 0x12);
}

int main() {
  RUN(test_xor_checksum_basic);
  RUN(test_constants);
  RUN(test_build_frame_get_info);
  RUN(test_build_frame_master_write);
  RUN(test_build_frame_channel_write);
  RUN(test_build_frame_crossover_write);
  RUN(test_build_frame_eq_band_write);
  RUN(test_build_frame_routing_write);
  RUN(test_build_frame_channel_name_write);
  RUN(test_build_frame_preset_name_write);
  RUN(test_eq_q_encoding);
  RUN(test_round_trip_master_read);
  RUN(test_round_trip_master_write);
  RUN(test_parse_multi_frame_first);
  RUN(test_parse_invalid_frames);
  RUN(test_build_frame_payload_too_large);
  printf("\nAll tests passed.\n");
  return 0;
}
