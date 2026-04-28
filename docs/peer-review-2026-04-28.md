# Peer review — 2026-04-28

Independent review of the v0.1 codebase (run by a sub-agent against
`/tmp/dsp408-py` as the canonical reference). Findings are reproduced
here verbatim with status annotations.

## What was addressed in v0.2

| # | Finding | Severity | Status |
|---|---|---|---|
| §2.1 | Channel writes use `CAT_STATE` (0x09) instead of `CAT_PARAM` (0x04) | Show-stopper | **Fixed** in `dsp408.cpp:486` — now matches Python `category_hint()` |
| §2.2 | No read-divergence retry on multi-frame channel reads | High | **Fixed** — `MAX_BLOB_RETRY=4` in `dsp408.cpp:566`, mirrors Python's `read_channel_state(retry_on_divergence=True)`. Bench observation: every channel needs at least 2 reads to converge; Ch6 needs 4 |
| §2.3 | `request_master_mute` before `master_known_=true` sends `lvl_raw=60` (0 dB) | High | **Fixed** — `dsp408.cpp:692` gates writes on `phase_==RUNNING` AND `master_known_=true` |
| §2.4 | `request_channel_*` before warmup completes silently writes default state | High | **Fixed** — same gate added to all `request_channel_*` (`dsp408.cpp:704-770`) using `ch_state_[ch].primed` |
| §2.5 | Crossover writes always send `slope=8 (Off)` on first use | High | **Fixed** by §2.4 (no writes before warmup) |
| §2.6 | `byte_254` handling differs from Python's hardcoded 0 | Low / docs | **Documented** in `dsp408.cpp` — preserve verbatim from device readback |
| §2.9 | No request serialization (rapid sliders could trip watchdog) | Low | Not yet — add bounded queue in v0.3 |
| §3 | Test coverage: unit tests for `build_frame`/`parse_frame` round-trips | High | **Added** in `tests/test_protocol.cpp` (11 tests, all pass) |

## Deferred to v0.3

- §2.7: 0.1 dB resolution on per-channel volume (currently rounded to 1 dB).
  Wire format supports it; UX trade-off is fine for v0.2.
- §2.8: Multi-frame WRITE support (296-byte channel-state replay).
  Required for `set_full_channel_state` / preset save.
- §2.10: `master_db_` widening from `int8_t`. Not needed yet.
- §2.11: Immediate post-write master poll. The 5-s periodic poll is
  fine for the current entity surface.
- Filter type / slope as select entities (currently the read-back
  values are preserved, but there's no HA-side way to change them
  without setting freq).
- 10-band parametric EQ (per-channel × per-band entities or service).
- Output routing matrix (4×8 mixer cells).
- Preset name + per-channel name read/write.
- Compressor / dynamics (firmware-inert in v1.06 anyway).
- Input-side processing (cat=0x03 plane).

## protocol.h additions still needed

These constants are in dsp408-py's `protocol.py` but missing from our
`protocol.h`. Adding them is mechanical; they unblock the v0.3
features above.

```cpp
// Routing matrix
static constexpr uint32_t CMD_ROUTING_BASE      = 0x2100;
static constexpr uint32_t CMD_ROUTING_HI_BASE   = 0x2200;
static constexpr uint8_t  ROUTING_ON  = 0x64;
static constexpr uint8_t  ROUTING_OFF = 0x00;

// Compressor + names
static constexpr uint32_t CMD_WRITE_COMPRESSOR_BASE   = 0x2300;
static constexpr uint32_t CMD_WRITE_CHANNEL_NAME_BASE = 0x2400;

// Crossover + EQ named symbols (we use literals today)
static constexpr uint32_t CMD_WRITE_CROSSOVER_BASE = 0x12000;
static constexpr uint32_t CMD_WRITE_EQ_BAND_BASE   = 0x10000;
static constexpr float    EQ_Q_BW_CONSTANT         = 256.0f;
static constexpr int      EQ_GAIN_RAW_OFFSET       = 600;
static constexpr int      EQ_GAIN_RAW_MIN          = 0;
static constexpr int      EQ_GAIN_RAW_MAX          = 1200;

// Globals + factory reset
static constexpr uint32_t CMD_WRITE_GLOBAL = 0x2000;
static constexpr uint8_t  FACTORY_RESET_PAYLOAD[8] = {
    0x06, 0x1F, 0x00, 0x00, 0x20, 0x4E, 0x00, 0x01,
};
static constexpr uint32_t CMD_RESET_MCU         = 0x60;
static constexpr uint32_t CMD_PRESET_SAVE_TRIGGER = 0x34;
static constexpr uint8_t  PRESET_SAVE_TRIGGER_BYTE = 0x01;

// Multi-frame channel state alias paths
static constexpr uint32_t CMD_WRITE_FULL_CHANNEL_LO_BASE = 0x10000;  // ch 0..3
static constexpr uint32_t CMD_WRITE_FULL_CHANNEL_HI_BASE = 0x0004;   // ch 4..7
static constexpr uint32_t CMD_FULL_CHANNEL_ALIAS_BASE    = 0x0000;

// Input-side
static constexpr uint32_t CMD_READ_INPUT_BASE          = 0x0077;
static constexpr uint32_t CMD_WRITE_INPUT_EQ_BAND_BASE = 0x0000;
static constexpr uint32_t CMD_WRITE_INPUT_MISC_BASE    = 0x0900;
static constexpr uint32_t CMD_WRITE_INPUT_DATAID10_BASE = 0x0A00;
static constexpr uint32_t CMD_WRITE_INPUT_NOISEGATE_BASE = 0x0B00;
static constexpr size_t   INPUT_BLOB_SIZE = 288;
static constexpr int      INPUT_CHANNEL_COUNT = 8;
```

The full peer-review report is on file in the v0.2 release notes.
