# dsp408-esphome

ESPHome external_component that talks to a Dayton Audio **DSP-408** directly
from an **ESP32-S3**'s native USB host peripheral. Plug the DSP-408 into the
ESP32-S3's USB-OTG port and the device shows up in Home Assistant as a
proper integration with sliders, switches, selects and text fields — no MQTT
glue, no USBIP, no Linux client, no kernel HID layer in the path.

<p align="center">
  <img src="docs/screenshots/device-page.png" alt="DSP-408 Test device page in Home Assistant — full v0.3 surface with HPF type/slope selects, channel name text, sliders for volume / delay / freq, polar / mute switches"
       width="700">
</p>

> *Above: the auto-generated device page Home Assistant builds from the
> ESPHome native API surface. Per-channel volume / mute / polar / delay
> / HPF freq+type+slope / LPF freq+type+slope / name across all 8
> outputs, master volume + mute, preset name, and the firmware identity
> sensor — all live, all bidirectional.*

## Status

**1.0** (2026-04-28). Validated on an ESP32-S3 DevKitC-1 + a real
DSP-408 (firmware `MYDW-AV1.06`), driven from a live Home Assistant
instance over WiFi. CI for the host-side unit tests + ESPHome config
validation runs on every push.

| Entity                     | Type          | Range / Notes                                        |
| -------------------------- | ------------- | ---------------------------------------------------- |
| Model identity             | `text_sensor` | `MYDW-AV1.06` on stock fw                            |
| Master volume              | `number`      | -60..+6 dB, 1 dB step                                |
| Master mute                | `switch`      | off=audible, on=muted                                |
| Preset name                | `text`        | 15-byte ASCII, persisted in device flash             |
| Channel volume × 8         | `number`      | -60..0 dB, **0.5 dB step** (firmware native 0.1 dB)  |
| Channel mute × 8           | `switch`      | per-output mute                                      |
| Channel polar × 8          | `switch`      | 180° phase invert                                    |
| Channel delay × 8          | `number`      | 0..359 samples (≈ 8.143 ms @ 44.1 kHz)               |
| Channel HPF cutoff × 8     | `number`      | 10..20000 Hz                                         |
| Channel LPF cutoff × 8     | `number`      | 100..22000 Hz                                        |
| Channel HPF type × 8       | `select`      | Butterworth / Bessel / Linkwitz-Riley                |
| Channel HPF slope × 8      | `select`      | 6..48 dB/oct, plus "Off"                             |
| Channel LPF type × 8       | `select`      | Butterworth / Bessel / Linkwitz-Riley                |
| Channel LPF slope × 8      | `select`      | 6..48 dB/oct, plus "Off"                             |
| Channel name × 8           | `text`        | 8-byte ASCII                                         |
| Channel compressor × 8     | `number` × 3  | attack ms / release ms / threshold (firmware-inert in v1.06 but writes round-trip) |
| Input polar × 8            | `switch`      | per-input phase invert (cat=0x03 plane; the one firmware-active input field) |
| EQ band write              | service       | `set_eq_band(channel, band, freq_hz, gain_db, q)`    |
| Routing matrix write       | service       | `set_routing(channel, input_idx, enabled)`           |
| Save channel snapshot      | service       | `save_channel_snapshot(channel)` — multi-frame WRITE of the cached blob (296 B, 5 USB frames) |

EQ (10 bands × 8 channels = 80 cells × 3 fields each) and the routing
matrix (4 inputs × 8 outputs = 32 cells) are exposed as user-defined
**service actions** rather than entities — too numerous for a useful
HA dashboard, more natural as scripted calls.

On connect, the bridge reads each channel's full 296-byte state blob
(via the multi-frame `cmd=0x77NN` path, with read-divergence retry up
to 4 attempts) and populates entities from device truth. Writes are
gated on warmup completion + cache-primed status to prevent the
"fresh-boot HA touch surges channel to 0 dB" hazard.

### Future work (post-1.0)

- Bounded request queue for fast slider drag operations (currently
  the firmware can drop a few mid-flight writes if you whip a slider
  back and forth — most users won't hit this, but it's on the list).
- Submit upstream as an ESPHome external_component PR.

## What it looks like booting

```
[I][logger:120]: Log initialized
[I][app:067]: Running through setup()
[C][dsp408:041]: DSP-408 USB host client initialised (VID 0x0483 PID 0x5750)
[C][component:246]: Setup wifi took 57ms
[C][esphome.ota:071]:   Address: dsp408-test.local:3232
[C][api:235]: Server:
[C][api:235]:   Address: dsp408-test.local:6053
[I][dsp408:077]: Claimed HID interface 0, EP-IN=0x82 EP-OUT=0x01
[I][dsp408:579]: CONNECT ok (status=0x00)
[I][dsp408:594]: GET_INFO: 'MYDW-AV1.06'
[I][dsp408:621]: MASTER read: -5 dB (audible)
[D][dsp408:639]: Warmup: reading channel 0 (attempt 1)
[D][dsp408:665]: Ch0: read divergence — retry attempt 2
[D][dsp408:676]: Ch0: converged after 2 attempts
[I][dsp408:713]: Ch0: +0 dB (audible) delay=0  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x01
[D][dsp408:665]: Ch1: read divergence — retry attempt 2
[D][dsp408:665]: Ch1: read divergence — retry attempt 3
[D][dsp408:676]: Ch1: converged after 3 attempts
[I][dsp408:713]: Ch1: -1 dB (audible) delay=0  HPF=2500Hz/0/1  LPF=20000Hz/0/1  subidx=0x04
[I][dsp408:713]: Ch2: -34 dB (muted) (POLAR) delay=20  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x00
[I][dsp408:713]: Ch3: +0 dB (audible) delay=0  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x07
[I][dsp408:713]: Ch4: +0 dB (audible) delay=0  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x08
[I][dsp408:713]: Ch5: +0 dB (audible) delay=0  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x09
[I][dsp408:713]: Ch6: +324 dB (muted) delay=0  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x55
[I][dsp408:713]: Ch7: +0 dB (audible) delay=0  HPF=20Hz/0/1  LPF=20000Hz/0/1  subidx=0x12
[I][dsp408:731]: DSP-408 ready (full state read; startup took 1741 ms)
```

A few notable lines:

- **`MYDW-AV1.06`** is the firmware identity string returned by the
  DSP-408 GET_INFO response — exposed as a `text_sensor` in HA.
- **`Master read: -5 dB`** reflects the persisted state from the
  previous test session — the DSP-408 keeps master volume in flash.
- **`Ch1: read divergence — retry attempt 2/3`** is the multi-frame
  read retry kicking in. The firmware occasionally emits a 2-byte-
  shifted variant of the channel state blob; we re-read up to 4
  times until two consecutive reads agree (a port of dsp408-py's
  `retry_on_divergence`). Every channel typically converges within
  2-3 attempts.
- **`Ch6: +324 dB (muted) subidx=0x55`** is a known firmware quirk
  on this particular spare unit — Ch6 is the only channel that
  occasionally still misreports gain even with retries. Captured in
  the bring-up notes; harmless because writes go through fine and
  state can be corrected via HA.
- **`startup took 1741 ms`** — boot to first-control on a fresh
  reset. ~310 ms USB enumeration + ~100 ms identity probe + ~1.3 s
  for the 8-channel state read pass with retries.

## Hardware

- **ESP32-S3** with native USB peripheral exposed (DevKitC-1, ESP32-S3-Box,
  custom board). The ESP32-S3's USB pins are GPIO19 (D-) and GPIO20 (D+);
  these are dedicated and not remappable.
- **Dayton Audio DSP-408** (VID `0x0483`, PID `0x5750`) connected to the
  ESP32-S3's USB-OTG port. The DSP-408 is bus-powered; the host port must
  supply 5V. The standard DevKitC-1 does **not** supply 5V on the OTG jack
  by default — wire VBUS externally or use a powered USB-A breakout.
- **Console**: the ESP32-S3 has two USB peripherals sharing the same
  internal PHY. ESPHome's USB host stack needs the PHY in host mode, which
  conflicts with the boot-ROM USB-CDC console. Logs go out **UART0**
  (the on-board CP2102/CH340 bridge on the DevKitC-1's other USB-C jack).

## Usage

Minimal config:

```yaml
external_components:
  - source: github://malaiwah/dsp408-esphome

esp32:
  board: esp32-s3-devkitc-1
  variant: esp32s3
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_ESP_CONSOLE_UART_DEFAULT: y
      CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG: n
      CONFIG_ESP_CONSOLE_USB_CDC: n

usb_host:

dsp408:
  id: my_dsp

text_sensor:
  - { platform: dsp408, dsp408_id: my_dsp, kind: MODEL, name: "DSP-408 Model" }

number:
  - { platform: dsp408, dsp408_id: my_dsp, kind: MASTER_VOLUME, name: "Master Volume" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_VOLUME, channel: 0, name: "Ch1 Volume" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_DELAY,  channel: 0, name: "Ch1 Delay" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_HPF_FREQ, channel: 0, name: "Ch1 HPF" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_LPF_FREQ, channel: 0, name: "Ch1 LPF" }

switch:
  - { platform: dsp408, dsp408_id: my_dsp, kind: MASTER_MUTE,  name: "Master Mute" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_MUTE, channel: 0, name: "Ch1 Mute" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_POLAR, channel: 0, name: "Ch1 Polar" }

select:
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_HPF_FILTER, channel: 0, name: "Ch1 HPF Type" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_HPF_SLOPE,  channel: 0, name: "Ch1 HPF Slope" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_LPF_FILTER, channel: 0, name: "Ch1 LPF Type" }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_LPF_SLOPE,  channel: 0, name: "Ch1 LPF Slope" }

text:
  - { platform: dsp408, dsp408_id: my_dsp, kind: PRESET_NAME, name: "Preset Name", mode: text }
  - { platform: dsp408, dsp408_id: my_dsp, kind: CHANNEL_NAME, channel: 0, name: "Ch1 Name", mode: text }
```

### Service actions (EQ + routing)

```yaml
api:
  encryption:
    key: !secret api_encryption_key
  actions:
    - action: set_eq_band
      variables: { channel: int, band: int, freq_hz: int, gain_db: float, q: float }
      then:
        - lambda: |-
            id(my_dsp).request_eq_band(
                static_cast<uint8_t>(channel),
                static_cast<uint8_t>(band),
                static_cast<uint16_t>(freq_hz),
                gain_db, q);
    - action: set_routing
      variables: { channel: int, input_idx: int, enabled: bool }
      then:
        - lambda: |-
            id(my_dsp).request_routing(
                static_cast<uint8_t>(channel),
                static_cast<uint8_t>(input_idx),
                enabled);
```

From HA: **Developer Tools → Services →** `esphome.dsp408_test_set_eq_band`
or `set_routing`. Or call from automations:

```yaml
service: esphome.dsp408_test_set_eq_band
data:
  channel: 0
  band: 5
  freq_hz: 1000
  gain_db: -3.0
  q: 4.0
```

See [`examples/dsp408-test.yaml`](examples/dsp408-test.yaml) for the full
v0.3 reference config (8 channels, all entity types, EQ + routing
service actions, WiFi/API/OTA), and
[`examples/dsp408-test-minimal.yaml`](examples/dsp408-test-minimal.yaml)
for a no-network bench-test variant.

### Adopting in Home Assistant

ESPHome devices auto-discover via mDNS. After flashing, the DSP-408 shows
up under **Settings → Devices & Services → Discovered**. Click "Configure",
paste the API encryption key from your `secrets.yaml`, and HA will
register all entities automatically.

### HA automation cookbook

A few patterns that come up:

**Crossover for a 2-way (LR @ 2.5 kHz, 24 dB/oct):**

```yaml
service: number.set_value
data:
  entity_id:
    - number.dsp_408_test_ch1_lpf
    - number.dsp_408_test_ch2_hpf
  value: 2500
---
service: select.select_option
data:
  entity_id:
    - select.dsp_408_test_ch1_lpf_type
    - select.dsp_408_test_ch2_hpf_type
  option: Linkwitz-Riley
---
service: select.select_option
data:
  entity_id:
    - select.dsp_408_test_ch1_lpf_slope
    - select.dsp_408_test_ch2_hpf_slope
  option: 24 dB/oct
```

**Time-align a tweeter that's 5 cm closer than the woofer**
(at 44.1 kHz, 5 cm ≈ 6.5 samples):

```yaml
service: number.set_value
data:
  entity_id: number.dsp_408_test_ch1_delay
  value: 7  # samples; firmware caps at 359 (≈ 8.143 ms / 277 cm @ 44.1 kHz)
```

**Set EQ band 5 to a 1 kHz / -3 dB / Q=4 cut on Ch1:**

```yaml
service: esphome.dsp_408_test_set_eq_band
data:
  channel: 0
  band: 5
  freq_hz: 1000
  gain_db: -3.0
  q: 4.0
```

**Wire IN1 → Out1 + Out2 (stereo pair from a single mono input):**

```yaml
service: esphome.dsp_408_test_set_routing
data: { channel: 0, input_idx: 0, enabled: true }
---
service: esphome.dsp_408_test_set_routing
data: { channel: 1, input_idx: 0, enabled: true }
```

**Save Ch1 + Ch2 cached state to the device's flash** (read-modify-write):

```yaml
service: esphome.dsp_408_test_save_channel_snapshot
data: { channel: 0 }
---
service: esphome.dsp_408_test_save_channel_snapshot
data: { channel: 1 }
```

### Practical entity-count ceiling

The ESPHome native API has a soft ceiling around ~60 entities per device
on this HA build (2026.3.1 + ESPHome 2026.4.1). Past that you may see
HA-side `CONNECTION_CLOSED` loops because the entity-list message
overflows somewhere. Each `select` entity is "fatter" than a `number` or
`switch` because it carries its options strings.

The reference example demonstrates all entity TYPES while staying under
the ceiling — selects on Ch1 only, channel-name text on Ch1 only, the
"everyday" 6 controls (volume/mute/polar/delay/HPF/LPF) on all 8
channels. Extending selects + names to all 8 outputs is mechanical; if
you do, expect to either trim something else or wait for an upstream
ESPHome fix.

## Architecture

```
   HA / MQTT
      │
   API/OTA (WiFi)
      │
┌─────┴────────┐
│ ESPHome      │   esp32-s3
│  ┌─────────┐ │
│  │ entities│◄┼── number / switch / text_sensor / text / select
│  └────┬────┘ │
│       │      │   in-process function calls (main loop task)
│  ┌────▼────┐ │
│  │ DSP408  │◄┼── parses frames, drives state machine,
│  └────┬────┘ │   reads each channel on connect (with retry)
│       │      │
│  ┌────▼────┐ │   esphome::usb_host::USBClient
│  │usb_host │ │   non-blocking transfer_in / transfer_out
│  └────┬────┘ │   lock-free queue → main loop, USB task at prio 5
└───────┼──────┘
        │ EP 0x01 OUT (interrupt, 64-byte HID reports)
        │ EP 0x82 IN  (interrupt, 64-byte HID reports)
        │
   ┌────▼─────┐
   │ DSP-408  │
   └──────────┘
```

The threading model is inherited from ESPHome's `usb_host` component:
a high-priority FreeRTOS task pumps `usb_host_lib_handle_events`, transfer
callbacks fire on that task, and a lock-free SPSC queue carries inbound
HID reports to the main loop where parsing and entity updates run.
Outbound `transfer_out` is non-blocking and safe to call directly from
entity `control()` handlers on the main loop.

## Provenance

The wire-format port is line-for-line traceable to
[**dsp408-py**](https://github.com/malaiwah/dsp408-py) — same 64-byte
HID frame layout, same XOR checksum, same command codes, same blob
offsets, same field semantics. dsp408-py did the heavy lifting of
reverse-engineering the device against the Windows GUI captures; this
project just turns that knowledge into ESP32 firmware.

If you're starting from scratch and want to understand **why** any
particular byte means what it means, read dsp408-py first. Especially:

- [`dsp408/protocol.py`](https://github.com/malaiwah/dsp408-py/blob/main/dsp408/protocol.py)
  — frame definition, command codes, blob layout. Many docstrings
  reference the original Windows captures byte-for-byte.
- [`dsp408/device.py`](https://github.com/malaiwah/dsp408-py/blob/main/dsp408/device.py)
  — high-level Device class. Lots of "this firmware does X if you
  Y" calibration notes from live testing on hardware.

The `docs/peer-review-2026-04-28.md` file in this repo has a full
feature-parity matrix vs dsp408-py.

## Building + flashing

Initial flash (USB):

```bash
cd examples
esphome run dsp408-test.yaml --device /dev/cu.usbserial-XXX
```

Subsequent flashes go over WiFi via OTA — ESPHome handles this
automatically when you re-run `esphome run` and pass the device's
`*.local` hostname.

For dev iterations on the components directory itself, the YAML uses
`source: type: local, path: ../components` so edits to `components/dsp408/*`
are picked up by the next `esphome run`.

## Testing

Host-side unit tests cover the protocol layer (frame build/parse,
checksum, multi-frame parsing, payload encoders for master / channel
/ crossover / EQ band / routing / channel name / preset name):

```bash
cd tests && make run
```

Output:

```
RUN test_xor_checksum_basic                          ... OK
RUN test_constants                                   ... OK
RUN test_build_frame_get_info                        ... OK
RUN test_build_frame_master_write                    ... OK
RUN test_build_frame_channel_write                   ... OK
RUN test_build_frame_crossover_write                 ... OK
RUN test_build_frame_eq_band_write                   ... OK
RUN test_build_frame_routing_write                   ... OK
RUN test_build_frame_channel_name_write              ... OK
RUN test_build_frame_preset_name_write               ... OK
RUN test_eq_q_encoding                               ... OK
RUN test_round_trip_master_read                      ... OK
RUN test_round_trip_master_write                     ... OK
RUN test_parse_multi_frame_first                     ... OK
RUN test_parse_invalid_frames                        ... OK
RUN test_build_frame_payload_too_large               ... OK
RUN test_build_frames_multi_channel_state            ... OK
RUN test_build_frames_multi_single_frame_passthrough ... OK
RUN test_build_frame_compressor_write                ... OK
RUN test_build_frame_input_misc_write                ... OK
RUN test_channel_volume_0_1db_encoding               ... OK

All tests passed.
```

End-to-end validation against a live device + Home Assistant is
documented in [`docs/bring-up-2026-04-28.md`](docs/bring-up-2026-04-28.md).

## License

[MIT](LICENSE).

## Related projects

- [**dsp408-py**](https://github.com/malaiwah/dsp408-py) — upstream
  Python protocol library and `dsp408-mqtt` bridge. Pioneered the
  reverse engineering. Read this first if you want to understand the
  protocol.

The earlier USBIP-over-WiFi appliance approach (host-side `usbipd` →
Linux `vhci-hcd` → kernel HID layer → hidapi → application) ran into
intractable disconnect cascades under burst HID polling load. Master
writes used to take ~80–150 ms via that chain; with this direct-HID-
over-USB-Host firmware they're 36 ms end-to-end (HA → entity → USB
transfer → ack), with no kernel timeout window to lose to.
