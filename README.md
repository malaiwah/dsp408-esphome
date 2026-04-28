# dsp408-esphome

ESPHome external_component that talks to a Dayton Audio **DSP-408** directly
from an **ESP32-S3**'s native USB host peripheral — no USBIP, no Linux client,
no kernel HID layer in the path. Plug the DSP-408 into the ESP32-S3's USB-OTG
port and ESPHome exposes the master volume, master mute, and per-channel
controls as standard Home Assistant entities.

This is the third leg of the DSP-408 integration project after
[dsp408-py](http://10.15.0.6:3300/malaiwah/dsp408-py) (the Python library
+ MQTT bridge) and
[dsp408-esp32-bridge](http://10.15.0.6:3300/malaiwah/dsp408-esp32-bridge)
(the USBIP-over-WiFi appliance). The USBIP path was abandoned after weeks of
fighting kernel HID timeouts and vhci-hcd disconnect cascades; everything
upstream of HID is much friendlier when the ESP32 firmware speaks the
device's wire protocol directly.

The protocol port is line-for-line traceable to dsp408-py's `protocol.py`
and `device.py` — same 64-byte HID frame layout, same XOR checksum, same
command codes, same field semantics.

## Status

**v0.1 — alpha**, bench-tested 2026-04-28. Currently exposes:

| Entity                   | Type          | Range / Notes |
| ------------------------ | ------------- | ------------- |
| Model identity           | `text_sensor` | `MYDW-AV1.06` on stock fw |
| Master volume            | `number`      | -60..+6 dB, 1 dB step |
| Master mute              | `switch`      | off=audible, on=muted |
| Channel volume (Ch1..8)  | `number`      | -60..0 dB, 1 dB step |
| Channel mute (Ch1..8)    | `switch`      | per-output mute |

Slated for follow-on iterations:

- Channel delay (0..359 samples)
- Per-channel name (8-byte ASCII)
- Crossover (HPF + LPF, freq + filter type + slope)
- Parametric EQ (10 bands × 8 channels with gain / freq / Q)
- Output routing matrix (4 inputs × 8 outputs)
- Preset name read/write
- Compressor / dynamics
- Input EQ / noisegate (the cat=0x03 plane)
- Multi-frame channel state read (296-byte blob) for state recovery on reconnect

## Hardware

- **ESP32-S3** with native USB peripheral exposed (DevKitC-1, ESP32-S3-Box,
  custom board). The ESP32-S3's USB pins are GPIO19 (D-) and GPIO20 (D+);
  these are dedicated and not remappable.
- **Dayton Audio DSP-408** (VID `0x0483`, PID `0x5750`) connected to the
  ESP32-S3's USB-OTG port. The DSP-408 is bus-powered from the host port,
  so the host port must supply 5V. The standard DevKitC-1 does **not**
  supply 5V on the OTG jack by default — you'll need either a powered
  USB-A breakout or to wire VBUS externally.
- **Console**: the ESP32-S3 has two USB peripherals sharing the same
  internal PHY. ESPHome's USB host stack needs the PHY in host mode, which
  conflicts with the boot-ROM USB-CDC console. Logs go out **UART0**
  (the on-board CP2102/CH340 bridge on the DevKitC-1's other USB-C jack)
  instead. The example YAML pins this via `CONFIG_ESP_CONSOLE_UART_DEFAULT`.

## Usage

```yaml
external_components:
  - source: github://malaiwah/dsp408-esphome  # or local path during dev

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
  - platform: dsp408
    dsp408_id: my_dsp
    kind: MODEL
    name: "DSP-408 Model"

number:
  - platform: dsp408
    dsp408_id: my_dsp
    kind: MASTER_VOLUME
    name: "Master Volume"
  - platform: dsp408
    dsp408_id: my_dsp
    kind: CHANNEL_VOLUME
    channel: 0
    name: "Ch1 Volume"

switch:
  - platform: dsp408
    dsp408_id: my_dsp
    kind: MASTER_MUTE
    name: "Master Mute"
  - platform: dsp408
    dsp408_id: my_dsp
    kind: CHANNEL_MUTE
    channel: 0
    name: "Ch1 Mute"
```

See [`examples/dsp408-test.yaml`](examples/dsp408-test.yaml) for a fully-
populated config (8 channels, WiFi, API, OTA) and
[`examples/dsp408-test-minimal.yaml`](examples/dsp408-test-minimal.yaml)
for a no-network bench-test variant that just outputs serial logs.

## Architecture

```
   HA / MQTT
      │
   API/OTA (WiFi)
      │
┌─────┴────────┐
│ ESPHome      │   esp32-s3
│  ┌─────────┐ │
│  │ entities│◄┼── number / switch / text_sensor
│  └────┬────┘ │
│       │      │
│  ┌────▼────┐ │   in-process function calls (main loop task)
│  │ DSP408  │◄┼── parses frames, drives state machine
│  └────┬────┘ │
│       │      │   esphome::usb_host::USBClient
│  ┌────▼────┐ │   non-blocking transfer_in / transfer_out
│  │usb_host │ │   lock-free queue → main loop, USB task at prio 5
│  └────┬────┘ │
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

## Design choices

- **No multi-frame reads in v0.1.** The 296-byte channel state blob
  (`cmd=0x77NN`) requires reassembling 5 sequential 64-byte HID reports
  and adds significant complexity to the state machine. We can drive the
  device entirely from a write-only protocol (master + per-channel writes)
  using local cached state, so v0.1 punts on reads. The blob is needed
  for state recovery after reconnect and for surface area like reading
  current EQ — that lands in v0.2.
- **Per-channel cache is RAM-only.** ESPHome's `restore_state` pattern
  could persist these to flash, but v0.1 just rebuilds the cache on
  every connect. This means a fresh boot sends "0 dB / unmuted" defaults
  to the DSP-408 the first time a user touches any channel control,
  which is fine for a bring-up release but should be addressed before
  it gets mounted somewhere users care about.
- **Master state is read-back-validated.** Master volume + mute round-trips
  cleanly (read returns 8 bytes including current state), so we re-poll
  every 5 seconds in steady state. This is a safety net for code paths
  we haven't yet covered (preset switch, factory reset, etc.).
- **Sequence number policy mirrors dsp408-py:** writes (`dir=0xA1`) use
  `seq=0` — empirically the firmware drops non-zero-seq writes on the
  output-param plane. Reads (`dir=0xA2`) get an incrementing seq and we
  match replies leniently because some firmware versions return `seq=0`
  for reads anyway.

## Building

```bash
cd examples
esphome run dsp408-test-minimal.yaml --device /dev/cu.usbserial-XXX
```

For dev iterations on the components directory itself, the YAML uses
`source: type: local, path: ../components` so edits to `components/dsp408/*`
are picked up by the next `esphome run`.

## License

[MIT](LICENSE).

## Related projects

- [dsp408-py](http://10.15.0.6:3300/malaiwah/dsp408-py) — Python protocol
  library and `dsp408-mqtt` bridge service that pioneered the wire-format
  reverse engineering
- [dsp408-esp32-bridge](http://10.15.0.6:3300/malaiwah/dsp408-esp32-bridge) —
  the (now-superseded) USBIP-over-WiFi appliance, useful for context on
  why the direct-HID-over-USBHost path is preferable
- [malaiwah-usbipdcpp_esp32](http://10.15.0.6:3300/malaiwah/malaiwah-usbipdcpp_esp32) —
  the underlying USBIP firmware fork, kept as a fallback for non-DSP-408
  USB devices on the home network (e.g. the DYMO label printer that
  works fine over USBIP because it's bulk, not interrupt-HID)
