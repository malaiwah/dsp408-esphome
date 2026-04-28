"""ESPHome external_component for the Dayton Audio DSP-408.

Talks to a USB-attached DSP-408 (VID 0x0483 / PID 0x5750) directly from
the ESP32-S3's native USB host peripheral. No USBIP, no Linux client,
no kernel HID layer in the path — wired straight at the HID-report
level using ESPHome's built-in ``usb_host`` component infrastructure.

The protocol is a port of dsp408-py (see https://gitea.malaiwah.com/
malaiwah/dsp408-py) — same 64-byte HID frame layout, same command
codes, same field semantics. v0.1 covers identity + master volume +
master mute + per-channel volume/mute basics; preset name, EQ, crossover,
and routing matrix are slated for follow-on iterations.
"""

import esphome.codegen as cg
from esphome.components.usb_host import register_usb_client, usb_device_schema
import esphome.config_validation as cv

CODEOWNERS = ["@malaiwah"]
DEPENDENCIES = ["esp32", "usb_host"]
AUTO_LOAD = ["usb_host"]
MULTI_CONF = True

# Used by sub-platform configs to attach entities to a specific DSP-408
# instance via ``dsp408_id:`` references.
CONF_DSP408_ID = "dsp408_id"

dsp408_ns = cg.esphome_ns.namespace("dsp408")
DSP408 = dsp408_ns.class_("DSP408", cg.Component)

# Default to the DSP-408's known USB IDs but allow override (e.g. if
# someone wants to use this on the sibling DSP-816 firmware which
# shares the protocol skeleton).
CONFIG_SCHEMA = usb_device_schema(DSP408, vid=0x0483, pid=0x5750)


async def to_code(config):
    var = await register_usb_client(config)
    return var
