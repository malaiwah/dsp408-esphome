"""DSP-408 number sub-platform.

Kinds:
  * MASTER_VOLUME    -- master volume in dB, range -60..+6 (1 dB step)
  * CHANNEL_VOLUME   -- per-channel output volume, -60..0 dB
                        (1 dB step; firmware accepts 0.1 dB resolution
                        but step=1 keeps HA UIs sane)
"""

import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL

from .. import CONF_DSP408_ID, DSP408, dsp408_ns

DEPENDENCIES = ["dsp408"]

CONF_KIND = "kind"
KIND_MASTER_VOLUME = "MASTER_VOLUME"
KIND_CHANNEL_VOLUME = "CHANNEL_VOLUME"

KINDS = {
    KIND_MASTER_VOLUME: KIND_MASTER_VOLUME,
    KIND_CHANNEL_VOLUME: KIND_CHANNEL_VOLUME,
}

DSP408Number = dsp408_ns.class_("DSP408Number", number.Number, cg.Component)


def _validate(config):
    kind = config[CONF_KIND]
    if kind == KIND_CHANNEL_VOLUME and CONF_CHANNEL not in config:
        raise cv.Invalid(
            "channel: 0..7 is required for kind: CHANNEL_VOLUME"
        )
    if kind == KIND_MASTER_VOLUME and CONF_CHANNEL in config:
        raise cv.Invalid("channel: must NOT be set for kind: MASTER_VOLUME")
    return config


CONFIG_SCHEMA = cv.All(
    number.number_schema(DSP408Number).extend(
        {
            cv.GenerateID(CONF_DSP408_ID): cv.use_id(DSP408),
            cv.Required(CONF_KIND): cv.enum(KINDS, upper=True),
            cv.Optional(CONF_CHANNEL): cv.int_range(min=0, max=7),
        }
    ),
    _validate,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_DSP408_ID])
    kind = config[CONF_KIND]
    if kind == KIND_MASTER_VOLUME:
        var = await number.new_number(
            config, min_value=-60.0, max_value=6.0, step=1.0
        )
        cg.add(var.set_parent(parent))
        cg.add(var.set_master())
        cg.add(parent.set_master_volume_number(var))
    else:  # CHANNEL_VOLUME
        ch = config[CONF_CHANNEL]
        var = await number.new_number(
            config, min_value=-60.0, max_value=0.0, step=1.0
        )
        cg.add(var.set_parent(parent))
        cg.add(var.set_channel(ch))
        cg.add(parent.set_channel_volume_number(ch, var))
