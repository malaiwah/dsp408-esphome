"""DSP-408 switch sub-platform.

Kinds:
  * MASTER_MUTE      -- toggle master mute (off = audible, on = muted)
  * CHANNEL_MUTE     -- per-output-channel mute (off = audible, on = muted)
"""

import esphome.codegen as cg
from esphome.components import switch as switch_
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL

from .. import CONF_DSP408_ID, DSP408, dsp408_ns

DEPENDENCIES = ["dsp408"]

CONF_KIND = "kind"
KIND_MASTER_MUTE = "MASTER_MUTE"
KIND_CHANNEL_MUTE = "CHANNEL_MUTE"

KINDS = {
    KIND_MASTER_MUTE: KIND_MASTER_MUTE,
    KIND_CHANNEL_MUTE: KIND_CHANNEL_MUTE,
}

DSP408Switch = dsp408_ns.class_("DSP408Switch", switch_.Switch, cg.Component)


def _validate(config):
    kind = config[CONF_KIND]
    if kind == KIND_CHANNEL_MUTE and CONF_CHANNEL not in config:
        raise cv.Invalid("channel: 0..7 is required for kind: CHANNEL_MUTE")
    if kind == KIND_MASTER_MUTE and CONF_CHANNEL in config:
        raise cv.Invalid("channel: must NOT be set for kind: MASTER_MUTE")
    return config


CONFIG_SCHEMA = cv.All(
    switch_.switch_schema(DSP408Switch).extend(
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
    var = await switch_.new_switch(config)
    cg.add(var.set_parent(parent))
    if config[CONF_KIND] == KIND_MASTER_MUTE:
        cg.add(var.set_master())
        cg.add(parent.set_master_mute_switch(var))
    else:
        ch = config[CONF_CHANNEL]
        cg.add(var.set_channel(ch))
        cg.add(parent.set_channel_mute_switch(ch, var))
