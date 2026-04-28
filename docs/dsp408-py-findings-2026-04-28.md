# dsp408-py findings — relayed from dsp408-esphome port

While porting `dsp408-py` to a C++ ESPHome external_component (see
[`dsp408-esphome`](http://10.15.0.6:3300/malaiwah/dsp408-esphome)) we did a
careful read of `dsp408/protocol.py` and `dsp408/device.py`. A peer-review
agent flagged a handful of issues in the Python upstream worth surfacing
to the dsp408-py thread. None are show-stoppers — everything below is
"this could bite us later" not "this is broken right now."

Listed in rough order of "likeliest to surprise a future maintainer."

## 1. `device.py:1186-1245` — `set_channel(...)` with `polar=None` defaults to False instead of priming the cache

`set_channel_volume` / `set_channel_mute` / `set_channel_polar` (the
"single-field" wrappers) call `_prime_channel_cache()` first to load the
real device state. But `set_channel()` itself, when called with
`polar=None`, falls back to `self._channel_cache[channel]['polar']` —
which is `False` if the channel was never primed. So a caller that does:

```python
dsp.set_channel(0, db=-10.0, muted=False)   # polar omitted
```

…on a freshly-opened Device silently flips the channel back to
non-inverted — even if the user previously had it inverted on the
device. The docstring at lines 1216–1219 says polar=None preserves the
cached value, which is true *given the cache is primed*; if it isn't,
the "cached value" is the default False.

**Suggested fix**: in `set_channel`, if `polar is None` and the channel
hasn't been primed yet, call `_prime_channel_cache(channel)` before
reading from the cache. One extra read (~20 ms) that prevents a real
audio surprise.

## 2. `device.py:1490-1500` — `load_factory_preset(preset_id)` is documented KNOWN-BROKEN but only validates `preset_id` range

The docstring is upfront ("This is known-broken — the firmware silently
ignores the request") but the method still happily fires off the write
and returns. A user reading the source might assume the call works
because no exception fires.

**Suggested fix**: raise `NotImplementedError` (or
`FirmwareLimitationError` if you have a domain class) with the same
explanation that's currently in the docstring. Lets callers gate on
the limitation programmatically.

## 3. `device.py:603-610` — `_exchange()` forces `seq=0` for ALL writes, but the comment narrowly cites `cat=0x04`

```python
seq = 0 if direction == DIR_WRITE else self._next_seq()
```

The inline comment immediately above says:

> WRITES (dir=a1) MUST use seq=0 — the device silently drops writes
> with non-zero seq on cat=0x04 cmds (per-channel volume, routing
> matrix, EQ).

But the rule applies to every category, not just `cat=0x04`. The code
is correct for all observed cases (CAT_STATE writes also use seq=0 in
captures), but a future maintainer reading the comment and seeing a
need to send a non-zero-seq write to a CAT_STATE cmd might think
they're allowed.

**Suggested fix**: either narrow the rule with a `category==CAT_PARAM`
guard (and verify experimentally that CAT_STATE writes accept non-zero
seq), or update the comment to "verified empirically that all writes
use seq=0 across every category we've tested."

## 4. `protocol.py:648` — `OFF_EQ_MODE = OFF_BYTE_252` deprecated alias still exported

```python
OFF_EQ_MODE     = OFF_BYTE_252  # DEPRECATED alias — will be removed; the
                                # name is misleading (see OFF_BYTE_252).
```

Still in `__all__` (line 832 = `"OFF_EQ_MODE"`). The "will be removed"
comment has aged.

**Suggested fix**: drop the alias from `__all__`, leave the symbol with
a deprecation warning at module-import time:

```python
import warnings as _warnings
def __getattr__(name):
    if name == "OFF_EQ_MODE":
        _warnings.warn(
            "OFF_EQ_MODE is deprecated, use OFF_BYTE_252",
            DeprecationWarning, stacklevel=2)
        return OFF_BYTE_252
    raise AttributeError(name)
```

## 5. `device.py:2113-2127` — `save_preset(...)` reassembles channel state with default retry budget; one bad blob corrupts EQ in flash

`save_preset(name)` does:

```python
for ch in range(8):
    blob = self.read_channel_state(ch)              # default retry_on_divergence=True, max_attempts=4
    self.set_full_channel_state(ch, blob)
```

The `read_channel_state` retry usually converges, but the docstring at
lines 798–810 admits the firmware's residual EQ-region quirk does
sometimes win the convergence race in 4 attempts. A 296-byte write of
the divergent blob would commit the corrupted EQ to flash.

**Suggested fix**: bump `max_attempts` to 8 here, AND/OR validate that
blob[0..79] (the EQ region) is byte-stable across two reads before
writing. The semantic region (248..295) is reliable per the docstring,
so you could read twice, take the latest, but only commit if the EQ
regions agree.

## 6. `device.py:732` — `connect()` returns `payload[0]` without a non-zero check

```python
def connect(self, *, warmup: bool = True) -> int:
    reply = self.read_raw(cmd=CMD_CONNECT, category=CAT_STATE)
    if not reply.payload:
        raise ProtocolError("CONNECT: empty payload")
    if warmup:
        ...
    return reply.payload[0]
```

The docstring says `0x00 = ok` but doesn't enumerate non-zero values.
Empirically nothing else has been observed, but if a future firmware
returns e.g. `0x01` for "already connected" callers won't notice.

**Suggested fix**: `if status != 0: raise ProtocolError(f"CONNECT
status 0x{status:02X}")` (or log a warning). Cheap defensive coding.

## 7. `device.py:1707-1711` — `set_eq_band` uses `CHANNEL_VOL_OFFSET` for EQ gain encoding

The constant value (600) is correct for both encodings, but if EQ gain
ever diverges from channel volume (e.g. firmware exposes ±60 dB for EQ
vs −60..0 dB for channel — which the docstring already hints at via
`EQ_GAIN_RAW_MAX = 1200`), the shared constant becomes a footgun.

**Suggested fix**: define `EQ_GAIN_RAW_OFFSET = 600` in `protocol.py`
alongside `EQ_GAIN_RAW_MIN`/`MAX` and use that here.

## 8. (FYI, no change needed) `device.py:603` — `_exchange` holds `_lock` across long writes

The `with self._lock:` block holds the lock across both the OUT submit
AND the read deadline (potentially 2-3 seconds for `factory_reset`). If
a second thread calls `close()` while a long write is outstanding,
`close()` will block until the write completes.

That's actually correct (avoids the `RuntimeError: device is closed`
mid-write that an aggressive close might cause), and `close()`'s
docstring at line 522 calls it out. Just flagging for future
refactoring — if anyone tries to "fix" this by narrowing the lock
they'll create a use-after-free.

## 9. Stale references to "captures-needed-from-windows.md on the reverse-engineering branch"

`device.py:21-46` and a few other places reference doc paths that may
or may not still exist in main. If the RE branch was merged, the paths
work; if not, consider committing a stub of those captures-needed
notes to main so the references resolve.

(Lower priority — purely doc hygiene.)

---

## What we DIDN'T find

- The `category_hint()` mapping in `protocol.py:65-86` agrees with all
  observed Windows-GUI captures. No miscategorisations spotted.
- The blob-offset table in `protocol.py:633-695` matches what we read
  back live from the bench DSP-408 (after porting it byte-for-byte).
  Channels 0–5 + 7 round-trip exactly; Ch6 still occasionally returns
  `+324 dB / subidx=0x55` on a single read but `retry_on_divergence`
  catches it cleanly.
- Master payload encoding `[lvl, 00, 00, 32, 00, 32, mute, 00]` — the
  two `0x32` constants at bytes 3 and 5 are preserved correctly by
  `set_master`. Verified on bench.
- The `seq=0` rule for writes — confirmed live on bench against both
  CAT_STATE and CAT_PARAM writes; the firmware accepts neither category
  on non-zero seq writes (reads-only get auto-incremented seq, which is
  what `_exchange` does correctly).
- The HID frame parser fix from 2026-04-22 (the `is_multi_frame_first`
  50-vs-48 byte payload extraction) — ported to C++ identically, gave
  byte-exact reassembly of 296-byte blobs on the first try.

## ESPHome port findings of interest to dsp408-py

One mechanism that might be worth back-porting from the C++ port:

**Aggressive read-divergence retry on first warmup**. In the ESPHome
port, channel reads during the warmup phase do `retry_on_divergence`
unconditionally (max 4 attempts), and we've observed every channel
needing at least 2 attempts to converge on first read. The Python lib
defaults `retry_on_divergence=True` for `read_channel_state` already —
no change needed. But if anyone uses
`read_channel_state(retry_on_divergence=False)` in MQTT polling (for
latency), they should be aware that single-shot reads have a non-zero
chance of returning visibly bogus data (e.g. our Ch6 with
`+324 dB / muted / subidx=0x55`). Worth documenting more
prominently in `dsp408/mqtt.py`'s polling path.

---

This note was generated 2026-04-28 alongside the
[dsp408-esphome v0.2 release](http://10.15.0.6:3300/malaiwah/dsp408-esphome).
File a separate issue (or just paste relevant sections into the
dsp408-py thread) for whichever items you want to act on.
