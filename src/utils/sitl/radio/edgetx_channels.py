#!/usr/bin/env python3
"""Reach `channelOutputs[]` - the radio's mixer outputs - in the EdgeTX simulator library.

    python3 src/utils/sitl/radio/edgetx_channels.py <libedgetx-...-simulator.dll>

The value wanted is the mixer's output: a stick after everything the model does to it,
out of the *shipped* simulator library, with no EdgeTX build.

**Linux:** the library exports both arrays (`channelOutputs`, `ex_chans`) by name, and
`by_name()` asks the loader. **Windows:** the DLL exports only the `OpenTxSimulator`
class (58 symbols), so the arrays are not reachable by name. They *are* reachable from
the exported function that reads them:

  ?checkOutputsChanged@OpenTxSimulator@@IEAAXXZ   is exported
  channelOutputs                                  is NOT

`checkOutputsChanged` compiles to a loop over 32 channels whose three bases are loaded
once, into three registers, by three consecutive rip-relative `lea`s:

    lea 0x160deae(%rip),%r15   # the Qt-side change cache (lastOutputs, lastMixers)
    lea  0x48cd23(%rip),%r12   # channelOutputs[32]   -- emitted with limit 1024/1536
    lea  0x48cd5c(%rip),%r13   # ex_chans[32]         -- emitted with limit 2048
    ...
    movswl (%r14,%r12,1),%r8d          # channelOutputs[i]
    cmp    %r8w,(%r14,%r15,1)          # against lastOutputs[i]
    ...
    movswl (%r14,%r13,1),%r8d          # ex_chans[i]
    cmp    %r8w,0x40(%r14,%r15,1)      # against lastMixers[i]

which is `radio/src/mixer.cpp`'s two adjacent `int16_t x[MAX_OUTPUT_CHANNELS]` globals,
in declaration order, 0x40 = 32 * 2 bytes apart.

**The anchor is that adjacency, and it aborts by name.** This scans the function for
rip-relative `lea`s, computes their targets, and requires **exactly one** pair exactly
`0x40` apart with both members in uninitialised `.data`. Anything else - a compiler that
keeps them in different registers, a version that separates the two globals, a third
coincidental pair - raises `AnchorLost` naming what it found, rather than returning a
plausible address.

**And a static address is not a proof.** `ChannelOutputs.verify()` is the other half: a
moved stick has to move the value. A plausible address that never changes cannot be
told from a plausible address that is wrong.
"""

import ctypes
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

CHECK_OUTPUTS = "?checkOutputsChanged@OpenTxSimulator@@IEAAXXZ"

# radio/src/myeeprom.h -- MAX_OUTPUT_CHANNELS is 32 on every colour-LCD target.
# It is not read from the binary; it is *checked* against it, because the pair
# spacing below is 2 * this and a mismatch would break the anchor loudly.
MAX_OUTPUT_CHANNELS = 32
ARRAY_BYTES = MAX_OUTPUT_CHANNELS * 2

# How far into the function the three bases are loaded. The whole loop is well
# under this; scanning further only adds candidate `lea`s from unrelated code.
SCAN_BYTES = 0x200

# EdgeTX channel units. `channelOutputs[]` is +-1024 for +-100%, and up to
# +-1536 with extended limits (LIMIT_EXT_PERCENT = 150) -- the two limits
# `checkOutputsChanged` itself passes to `channelOutValueChange`.
UNIT_100_PERCENT = 1024
UNIT_MAX = 1536


class AnchorLost(Exception):
    """The pattern this was written against is not in this binary any more."""


def _rip_lea_targets(code, code_rva):
    """Every `lea r64, [rip+disp32]` in `code`, as (offset, register, target rva).

    REX.W with REX.R optional: 0x48 or 0x4C, then 0x8D, then a modrm whose
    mod=00 and rm=101 (rip-relative) -- i.e. modrm & 0xC7 == 0x05.
    """
    out = []
    for i in range(len(code) - 6):
        rex = code[i]
        if rex not in (0x48, 0x4C) or code[i + 1] != 0x8D:
            continue
        modrm = code[i + 2]
        if modrm & 0xC7 != 0x05:
            continue
        disp = struct.unpack_from("<i", code, i + 3)[0]
        reg = ((modrm >> 3) & 7) | (8 if rex == 0x4C else 0)
        out.append((i, reg, code_rva + i + 7 + disp))
    return out


def locate(dll_path):
    """(channel_outputs_rva, ex_chans_rva) for this DLL. Raises `AnchorLost`.

    **Windows only, and only because Windows leaves no choice.** On Linux the two
    arrays are exported by name and none of this runs -- see `by_name()`.
    """
    from pe_exports import Image
    img = Image(dll_path)
    exports = img.exports()
    if CHECK_OUTPUTS not in exports:
        raise AnchorLost(
            f"{os.path.basename(dll_path)} does not export {CHECK_OUTPUTS!r}. "
            f"It exports {len(exports)} symbols; without this one there is no "
            f"anchor for channelOutputs[] at all.")
    fn_rva = exports[CHECK_OUTPUTS]
    code = img.read(fn_rva, SCAN_BYTES)

    leas = _rip_lea_targets(code, fn_rva)
    pairs = []
    for _, _, a in leas:
        for _, _, b in leas:
            if b - a != ARRAY_BYTES:
                continue
            # Both must be in .data and past the file-backed part of it: these
            # are zero-initialised globals, so they have no bytes in the image.
            sect_a, sect_b = img.section_of(a), img.section_of(b)
            if not sect_a or not sect_b or sect_a[0] != ".data" or sect_b[0] != ".data":
                continue
            if img.offset(a) is not None or img.offset(b) is not None:
                continue
            pairs.append((a, b))

    if len(pairs) != 1:
        found = ", ".join(f"{a:#x}/{b:#x}" for a, b in pairs) or "none"
        raise AnchorLost(
            f"ANCHOR LOST in {CHECK_OUTPUTS} at rva {fn_rva:#x}: expected exactly "
            f"one pair of zero-initialised .data globals {ARRAY_BYTES:#x} bytes "
            f"apart among {len(leas)} rip-relative lea target(s); found {found}. "
            f"Either MAX_OUTPUT_CHANNELS is no longer {MAX_OUTPUT_CHANNELS}, or "
            f"channelOutputs and ex_chans are no longer adjacent. Disassemble the "
            f"function and re-establish the anchor -- do not relax this test.")
    return pairs[0]


def by_name(lib):
    """(channelOutputs, ex_chans) as live views, resolved by SYMBOL — the ELF case.

    The Linux build exports both arrays in `.dynsym`, so the loader answers for
    them and there is nothing to anchor, nothing to disassemble and nothing to
    keep in step with a compiler's register allocation. `ctypes.in_dll` either
    finds the symbol or raises `ValueError` naming it — which is the same
    property the anchor was written to have, obtained for free.

    Note the two are NOT in the same order as in the Windows build: there
    `ex_chans` sits above `channelOutputs`, here below. That is exactly the kind
    of assumption resolving by name removes, so nothing here checks it.
    """
    array = (ctypes.c_int16 * MAX_OUTPUT_CHANNELS).in_dll(lib, "channelOutputs")
    ex = (ctypes.c_int16 * MAX_OUTPUT_CHANNELS).in_dll(lib, "ex_chans")
    return array, ex


class _DlInfo(ctypes.Structure):
    _fields_ = [("dli_fname", ctypes.c_char_p), ("dli_fbase", ctypes.c_void_p),
                ("dli_sname", ctypes.c_char_p), ("dli_saddr", ctypes.c_void_p)]


def _loaded_object(address):
    """(load address, file) of the loaded ELF object holding `address` — dladdr(3).

    What the Windows path gets for free — the HMODULE IS the load address — the ELF
    path has to ask for: `dlopen`'s handle is the loader's own record, not an address
    inside the image. `dladdr` is in libc since glibc 2.34 and in libdl before it.
    """
    import ctypes.util
    fn = None
    for name in (None, ctypes.util.find_library("dl")):
        try:
            fn = ctypes.CDLL(name).dladdr
            break
        except (OSError, AttributeError):
            continue
    if fn is None:
        raise AnchorLost("dladdr(3) is not available in this process, so the load address "
                         "of the simulator library -- and with it an rva -- cannot be named")
    fn.argtypes = [ctypes.c_void_p, ctypes.POINTER(_DlInfo)]
    fn.restype = ctypes.c_int
    info = _DlInfo()
    if not fn(address, ctypes.byref(info)) or not info.dli_fbase:
        raise AnchorLost(f"dladdr(3) knows no loaded object at {address:#x}")
    return info.dli_fbase, os.fsdecode(info.dli_fname or b"")


class ChannelOutputs:
    """`channelOutputs[]` of a *loaded* simulator library, as a live view.

    Two ways in, and which one applies is a property of the binary rather than a
    preference. Pass `lib` and the arrays are resolved **by name**, which is what
    the Linux build allows. Pass `base` alone — the module's load address, which
    `ctypes.CDLL._handle` is on Windows — and the anchored search runs instead.

    The attributes mean the same on both paths: `base` is the module's HANDLE, the
    value a caller compares with `lib._handle` to ask "is this the library I
    loaded?"; `load_base` is the address the image is mapped at; `rva` and
    `ex_chans_rva` are offsets from `load_base`. On Windows the handle IS the load
    address, so `base == load_base` there; on Linux the handle is `dlopen`'s record
    and the load address comes from `dladdr(3)`, so `address == load_base + rva`
    holds on both and `address == base + rva` on Windows only.
    """

    def __init__(self, dll_path, base=None, lib=None):
        self.dll_path = dll_path
        self.base = base
        if lib is not None:
            self._array, self._ex = by_name(lib)
            self.address = ctypes.addressof(self._array)
            self.ex_address = ctypes.addressof(self._ex)
            self.base = lib._handle
            # The symbol lookup searches the library AND what it pulled in, so name
            # resolution alone does not say the arrays are this library's. The loader
            # does: both addresses must lie in the object ctypes opened, or this reads
            # a namesake somewhere else.
            want = os.path.realpath(lib._name)
            self.load_base = None
            for what, addr in (("channelOutputs", self.address), ("ex_chans", self.ex_address)):
                fbase, fname = _loaded_object(addr)
                if os.path.realpath(fname) != want:
                    raise AnchorLost(
                        f"{what} resolved to {addr:#x} inside {fname}, not inside {want} -- "
                        f"a symbol of that name in another loaded object")
                if self.load_base not in (None, fbase):
                    raise AnchorLost(f"channelOutputs and ex_chans report two load addresses, "
                                     f"{self.load_base:#x} and {fbase:#x}")
                self.load_base = fbase
            self.rva = self.address - self.load_base
            self.ex_chans_rva = self.ex_address - self.load_base
            self.how = "by symbol name"
            return
        self.load_base = base
        self.rva, self.ex_chans_rva = locate(dll_path)
        self.address = base + self.rva
        self.ex_address = base + self.ex_chans_rva
        self.how = "by anchored search"
        self._array = (ctypes.c_int16 * MAX_OUTPUT_CHANNELS).from_address(self.address)
        self._ex = (ctypes.c_int16 * MAX_OUTPUT_CHANNELS).from_address(self.ex_address)

    def read(self):
        """All 32 channels, in EdgeTX units (+-1024 = +-100%)."""
        return list(self._array)

    def read_mixer(self):
        """`ex_chans[]` -- the raw mixer sum before limits. Not the channel output;
        read only so that a caller can tell the two apart when diagnosing."""
        return list(self._ex)

    def plausible(self):
        """Cheap sanity gate: every channel inside the units the emitter itself
        declares. Not a proof -- an all-zero array passes it, which is why
        `verify()` exists."""
        return all(-UNIT_MAX <= v <= UNIT_MAX for v in self.read())

    def verify(self, move, settle, channel=0, minimum=200):
        """A moved stick must move the value. Returns (ok, detail).

        `move(value)` puts a stick at an EdgeTX input value; `settle()` waits for
        the radio's own mixer pass to run. Both are the caller's, because this
        module deliberately knows nothing about how the simulator is driven.
        """
        if not self.plausible():
            return False, (f"channelOutputs at {self.address:#x} holds values outside "
                           f"+-{UNIT_MAX}: {self.read()[:8]} -- that is not this array")
        readings = []
        for value in (-UNIT_100_PERCENT, UNIT_100_PERCENT, 0):
            move(value)
            settle()
            readings.append(self.read()[channel])
        span = max(readings) - min(readings)
        if span < minimum:
            return False, (
                f"CH{channel + 1} moved by {span} units across full stick travel "
                f"(readings {readings}); expected at least {minimum}. Either the "
                f"model has no mix on this channel, or the mixer is not running, "
                f"or {self.address:#x} is not channelOutputs -- ex_chans is at "
                f"{self.ex_address:#x} and reads "
                f"{self.read_mixer()[channel]}")
        return True, (f"CH{channel + 1} spans {span} units across full stick travel "
                      f"(readings {readings})")


def attach(lib, dll_path=None):
    """`channelOutputs[]` of a library already loaded - by `edgetx_sim.Sim` or by any
    other host that owns the simulator instance (one library instance is one radio).

    `lib` is a `ctypes.CDLL`. On Windows its `_handle` is the HMODULE, which is exactly the
    module's load address, so the anchored RVA needs no runtime search. `dll_path`
    defaults to the library's own `_name`, because the anchor is found by parsing the file
    on disk, not the mapped image.

    `verify()` still applies: a static address is not a proof, and the caller still has to
    move a stick and watch the value move.
    """
    if os.name != "nt":
        # Nothing about the returned object differs, `verify()` included: a symbol
        # is a stronger claim than an anchored address and still not a proof, so
        # the caller still has to move a stick and watch the value move.
        return ChannelOutputs(dll_path or lib._name, lib=lib)
    return ChannelOutputs(dll_path or lib._name, lib._handle)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    from pe_exports import Image
    path = sys.argv[1]
    try:
        chans, ex = locate(path)
    except AnchorLost as exc:
        print(f"FAIL  {exc}")
        return 1
    img = Image(path)
    print(f"{os.path.basename(path)}")
    print(f"  ImageBase        {img.image_base:#x}")
    print(f"  checkOutputsChanged rva {img.exports()[CHECK_OUTPUTS]:#x}")
    print(f"  channelOutputs   rva {chans:#010x}  ({MAX_OUTPUT_CHANNELS} x int16)")
    print(f"  ex_chans         rva {ex:#010x}")
    print(f"  static load addr {img.image_base + chans:#x} "
          f"(the real one is the module handle + rva)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
