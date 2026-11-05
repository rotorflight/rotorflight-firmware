#!/usr/bin/env python3
"""Minimal ctypes binding to the EdgeTX simulator library that Companion ships.

The simulator ships as a library - `libedgetx-<radio>-simulator.dll` on Windows, `.so`
on Linux - exporting the whole `OpenTxSimulator` class, and Python `ctypes` calls the
exports directly. No build step, no Qt event loop, not even a `QCoreApplication`: on
Linux `isRunning()` goes true with no Qt application object anywhere in the process.

**Both platforms are bound from one file.** The symbol names differ (MSVC versus
Itanium mangling, read out of each binary rather than derived), the library suffix
differs, and the way a `QString` is built differs - Qt 6 on Linux does not export the
`fromUtf8` overload the Windows side uses, so a plain `QString(const QChar*, qsizetype)`
is used instead. Nothing else does.

Where the library comes from: Companion's install (`bin/` on Windows) or its AppImage
(`--appimage-extract`, `usr/lib/companion212`, with `LD_LIBRARY_PATH` at `usr/lib`).
**Status with Companion 2.12.5:** Linux works (also under WSL1, once `.note.ABI-tag` is
stripped from the bundled `libQt6Core.so.6`); the Windows library binds and finds its
channel array, but `start()` faults reading a null pointer.

`setSdPath` is mandatory before `start()`: the simulator writes `RADIO/` and `MODELS/`
into that directory.
"""

import ctypes
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from edgetx_channels import (ChannelOutputs, UNIT_100_PERCENT,   # noqa: E402
                             attach as attach_channels)

# Where Companion's simulator libraries are: pass `bindir`, or set EDGETX_SIM_BINDIR.
DEFAULT_VENDOR = os.environ.get("EDGETX_SIM_BINDIR", "")
DEFAULT_SD = os.path.join(os.getcwd(), "edgetx-sd")

SYMBOLS = {
    "ctor":      "??0OpenTxSimulator@@QEAA@XZ",
    "init":      "?init@OpenTxSimulator@@UEAAXXZ",
    "start":     "?start@OpenTxSimulator@@UEAAXPEBD_N@Z",
    "stop":      "?stop@OpenTxSimulator@@UEAAXXZ",
    "isRunning": "?isRunning@OpenTxSimulator@@UEAA_NXZ",
    "setSdPath": "?setSdPath@OpenTxSimulator@@UEAAXAEBVQString@@0@Z",
    "setSwitch": "?setSwitch@OpenTxSimulator@@UEAAXEC@Z",
    "setInput":  "?setInputValue@OpenTxSimulator@@UEAAXHEF@Z",
    "setAnalog": "?setAnalogValue@OpenTxSimulator@@UEAAXEF@Z",
    "sendIntTlm": "?sendInternalModuleTelemetry@OpenTxSimulator@@UEAAXEVQByteArray@@@Z",
    # The screen half. `checkLcdChanged` is `IEAA`, a *protected* member, and is
    # exported all the same.
    "getLcd":     "?getLcd@OpenTxSimulator@@UEAAPEAEXZ",
    "lcdChanged": "?checkLcdChanged@OpenTxSimulator@@IEAA_NXZ",
    "touchEvent": "?touchEvent@OpenTxSimulator@@UEAAXHHH@Z",
    "setKey":     "?setKey@OpenTxSimulator@@UEAAXE_N@Z",
    # Read out of the shipped DLL's own export strings rather than derived from the
    # header -- `SimulatorInterface::setTrim` is `(unsigned int, int)`, which mangles to
    # `XIH`, and a guessed mangling fails at bind time with a name nobody can grep for.
    # The trim BUTTONS (`setTrimSwitch`) are a different thing: readable as a switch
    # source whatever the trim mode is, which is how a model drives an in-flight
    # adjustment from a trim.
    "setTrim":    "?setTrim@OpenTxSimulator@@UEAAXIH@Z",
    "setTrimSwitch": "?setTrimSwitch@OpenTxSimulator@@UEAAXE_N@Z",
}
QT_FROM_UTF8 = "?fromUtf8@QString@@SA?AV1@PEBD_J@Z"
QT_QBA_CTOR = "??0QByteArray@@QEAA@PEBD_J@Z"

# The same class, built by GCC for Linux. **Read out of the library's own
# dynamic symbol table, never derived from the header** -- the note on `setTrim`
# above is the reason, and it applies twice as hard here: a guessed Itanium
# mangling fails at bind time with a name nobody can grep for.
#
# Two things measured while reading them, both worth not rediscovering:
#   * the complete-object and base-object constructors (`C1Ev`, `C2Ev`) are at
#     the SAME address in this build, so the documented trap of picking the
#     wrong one cannot arise here. `C1` is bound anyway, because that is the
#     one that is right in general.
#   * the argument codes line up with the MSVC ones exactly: `setSwitch(uchar,
#     signed char)` is `Eha`, `setTrim(uint, int)` is `Eji`. Nothing about the
#     signatures changed with the compiler.
SYMBOLS_ELF = {
    "ctor":          "_ZN15OpenTxSimulatorC1Ev",
    "init":          "_ZN15OpenTxSimulator4initEv",
    "start":         "_ZN15OpenTxSimulator5startEPKcb",
    "stop":          "_ZN15OpenTxSimulator4stopEv",
    "isRunning":     "_ZN15OpenTxSimulator9isRunningEv",
    "setSdPath":     "_ZN15OpenTxSimulator9setSdPathERK7QStringS2_",
    "setSwitch":     "_ZN15OpenTxSimulator9setSwitchEha",
    "setInput":      "_ZN15OpenTxSimulator13setInputValueEihs",
    "setAnalog":     "_ZN15OpenTxSimulator14setAnalogValueEhs",
    "sendIntTlm":    "_ZN15OpenTxSimulator27sendInternalModuleTelemetryEh10QByteArray",
    "getLcd":        "_ZN15OpenTxSimulator6getLcdEv",
    "lcdChanged":    "_ZN15OpenTxSimulator15checkLcdChangedEv",
    "touchEvent":    "_ZN15OpenTxSimulator10touchEventEiii",
    "setKey":        "_ZN15OpenTxSimulator6setKeyEhb",
    "setTrim":       "_ZN15OpenTxSimulator7setTrimEji",
    "setTrimSwitch": "_ZN15OpenTxSimulator13setTrimSwitchEhb",
}

# **`QString::fromUtf8(const char*, qsizetype)` is NOT exported by Qt 6 on Linux.**
# The only exported overload takes a `QByteArrayView`, which is a struct passed by
# value -- an ABI detail to get right for no gain. A plain constructor avoids the
# question entirely: `QString(const QChar*, qsizetype)` is exported, takes a
# pointer and a length, and UTF-16 is something Python already produces exactly.
# So this half of the binding is not a port of the Windows call; it is a simpler
# call that happens to be available here.
QT_QSTRING_FROM_UTF16 = "_ZN7QStringC1EPK5QCharx"
QT_QBA_CTOR_ELF = "_ZN10QByteArrayC1EPKcx"

WINDOWS = os.name == "nt"
LIB_SUFFIX = ".dll" if WINDOWS else ".so"
QT_CORE = "Qt6Core.dll" if WINDOWS else "libQt6Core.so.6"

QSTRING_SIZE = 24      # Qt6 QArrayDataPointer: {Data* d; char16_t* ptr; qsizetype size}
QBYTEARRAY_SIZE = 24
OBJ_BUF = 65536        # sizeof(OpenTxSimulator) is not knowable from outside

# `getLcd()` returns a bare pointer and no length, so the caller has to know the
# framebuffer's shape. RGB565, pinned empirically rather than from documentation:
# the same buffer read as ARGB8888 renders as four tiled half-images, as RGB565
# as a pixel-exact screen. Reading a radio that is not in this table would read
# past the end of the buffer, so an unknown one is refused by name.
LCD_RESOLUTION = {
    "tx16smk3": (800, 480),
    "tx15": (480, 320),
    "tx16s": (480, 272),
}
LCD_BYTES_PER_PIXEL = 2

# `touchEvent(event, x, y)`.
TOUCH_RELEASED = 0
TOUCH_PRESSED = 1

# The uint8_t on the telemetry senders is a PROTOCOL, not a module index -- 2 is
# CRSF, and 0 silently feeds EdgeTX's S.PORT parser, which looks like delivery
# and creates no sensor.
PROTOCOL_CROSSFIRE = 2


#: The simulator library is built with Visual Studio 2022 17.10 or later, whose
#: `std::mutex` needs `msvcp140.dll` 14.40 or newer. An older copy in the process -
#: Python distributions such as Anaconda ship one (14.27) next to python.exe, and the
#: loader finds it first - makes the first lock in `start()` read a null pointer.
MIN_MSVCP140 = (14, 40)


def _file_version(path):
    size = ctypes.windll.version.GetFileVersionInfoSizeW(path, None)
    buf = ctypes.create_string_buffer(size)
    ctypes.windll.version.GetFileVersionInfoW(path, 0, size, buf)
    value, length = ctypes.c_void_p(), ctypes.c_uint()
    ctypes.windll.version.VerQueryValueW(buf, "\\", ctypes.byref(value), ctypes.byref(length))
    ms, ls = ctypes.cast(value, ctypes.POINTER(ctypes.c_uint32))[2:4]
    return (ms >> 16, ms & 0xFFFF, ls >> 16, ls & 0xFFFF)


def _system_cpp_runtime():
    """Make sure the C++ runtime in this process is new enough for the simulator."""
    k32 = ctypes.WinDLL("kernel32")
    k32.GetModuleHandleW.restype = ctypes.c_void_p
    k32.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32]
    handle = k32.GetModuleHandleW("msvcp140.dll")
    if not handle:
        # not loaded yet: take the system's copy, so the simulator binds to it
        ctypes.WinDLL(os.path.join(os.environ.get("SystemRoot", r"C:\Windows"),
                                   "System32", "msvcp140.dll"))
        handle = k32.GetModuleHandleW("msvcp140.dll")
    name = ctypes.create_unicode_buffer(1024)
    k32.GetModuleFileNameW(handle, name, 1024)
    version = _file_version(name.value)
    if version[:2] < MIN_MSVCP140:
        raise RuntimeError(
            f"{name.value} is version {'.'.join(map(str, version))}; the EdgeTX simulator "
            f"needs msvcp140.dll {MIN_MSVCP140[0]}.{MIN_MSVCP140[1]} or newer and would "
            f"crash in start(). Something loaded the older copy first: import edgetx_sim "
            f"before it, or install the current Visual C++ redistributable.")


class Sim:
    def __init__(self, radio="tx16smk3", bindir=None, sd_path=None):
        self.radio = radio
        self.bindir = os.path.abspath(bindir or DEFAULT_VENDOR)
        self.dll_path = os.path.join(
            self.bindir, f"libedgetx-{radio}-simulator{LIB_SUFFIX}")
        if not os.path.exists(self.dll_path):
            raise FileNotFoundError(
                f"{self.dll_path} not found: one library per radio is in Companion's "
                f"bin directory, so this is the wrong radio name or the wrong directory.")
        self.sd_path = os.path.abspath(sd_path or DEFAULT_SD)
        self._sd_set = False
        self._started = False

        symbols = SYMBOLS if WINDOWS else SYMBOLS_ELF
        qt_path = os.path.join(self.bindir if WINDOWS
                               else os.path.dirname(self.bindir), QT_CORE)

        if WINDOWS:
            # The loader needs the directory on the search path; there is no
            # equivalent step on Linux, where the image sets LD_LIBRARY_PATH and
            # the library's own dependencies resolve beside it.
            _system_cpp_runtime()
            os.add_dll_directory(self.bindir)
            os.environ["PATH"] = self.bindir + os.pathsep + os.environ["PATH"]
            self.lib = ctypes.CDLL(self.dll_path, winmode=0)
        else:
            self.lib = ctypes.CDLL(self.dll_path)
        if not os.path.exists(qt_path):
            raise FileNotFoundError(
                f"{qt_path} not found. The simulator brings its own Qt and this "
                f"binding calls into that same one deliberately: the library is "
                f"linked -Bsymbolic, so a Qt object built by a DIFFERENT Qt build "
                f"and handed to it is silent corruption rather than an error.")
        self.qtcore = ctypes.CDLL(qt_path, winmode=0) if WINDOWS             else ctypes.CDLL(qt_path)

        V = ctypes.c_void_p

        def bind(lib, name, restype, argtypes):
            addr = ctypes.cast(getattr(lib, name), ctypes.c_void_p).value
            return ctypes.CFUNCTYPE(restype, *argtypes)(addr)

        self._ctor = bind(self.lib, symbols["ctor"], V, [V])
        self._init = bind(self.lib, symbols["init"], None, [V])
        self._start = bind(self.lib, symbols["start"], None,
                           [V, ctypes.c_char_p, ctypes.c_bool])
        self._stop = bind(self.lib, symbols["stop"], None, [V])
        self._running = bind(self.lib, symbols["isRunning"], ctypes.c_bool, [V])
        self._setsd = bind(self.lib, symbols["setSdPath"], None, [V, V, V])
        self._setswitch = bind(self.lib, symbols["setSwitch"], None,
                               [V, ctypes.c_ubyte, ctypes.c_byte])
        self._setinput = bind(self.lib, symbols["setInput"], None,
                              [V, ctypes.c_int, ctypes.c_ubyte, ctypes.c_short])
        self._setanalog = bind(self.lib, symbols["setAnalog"], None,
                               [V, ctypes.c_ubyte, ctypes.c_short])
        self._sendtlm = bind(self.lib, symbols["sendIntTlm"], None,
                             [V, ctypes.c_ubyte, V])
        self._getlcd = bind(self.lib, symbols["getLcd"], V, [V])
        self._lcdchanged = bind(self.lib, symbols["lcdChanged"], ctypes.c_bool, [V])
        self._touch = bind(self.lib, symbols["touchEvent"], None,
                           [V, ctypes.c_int, ctypes.c_int, ctypes.c_int])
        self._setkey = bind(self.lib, symbols["setKey"], None,
                            [V, ctypes.c_ubyte, ctypes.c_bool])
        self._settrim = bind(self.lib, symbols["setTrim"], None,
                             [V, ctypes.c_uint, ctypes.c_int])
        self._settrimsw = bind(self.lib, symbols["setTrimSwitch"], None,
                               [V, ctypes.c_ubyte, ctypes.c_bool])
        if WINDOWS:
            # A static member returning QString by value: the hidden result
            # pointer is the first argument.
            self._qstr = bind(self.qtcore, QT_FROM_UTF8, V,
                              [V, ctypes.c_char_p, ctypes.c_int64])
            self._qba = bind(self.qtcore, QT_QBA_CTOR, V,
                             [V, ctypes.c_char_p, ctypes.c_int64])
        else:
            # A constructor: `this` first, then UTF-16 data and its length in
            # CHARACTERS -- not bytes, which is the mistake this shape invites.
            self._qstr = bind(self.qtcore, QT_QSTRING_FROM_UTF16, V,
                              [V, ctypes.c_void_p, ctypes.c_int64])
            self._qba = bind(self.qtcore, QT_QBA_CTOR_ELF, V,
                             [V, ctypes.c_char_p, ctypes.c_int64])

        self._buf = ctypes.create_string_buffer(OBJ_BUF)
        self.this = ctypes.cast(self._buf, ctypes.c_void_p)
        self._keep = []
        self._ctor(self.this)

        # On Windows the module handle IS the load base, so the anchored RVA
        # needs no runtime search. On Linux nothing is anchored at all: the two
        # arrays are exported by name, so the loader resolves them and the whole
        # apparatus that found them by disassembly does not run.
        self.channels = attach_channels(self.lib, self.dll_path)

    # ------------------------------------------------------------- Qt glue

    def _qstring(self, text):
        buf = ctypes.create_string_buffer(QSTRING_SIZE)
        self._keep.append(buf)
        if WINDOWS:
            data = text.encode("utf-8")
            self._qstr(ctypes.cast(buf, ctypes.c_void_p), data, len(data))
        else:
            # UTF-16LE, and the length is in CHARACTERS. Encoding to bytes and
            # passing the byte count would build a string twice as long as the
            # text, reading past the buffer -- which is why the count is derived
            # from the encoded buffer rather than from len(text): a character
            # outside the basic plane is two QChars, and len(text) is one.
            data = text.encode("utf-16-le")
            self._keep.append(data)
            self._qstr(ctypes.cast(buf, ctypes.c_void_p),
                       ctypes.cast(ctypes.c_char_p(data), ctypes.c_void_p),
                       len(data) // 2)
        return ctypes.cast(buf, ctypes.c_void_p)

    # ------------------------------------------------------------ lifecycle

    def set_sd_path(self, path=None, settings=None):
        path = os.path.abspath(path or self.sd_path)
        os.makedirs(path, exist_ok=True)
        self.sd_path = path
        self._setsd(self.this, self._qstring(path), self._qstring(settings or path))
        self._sd_set = True

    def start(self, model=None):
        if not self._sd_set:
            # Refusing rather than defaulting silently: the failure this prevents
            # is the simulator writing RADIO/ and MODELS/ into whatever directory
            # the process happened to start in.
            raise RuntimeError("set_sd_path() before start() -- setSdPath is mandatory")
        self._init(self.this)
        self._start(self.this, model.encode() if model else None, False)
        self._started = True

    def stop(self):
        if self._started:
            self._stop(self.this)
            self._started = False

    @property
    def running(self):
        return self._running(self.this)

    def wait_running(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.running:
                return True
            time.sleep(0.05)
        return False

    # ---------------------------------------------------------------- input

    def analog(self, index, value):
        """A raw stick/pot position, in EdgeTX units (+-1024 = +-100 %)."""
        self._setanalog(self.this, index, value)

    def input_value(self, kind, index, value):
        self._setinput(self.this, kind, index, value)

    def switch(self, index, state):
        """`state` is -1 / 0 / 1 -- the mangled name says `C`, a *signed* char.

        `index` is the radio's switch **ORDER** (SA = 0, SB = 1, ... SH = 7 on
        the MK3), which is a different numbering from the `swsrc` a widget
        stores. SA at state +1 puts +100 % on its mix, state -1 puts -100 % on it.
        """
        self._setswitch(self.this, index, state)

    def key(self, code, pressed):
        """A hardware key, by EdgeTX's own `EnumKeys` code."""
        self._setkey(self.this, code, bool(pressed))

    def trim(self, index, value):
        """A trim's stored VALUE, not a trim button press.

        `setTrimValue()` (`edgetx.cpp:568`) writes into the active flight
        mode's `trim[index]`, and `index` is the stick order -- the same order
        `analog()` uses, i.e. 0 Rud, 1 Ele, 2 Thr, 3 Ail.

        **It can refuse, and it refuses silently here**: `setTrimValue` returns
        false for `TRIM_MODE_NONE` and `TRIM_MODE_3POS`, and the simulator's
        `setTrim` throws that answer away. A caller that wants to know whether a
        trim moved has to read the wire back -- which is the same rule the
        switch sweep already runs on.

        **This is NOT the trim button** -- see `trim_switch()`, which is the one
        a model uses for in-flight tuning and which works whatever the mode is.
        """
        self._settrim(self.this, int(index), int(value))

    def trim_switch(self, index, pressed):
        """A trim BUTTON, held or released. `simuSetTrim`, `simpgmspace.cpp:112`.

        **A trim button is not the trim, and a model whose trims are
        `TRIM_MODE_NONE` still reads its buttons.** `getSwitch()` resolves
        `SWSRC_FIRST_TRIM + idx` to `trimDown(idx)` (`switches.cpp`) -- the raw key
        state -- and the trim mode never enters that path. So a mixer line, a logical
        switch or a special function triggered by `TrimEleUp` fires on the press
        whatever the model does with the stored value.

        `index` runs over `2 * MAX_TRIMS` -- 12 here, two per trim -- and which
        end of which trim an index is depends on the stick mode
        (`inputMappingConvertMode`, `switches.cpp:720`). **Measure it rather than
        deriving it**: `switch_sweep.Sweep.run` drives this method directly.
        """
        self._settrimsw(self.this, int(index), bool(pressed))

    def touch(self, event, x, y):
        """`TOUCH_PRESSED` / `TOUCH_RELEASED` at LCD pixel coordinates."""
        self._touch(self.this, int(event), int(x), int(y))

    # ------------------------------------------------------------------ screen

    @property
    def lcd_size(self):
        try:
            return LCD_RESOLUTION[self.radio]
        except KeyError:
            raise RuntimeError(
                f"no framebuffer shape recorded for radio {self.radio!r}; "
                f"getLcd() returns a bare pointer and reading it at a guessed "
                f"size reads past the end of the buffer. Add it to "
                f"LCD_RESOLUTION once it has been measured.") from None

    def lcd_changed(self):
        """True once per repaint. Clears the flag, so ask at most once a frame."""
        return self._lcdchanged(self.this)

    def lcd_raw(self):
        """The framebuffer as `bytes`: RGB565, little-endian, row-major."""
        width, height = self.lcd_size
        pointer = self._getlcd(self.this)
        if not pointer:
            raise RuntimeError("getLcd() returned NULL -- the radio is not running")
        return ctypes.string_at(pointer, width * height * LCD_BYTES_PER_PIXEL)

    # --------------------------------------------------------------- output

    def send_internal_telemetry(self, frame, protocol=PROTOCOL_CROSSFIRE):
        """Push a raw CRSF frame into the internal module's receive path.

        Whether a **sensor** appears is a different question and not this call's:
        Rotorflight's `0x88` custom frames are popped and decoded by the Lua suite on
        the SD card, not by EdgeTX's auto-discovery.
        """
        buf = ctypes.create_string_buffer(QBYTEARRAY_SIZE)
        data = bytes(frame)
        self._qba(ctypes.cast(buf, ctypes.c_void_p), data, len(data))
        self._sendtlm(self.this, protocol, ctypes.cast(buf, ctypes.c_void_p))

    # -------------------------------------------------------------- context

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


def settle(seconds=0.25):
    """One mixer pass and then some. The radio runs on its own thread; nothing
    here polls a flag, because EdgeTX exposes none for 'the mixer has run'."""
    time.sleep(seconds)


__all__ = ["Sim", "settle", "ChannelOutputs", "UNIT_100_PERCENT",
           "DEFAULT_VENDOR", "DEFAULT_SD", "LCD_RESOLUTION",
           "TOUCH_PRESSED", "TOUCH_RELEASED"]
