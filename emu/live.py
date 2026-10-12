"""Play the emulated unit from the computer keyboard, in real time.

    uv run python -m emu.live SNAPSHOT --syx FIRMWARE.syx

A window with the panel and the clickable control surface of emu/gui.py,
plus a keyboard map: holding a key holds the button, so FUNC chords work
the way they do on the unit. Press `?` in the window for the map; the
on-screen keys also carry their shortcut. docs/LIVE.md has the full table.

What is different from emu/gui.py, and why:

* **Real time, not a fixed fraction of it.** The worker runs one timer
  deadline step at a time (emu.longrun.spin with a floor of one
  instruction), so input is never more than one step (at most ~17 ms of
  emulated time) away from delivery, and it sleeps off whatever it is ahead
  of the wall clock. A key press wakes it at once.
* **Idle skipping.** After boot the firmware spends most of its time in a
  `bra.s *` loop (on Digitone II 1.12 the priority-0 task parks at
  `0x400cf246`), and each pass costs two Python callbacks (the fast
  stepper's block hook and build()'s idle hook). When the CPU reaches an
  idle loop, has been through one reschedule (vector 32) since, and nothing
  else ran in between, the rest of the current timer step is credited as
  executed and the step ends. Nothing can leave a `bra.s *` but an
  interrupt, and interrupts are only delivered at step boundaries, so this
  changes no guest-visible order; it only stops paying for the spin.
  `--no-idle-skip` turns it off for comparison.
* **One transition per delivery, with a short dwell.** Button state changes
  reach the firmware at least `--dwell-ms` of emulated time apart (default
  25 ms), so a fast tap still arrives as a press and then a release, as it
  would from the panel MCU. Encoder detents are not paced.
* **Saved timer cadence.** Snapshots saved with tools/guirun.py --save-at
  (or `--save` here) carry their timers; they are restored, not rebuilt.
  A snapshot taken after boot starts in seconds; boot400M still works but
  takes minutes to reach the main screen.
* **Encoders step.** The firmware filters encoder counts (a dead zone after
  every pause, then one step per `divisor` counts), so one count per key
  press changed nothing. By default a hook gives one parameter step per
  key press; see LiveEmulator._make_detent_hook. `--encoder-mode raw`
  sends raw counts instead.
* **DTIM0 counts.** The firmware time-stamps panel input with DMA timer 0,
  which nothing in the firmware file starts; here it counts (`--dtim0-hz`).

Headless use, for tests and for anyone without a display (no Tk window is
created): `--script FILE` runs key events from a file through exactly the
same KeyInput -> inbox -> panel path the window uses, and `png NAME` lines
save the screen. `--bench SECONDS` measures speed with no input.

`--ssi0-hz` (with `--ips 150e6`) runs the audio clock, so PLAY runs the
sequencer; see docs/LIVE.md "Audio clock". No sound: the DSP is not
emulated (emu/dsp.py only answers its ready line).

DIN MIDI (UART9, emu/midi.py) is modelled: `--midi-log`, `--midi-file`,
and with python-rtmidi a virtual `--midi-port` out and `--midi-in`. The
panel LEDs (UART8 messages to the panel MCU, emu/leds.py) are drawn on the
keys and saved next to every headless `png`.
"""
import argparse
import collections
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from unicorn import UC_HOOK_MEM_READ
from unicorn.m68k_const import UC_M68K_REG_A7, UC_M68K_REG_PC

from emu import config, device as devices, panel, panelin, symbols
from emu.dspboot import find_idle_spins, MAIN_LOAD
from emu.dtim import Dtims, Timers
from emu.longrun import build, spin, _fast_stepper
from emu.pit import INSTR_PER_SEC, Pits, intro_running

W, H = panel.W, panel.H

# The RTOS's own idle (`move.w #$2000,sr; bra.s *` with no task current).
# A reschedule trap there would run with no current task, so it is only
# skipped, never yielded from. Same address in both known builds' RTOS.
RTOS_IDLE = 0x40001c08

# Blocks allowed between two idle-loop hits for them to count as "nothing
# else ran". A reschedule that finds no other ready task and returns to
# the loop is a few dozen blocks; any real task work is far more.
IDLE_QUIET_BLOCKS = 400

# Emulated time the worker may fall behind the wall clock before it stops
# trying to catch up. Without a cap, a busy stretch is followed by a burst
# of fast-forward (a sequencer that speeds up after it slowed down).
MAX_LAG_S = 0.25

# The firmware's shared encoder accumulator (every view's encoder handler
# calls it): (handler, encoder, delta, cfg, pushed) -> steps. Checked by its
# first bytes before use; see _setup and docs/LIVE.md.
ENC_ACCUMULATE = 0x4011364e
ENC_ACCUMULATE_BYTES = bytes.fromhex('4fefffd048d77cfc142f0047266f003c206f0040')
# A pending keyboard detent the firmware has not consumed after this long
# (it can drop rotation, e.g. just after an encoder push) is forgotten.
DETENT_TTL_S = 0.5

# DTIM0's counter register (MCF5441x DMA timer 0, DTCN0). See _setup.
DTIM0_DTCN = 0xFC07000C
# Its rate on the unit is not known: no code in the firmware file programs
# DTIM0. The bus clock with no prescale is the plain guess.
DTIM0_HZ = 132_000_000

# The audio clock (--ssi0-hz). On the unit the SHARC clocks SSI0, and
# every completed eDMA50 major loop (64 minors of 32 bytes) raises vector
# 170. The Digitone II 1.12 vector-170 handler 0x400d55d4 scans the RX
# bank just filled (32 rows of 64 bytes at 0x4e6e7100 or 0x4e6e7900) for
# the peer's sync word 0x007fffff at the head of row 0; after 64 aligned
# blocks it installs the real channel-50 ISR (0x40025386), which forces
# vector 191, the frame handler that runs the sequencer and the DSP frame.
# 96,000 requests/s gives one major loop per 32-sample block at 48 kHz
# (1,500 per second), the DSP's own block rate (dn2-cfw notes/T3.4).
SSI0_HZ_DN2 = 96_000
SSI0_MINORS_PER_BLOCK = 64
DSP_SYNC_WORD = b'\x00\x7f\xff\xff'
DSP_SYNC_BANKS = (0x4E6E7100, 0x4E6E7900)
DSP_SYNC_ROWS, DSP_SYNC_ROW_BYTES = 32, 0x40


# --- keyboard map -------------------------------------------------------

# Firmware control codes (the factory-test name table, emu/panelin.py).
# Button codes and encoder ROTATION codes are separate spaces.
BUTTON, ENCODER, HOST = 'button', 'encoder', 'host'

# (key, kind, code, label). Keys are Tk keysyms after normalize(); a US /
# ANSI layout is assumed. --keymap FILE overrides any entry.
DEFAULT_KEYS = [
    # 16 trig keys on two letter rows
    *[(k, BUTTON, 25 + i, 'TRIG %d' % (i + 1)) for i, k in enumerate('qwertyui')],
    *[(k, BUTTON, 33 + i, 'TRIG %d' % (i + 9)) for i, k in enumerate('asdfghjk')],
    # parameter page keys, right of the trig rows
    ('o', BUTTON, 1, 'TRIG page'),
    ('p', BUTTON, 2, 'SRC (SYN)'),
    ('bracketleft', BUTTON, 3, 'FLTR'),
    ('bracketright', BUTTON, 4, 'AMP'),
    ('l', BUTTON, 5, 'FX'),
    ('semicolon', BUTTON, 6, 'MOD'),
    ('apostrophe', BUTTON, 22, 'PAGE'),
    # modifiers: hold them
    ('space', BUTTON, 17, 'FUNC'),
    ('Tab', BUTTON, 16, 'TRK'),
    # confirm and arrows
    ('Return', BUTTON, 10, 'YES'),
    ('BackSpace', BUTTON, 12, 'NO'),
    ('Delete', BUTTON, 12, 'NO'),
    ('Escape', BUTTON, 12, 'NO'),
    ('Up', BUTTON, 11, 'UP'),
    ('Left', BUTTON, 13, 'LEFT'),
    ('Down', BUTTON, 14, 'DOWN'),
    ('Right', BUTTON, 15, 'RIGHT'),
    # transport, in panel order
    ('0', BUTTON, 19, 'RECORD'),
    ('minus', BUTTON, 20, 'PLAY'),
    ('equal', BUTTON, 21, 'STOP'),
    # the rest of the panel, bottom letter row and around
    ('z', BUTTON, 18, 'KEYBOARD'),
    ('x', BUTTON, 54, 'MINUS'),
    ('c', BUTTON, 52, 'PLUS'),
    ('v', BUTTON, 51, 'ARP'),
    ('b', BUTTON, 50, 'VOICE'),
    ('n', BUTTON, 53, 'STACK'),
    ('m', BUTTON, 7, 'PRESET'),
    ('comma', BUTTON, 23, 'PTN'),
    ('period', BUTTON, 24, 'SONG'),
    ('slash', BUTTON, 9, 'TEMPO'),
    ('grave', BUTTON, 8, 'SETTINGS'),
    ('backslash', BUTTON, 49, 'LEVEL push'),
    # encoder pushes (hold one while turning for press-and-turn)
    *[('F%d' % (i + 1), BUTTON, 41 + i, 'ENCODER %s push' % 'ABCDEFGH'[i])
      for i in range(8)],
    # encoders: the number key turns clockwise, with Shift anticlockwise
    *[(str(i + 1), ENCODER, i + 1, 'ENCODER %s' % 'ABCDEFGH'[i])
      for i in range(8)],
    ('9', ENCODER, 9, 'LEVEL/DATA'),
    # host keys
    ('question', HOST, 'help', 'show or hide this map'),
]

# Shifted characters back to the key they are typed on (US layout), so
# Shift changes an encoder's direction and nothing else.
SHIFTED = {
    'exclam': '1', 'at': '2', 'numbersign': '3', 'dollar': '4',
    'percent': '5', 'asciicircum': '6', 'ampersand': '7', 'asterisk': '8',
    'parenleft': '9', 'parenright': '0', 'underscore': 'minus',
    'plus': 'equal', 'braceleft': 'bracketleft', 'braceright': 'bracketright',
    'colon': 'semicolon', 'quotedbl': 'apostrophe', 'bar': 'backslash',
    'less': 'comma', 'greater': 'period', 'asciitilde': 'grave',
    'ISO_Left_Tab': 'Tab',
}
ALIASES = {
    'quoteright': 'apostrophe', 'quoteleft': 'grave', 'KP_Enter': 'Return',
    **{'KP_%d' % i: str(i) for i in range(10)},
}
# Keys whose shifted form means something else and must not be folded.
UNFOLDED = {'question'}

# How each key is shown to a person.
PRETTY = {
    'space': 'Space', 'Tab': 'Tab', 'Return': 'Return', 'BackSpace': 'Delete',
    'Delete': 'Fwd Del', 'Escape': 'Esc', 'Up': 'Up', 'Down': 'Down',
    'Left': 'Left', 'Right': 'Right', 'minus': '-', 'equal': '=',
    'bracketleft': '[', 'bracketright': ']', 'semicolon': ';',
    'apostrophe': "'", 'comma': ',', 'period': '.', 'slash': '/',
    'grave': '`', 'backslash': '\\', 'question': '?',
}


def normalize(keysym):
    """-> (key, shifted) for a Tk keysym: lower case, Shift folded away."""
    if keysym in UNFOLDED:
        return keysym, False
    keysym = ALIASES.get(keysym, keysym)
    if keysym in SHIFTED:
        return SHIFTED[keysym], True
    if len(keysym) == 1 and keysym.isalpha():
        return keysym.lower(), keysym.isupper()
    return keysym, False


# Tk 9 reports a printable key's keysym as the character itself (%K is ','
# for the comma key, '!' for Shift+1), not as the X name ('comma',
# 'exclam') that Tk 8 gave. Map the characters back to the names above.
CHAR_KEYSYMS = {
    ',': 'comma', '.': 'period', '-': 'minus', '=': 'equal',
    ';': 'semicolon', "'": 'apostrophe', '`': 'grave', '/': 'slash',
    '\\': 'backslash', '[': 'bracketleft', ']': 'bracketright',
    '?': 'question', '!': 'exclam', '@': 'at', '#': 'numbersign',
    '$': 'dollar', '%': 'percent', '^': 'asciicircum', '&': 'ampersand',
    '*': 'asterisk', '(': 'parenleft', ')': 'parenright', '_': 'underscore',
    '+': 'plus', '{': 'braceleft', '}': 'braceright', ':': 'colon',
    '"': 'quotedbl', '|': 'bar', '<': 'less', '>': 'greater',
    '~': 'asciitilde',
}

# macOS virtual key codes (Carbon kVK_*: the physical key, whatever the
# keyboard layout), named after the key at that position on a US keyboard,
# which is how DEFAULT_KEYS is laid out. Tk on macOS (8.6.11+, 9) carries
# the virtual key code in bits 24-31 of the event's keycode. Mapping by
# position keeps the map usable on Swiss, French, German... layouts, where
# the US punctuation keys carry other characters and Shift + digit gives
# '+', '"', '*', 'ç'..., and the key release always matches its press.
MAC_VK = {
    0x0C: 'q', 0x0D: 'w', 0x0E: 'e', 0x0F: 'r', 0x11: 't', 0x10: 'y',
    0x20: 'u', 0x22: 'i', 0x1F: 'o', 0x23: 'p', 0x21: 'bracketleft',
    0x1E: 'bracketright', 0x00: 'a', 0x01: 's', 0x02: 'd', 0x03: 'f',
    0x05: 'g', 0x04: 'h', 0x26: 'j', 0x28: 'k', 0x25: 'l', 0x29: 'semicolon',
    0x27: 'apostrophe', 0x2A: 'backslash', 0x06: 'z', 0x07: 'x', 0x08: 'c',
    0x09: 'v', 0x0B: 'b', 0x2D: 'n', 0x2E: 'm', 0x2B: 'comma',
    0x2F: 'period', 0x2C: 'slash', 0x32: 'grave', 0x0A: 'grave',
    0x12: '1', 0x13: '2', 0x14: '3', 0x15: '4', 0x17: '5', 0x16: '6',
    0x1A: '7', 0x1C: '8', 0x19: '9', 0x1D: '0', 0x1B: 'minus', 0x18: 'equal',
    0x31: 'space', 0x30: 'Tab', 0x24: 'Return', 0x4C: 'Return',
    0x33: 'BackSpace', 0x75: 'Delete', 0x35: 'Escape',
    0x7E: 'Up', 0x7B: 'Left', 0x7D: 'Down', 0x7C: 'Right',
    0x7A: 'F1', 0x78: 'F2', 0x63: 'F3', 0x76: 'F4', 0x60: 'F5', 0x61: 'F6',
    0x62: 'F7', 0x64: 'F8',
    0x52: '0', 0x53: '1', 0x54: '2', 0x55: '3', 0x56: '4', 0x57: '5',
    0x58: '6', 0x59: '7', 0x5B: '8', 0x5C: '9',
}


def event_key(keysym, keycode, state, darwin=sys.platform == 'darwin'):
    """A Tk key event -> (key for KeyMap, Shift held).

    '?' always opens the map, whatever key types it. On macOS the key is
    the physical one (MAC_VK), so the map does not depend on the layout;
    elsewhere it is the keysym, with Tk 9's character keysyms renamed.
    """
    shift = bool(state & 0x1)
    if keysym in ('question', '?'):
        return 'question', False
    if darwin:
        key = MAC_VK.get((keycode >> 24) & 0xFF)
        if key is not None:
            return key, shift
    return CHAR_KEYSYMS.get(keysym, keysym), shift


def pretty(key):
    if key in PRETTY:
        return PRETTY[key]
    return key.upper() if len(key) == 1 else key


class Action:
    __slots__ = ('kind', 'code', 'label', 'key')

    def __init__(self, kind, code, label, key):
        self.kind, self.code, self.label, self.key = kind, code, label, key

    def __repr__(self):
        return '<%s %s %r on %s>' % (self.kind, self.code, self.label, self.key)


class KeyMap:
    """Keysym -> Action. Built from DEFAULT_KEYS, optionally overridden."""

    def __init__(self, entries=DEFAULT_KEYS):
        self.by_key = {}
        for key, kind, code, label in entries:
            self.by_key[key] = Action(kind, code, label, key)

    @classmethod
    def load(cls, path=None):
        """Defaults, then each `KEY KIND CODE [LABEL]` line of `path`.

        KIND is button, encoder or none (none unmaps the key). Lines
        starting with # are comments.
        """
        km = cls()
        if not path:
            return km
        with open(path) as fh:
            for raw in fh:
                line = raw.split('#', 1)[0].strip()
                if not line:
                    continue
                parts = line.split(None, 3)
                key = parts[0]
                kind = parts[1] if len(parts) > 1 else 'none'
                if kind == 'none':
                    km.by_key.pop(key, None)
                    continue
                code = int(parts[2], 0)
                label = parts[3] if len(parts) > 3 else '%s %d' % (kind, code)
                km.by_key[key] = Action(kind, code, label, key)
        return km

    def lookup(self, key):
        return self.by_key.get(key)

    def shortcuts(self):
        """-> {(kind, code): 'key text'} for labelling the control surface."""
        out = {}
        for key, act in self.by_key.items():
            if act.kind == HOST:
                continue
            text = pretty(key)
            if act.kind == ENCODER:
                text = '%s/Sh+%s' % (text, text)
            slot = (act.kind, act.code)
            out[slot] = out[slot] + ' ' + text if slot in out else text
        return out

    def table(self):
        """-> [(keys, does)] rows for the help overlay, in panel order."""
        rows = collections.OrderedDict()
        for key, act in self.by_key.items():
            if act.kind == ENCODER:
                text = '%s / Shift+%s' % (pretty(key), pretty(key))
                does = '%s right / left' % act.label
            else:
                text, does = pretty(key), act.label
            rows.setdefault(does, []).append(text)
        return [(' '.join(keys), does) for does, keys in rows.items()]


class KeyInput:
    """Key down/up events -> panel events, shared by the window and --script.

    Holding a key holds its button: the press goes out on key down and the
    release on key up, so a modifier held on one key while another is
    tapped is a real chord. Auto-repeat is absorbed for buttons (the
    firmware makes its own repeats) and passed through for encoders, so a
    held number key keeps turning.

    Key up is delayed by `release_delay` seconds and cancelled if the same
    key comes down again first. That covers platforms whose auto-repeat
    sends up/down pairs; on others it only delays a release by that much.
    """

    def __init__(self, keymap, send, release_delay=0.04, clock=time.monotonic):
        self.keymap = keymap
        self.send = send
        self.release_delay = release_delay
        self.clock = clock
        self.down = {}            # key -> Action, buttons currently held
        self.pending_up = {}      # key -> due time
        self.last = ''
        self._enc = (None, 0.0, 0)  # (code and direction, time, streak)

    # Detents per key event while one encoder key is held (auto-repeat):
    # (events in the streak so far, detents per event). A tap is one detent.
    ACCEL = ((8, 1), (16, 2), (24, 4), (None, 8))
    ACCEL_GAP = 0.25

    def _encoder_detents(self, code, shift):
        """1 for a tap; more per event the longer an encoder key repeats."""
        now = self.clock()
        last, t, streak = self._enc
        same = last == (code, shift) and now - t < self.ACCEL_GAP
        streak = streak + 1 if same else 0
        self._enc = ((code, shift), now, streak)
        for limit, n in self.ACCEL:
            if limit is None or streak < limit:
                return n
        return 1

    def key_down(self, keysym, shift=False):
        """-> the Action taken, or None for an unmapped key."""
        key, shifted = normalize(keysym)
        shift = shift or shifted
        repeat = key in self.pending_up or key in self.down
        self.pending_up.pop(key, None)
        act = self.keymap.lookup(key)
        self.last = '%s%s' % ('Shift+' if shift else '', keysym)
        if act is None:
            return None
        if act.kind == ENCODER:
            n = self._encoder_detents(act.code, shift)
            self.send('encoder', act.code, -n if shift else n)
        elif act.kind == BUTTON:
            if not repeat:
                self.down[key] = act
                self.send('press', act.code, 0)
        return act

    def key_up(self, keysym):
        key, _ = normalize(keysym)
        if key in self.down:
            self.pending_up[key] = self.clock() + self.release_delay

    def poll(self):
        """Send the releases that are due. Call often (every ~10 ms)."""
        if not self.pending_up:
            return
        now = self.clock()
        for key, due in list(self.pending_up.items()):
            if due <= now:
                del self.pending_up[key]
                act = self.down.pop(key, None)
                if act is not None:
                    self.send('release', act.code, 0)

    def release_all(self):
        """Let go of everything at once (focus lost, window closed)."""
        for key, act in list(self.down.items()):
            self.send('release', act.code, 0)
        self.down.clear()
        self.pending_up.clear()

    def held_labels(self):
        return [act.label for act in self.down.values()]


# --- live MIDI ports (virtual CoreMIDI / ALSA ports, via python-rtmidi) ----

def _rtmidi():
    try:
        import rtmidi
    except ImportError:
        raise SystemExit('--midi-port / --midi-in need python-rtmidi: run with '
                         '`uv run --with python-rtmidi python -m emu.live ...`')
    return rtmidi


class LiveMidiOut:
    """A virtual MIDI output named `name` that other apps (a DAW, a synth
    via an interface) can read. Bytes from the emulated DIN out are parsed
    and each message is sent as soon as it is complete. Only a virtual port
    is ever opened: no hardware device is touched."""

    def __init__(self, name):
        from emu.midi import Parser
        self.out = _rtmidi().MidiOut()
        self.out.open_virtual_port(name)
        self.parser = Parser()
        self.sent = 0

    def byte(self, t, b):
        for _, msg in self.parser.feed(t, b):
            self.out.send_message(list(msg))
            self.sent += 1

    def close(self):
        try:
            self.out.close_port()
        except Exception:                # noqa: BLE001
            pass


class LiveMidiIn:
    """A virtual MIDI input named `name`: what other apps send to it goes
    to the emulated unit's DIN in. Virtual port only."""

    def __init__(self, name, post):
        self.inp = _rtmidi().MidiIn()
        self.inp.ignore_types(sysex=False, timing=False, active_sense=True)
        self.inp.open_virtual_port(name)
        self.inp.set_callback(lambda event, data=None: post(bytes(event[0])))

    def close(self):
        try:
            self.inp.cancel_callback()
            self.inp.close_port()
        except Exception:                # noqa: BLE001
            pass


# --- the emulator worker ----------------------------------------------------

def frame_to_fb(buf, fb):
    """Unpack one 1024-byte panel frame into `fb` (W*H bytes, 0 or 1).

    Page order is inverted (page 0 is the bottom eight rows); see
    emu/panel.py.
    """
    for x in range(W):
        col = buf[8 * x:8 * x + 8]
        for page in range(8):
            byte = col[page]
            base = (8 * (7 - page)) * W + x
            for n in range(8):
                fb[base + n * W] = (byte >> n) & 1


class LiveEmulator(threading.Thread):
    """Runs the firmware in real time and publishes the panel. No widgets.

    The attributes emu/gui.py's App reads (fb, version, stats, error,
    device, button_names, encoder_names, captured, pause, stop_flag, inbox)
    are all here, so the App's window can show this worker unchanged.
    """

    daemon = True

    def __init__(self, snapshot, syx=None, speed=1.0, idle_skip=True,
                 dwell_ms=25.0, post_intro_ips=4 * INSTR_PER_SEC,
                 keep_frames=900, verbose=False, save_at_exit=None,
                 dtim0_hz=DTIM0_HZ, encoder_mode='step', raw_counts=4,
                 ssi0_hz=None, dsp_sync=True, ips=None, midi=True,
                 midi_port=None, midi_in_port=None, midi_log=None,
                 midi_file=None, midi_bpm=120.0):
        super().__init__()
        self.midi_enabled = midi           # model UART9 (DIN MIDI)
        self.midi_port = midi_port         # name of a virtual CoreMIDI out
        self.midi_in_port = midi_in_port   # name of a virtual CoreMIDI in
        self.midi_log_path = midi_log      # text log written at the end
        self.midi_file = midi_file         # .mid written at the end
        self.midi_bpm = midi_bpm
        self.midi = None
        self.leds = None                   # emu/leds.py PanelLeds
        self._uart_mark = 0
        self.midi_inbox = collections.deque()
        self._midi_live = None
        self.ips = ips                     # emulated CPU rate; None: as saved
        self.dtim0_hz = dtim0_hz
        self.ssi0_hz = ssi0_hz             # None: no audio clock (as before)
        self.dsp_sync = dsp_sync           # plant the SHARC's sync word
        self.ssi0 = None
        self.encoder_mode = encoder_mode   # 'step' or 'raw'
        self.raw_counts = raw_counts
        self._detents = {}                  # channel -> [count, emu_s]
        self.snapshot = snapshot
        self.syx = syx
        self.speed = speed               # 1.0 real time, 0 = unthrottled
        self.idle_skip = idle_skip
        self.dwell_s = dwell_ms / 1000.0
        self.post_intro_ips = post_intro_ips
        self.verbose = verbose
        self.save_at_exit = save_at_exit
        self.fb = bytearray(W * H)
        self.raw = None                  # newest published 1024-byte frame
        self.version = 0
        self.captured = collections.deque(maxlen=keep_frames)
        self.inbox = collections.deque()
        self.wake = threading.Event()
        self.pause = threading.Event()
        self.stop_flag = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.device = None
        self.held = None
        self.button_names = {}
        self.encoder_names = {}
        self.device_error = None
        self.latency = collections.deque(maxlen=500)
        self.deliveries = collections.deque(maxlen=500)
        self.frame_times = collections.deque(maxlen=4000)
        self.speed_log = collections.deque(maxlen=2000)   # per 0.5 s window
        self.stats = {
            'status': 'loading snapshot', 'frames': 0, 'fps': 0.0,
            'px': 0, 'panel_lit': 0, 'source': 'panel', 'pc': 0, 'tcb': 0,
            'tasks': 0, 'prints': 0, 'pit': (0, 0, 0), 'dtim3': 0,
            'mainloop': 0, 'jobs': 0, 'instrs': 0, 'wall_ips': 0.0,
            'real': 0.0, 'load': 0.0, 'emu_s': 0.0, 'lag_ms': 0.0,
            'latency_ms': None, 'load_s': 0.0, 'terminal': False,
            'idle_skips': 0, 'latches': 0,
            'ssi_blocks': 0, 'vec170': 0, 'vec191': 0,
            'midi_out': 0, 'midi_in': 0,
        }
        self._latch = None
        self._watch = None               # (t_input, raw_before, instrs)
        self._last_button_emu = -1e9
        self._frame_t = time.monotonic()

    # -- called from other threads -----------------------------------------

    def post(self, kind, code, arg=0):
        """Queue one panel event (press, release, encoder, release_all)."""
        self.inbox.append((kind, code, arg, time.monotonic()))
        self.wake.set()

    def post_midi(self, data):
        """Queue bytes for the emulated unit's MIDI in (any thread)."""
        self.midi_inbox.append(bytes(data))
        self.wake.set()

    def emu_time(self):
        """Emulated seconds, inside a step too (worker thread)."""
        s = self._stepper
        return self._emu_s + s.blocks * s.per_block / self._step_ips

    def stop(self):
        self.stop_flag.set()
        self.pause.clear()
        self.wake.set()

    # -- worker ---------------------------------------------------------------

    def _setup(self):
        t0 = time.monotonic()
        extra = {'syx': self.syx} if self.syx else {}
        if self.ssi0_hz:
            extra['ssi0_request_hz'] = int(self.ssi0_hz)
            extra['ssi0_legacy_upgrade'] = not _has_ssi0_state(self.snapshot)
            # One event per major loop (64 minors): exact at the interrupt,
            # 64 times fewer emulator stops than one per request.
            extra['ssi0_per_event'] = SSI0_MINORS_PER_BLOCK
        m, ev, st, pc, inq, at = build(
            self.snapshot, unblock=True, softfloat=True, bitmap=True,
            dsp=True, on_pixel=None, deferred_components=('timers',), **extra)
        profile = symbols.resolve(open(config.main_image(), 'rb').read())
        try:
            self.device, _fw = devices.identify(config.firmware(self.syx))
            self.held = panelin.Held(self.device)
            self.button_names = panelin.control_names(m, profile, 'button')
            self.encoder_names = panelin.control_names(m, profile, 'encoder')
        except BaseException as exc:                 # DeviceError is SystemExit
            self.device_error = '%s: %s' % (type(exc).__name__, exc)

        intro = intro_running(m, profile.intro_pit3_isr)
        timers = ev['restore_checkpoint_timers']()
        if timers is None:
            timers = Timers(Pits(m, hold=intro),
                            Dtims(m, channels=(3,), hold=intro))
            ev['checkpoint_components']['timers'] = timers
        self._pending_ips = None
        if intro and profile.intro_done is not None:
            def handover(uc, a, s, d):
                timers.release()
                if self.post_intro_ips:
                    self._pending_ips = self.post_intro_ips
            at(profile.intro_done, handover)

        if self.ips:
            # The emulated ColdFire's speed. digikit's default (4.68M, 4x
            # after the intro) was calibrated to the screen's frame rate;
            # the unit's MCF54418 runs at 250 MHz. With the audio clock on,
            # the frame handler runs 1,500 times a second, and too low a
            # rate leaves no time for the UI task. Idle skipping makes the
            # spare time free, so a realistic rate costs only the real work.
            for source in timers.sources:
                source.ips = int(self.ips)
            self.post_intro_ips = 0
        self.ssi0 = ev.get('ssi0_dma')
        self.intfrc = None
        if self.ssi0 is not None:
            # The sequencer runs on software-forced interrupts that the
            # frame handler raises (emu/intfrc.py); vector 191 itself stays
            # with the SSI model.
            from emu import intfrc
            self.intfrc = intfrc.install(m, ev, skip=(191,))
            if not self.ssi0._checkpoint_restored or self.ips:
                self.ssi0.ips = timers.sources[0].ips
            self.ssi0.align(timers.now)
            # Half a period out of phase with the timers. A snapshot is saved
            # on a timer deadline, and at --ips 150e6 every timer period is a
            # whole number of SSI periods, so aligned on `timers.now` each
            # SSI event fell a few instructions before a PIT/DTIM deadline:
            # the timer then found the CPU in the channel-50 ISR (IPL 7) and
            # the tick was dropped, every time (PIT0, PIT2 and DTIM3 ~100%
            # missed, the RTOS tick and the UI loop stopped). The real phase
            # is unknown; half a period leaves the frame handler time to
            # finish before the next timer tick.
            if self.ssi0.next is not None and not self.ssi0._checkpoint_restored:
                self.ssi0.next += self.ssi0.period / 2
            if self.dsp_sync:
                # Stand-in for the SHARC: its sync word at the head of every
                # row of both RX banks. The SSI model never writes RX data,
                # so these stay put; the rest of each row reads as silence.
                for bank in DSP_SYNC_BANKS:
                    for row in range(DSP_SYNC_ROWS):
                        m.uc.mem_write(bank + row * DSP_SYNC_ROW_BYTES,
                                       DSP_SYNC_WORD)

        # Panel LEDs: seeded from the firmware's own copies, then decoded
        # from the UART8 stream to the panel MCU (emu/leds.py).
        from emu.leds import PanelLeds
        self.leds = PanelLeds()
        img = open(config.main_image(), 'rb').read()
        if PanelLeds.image_matches(img, MAIN_LOAD):
            self.leds.from_memory(m.uc, img, MAIN_LOAD)
        else:
            print('[live] LED tables not found in this image; LEDs from the '
                  'wire only', flush=True)
        self._uart_mark = len(ev['uart_out'])

        stats = self.stats
        if profile.mainloop is not None:
            at(profile.mainloop, lambda uc, a, s, d: stats.__setitem__(
                'mainloop', stats['mainloop'] + 1))
        if profile.job_pump is not None:
            at(profile.job_pump, lambda uc, a, s, d: stats.__setitem__(
                'jobs', stats['jobs'] + 1))

        # The untorn frame, grabbed at the double-buffer diff's entry
        # (emu/panel.py). Only a reference is stored here; work happens in
        # _publish, outside the emulator.
        if profile.panel_diff is not None and profile.fb_front is not None:
            def latch(uc, a, s, d):
                stats['latches'] += 1
                buf = panel.read(m, profile.fb_front)
                if buf is not None:
                    self._latch = buf
            at(profile.panel_diff, latch)

        # Until the firmware renders again, show what the panel already
        # holds: after the diff's swap that is the BACK buffer (emu/panel.py).
        # A static screen (a menu) may not be re-rendered for a long time.
        for ptr in (profile.get('fb_back'), profile.fb_front):
            buf = panel.read(m, ptr) if ptr is not None else None
            if buf is not None and any(buf):
                self._latch = buf
                break
        stepper = _fast_stepper(m)
        self._stepper = stepper
        self._emu_s = 0.0
        self._step_ips = timers.sources[0].ips
        if self.midi_enabled:
            # DIN MIDI on UART9 (emu/midi.py): out by eDMA 37 and by PIO,
            # in by eDMA 36. Bytes are logged with emulated time stamps.
            from emu import midi as midimod
            if self.midi_port:
                self._midi_live = LiveMidiOut(self.midi_port)
            self.midi = midimod.install(
                m, ev, clock=self.emu_time,
                on_byte=self._midi_live.byte if self._midi_live else None)
            if self.midi is None:
                print('[live] UART9 is not set up as on 1.12; no MIDI',
                      flush=True)
            elif self.midi_in_port:
                self._midi_in = LiveMidiIn(self.midi_in_port, self.post_midi)
        if self.dtim0_hz:
            # DTIM0's counter is the firmware's free-running time base: the
            # panel driver stamps every key and encoder record with it
            # (0x4011fc8c, 0x4011f8bc, 0x4011ff0c), and tap tempo divides by
            # it. Nothing in the main OS or the bootstrap programs DTIM0, so
            # the code that starts it runs before them and the emulator
            # never does: the counter reads 0 for ever. Here it counts at
            # dtim0_hz of emulated time, interpolated inside the step from
            # the fast stepper's block count.
            hz = self.dtim0_hz

            def dtcn(uc, typ, addr, size, val, data):
                t = self.emu_time()
                uc.mem_write(DTIM0_DTCN, struct.pack(
                    '>I', int(t * hz) & 0xFFFFFFFF))
            m.uc.hook_add(UC_HOOK_MEM_READ, dtcn, begin=DTIM0_DTCN,
                          end=DTIM0_DTCN + 3)
        self.stats['encoder_mode'] = self.encoder_mode
        if self.encoder_mode == 'step':
            img = open(config.main_image(), 'rb').read()
            off = ENC_ACCUMULATE - MAIN_LOAD
            if img[off:off + len(ENC_ACCUMULATE_BYTES)] != ENC_ACCUMULATE_BYTES:
                print('[live] encoder accumulator not found in this image; '
                      'encoders fall back to raw counts', flush=True)
                self.encoder_mode = self.stats['encoder_mode'] = 'raw'
            else:
                at(ENC_ACCUMULATE, self._make_detent_hook(m))
        self.idle = {'steps': -1, 'blocks': 0, 'yielded': False,
                     'skipped_blocks': 0, 'skips': 0, 'yields': 0}
        if self.idle_skip:
            idle = self.idle
            spins = ev.get('idle_spins', {'n': 1})

            def on_idle(uc, addr, size, data):
                s = stepper
                quiet = (s.steps == idle['steps']
                         and s.blocks - idle['blocks'] <= IDLE_QUIET_BLOCKS)
                idle['steps'], idle['blocks'] = s.steps, s.blocks
                if not quiet:
                    idle['yielded'] = False
                    return
                if not idle['yielded'] and addr != RTOS_IDLE:
                    # One reschedule first, as build()'s own idle hook
                    # does every 20000 passes -- unless that hook is about
                    # to raise it on this very pass.
                    idle['yielded'] = True
                    if spins['n'] % 20000:
                        idle['yields'] += 1
                        m.raise_vector(32)
                    return
                if s.left > 0:
                    idle['skipped_blocks'] += s.left
                    idle['skips'] += 1
                    s.blocks += s.left
                    s.left = 0
                    uc.emu_stop()
            for addr in find_idle_spins(
                    open(config.main_image(), 'rb').read(), MAIN_LOAD):
                at(addr, on_idle)
        self.stats['load_s'] = time.monotonic() - t0
        return m, ev, pc, timers, profile, stepper

    def write_midi(self):
        """Write --midi-log / --midi-file from what was sent so far."""
        from emu import midi as midimod
        log = self.midi.log if self.midi is not None else []
        if self.midi_log_path:
            midimod.write_log(self.midi_log_path, log)
            print('[live] MIDI log %s (%d bytes)' % (self.midi_log_path, len(log)),
                  flush=True)
        if self.midi_file:
            msgs = midimod.parse([(w, b) for _, w, b, _ in log])
            with open(self.midi_file, 'wb') as fh:
                fh.write(midimod.smf(msgs, bpm=self.midi_bpm, t0=0.0))
            print('[live] MIDI file %s (%d messages, time 0 = start of the run)'
                  % (self.midi_file, len(msgs)), flush=True)

    def _make_detent_hook(self, m):
        """One key press = one parameter step, whatever the parameter.

        The unit's encoders are high-resolution and the firmware filters
        them (0x4011364e, per encoder state at handler+0x20+20*n):
        +16 is an idle meter that rises by one every handler tick to 48 and
        swallows rotation until 3*|counts| has brought it under 25 (about
        8 counts after any pause), +12 accumulates counts and a step is
        taken per `divisor` counts of the parameter's own config, +8 is a
        velocity that multiplies steps. A key press has no physical speed,
        so for a press this hook clears the three fields and hands the
        accumulator exactly `divisor` counts per detent: one step of the
        parameter's own size (its press-and-turn size while the encoder is
        held down). Only calls with keyboard detents pending are touched.
        """
        uc = m.uc
        detents = self._detents

        def hook(_uc, a, s, d):
            if not detents:
                return
            sp = uc.reg_read(UC_M68K_REG_A7)
            handler, enc, delta, cfg, pushed = struct.unpack(
                '>IIiII', bytes(uc.mem_read(sp + 4, 20)))
            item = detents.pop(enc, None)
            if item is None and len(detents) == 1:
                item = detents.popitem()[1]
            if item is None or not item[0]:
                return
            base = cfg + (0x10 if pushed & 0xFF else 4)
            divisor = struct.unpack('>i', bytes(uc.mem_read(base + 4, 4)))[0]
            if not 0 < divisor < 0x10000:
                return
            state = handler + 0x20 + 0x14 * enc if enc < 10 else handler + 0xE8
            uc.mem_write(state + 8, bytes(12))           # velocity, acc, idle
            uc.mem_write(sp + 0xC, struct.pack('>i', item[0] * divisor))
            self.stats['detents'] = self.stats.get('detents', 0) + abs(item[0])
        return hook

    def run(self):
        try:
            m, ev, pc, timers, profile, stepper = self._setup()
        except BaseException as exc:                 # noqa: BLE001
            self.error = '%s: %s' % (type(exc).__name__, exc)
            self.stats['status'] = 'failed to load'
            print('[live] FAILED TO LOAD: %s' % self.error, flush=True)
            self.ready.set()
            return
        self._m, self._ev, self._timers = m, ev, timers
        self.ready.set()
        stats = self.stats
        stats['status'] = 'running'
        emu_s = 0.0
        t0 = time.monotonic()
        win_t, win_emu, win_instrs, win_skip = t0, 0.0, 0, 0
        per_block = stepper.per_block
        asyncs = tuple(e for e in (self.ssi0, getattr(self, 'intfrc', None),
                                   self.midi)
                       if e is not None)
        while not self.stop_flag.is_set():
            if self.pause.is_set():
                stats['status'] = 'paused'
                time.sleep(0.05)
                t0 = time.monotonic() - (emu_s / self.speed if self.speed else 0)
                continue
            if self._pending_ips:
                for source in timers.sources:
                    source.ips = self._pending_ips
                if self.ssi0 is not None:
                    self.ssi0.ips = self._pending_ips
                self._pending_ips = None
            ips = timers.sources[0].ips
            pc = self._deliver(m, profile, pc, emu_s, ips)
            self._emu_s, self._step_ips = emu_s, ips
            if self.midi_inbox and self.midi is not None:
                data = b''.join(self.midi_inbox.popleft()
                                for _ in range(len(self.midi_inbox)))
                if self.midi.feed(data):
                    stats['midi_in'] += len(data)
                pc = m.uc.reg_read(UC_M68K_REG_PC)
            pc, executed, why = spin(m, pc, 1, pits=timers, fast=True,
                                     async_events=asyncs, timers_first=True)
            if why != 'limit':
                stats['status'] = 'halted: %s' % why
                print('[live] HALTED: %s at pc=0x%08x' % (why, pc), flush=True)
                break
            stats['instrs'] += executed
            emu_s += executed / ips
            stats['emu_s'] = emu_s
            out = ev['uart_out']
            if len(out) != self._uart_mark:
                self.leds.feed(bytes(out[self._uart_mark:]))
                self._uart_mark = len(out)
            self._publish()
            now = time.monotonic()
            if now - win_t >= 0.5:
                dt = now - win_t
                skipped = self.idle['skipped_blocks'] * per_block
                stats['wall_ips'] = (stats['instrs'] - win_instrs) / dt
                stats['real'] = (emu_s - win_emu) / dt
                ran = stats['instrs'] - win_instrs
                stats['load'] = (1.0 - (skipped - win_skip) / ran) if ran else 0.0
                stats['idle_skips'] = self.idle['skips']
                stats['pc'] = pc
                stats['tasks'] = len(ev['tasks'])
                fired = timers.fired
                stats['pit'] = (fired.get('PIT0', 0), fired.get('PIT2', 0),
                                fired.get('PIT3', 0))
                stats['dtim3'] = fired.get('DTIM3', 0)
                if self.midi is not None:
                    stats['midi_out'] = len(self.midi.log)
                if self.ssi0 is not None:
                    stats['ssi_blocks'] = self.ssi0.major_loops[50]
                    stats['vec170'] = self.ssi0.vector170
                    stats['vec191'] = self.ssi0.vector191
                    stats['forced'] = dict(self.intfrc.taken)
                if now - self._frame_t > 1.0:
                    stats['fps'] = 0.0
                self.speed_log.append((round(stats['real'], 3),
                                       round(stats['load'], 3),
                                       round(stats['wall_ips'] / 1e6, 2)))
                win_t, win_emu, win_instrs, win_skip = (
                    now, emu_s, stats['instrs'], skipped)
            if self.speed:
                ahead = emu_s / self.speed - (now - t0)
                stats['lag_ms'] = max(0.0, -ahead * 1000.0)
                if ahead > 0.001:
                    self.wake.wait(min(ahead, 0.1))
                    self.wake.clear()
                elif ahead < -MAX_LAG_S:
                    t0 = now - emu_s / self.speed - MAX_LAG_S
        else:
            stats['status'] = 'stopped'
        if self.midi is not None:
            stats['midi_out'] = len(self.midi.log)
            self.write_midi()
        if self._midi_live is not None:
            self._midi_live.close()
        if getattr(self, '_midi_in', None) is not None:
            self._midi_in.close()
        if self.save_at_exit:
            from emu.checkpoint import save_longrun
            save_longrun(m, ev, timers, self.save_at_exit,
                         extra={'instrs': stats['instrs'], 'live': True})
            print('[live] saved %s' % self.save_at_exit, flush=True)

    def _deliver(self, m, profile, pc, emu_s, ips):
        """Send due input at a step boundary. -> the PC (feeding moves it)."""
        if not self.inbox:
            return pc
        if self.held is None:
            self.inbox.clear()
            return pc
        out = bytearray()
        first_t = None
        first_act = None           # earliest press or turn: what a person
        sent_button = False        # waits to see the screen answer
        while self.inbox:
            item = self.inbox[0]
            kind, code, arg = item[0], item[1], item[2]
            t_in = item[3] if len(item) > 3 else time.monotonic()
            if kind == 'encoder':
                self.inbox.popleft()
                channel = self.device.encoder_channel(code)
                if channel is None:
                    continue
                if self.encoder_mode == 'step':
                    for ch, (n, t) in list(self._detents.items()):
                        if emu_s - t > DETENT_TTL_S:
                            del self._detents[ch]
                    n = self._detents.get(channel, [0, emu_s])[0] + arg
                    self._detents[channel] = [n, emu_s]
                else:
                    arg *= self.raw_counts
                while arg:
                    step = max(-30, min(30, arg))
                    out += panelin.encode_encoder(channel, step)
                    arg -= step
            else:
                # One button transition per delivery, each at least dwell_s
                # of emulated time after the previous one.
                if sent_button or emu_s - self._last_button_emu < self.dwell_s:
                    break
                self.inbox.popleft()
                sent_button = True
                self._last_button_emu = emu_s
                if kind == 'press':
                    pos = self.held.press(code)
                    out += panelin.encode_buttons(*pos) if pos else b''
                elif kind == 'release':
                    pos = self.held.release(code)
                    out += panelin.encode_buttons(*pos) if pos else b''
                elif kind == 'release_all':
                    for pos in self.held.release_all():
                        out += panelin.encode_buttons(*pos)
            first_t = t_in if first_t is None else min(first_t, t_in)
            if kind in ('press', 'encoder'):
                first_act = t_in if first_act is None else min(first_act, t_in)
        if not out:
            return pc
        new_pc = panelin.feed(m, profile, bytes(out))
        now = time.monotonic()
        self.deliveries.append((first_t, now, self.stats['instrs'], bytes(out)))
        if first_act is not None:
            # Latency is timed from the newest press or turn; a release or
            # a held modifier that changes nothing on screen is not waited on.
            self._watch = (first_act, self.raw, emu_s)
        if self.verbose:
            print('[live] input --feed %d:%s (queued %.1f ms)'
                  % (self.stats['instrs'], out.hex(), (now - first_t) * 1000),
                  flush=True)
        return new_pc

    def _publish(self):
        buf = self._latch
        if buf is None or buf == self.raw:
            return
        now = time.monotonic()
        self.raw = buf
        frame_to_fb(buf, self.fb)
        self.captured.append(bytes(self.fb))
        self.frame_times.append((now, self.stats['instrs']))
        s = self.stats
        s['frames'] += 1
        s['fps'] = 1.0 / max(1e-6, now - self._frame_t)
        s['panel_lit'] = int.from_bytes(buf, 'big').bit_count()
        self._frame_t = now
        self.version += 1
        if self._watch is not None:
            t_in, before, emu0 = self._watch
            if now - t_in > 2.0:
                self._watch = None        # nothing visible came of it
            elif buf != before:
                ms = (now - t_in) * 1000.0
                self.latency.append((ms, (s['emu_s'] - emu0) * 1000.0))
                s['latency_ms'] = ms
                self._watch = None


# --- the window -----------------------------------------------------------

def _window_classes():
    """The Tk classes, built only when a window is wanted (import tkinter
    lazily, so --script and --bench run anywhere)."""
    import tkinter as tk
    from emu import gui

    def hex_colour(rgb):
        return '#%02x%02x%02x' % rgb

    class LedKey(gui.CanvasKey):
        """A key with its LED: a lit strip along the top of the key face,
        in the colour the firmware sent the panel (emu/leds.py)."""

        led = None                    # (r, g, b) 8-bit, or None: no LED

        def set_led(self, rgb):
            if rgb != self.led:
                self.led = rgb
                self._draw()

        def _draw(self):
            super()._draw()
            if self.led is None:
                return
            width = int(self.cget('width'))
            inset = 3 if self.pressed else 1
            self.create_rectangle(inset + 4, inset + 4, width - 7, inset + 8,
                                  fill=hex_colour(self.led), outline='')

    class LiveControls(gui.Controls):
        """emu/gui.py's control surface with each key's shortcut on it,
        and the panel LEDs drawn on the keys (plus the 8 round LEDs)."""

        def __init__(self, master, device, button_names, encoder_names, send,
                     scale, shortcuts):
            self.shortcuts = shortcuts
            self.led_keys = {}            # button code -> [LedKey]
            super().__init__(master, device, button_names, encoder_names,
                             send, scale)
            box = tk.Frame(self, bg=self.BG)
            box.grid(row=2, column=0, pady=(0, 6))
            tk.Label(box, text='PAGE LEDS', bg=self.BG, fg='#7f8b9c',
                     font=('TkFixedFont', 7)).pack(side='left', padx=(0, 6))
            self.round_canvas = tk.Canvas(box, width=8 * 18, height=14,
                                          bg=self.BG, highlightthickness=0)
            self.round_canvas.pack(side='left')
            self.round_dots = [self.round_canvas.create_oval(
                3 + 18 * i, 2, 13 + 18 * i, 12, fill='#202428', outline='#434c57')
                for i in range(8)]

        def _button(self, box, code, text, accent=None, width=48, height=32):
            accent = accent or self.AMBER
            if code in self._latching:
                widget = LedKey(box, text, accent=accent, width=width,
                                height=height,
                                on_press=lambda c=code: self._toggle_latch(c, widget))
            else:
                widget = LedKey(box, text, accent=accent, width=width,
                                height=height,
                                on_press=lambda c=code: self.send('press', c, 0),
                                on_release=lambda c=code: self.send('release', c, 0))
            self.led_keys.setdefault(code, []).append(widget)
            return widget

        def set_leds(self, leds):
            """Draw a PanelLeds state on the keys and the round LEDs."""
            for led, code in leds.led_codes.items():
                rgb = leds.rgb8(led)
                for widget in self.led_keys.get(code, ()):
                    widget.set_led(rgb)
            for n, led in enumerate(sorted(leds.led_names)[:8]):
                rgb = leds.rgb8(led)
                self.round_canvas.itemconfigure(
                    self.round_dots[n],
                    fill=hex_colour(rgb) if any(rgb) else '#202428')

        def _group_labels(self, group):
            labels = super()._group_labels(group)
            out = {}
            for code, text in labels.items():
                key = self.shortcuts.get((group.kind, code))
                if key and group.kind == 'encoder':
                    out[code] = '%s %s' % (text, key.split('/')[0])
                elif key:
                    out[code] = '%s\n%s' % (text, key)
                else:
                    out[code] = text
            return out

    class LiveApp(gui.App):
        def __init__(self, snapshot, syx, keymap, speed=1.0, idle_skip=True,
                     dwell_ms=25.0, scale=None, verbose=False,
                     dtim0_hz=DTIM0_HZ, encoder_mode='step', raw_counts=4,
                     save_at_exit=None, ssi0_hz=None, dsp_sync=True, ips=None,
                     live_extra=None):
            self.keymap = keymap
            self._live_extra = dict(live_extra or {})
            self._live = dict(syx=syx, speed=speed, idle_skip=idle_skip,
                              dwell_ms=dwell_ms, verbose=verbose,
                              dtim0_hz=dtim0_hz, encoder_mode=encoder_mode,
                              raw_counts=raw_counts, save_at_exit=save_at_exit,
                              ssi0_hz=ssi0_hz, dsp_sync=dsp_sync, ips=ips)
            self.keys = None
            self.help = None
            self._leds_shown = -1
            super().__init__(snapshot, syx=syx, scale=scale)
            self.title('Digitone II live (? for the key map)')
            self.keys = KeyInput(keymap, self.send_input)
            self.bind_all('<KeyPress>', self._key_down)
            self.bind_all('<KeyRelease>', self._key_up)
            # Tab would move the focus; here it is TRK.
            self.bind_all('<Tab>', self._key_down)
            self.bind_all('<Shift-Tab>', self._key_down)
            self.bind_all('<<PrevWindow>>', lambda e: 'break')
            # Tk 9 maps Tab to the virtual event <<NextWindow>>, whose 'all'
            # binding (focus traversal) wins over the plain <Tab> binding
            # above, so a plain Tab never reached _key_down. Remove the
            # virtual events; then Tab and Shift+Tab are TRK.
            for virtual in ('<<NextWindow>>', '<<PrevWindow>>'):
                try:
                    self.event_delete(virtual)
                except tk.TclError:
                    pass
            self.bind('<FocusOut>', self._lost_focus)
            try:
                self.bind('<Deactivate>', self._lost_focus)
            except tk.TclError:
                pass
            self.after(10, self._poll_keys)
            if sys.platform == 'darwin':
                # Cmd-Q must stop the worker first, or the interpreter kills
                # it inside emu_start on the way out (see gui.App.quit_all).
                self.createcommand('::tk::mac::Quit', self.quit_all)

        def start(self):
            o = self._live
            self.emu = LiveEmulator(self.snapshot, syx=o['syx'],
                                    speed=o['speed'],
                                    idle_skip=o['idle_skip'],
                                    dwell_ms=o['dwell_ms'],
                                    verbose=o['verbose'],
                                    dtim0_hz=o['dtim0_hz'],
                                    encoder_mode=o['encoder_mode'],
                                    raw_counts=o['raw_counts'],
                                    save_at_exit=o['save_at_exit'],
                                    ssi0_hz=o['ssi0_hz'],
                                    dsp_sync=o['dsp_sync'],
                                    ips=o['ips'], **self._live_extra)
            self.emu.start()

        def send_input(self, kind, code, arg):
            if self.emu:
                self.emu.post(kind, code, arg)

        def _ensure_controls(self):
            if self.controls is not None or not self.emu or not self.emu.device:
                return
            self.controls = LiveControls(
                self.surface, self.emu.device, self.emu.button_names,
                self.emu.encoder_names, self.send_input, self.scale,
                self.keymap.shortcuts())
            self.controls.pack()
            self.panel = self.controls.panel

        def _key_down(self, event):
            if self.keys is None:
                return 'break'
            if sys.platform == 'darwin' and event.state & 0x8:
                return None          # Command-key shortcuts belong to the Mac
            key, shift = event_key(event.keysym, event.keycode, event.state)
            act = self.keys.key_down(key, shift)
            if act is not None and act.kind == HOST and act.code == 'help':
                self.toggle_help()
            return 'break'

        def _key_up(self, event):
            if self.keys is not None:
                self.keys.key_up(
                    event_key(event.keysym, event.keycode, event.state)[0])
            return 'break'

        def _lost_focus(self, event=None):
            if self.keys is not None and self.keys.down:
                self.keys.release_all()

        def _poll_keys(self):
            if self.keys is not None:
                self.keys.poll()
            self.after(10, self._poll_keys)

        def restart(self):
            if self.keys is not None:
                self.keys.release_all()
            super().restart()

        def quit_all(self):
            if self.emu:
                self.emu.stop()
                # --save writes the snapshot in the worker after its loop
                # ends; wait for it before the window (and the process) go.
                self.emu.join(timeout=60 if self.emu.save_at_exit else 3)
            super().quit_all()

        def toggle_help(self):
            if self.help is not None:
                self.help.destroy()
                self.help = None
                return
            top = tk.Toplevel(self, bg='#15181d')
            top.title('Key map')
            rows = self.keymap.table()
            text = '\n'.join('%-22s %s' % (k, d) for k, d in rows)
            text += ('\n\nHold a key to hold the button (FUNC = Space).'
                     '\nShift + number turns that encoder left.'
                     '\nHold F1-F8 while turning for press-and-turn.'
                     '\nPress ? again to close.')
            tk.Label(top, text=text, justify='left', bg='#15181d',
                     fg='#d5dbe3', font=('TkFixedFont', 11)).pack(
                         padx=14, pady=12)
            top.bind('<KeyPress>', self._key_down)
            top.bind('<KeyRelease>', self._key_up)
            self.help = top

        def tick(self):
            e = self.emu
            if e is not None:
                if e.error:
                    self.status.configure(text=e.error, fg='#ff8f8f')
                else:
                    self._ensure_controls()
                    if self.panel is not None and e.version != self.shown:
                        self.panel.draw(e.fb)
                        self.shown = e.version
                    if (self.controls is not None and e.leds is not None
                            and e.leds.version != self._leds_shown):
                        self._leds_shown = e.leds.version
                        self.controls.set_leds(e.leds)
                    s = e.stats
                    lat = s['latency_ms']
                    self.frames_lbl.configure(
                        text='%.2fx real time   CPU load %.0f%%   %.1fM instr/s   '
                             'MIDI out %d in %d bytes'
                             % (s['real'], 100 * s['load'], s['wall_ips'] / 1e6,
                                s['midi_out'], s['midi_in']))
                    held = ', '.join(self.keys.held_labels()) if self.keys else ''
                    self.status.configure(
                        text='%s   last key %s   held: %s\n'
                             'last reaction %s   lag %.0f ms   frames %d   '
                             'emulated %.1f s   ? = key map'
                             % (s['status'], self.keys.last if self.keys else '',
                                held or '-',
                                '%.0f ms' % lat if lat is not None else '-',
                                s['lag_ms'], s['frames'], s['emu_s']),
                        fg='#9aa7b8')
            self.after(30, self.tick)

    return LiveApp


# --- headless -------------------------------------------------------------

def parse_key(spec):
    """'shift+1' -> ('1', True); 'space' -> ('space', False)."""
    shift = False
    if spec.lower().startswith('shift+'):
        shift, spec = True, spec[6:]
    return spec, shift


def wait_frames(emu, keys, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        keys.poll()
        time.sleep(0.005)
        if emu.error or not emu.is_alive():
            raise RuntimeError('emulator stopped: %s' % (emu.error
                                                         or emu.stats['status']))


def wait_emulated(emu, keys, seconds):
    """Wait until `seconds` of EMULATED time have passed (whatever the
    emulator's speed against the wall clock)."""
    end = emu.stats['emu_s'] + seconds
    while emu.stats['emu_s'] < end:
        wait_frames(emu, keys, 0.005)


def save_png(emu, path):
    """The screen as PNG + ASCII, and the LEDs as PATH.leds.{json,txt,png}."""
    if emu.raw is None:
        raise RuntimeError('no frame yet for %s' % path)
    panel.write_png(emu.raw, path)
    base = os.path.splitext(path)[0]
    with open(base + '.txt', 'w') as fh:
        fh.write('\n'.join(panel.ascii_art(emu.raw)) + '\n')
    if emu.leds is not None:
        emu.leds.save(base, names=emu.button_names)


def run_script(emu, keymap, lines, outdir, log=print):
    """Drive `emu` from a script, through KeyInput, in real time.

    Commands, one per line (# starts a comment):
        wait S          let S seconds of wall time pass
        wait-emu S      let S seconds of emulated time pass
        down KEY        key down (KEY is a keysym; shift+KEY for Shift)
        up KEY          key up
        tap KEY [S]     down, hold S seconds (default 0.12), up
        png NAME        save the screen as OUTDIR/NAME.png (+ .txt)
        midi-in HEX     send bytes to the unit's DIN MIDI in (e.g. 90 3c 64)
    """
    keys = KeyInput(keymap, emu.post)
    for n, raw in enumerate(lines, 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        cmd, *args = line.split()
        if cmd == 'wait':
            wait_frames(emu, keys, float(args[0]))
        elif cmd == 'wait-emu':
            wait_emulated(emu, keys, float(args[0]))
        elif cmd in ('down', 'tap'):
            key, shift = parse_key(args[0])
            act = keys.key_down(key, shift)
            log('[script] %-5s %-10s -> %s' % (cmd, args[0], act))
            if cmd == 'tap':
                wait_frames(emu, keys, float(args[1]) if len(args) > 1 else 0.12)
                keys.key_up(key)
                wait_frames(emu, keys, keys.release_delay + 0.01)
        elif cmd == 'up':
            keys.key_up(parse_key(args[0])[0])
            wait_frames(emu, keys, keys.release_delay + 0.01)
        elif cmd == 'midi-in':
            data = bytes.fromhex(''.join(args))
            emu.post_midi(data)
            log('[script] midi-in %s (emulated %.3f s)' % (data.hex(),
                                                          emu.stats['emu_s']))
            wait_frames(emu, keys, 0.01)
        elif cmd == 'png':
            path = os.path.join(outdir, args[0] + '.png')
            save_png(emu, path)
            log('[script] png %s (emulated %.2f s, frame %d)'
                % (path, emu.stats['emu_s'], emu.stats['frames']))
        else:
            raise SystemExit('script line %d: unknown command %r' % (n, cmd))
    keys.release_all()
    wait_frames(emu, keys, 0.1)


def _has_ssi0_state(snapshot):
    """-> True when the snapshot already carries an SSI0 component."""
    from emu.snapshot import _load_blob
    return 'ssi0_dma' in (_load_blob(snapshot).get('components') or {})


def summary(emu):
    s = emu.stats
    lat = [ms for ms, _ in emu.latency]
    lat_emu = sorted(e for _, e in emu.latency)
    lines = [
        'snapshot load %.1f s, emulated %.2f s, %d frames, %.0fM instructions'
        % (s['load_s'], s['emu_s'], s['frames'], s['instrs'] / 1e6),
        'last window: %.2fx real time, CPU load %.0f%%, %.1fM instr/s, '
        'idle skips %d' % (s['real'], 100 * s['load'], s['wall_ips'] / 1e6,
                           s['idle_skips']),
    ]
    if emu.ssi0 is not None:
        lines.append('audio clock %d Hz: %d blocks, vector 170 x%d, '
                     'vector 191 x%d' % (emu.ssi0_hz, s['ssi_blocks'],
                                         s['vec170'], s['vec191']))
        lines.append('forced interrupts taken: %s' % (
            ', '.join('vector %d x%d' % kv for kv in sorted(s.get('forced', {}).items()))
            or 'none'))
    if emu.midi is not None:
        from emu.midi import parse
        msgs = parse([(w, b) for _, w, b, _ in emu.midi.log])
        lines.append('DIN MIDI: out %d bytes (%d messages), in %d bytes; '
                     'vectors 156/157/181 x%d/%d/%d'
                     % (len(emu.midi.log), len(msgs), emu.midi.rx_bytes,
                        emu.midi.taken[156], emu.midi.taken[157],
                        emu.midi.taken[181]))
    if lat:
        lat_sorted = sorted(lat)
        lines.append('input -> changed frame: n=%d  median %.0f ms  min %.0f  '
                     'max %.0f ms (wall); emulated median %.0f ms'
                     % (len(lat), lat_sorted[len(lat) // 2], lat_sorted[0],
                        lat_sorted[-1], lat_emu[len(lat_emu) // 2]))
    if emu.deliveries:
        q = sorted((d[1] - d[0]) * 1000 for d in emu.deliveries)
        lines.append('input queued before delivery: n=%d  median %.1f ms  '
                     'max %.1f ms' % (len(q), q[len(q) // 2], q[-1]))
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Live keyboard control of the emulated unit.',
        epilog='docs/LIVE.md has the key map and the limits.')
    ap.add_argument('snapshot')
    ap.add_argument('--syx', required=True)
    ap.add_argument('--sections', help='sets DT2_SECTIONS')
    ap.add_argument('--keymap', help='file of KEY KIND CODE [LABEL] overrides')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='1 = real time (default), 0 = as fast as possible')
    ap.add_argument('--no-idle-skip', action='store_true')
    ap.add_argument('--dwell-ms', type=float, default=25.0,
                    help='emulated time between button transitions')
    ap.add_argument('--dtim0-hz', type=float, default=DTIM0_HZ,
                    help='rate of the modelled DTIM0 time base; 0 leaves it '
                         'at 0 as plain digikit does')
    ap.add_argument('--encoder-mode', choices=('step', 'raw'), default='step',
                    help='step: one key press = one parameter step (default); '
                         'raw: send --raw-counts encoder counts per press and '
                         'let the firmware filter them as from the panel')
    ap.add_argument('--raw-counts', type=int, default=4)
    ap.add_argument('--ssi0-hz', type=int, nargs='?', const=SSI0_HZ_DN2,
                    help='run the audio clock (SSI0/eDMA48/50 model) at this '
                         'request rate; bare flag = %d (Digitone II: one '
                         'block per 32 samples at 48 kHz)' % SSI0_HZ_DN2)
    ap.add_argument('--ips', type=float,
                    help='emulated CPU instructions per second (default: the '
                         'snapshot\'s timer rate); try 150e6 with --ssi0-hz')
    ap.add_argument('--no-dsp-sync', action='store_true',
                    help='with --ssi0-hz: do not plant the SHARC sync word')
    ap.add_argument('--no-midi', action='store_true',
                    help='do not model DIN MIDI (UART9)')
    ap.add_argument('--midi-log', help='write the DIN MIDI out bytes, with '
                    'emulated time stamps, and the parsed messages here')
    ap.add_argument('--midi-file', help='write DIN MIDI out as a Standard '
                    'MIDI File here (time 0 = start of the run)')
    ap.add_argument('--midi-bpm', type=float, default=120.0,
                    help='tempo written into --midi-file (default 120; use '
                         'the project tempo so bars line up)')
    ap.add_argument('--midi-port', nargs='?', const='Digitone II (emulator)',
                    help='open a virtual MIDI output with this name and send '
                         'DIN MIDI out to it live (needs python-rtmidi)')
    ap.add_argument('--midi-in', nargs='?', const='Digitone II (emulator) in',
                    help='open a virtual MIDI input with this name; what is '
                         'sent to it reaches the DIN MIDI in (needs python-rtmidi)')
    ap.add_argument('--scale', type=int)
    ap.add_argument('--verbose', action='store_true',
                    help='print every delivered input as a guirun --feed')
    ap.add_argument('--script', help='headless: run key events from FILE')
    ap.add_argument('--out', default='out/live', help='headless PNG directory')
    ap.add_argument('--bench', type=float,
                    help='headless: run SECONDS with no input and report speed')
    ap.add_argument('--save', help='save a snapshot here when the run ends')
    ap.add_argument('--report', help='headless: write speed and latency JSON')
    ap.add_argument('--print-keymap', action='store_true')
    args = ap.parse_args(argv)
    if args.sections:
        os.environ['DT2_SECTIONS'] = args.sections
    keymap = KeyMap.load(args.keymap)
    if args.print_keymap:
        for keys, does in keymap.table():
            print('%-24s %s' % (keys, does))
        return 0
    if not os.path.exists(args.snapshot):
        raise SystemExit('no such snapshot: %s' % args.snapshot)
    headless = args.script or args.bench is not None
    if not headless:
        LiveApp = _window_classes()
        LiveApp(args.snapshot, args.syx, keymap, speed=args.speed,
                idle_skip=not args.no_idle_skip, dwell_ms=args.dwell_ms,
                scale=args.scale, verbose=args.verbose,
                dtim0_hz=args.dtim0_hz, encoder_mode=args.encoder_mode,
                raw_counts=args.raw_counts, save_at_exit=args.save,
                ssi0_hz=args.ssi0_hz, ips=args.ips,
                dsp_sync=not args.no_dsp_sync,
                live_extra=dict(midi=not args.no_midi, midi_port=args.midi_port,
                    midi_in_port=args.midi_in, midi_log=args.midi_log,
                    midi_file=args.midi_file, midi_bpm=args.midi_bpm)).mainloop()
        return 0

    emu = LiveEmulator(args.snapshot, syx=args.syx, speed=args.speed,
                       idle_skip=not args.no_idle_skip, dwell_ms=args.dwell_ms,
                       verbose=args.verbose, save_at_exit=args.save,
                       dtim0_hz=args.dtim0_hz,
                       encoder_mode=args.encoder_mode,
                       raw_counts=args.raw_counts,
                       ssi0_hz=args.ssi0_hz, dsp_sync=not args.no_dsp_sync,
                       ips=args.ips, midi=not args.no_midi, midi_port=args.midi_port,
                       midi_in_port=args.midi_in, midi_log=args.midi_log,
                       midi_file=args.midi_file, midi_bpm=args.midi_bpm)
    t0 = time.monotonic()
    emu.start()
    emu.ready.wait()
    if emu.error:
        print(emu.error)
        return 2
    print('[live] loaded in %.1f s' % (time.monotonic() - t0), flush=True)
    while emu.raw is None and emu.is_alive():
        time.sleep(0.01)
    print('[live] first frame %.1f s after start' % (time.monotonic() - t0),
          flush=True)
    try:
        if args.script:
            os.makedirs(args.out, exist_ok=True)
            with open(args.script) as fh:
                run_script(emu, keymap, fh.readlines(), args.out)
        if args.bench is not None:
            wait_frames(emu, KeyInput(keymap, emu.post), args.bench)
    finally:
        emu.stop()
        emu.join(timeout=30)
    print(summary(emu))
    if args.report:
        import json
        with open(args.report, 'w') as fh:
            json.dump({'stats': {k: v for k, v in emu.stats.items()
                                 if isinstance(v, (int, float, str))},
                       'latency_wall_ms': [round(a, 1) for a, _ in emu.latency],
                       'latency_emulated_ms': [round(b, 1) for _, b in emu.latency],
                       'queued_ms': [round((d[1] - d[0]) * 1000, 2)
                                     for d in emu.deliveries],
                       'speed_windows': list(emu.speed_log)},
                      fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
