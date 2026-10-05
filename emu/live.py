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

No sound: the DSP is not emulated (emu/dsp.py only answers its ready line).
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
from unicorn.m68k_const import UC_M68K_REG_A7

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
                 dtim0_hz=DTIM0_HZ, encoder_mode='step', raw_counts=4):
        super().__init__()
        self.dtim0_hz = dtim0_hz
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

    def stop(self):
        self.stop_flag.set()
        self.pause.clear()
        self.wake.set()

    # -- worker ---------------------------------------------------------------

    def _setup(self):
        t0 = time.monotonic()
        extra = {'syx': self.syx} if self.syx else {}
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
        self._emu_s = 0.0
        self._step_ips = timers.sources[0].ips
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
                t = self._emu_s + stepper.blocks * stepper.per_block / self._step_ips
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
        while not self.stop_flag.is_set():
            if self.pause.is_set():
                stats['status'] = 'paused'
                time.sleep(0.05)
                t0 = time.monotonic() - (emu_s / self.speed if self.speed else 0)
                continue
            if self._pending_ips:
                for source in timers.sources:
                    source.ips = self._pending_ips
                self._pending_ips = None
            ips = timers.sources[0].ips
            pc = self._deliver(m, profile, pc, emu_s, ips)
            self._emu_s, self._step_ips = emu_s, ips
            pc, executed, why = spin(m, pc, 1, pits=timers, fast=True)
            if why != 'limit':
                stats['status'] = 'halted: %s' % why
                print('[live] HALTED: %s at pc=0x%08x' % (why, pc), flush=True)
                break
            stats['instrs'] += executed
            emu_s += executed / ips
            stats['emu_s'] = emu_s
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

    class LiveControls(gui.Controls):
        """emu/gui.py's control surface with each key's shortcut on it."""

        def __init__(self, master, device, button_names, encoder_names, send,
                     scale, shortcuts):
            self.shortcuts = shortcuts
            super().__init__(master, device, button_names, encoder_names,
                             send, scale)

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
                     save_at_exit=None):
            self.keymap = keymap
            self._live = dict(syx=syx, speed=speed, idle_skip=idle_skip,
                              dwell_ms=dwell_ms, verbose=verbose,
                              dtim0_hz=dtim0_hz, encoder_mode=encoder_mode,
                              raw_counts=raw_counts, save_at_exit=save_at_exit)
            self.keys = None
            self.help = None
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
                                    save_at_exit=o['save_at_exit'])
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
                    s = e.stats
                    lat = s['latency_ms']
                    self.frames_lbl.configure(
                        text='%.2fx real time   CPU load %.0f%%   %.1fM instr/s'
                             % (s['real'], 100 * s['load'], s['wall_ips'] / 1e6))
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


def save_png(emu, path):
    if emu.raw is None:
        raise RuntimeError('no frame yet for %s' % path)
    panel.write_png(emu.raw, path)
    with open(os.path.splitext(path)[0] + '.txt', 'w') as fh:
        fh.write('\n'.join(panel.ascii_art(emu.raw)) + '\n')


def run_script(emu, keymap, lines, outdir, log=print):
    """Drive `emu` from a script, through KeyInput, in real time.

    Commands, one per line (# starts a comment):
        wait S          let S seconds of wall time pass
        down KEY        key down (KEY is a keysym; shift+KEY for Shift)
        up KEY          key up
        tap KEY [S]     down, hold S seconds (default 0.12), up
        png NAME        save the screen as OUTDIR/NAME.png (+ .txt)
    """
    keys = KeyInput(keymap, emu.post)
    for n, raw in enumerate(lines, 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        cmd, *args = line.split()
        if cmd == 'wait':
            wait_frames(emu, keys, float(args[0]))
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
        elif cmd == 'png':
            path = os.path.join(outdir, args[0] + '.png')
            save_png(emu, path)
            log('[script] png %s (emulated %.2f s, frame %d)'
                % (path, emu.stats['emu_s'], emu.stats['frames']))
        else:
            raise SystemExit('script line %d: unknown command %r' % (n, cmd))
    keys.release_all()
    wait_frames(emu, keys, 0.1)


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
                raw_counts=args.raw_counts, save_at_exit=args.save).mainloop()
        return 0

    emu = LiveEmulator(args.snapshot, syx=args.syx, speed=args.speed,
                       idle_skip=not args.no_idle_skip, dwell_ms=args.dwell_ms,
                       verbose=args.verbose, save_at_exit=args.save,
                       dtim0_hz=args.dtim0_hz,
                       encoder_mode=args.encoder_mode,
                       raw_counts=args.raw_counts)
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
