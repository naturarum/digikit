"""Panel LEDs, decoded from what the firmware sends the panel MCU on UART8.

Read from Digitone II 1.12 (dn2-cfw notes/T2.4c-midi-leds-speed.md), all
through the UART8 transmit ring `FUN_4000220c` (eDMA 35, emu/edma.py):

* `B9 index value` (`FUN_40132428`, `FUN_401320e8`): LED `index` (0-53)
  shows palette entry `value`. Views set LEDs by **LED code**, a numbering
  of their own: `FUN_4002c318(manager, led_code, mode, colour)` ->
  `FUN_40132428(led_code, value)`, which looks the wire index up in the int
  table at 0x401f3a14 (on 1.12 simply `led_code - 1`). The firmware keeps
  the state in 0x466b8420[54] (with an alternate layer at 0x466b8456, used
  where 0x466b848c[n] is set).
* **Which key an LED code belongs to** comes from the factory UI test
  (constructor `FUN_400d2626`): 45 button LED codes at 0x401f37b0, paired
  entry by entry with the button codes at 0x401f3628 (the test lights LED
  i when button i is pressed), and the 8 round LEDs at 0x401f36dc (LED
  codes 42-49; the main screen uses them as the pattern page indicator).
  On 1.12: TRK 1, KEYBOARD 2, FUNC 3, PTN 4, SONG 5, PRESET 6, SETTINGS 7,
  VOICE 8, TEMPO 9, RECORD 10, PLAY 11, STOP 12, YES 13, NO 14, UP 15,
  DOWN 16, LEFT 17, RIGHT 18, PAGE 19, page keys TRIG..MOD 20-25,
  TRIG 1-8 26-33, TRIG 9-16 34-41, ARP 50, PLUS 51, STACK 52, MINUS 53.
* `B5 index R G B` (`FUN_401322fc`): palette entry `index` (0-135) is this
  colour, 6 bits a channel (0-63). `FUN_40132388` sends the whole palette:
  0 black, 1 a dim white (the unlit-key glow, scaled by the LED brightness
  setting), then from 2 on sixteen levels each of red (2-17), green
  (18-33), blue (34-49), yellow (50-65), cyan (66-81), magenta (82-97),
  white (98-113), orange (114-129), and six specials (130-135); one of
  three tables (0x4020905c + 0x192 * n) by the brightness setting. The
  firmware caches it at 0x402a10d0 (one long per entry, 0x00RRGGBB).
* Others, not LEDs: `B7 x`, `BC x` (`FUN_4011fb28`, `FUN_4011fb94`, from
  the render loop), `BB a b c` (`FUN_4011fb58`), `B8` (`FUN_40131eec`),
  `0x40|x 00` (`FUN_4011faf6`) and `0x10|x n` + 8 bytes (`FUN_40131f04`).

A snapshot taken after boot has already sent the palette and the LEDs, so
`from_memory()` seeds the state from the firmware's own copies; the wire
updates it from then on.
"""
from __future__ import annotations

import json
import struct
import zlib

N_LEDS = 54
N_PALETTE = 136
# Digitone II 1.12 RAM copies (checked against the image before use).
LED_STATE = 0x466B8420
LED_ALT = LED_STATE + N_LEDS
LED_ALT_ON = LED_ALT + N_LEDS
PALETTE_CACHE = 0x402A10D0
LED_CODE_TO_INDEX = 0x401F3A14
TEST_LED_CODES, TEST_BUTTONS, TEST_N = 0x401F37B0, 0x401F3628, 45
TEST_ROUND_LEDS, TEST_N_ROUND = 0x401F36DC, 8
# The image's own code must hold these literals for the RAM copies and
# tables to mean what is described above (1.12 addresses).
ANCHORS = ((0x40132428, 0xAC, (LED_CODE_TO_INDEX, LED_STATE)),
           (0x401322FC, 0x8C, (PALETTE_CACHE,)),
           (0x400D2626, 0x3C8, (TEST_LED_CODES, TEST_BUTTONS, TEST_ROUND_LEDS)))

# Message lengths by first byte; see the module docstring.
FIXED = {0xB5: 5, 0xB7: 2, 0xB8: 1, 0xB9: 3, 0xBB: 4, 0xBC: 2}

COLOURS = ('red', 'green', 'blue', 'yellow', 'cyan', 'magenta', 'white',
           'orange')


def describe_value(value):
    """Palette index -> 'off', 'glow', 'red 16/16' ..."""
    if value == 0:
        return 'off'
    if value == 1:
        return 'glow'
    if 2 <= value < 130:
        group, level = divmod(value - 2, 16)
        return '%s %d/16' % (COLOURS[group], level + 1)
    return 'special %d' % value


def message_length(head):
    """Bytes in the UART8 message starting with `head`, or None."""
    if head in FIXED:
        return FIXED[head]
    if head & 0xD0 == 0x10:          # FUN_40131f04: (x & 0xef) | 0x10, 0x10-0x3f
        return 10
    if 0x40 <= head < 0x80:          # FUN_4011faf6: (x & 0xbf) | 0x40
        return 2
    return None


class PanelLeds:
    def __init__(self):
        self.values = [0] * N_LEDS
        self.palette = [(0, 0, 0)] * N_PALETTE
        self.led_codes = {}         # wire index -> button code
        self.led_names = {}         # wire index -> name for round LEDs
        self.version = 0
        self.changes = 0
        self.skipped = 0
        self.other = {}
        self._buf = bytearray()

    # -- seeding -------------------------------------------------------

    @staticmethod
    def image_matches(img, load_addr):
        """-> True when the image's LED code holds the expected literals."""
        for func, size, lits in ANCHORS:
            off = func - load_addr
            body = img[off:off + size]
            if len(body) < size:
                return False
            for lit in lits:
                if struct.pack('>I', lit) not in body:
                    return False
        return True

    def from_memory(self, uc, img=None, load_addr=None):
        """Seed palette, values and the LED -> code map from guest memory."""
        pal = struct.unpack('>%dI' % N_PALETTE,
                            bytes(uc.mem_read(PALETTE_CACHE, 4 * N_PALETTE)))
        self.palette = [((v >> 16) & 0x3F, (v >> 8) & 0x3F, v & 0x3F) for v in pal]
        vals = bytes(uc.mem_read(LED_STATE, N_LEDS))
        alt = bytes(uc.mem_read(LED_ALT, N_LEDS))
        on = bytes(uc.mem_read(LED_ALT_ON, N_LEDS))
        self.values = [alt[i] if on[i] else vals[i] for i in range(N_LEDS)]
        if img is not None and load_addr is not None:
            self.map_from_image(img, load_addr)
        self.version += 1

    def map_from_image(self, img, load_addr):
        """Wire index -> button code, from the factory UI test's tables."""
        def ints(addr, n):
            off = addr - load_addr
            return struct.unpack('>%di' % n, img[off:off + 4 * n])
        to_index = ints(LED_CODE_TO_INDEX, 64)

        def index(led_code):
            return to_index[led_code] if 0 <= led_code < 64 else -1
        self.led_codes, self.led_names = {}, {}
        for led_code, button in zip(ints(TEST_LED_CODES, TEST_N),
                                    ints(TEST_BUTTONS, TEST_N)):
            i = index(led_code)
            if 0 <= i < N_LEDS:
                self.led_codes[i] = button
        for n, led_code in enumerate(ints(TEST_ROUND_LEDS, TEST_N_ROUND)):
            i = index(led_code)
            if 0 <= i < N_LEDS:
                self.led_names[i] = 'round %d' % (n + 1)

    # -- the wire ------------------------------------------------------

    def feed(self, data):
        """Bytes sent on UART8 -> state. -> number of LED changes."""
        buf = self._buf
        buf += data
        changed = 0
        i = 0
        while i < len(buf):
            n = message_length(buf[i])
            if n is None:
                self.skipped += 1
                i += 1
                continue
            if i + n > len(buf):
                break
            msg = buf[i:i + n]
            i += n
            head = msg[0]
            if head == 0xB9 and msg[1] < N_LEDS:
                if self.values[msg[1]] != msg[2]:
                    self.values[msg[1]] = msg[2]
                    changed += 1
            elif head == 0xB5 and msg[1] < N_PALETTE:
                rgb = (msg[2] & 0x3F, msg[3] & 0x3F, msg[4] & 0x3F)
                if self.palette[msg[1]] != rgb:
                    self.palette[msg[1]] = rgb
                    changed += 1
            else:
                self.other[head] = self.other.get(head, 0) + 1
        del buf[:i]
        if changed:
            self.changes += changed
            self.version += 1
        return changed

    # -- reading -------------------------------------------------------

    def rgb6(self, led):
        v = self.values[led]
        return self.palette[v] if v < N_PALETTE else (0, 0, 0)

    def rgb8(self, led, gamma=0.5):
        """8-bit colour for drawing; `gamma` < 1 lifts dim levels so a
        1/63 glow is visible on a screen."""
        return tuple(int(round(255 * (c / 63.0) ** gamma)) if c else 0
                     for c in self.rgb6(led))

    def by_code(self):
        """-> {control code: (led, value)}."""
        return {code: (led, self.values[led]) for led, code in self.led_codes.items()}

    def snapshot(self, names=None):
        """JSON-friendly state: one row per LED."""
        names = names or {}
        rows = []
        for led in range(N_LEDS):
            code = self.led_codes.get(led)
            v = self.values[led]
            rows.append({'led': led, 'code': code,
                         'name': (names.get(code) if code else None)
                         or self.led_names.get(led),
                         'value': v, 'is': describe_value(v),
                         'rgb6': list(self.rgb6(led))})
        return rows

    def save(self, path_base, names=None):
        """Write PATH.leds.json, PATH.leds.txt and PATH.leds.png."""
        rows = self.snapshot(names)
        with open(path_base + '.leds.json', 'w') as fh:
            json.dump(rows, fh, indent=0)
        with open(path_base + '.leds.txt', 'w') as fh:
            for r in rows:
                if r['value'] > 1:
                    fh.write('led %2d  button %-3s %-16s %3d  %s\n'
                             % (r['led'], r['code'], r['name'] or '-',
                                r['value'], r['is']))
        with open(path_base + '.leds.png', 'wb') as fh:
            fh.write(self.png())

    def png(self, cell=28, gap=4):
        """A plain picture: the 16 trig keys on two rows, the 8 round LEDs,
        then every other key LED in wire order on two rows."""
        by_code = {code: led for led, code in self.led_codes.items()}
        trigs = [by_code.get(c) for c in range(25, 41)]
        rounds = sorted(self.led_names)
        other = [i for i in sorted(self.led_codes) if i not in trigs]
        half = (len(other) + 1) // 2
        grid = [trigs[:8], trigs[8:], rounds, other[:half], other[half:]]
        cols = max(len(r) for r in grid)
        w = cols * (cell + gap) + gap
        h = len(grid) * (cell + gap) + gap + gap
        px = [[(24, 26, 30)] * w for _ in range(h)]
        for r, codes in enumerate(grid):
            y0 = gap + r * (cell + gap) + (gap if r >= 2 else 0)
            for c, led in enumerate(codes):
                x0 = gap + c * (cell + gap)
                col = self.rgb8(led) if led is not None else (0, 0, 0)
                if col == (0, 0, 0):
                    col = (8, 8, 10)
                for y in range(y0, y0 + cell):
                    row = px[y]
                    for x in range(x0, x0 + cell):
                        edge = y in (y0, y0 + cell - 1) or x in (x0, x0 + cell - 1)
                        row[x] = (70, 76, 84) if edge else col
        raw = b''.join(b'\x00' + bytes(v for p in row for v in p) for row in px)

        def chunk(kind, data):
            return (struct.pack('>I', len(data)) + kind + data
                    + struct.pack('>I', zlib.crc32(kind + data) & 0xFFFFFFFF))
        return (b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
