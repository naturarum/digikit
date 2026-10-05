"""Panel LED decoding (emu/leds.py); no firmware image required."""
import json
import struct

from emu import leds


class FakeUc:
    def __init__(self):
        self.memory = {}

    def mem_read(self, address, size):
        return bytes(self.memory.get(address + i, 0) for i in range(size))

    def mem_write(self, address, data):
        for i, value in enumerate(data):
            self.memory[address + i] = value


def test_message_lengths():
    assert leds.message_length(0xB9) == 3
    assert leds.message_length(0xB5) == 5
    assert leds.message_length(0xBC) == 2
    assert leds.message_length(0xB8) == 1
    assert leds.message_length(0x10) == 10 and leds.message_length(0x16) == 10
    assert leds.message_length(0x35) == 10
    assert leds.message_length(0x41) == 2
    assert leds.message_length(0x00) is None and leds.message_length(0xF0) is None


def test_describe_value():
    assert leds.describe_value(0) == 'off'
    assert leds.describe_value(1) == 'glow'
    assert leds.describe_value(17) == 'red 16/16'
    assert leds.describe_value(18) == 'green 1/16'
    assert leds.describe_value(0x71) == 'white 16/16'
    assert leds.describe_value(0x83) == 'special 131'


def test_feed_split_messages_and_palette():
    p = leds.PanelLeds()
    stream = bytes.fromhex('bc01' 'b91971' '1010' + '00' * 8 + 'b5113f0000' 'b8' 'b90811')
    # in two arbitrary pieces: a message cut in half waits for the rest
    assert p.feed(stream[:4]) == 0
    p.feed(stream[4:])
    assert p.values[25] == 0x71 and p.values[8] == 0x11
    assert p.palette[0x11] == (63, 0, 0)
    assert p.rgb6(8) == (63, 0, 0)
    assert p.rgb8(8) == (255, 0, 0)
    assert p.other[0xBC] == 1 and p.other[0x10] == 1 and p.other[0xB8] == 1
    assert p.skipped == 0


def test_unknown_bytes_skipped():
    p = leds.PanelLeds()
    p.feed(bytes.fromhex('00ffb90a21'))
    assert p.skipped == 2 and p.values[10] == 0x21


def build_image(load=0x40000000):
    """A tiny image with the 1.12 anchors and tables at their addresses."""
    size = 0x0040_0000
    img = bytearray(size)

    def put(addr, data):
        img[addr - load:addr - load + len(data)] = data
    put(0x40132428, struct.pack('>II', leds.LED_CODE_TO_INDEX, leds.LED_STATE))
    put(0x401322FC, struct.pack('>I', leds.PALETTE_CACHE))
    put(0x400D2626, struct.pack('>III', leds.TEST_LED_CODES, leds.TEST_BUTTONS,
                                leds.TEST_ROUND_LEDS))
    put(leds.LED_CODE_TO_INDEX, struct.pack('>64i', -1, *range(63)))
    codes = [26, 27, 11, 12] + [0] * 41
    buttons = [25, 26, 20, 21] + [0] * 41
    put(leds.TEST_LED_CODES, struct.pack('>45i', *codes))
    put(leds.TEST_BUTTONS, struct.pack('>45i', *buttons))
    put(leds.TEST_ROUND_LEDS, struct.pack('>8i', *range(42, 50)))
    return bytes(img), load


def test_from_memory_and_mapping(tmp_path):
    img, load = build_image()
    assert leds.PanelLeds.image_matches(img, load)
    assert not leds.PanelLeds.image_matches(bytes(len(img)), load)
    uc = FakeUc()
    uc.mem_write(leds.PALETTE_CACHE + 4 * 0x71, struct.pack('>I', 0x3F3F3F))
    uc.mem_write(leds.LED_STATE + 25, b'\x71')        # wire index 25 = LED code 26
    uc.mem_write(leds.LED_STATE + 10, b'\x05')
    uc.mem_write(leds.LED_ALT + 10, b'\x21')          # alternate layer on index 10
    uc.mem_write(leds.LED_ALT_ON + 10, b'\x01')
    p = leds.PanelLeds()
    p.from_memory(uc, img, load)
    assert p.led_codes == {25: 25, 26: 26, 10: 20, 11: 21}
    assert p.led_names[41] == 'round 1' and p.led_names[48] == 'round 8'
    assert p.values[25] == 0x71 and p.values[10] == 0x21
    assert p.rgb8(25) == (255, 255, 255)
    base = str(tmp_path / 'x')
    p.save(base, names={25: 'TRIG 1', 20: 'PLAY'})
    rows = json.load(open(base + '.leds.json'))
    assert rows[25]['name'] == 'TRIG 1' and rows[25]['is'] == 'white 16/16'
    assert rows[41]['name'] == 'round 1'
    text = open(base + '.leds.txt').read()
    assert 'TRIG 1' in text and 'PLAY' in text
    png = open(base + '.leds.png', 'rb').read()
    assert png.startswith(b'\x89PNG\r\n\x1a\n') and b'IEND' in png
