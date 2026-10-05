"""emu/live.py --script commands added for T2.4c (wait-emu, midi-in); no emulator."""

from emu import live


class FakeEmu:
    """Advances emulated time by 10 ms per wall poll; records posts."""

    def __init__(self):
        self.stats = {'emu_s': 0.0}
        self.posted, self.midi = [], []
        self.error = None
        self.leds = None

    def is_alive(self):
        self.stats['emu_s'] += 0.01
        return True

    def post(self, *a):
        self.posted.append(a)

    def post_midi(self, data):
        self.midi.append((round(self.stats['emu_s'], 2), bytes(data)))


def test_wait_emu_and_midi_in(tmp_path):
    emu = FakeEmu()
    lines = ['wait-emu 0.5', 'midi-in 90 3c 64', 'wait-emu 0.25', 'midi-in fc']
    live.run_script(emu, live.KeyMap(), lines, str(tmp_path), log=lambda *a: None)
    assert [d for _, d in emu.midi] == [b'\x90\x3c\x64', b'\xfc']
    t0, t1 = emu.midi[0][0], emu.midi[1][0]
    assert t0 >= 0.5 and t1 - t0 >= 0.25
