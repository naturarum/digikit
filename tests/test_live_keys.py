"""emu/live.py's keyboard layer: no firmware, no emulator, no display."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from emu import live, panel


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make():
    sent = []
    clock = Clock()
    keys = live.KeyInput(live.KeyMap(), lambda *a: sent.append(a), clock=clock)
    return keys, sent, clock


def test_every_control_has_a_key():
    km = live.KeyMap()
    buttons = {a.code for a in km.by_key.values() if a.kind == live.BUTTON}
    encoders = {a.code for a in km.by_key.values() if a.kind == live.ENCODER}
    assert buttons == set(range(1, 55))
    assert encoders == set(range(1, 10))


def test_normalize_folds_shift():
    assert live.normalize('Q') == ('q', True)
    assert live.normalize('exclam') == ('1', True)
    assert live.normalize('quoteright') == ('apostrophe', False)
    assert live.normalize('KP_3') == ('3', False)
    assert live.normalize('question') == ('question', False)


def test_hold_is_a_chord_and_repeat_is_absorbed():
    keys, sent, clock = make()
    keys.key_down('space')
    keys.key_down('space')          # auto-repeat
    keys.key_down('bracketleft')
    keys.key_up('bracketleft')
    clock.t += 0.1
    keys.poll()
    keys.key_up('space')
    clock.t += 0.1
    keys.poll()
    assert sent == [('press', 17, 0), ('press', 3, 0), ('release', 3, 0),
                    ('release', 17, 0)]


def test_up_down_pair_inside_release_delay_keeps_the_button_held():
    keys, sent, clock = make()
    keys.key_down('Tab')
    keys.key_up('Tab')
    clock.t += 0.01
    keys.key_down('Tab')            # X11-style auto-repeat pair
    clock.t += 0.1
    keys.poll()
    assert sent == [('press', 16, 0)]


def test_encoder_direction_and_acceleration():
    keys, sent, clock = make()
    keys.key_down('4')
    clock.t += 1
    keys.key_down('dollar', shift=True)
    assert sent == [('encoder', 4, 1), ('encoder', 4, -1)]
    sent.clear()
    clock.t += 1
    for _ in range(30):
        keys.key_down('4')
        clock.t += 0.05
    steps = [a[2] for a in sent]
    assert steps[:8] == [1] * 8 and steps[8] == 2 and max(steps) == 8


def test_release_all():
    keys, sent, clock = make()
    keys.key_down('space')
    keys.key_down('F4')
    keys.release_all()
    assert sorted(sent[2:]) == [('release', 17, 0), ('release', 44, 0)]
    assert keys.down == {}


def test_keymap_override(tmp_path):
    f = tmp_path / 'map.txt'
    f.write_text('# test\nz none\nk button 25 TRIG 1 again\n')
    km = live.KeyMap.load(str(f))
    assert km.lookup('z') is None
    assert km.lookup('k').code == 25


def test_frame_to_fb_matches_panel_lit():
    buf = bytes((i * 37 + 11) & 0xFF for i in range(panel.SIZE))
    fb = bytearray(live.W * live.H)
    live.frame_to_fb(buf, fb)
    lit = {(i % live.W, i // live.W) for i, v in enumerate(fb) if v}
    assert lit == panel.lit(buf)


def test_event_key_by_position_on_macos():
    # Swiss French: the key right of P types 'è', Shift+1 types '+', the key
    # at US Y types 'z'. By virtual key code they are FLTR, encoder A left
    # and TRIG 6, as on a US keyboard.
    ek = live.event_key
    assert ek('è', 0x21 << 24 | 0xe8, 0, darwin=True) == ('bracketleft', False)
    assert ek('+', 0x12 << 24 | 0x31, 1, darwin=True) == ('1', True)
    assert ek('z', 0x10 << 24 | 0x7a, 0, darwin=True) == ('y', False)
    assert ek('?', 0x1b << 24 | 0x27, 1, darwin=True) == ('question', False)
    assert ek('Tab', 0x30 << 24, 1, darwin=True) == ('Tab', True)


def test_event_key_tk9_character_keysyms():
    # Tk 9 gives ',' and '!' where Tk 8 gave 'comma' and 'exclam'.
    ek = live.event_key
    assert ek(',', 0, 0, darwin=False) == ('comma', False)
    assert ek('!', 0, 1, darwin=False) == ('exclam', True)
    keys, sent, clock = make()
    keys.key_down(*ek('!', 0, 1, darwin=False))
    keys.key_down(*ek(',', 0, 0, darwin=False))
    assert sent == [('encoder', 1, -1), ('press', 23, 0)]


def test_every_mapped_key_has_a_position():
    km = live.KeyMap()
    placed = set(live.MAC_VK.values()) | {'question'}
    assert set(km.by_key) <= placed
