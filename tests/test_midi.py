"""DIN MIDI model (emu/midi.py) and its MIDI helpers; no firmware image required."""

# pyright: reportMissingImports=false

import struct

from unicorn.m68k_const import UC_M68K_REG_SR

from emu import midi
from emu.edma import ATTR, BITER, CITER, CSR, DADDR, NBYTES, SADDR, SOFF, TCD_BASE


class FakeUc:
    def __init__(self):
        self.memory = {}
        self.regs = {UC_M68K_REG_SR: 0x2000}
        self.hooks = []

    def hook_add(self, kind, fn, begin=1, end=0, **kw):
        self.hooks.append((kind, fn, begin, end))
        return len(self.hooks)

    def mem_read(self, address, size):
        return bytes(self.memory.get(address + i, 0) for i in range(size))

    def mem_write(self, address, data):
        for i, value in enumerate(data):
            self.memory[address + i] = value

    def reg_read(self, register):
        return self.regs.get(register, 0)

    def write(self, address, size, value):
        """A guest write: the write hooks covering it, then memory."""
        for kind, fn, begin, end in list(self.hooks):
            if begin <= address <= end and fn.__name__ in (
                    '_on_serq', '_on_udr', '_on_intc1_mask'):
                fn(self, None, address, size, value, None)
        self.mem_write(address, value.to_bytes(size, 'big'))


class FakeMachine:
    def __init__(self):
        self.uc = FakeUc()
        self.after_rte = []
        self.raised = []

    def raise_vector(self, vector, level=None):
        self.raised.append((vector, level))
        return True


def tcd(chan):
    return TCD_BASE + chan * 0x20


def setup_uart9(m, tx_bytes=b''):
    """The 1.12 descriptors: TX ring 0x4e6c9000 (4 KB), RX ring 0x4e6c0000 (32 KB)."""
    uc = m.uc
    uc.mem_write(tcd(37) + SADDR, struct.pack('>I', 0x4E6C9000))
    uc.mem_write(tcd(37) + ATTR, struct.pack('>H', 0x6000))
    uc.mem_write(tcd(37) + SOFF, struct.pack('>h', 1))
    uc.mem_write(tcd(37) + NBYTES, struct.pack('>I', 1))
    uc.mem_write(tcd(37) + DADDR, struct.pack('>I', midi.UDR))
    uc.mem_write(tcd(37) + CITER, struct.pack('>H', len(tx_bytes)))
    uc.mem_write(tcd(37) + BITER, struct.pack('>H', len(tx_bytes)))
    uc.mem_write(tcd(37) + CSR, struct.pack('>H', 0x000A))
    uc.mem_write(0x4E6C9000, tx_bytes)
    uc.mem_write(tcd(36) + SADDR, struct.pack('>I', midi.UDR))
    uc.mem_write(tcd(36) + ATTR, struct.pack('>H', 0x0078))
    uc.mem_write(tcd(36) + DADDR, struct.pack('>I', 0x4E6C0000))
    # ICR1 for sources 28, 29, 53 = level 4
    for src in (28, 29, 53):
        uc.mem_write(0xFC04C040 + src, b'\x04')


def make(tx=b''):
    m = FakeMachine()
    setup_uart9(m, tx)
    t = [1.0]
    sent = []
    model = midi.install(m, {}, clock=lambda: t[0], on_byte=lambda tt, b: sent.append(b))
    return m, model, t, sent


def test_install_refuses_other_set_up():
    m = FakeMachine()
    assert midi.install(m, {}) is None


def test_dma_transfer_logged_with_wire_times_and_completion():
    m, model, t, sent = make(b'\x90\x3c\x64')
    m.uc.write(midi.SERQ, 1, 37)
    assert model.bytes_out == b'\x90\x3c\x64'
    assert sent == [0x90, 0x3C, 0x64]
    (t0, w0, _, p0), (_, w1, _, _), (_, w2, _, _) = model.log
    assert p0 == 'dma' and t0 == w0 == 1.0
    assert abs((w1 - w0) - 10 / 31250) < 1e-12 and abs((w2 - w1) - 10 / 31250) < 1e-12
    # SADDR advanced, CITER reloaded, DONE set, completion owed
    assert struct.unpack('>I', m.uc.mem_read(tcd(37) + SADDR, 4))[0] == 0x4E6C9003
    assert struct.unpack('>H', m.uc.mem_read(tcd(37) + CSR, 2))[0] & 0x80
    assert model.tx_pending == 1
    assert model.deliver() and m.raised == [(157, 4)]
    assert model.tx_pending == 0 and not model.deliver()


def test_tx_ring_wraps_at_4k():
    m, model, t, sent = make()
    m.uc.mem_write(tcd(37) + SADDR, struct.pack('>I', 0x4E6C9FFF))
    m.uc.mem_write(tcd(37) + CITER, struct.pack('>H', 2))
    m.uc.mem_write(0x4E6C9FFF, b'\xf8')
    m.uc.mem_write(0x4E6C9000, b'\xfa')
    m.uc.write(midi.SERQ, 1, 37)
    assert model.bytes_out == b'\xf8\xfa'
    assert struct.unpack('>I', m.uc.mem_read(tcd(37) + SADDR, 4))[0] == 0x4E6C9001


def test_other_channels_ignored():
    m, model, t, sent = make(b'\x90\x3c\x64')
    m.uc.write(midi.SERQ, 1, 35)
    assert model.log == []


def test_pio_byte_and_status_ready():
    m, model, t, sent = make()
    model._on_usr(m.uc, None, midi.USR, 1, 0, None)
    assert m.uc.mem_read(midi.USR, 1)[0] & 0x0C == 0x0C
    t[0] = 2.0
    m.uc.write(midi.UDR, 1, 0xF8)
    assert model.log == [(2.0, 2.0, 0xF8, 'pio')]


def test_uart_interrupt_follows_intc_mask():
    m, model, t, sent = make()
    assert not model.deliver()
    m.uc.write(midi.INTC1_CIMR, 1, 53)
    assert model.deliver() and m.raised[-1] == (181, 4)
    m.uc.write(midi.INTC1_SIMR, 1, 53)
    assert not model.deliver()


def test_ipl_blocks_delivery():
    m, model, t, sent = make(b'\xfa')
    m.uc.write(midi.SERQ, 1, 37)
    m.uc.regs[UC_M68K_REG_SR] = 0x2400
    assert not model.deliver() and model.tx_pending == 1
    m.uc.regs[UC_M68K_REG_SR] = 0x2000
    assert model.deliver()


def test_feed_writes_ring_and_raises_vector_156():
    m, model, t, sent = make()
    assert model.feed(b'\x90\x3c\x64')
    assert m.uc.mem_read(0x4E6C0000, 3) == b'\x90\x3c\x64'
    assert struct.unpack('>I', m.uc.mem_read(tcd(36) + DADDR, 4))[0] == 0x4E6C0003
    assert m.raised == [(156, 4)] and model.rx_bytes == 3


def test_feed_wraps_at_32k():
    m, model, t, sent = make()
    m.uc.mem_write(tcd(36) + DADDR, struct.pack('>I', 0x4E6C7FFF))
    model.feed(b'\xfa\xf8')
    assert m.uc.mem_read(0x4E6C7FFF, 1) == b'\xfa' and m.uc.mem_read(0x4E6C0000, 1) == b'\xf8'
    assert struct.unpack('>I', m.uc.mem_read(tcd(36) + DADDR, 4))[0] == 0x4E6C0001


def test_receive_semaphore_from_callback():
    m, model, t, sent = make()
    uc = m.uc
    uc.mem_write(midi.RX_CALLBACK_PTR, struct.pack('>I', 0x4011F3AC))
    uc.mem_write(0x4011F3AC, struct.pack('>HI', 0x4879, 0x445290C4))
    uc.mem_write(0x445290C4 + 0x28, struct.pack('>I', 0x445F03B8))
    assert midi.receive_semaphore(m) == 0x445F03B8
    ev = {'unblock_skip': set()}
    midi.install(m, ev)
    assert ev['unblock_skip'] == {0x445F03B8}


def test_parser_running_status_and_realtime():
    data = bytes.fromhex('fa903c64f8403c00b00700f0417f00f7fc')
    msgs = midi.parse([(i * 0.001, b) for i, b in enumerate(data)])
    hexes = [m.hex() for _, m in msgs]
    assert hexes == ['fa', '903c64', 'f8', '90403c', 'b00700', 'f0417f00f7', 'fc']
    # the running-status message is timed at its first data byte
    assert abs(msgs[3][0] - 0.005) < 1e-9


def test_describe():
    assert midi.describe(b'\x90\x3c\x64') == 'note-on ch1 60 vel 100'
    assert midi.describe(b'\x91\x3c\x00') == 'note-off ch2 60 (vel 0)'
    assert midi.describe(b'\xf8') == 'clock'
    assert midi.describe(b'\xb0\x07\x64') == 'cc ch1 7 = 100'


def test_smf_layout_and_timing():
    msgs = [(1.0, b'\xfa'), (1.0, b'\x90\x3c\x64'), (1.5, b'\xf8'), (1.5, b'\x90\x3c\x00'),
            (2.0, b'\xfc')]
    data = midi.smf(msgs, bpm=120, ppq=960, t0=1.0)
    assert data[:4] == b'MThd' and struct.unpack('>IHHH', data[4:14]) == (6, 0, 1, 960)
    assert data[14:18] == b'MTrk'
    body = data[22:]
    assert body.startswith(b'\x00\xff\x51\x03\x07\xa1\x20')      # 500000 us
    assert b'\xff\x06\x05start' in body and b'\xff\x06\x04stop' in body
    assert b'\xf7' not in body                                     # clock dropped, no escapes
    # 0.5 s at 120 BPM, 960 ppq = 960 ticks = vlq 87 40
    assert b'\x87\x40\x90\x3c\x00' in body
    assert body.endswith(b'\x00\xff\x2f\x00')
    esc = midi.smf(msgs, bpm=120, t0=1.0, realtime='escape')
    assert b'\xf7\x01\xf8' in esc


def test_vlq():
    assert midi._vlq(0) == b'\x00'
    assert midi._vlq(127) == b'\x7f'
    assert midi._vlq(128) == b'\x81\x00'
    assert midi._vlq(0x0FFFFFFF) == b'\xff\xff\xff\x7f'


def test_resume_in_flight_owes_completion():
    m = FakeMachine()
    setup_uart9(m)
    m.uc.mem_write(midi.TX_STATE, struct.pack('>I', 1))
    model = midi.install(m, {})
    assert model.tx_pending == 1
    m2 = FakeMachine()
    setup_uart9(m2)
    m2.uc.mem_write(midi.TX_STATE, struct.pack('>I', 2))
    assert midi.install(m2, {}).uart_unmasked
