"""DIN MIDI of the emulated unit: UART9 at 31,250 baud, out and in.

Found on Digitone II 1.12 (dn2-cfw notes/T2.4c-midi-leds-speed.md); the
addresses below are read from the running image, not assumed:

* `0x400cedfc` initialises UART8 at 156,250 baud (the panel MCU) and
  UART9 at 0x7a12 = **31,250 baud** (`FUN_40002844(0x7a12, 0x4011f3ac)`;
  UBG2 = 132 MHz / (32 x baud)).
* **Out, bulk: eDMA channel 37** (TCD37 at 0xFC0454A0). SADDR walks a
  4 KB ring (ATTR 0x6000: source modulo 4096, base `[0x4058e580]` =
  0x4e6c9000), DADDR is UART9's data register 0xEC07400C, one byte per
  minor, CITER = bytes to send. `FUN_40002612(len, buf)` copies into the
  ring and starts the channel (SERQ 0x25). Completion is **vector 157**
  (`FUN_40002102`), which chains whatever was queued meanwhile. Every MIDI
  task message (notes, CCs: `FUN_4012a36c`) leaves this way when bit 0 of
  `[0x446181f0]` is set (bit 1 is USB).
* **Out, single bytes: programmed I/O.** `FUN_40002746(byte)` (realtime
  bytes such as clock, via `FUN_401233de`) writes the data register
  directly when no DMA transfer is running and USR has TXRDY; otherwise it
  queues the byte in a 1 KB ring (`[0x4058e584]`), pauses channel 37 (CERQ)
  and unmasks INTC1 source 53. The UART9 interrupt, **vector 181**
  (0x40001f86), sends one byte per TXRDY and masks source 53 (SIMR 0x35)
  when the ring is empty, then resumes the DMA.
* **In: eDMA channel 36** (TCD36) moves each received byte into a 32 KB ring
  (ATTR 0x0078: destination modulo 32768, base `[0x4058e588]` =
  0x4e6c0000); DADDR is the producer pointer. **Vector 156** (0x400021a0)
  drains ring[[0x4058e5a8]] up to DADDR and calls `[0x4058e5b8]`
  (0x4011f3ac) once per byte.

This model, like emu/edma.py's UART8 TX, runs a DMA transfer at once when
it is started and queues its completion interrupt; delivery waits for an
IPL that allows it (spin() step boundaries and after every `rte`). Each
byte is logged with the emulated time it was handed to the UART and the
time it would start on the wire at 31,250 baud (10 bits a byte, back to
back).

Optional live output to a CoreMIDI port is in emu/live.py (--midi-port).
"""
# pyright: reportMissingImports=false
from __future__ import annotations

import struct

from unicorn import UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.m68k_const import UC_M68K_REG_SR

from emu.edma import (ATTR, BITER, CITER, CSR, DADDR, EDMA_BASE, NBYTES, SADDR,
                      SERQ, SLAST, SOFF, TCD_BASE)
from emu.pit import interrupt_level

UART9 = 0xEC074000
USR, UDR = UART9 + 0x04, UART9 + 0x0C     # status / data (UTB on write)
USR_TXRDY, USR_TXEMP = 0x04, 0x08
CERQ = EDMA_BASE + 0x19
INTC1_SIMR, INTC1_CIMR = 0xFC04C01C, 0xFC04C01D
TX_CHAN, RX_CHAN = 37, 36
TX_VECTOR, RX_VECTOR, UART_VECTOR = 157, 156, 181
UART_SOURCE = UART_VECTOR - 128             # INTC1 source 53
BAUD = 31_250
BYTE_S = 10.0 / BAUD                        # start + 8 data + stop


def _tcd(chan):
    return TCD_BASE + chan * 0x20


class DinMidi:
    """UART9 + eDMA 36/37 as far as the firmware's MIDI driver uses them.

    `clock()` returns the emulated time in seconds; `on_byte(t, byte)` is
    called for every byte sent (after it is logged), e.g. to forward it to a
    live port.
    """

    def __init__(self, machine, clock=lambda: 0.0, on_byte=None):
        self.m = machine
        self.clock = clock
        self.on_byte = on_byte
        self.log = []               # (t_handed, t_wire, byte, 'dma'|'pio')
        self.wire_free = 0.0        # when the line is next idle
        self.tx_pending = 0         # queued channel-37 completions
        self.uart_unmasked = False  # INTC1 source 53 (TXRDY interrupt)
        self.rx_pending = False     # vector 156 owed
        self.rx_bytes = 0
        self.rx_semaphore = None
        self.taken = {TX_VECTOR: 0, RX_VECTOR: 0, UART_VECTOR: 0}
        uc = machine.uc
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_serq, begin=SERQ, end=SERQ)
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_udr, begin=UDR, end=UDR)
        uc.hook_add(UC_HOOK_MEM_READ, self._on_usr, begin=USR, end=USR)
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_intc1_mask,
                    begin=INTC1_SIMR, end=INTC1_CIMR)
        machine.after_rte.append(self.deliver)

    # -- output ---------------------------------------------------------

    def _record(self, data, path):
        t = self.clock()
        for byte in data:
            wire = max(t, self.wire_free)
            self.wire_free = wire + BYTE_S
            self.log.append((t, wire, byte, path))
            if self.on_byte is not None:
                self.on_byte(t, byte)

    @property
    def bytes_out(self):
        return bytes(b for _, _, b, _ in self.log)

    def _on_udr(self, uc, access, address, size, value, data):
        if size == 1:
            self._record(bytes([value & 0xFF]), 'pio')

    def _on_usr(self, uc, access, address, size, value, data):
        # The transmitter is always ready: bytes leave at once here.
        uc.mem_write(USR, bytes([USR_TXRDY | USR_TXEMP]))

    def _on_intc1_mask(self, uc, access, address, size, value, data):
        if size != 1 or value & 0x80:
            return
        if value & 0x40 or (value & 0x3F) == UART_SOURCE:   # 0x40: all sources
            self.uart_unmasked = address == INTC1_CIMR

    def _on_serq(self, uc, access, address, size, value, data):
        if size != 1 or value & 0x80:
            return
        if value & 0x40 or (value & 0x3F) == TX_CHAN:
            self._run_tx()

    def _run_tx(self):
        """Channel 37's whole major loop, now. -> bytes moved."""
        uc = self.m.uc
        base = _tcd(TX_CHAN)
        rd = lambda off, n: bytes(uc.mem_read(base + off, n))
        if struct.unpack('>I', rd(DADDR, 4))[0] != UDR:
            return 0                      # not the UART9 transmit set-up
        citer = struct.unpack('>H', rd(CITER, 2))[0] & 0x7FFF
        if not citer:
            return 0
        biter = struct.unpack('>H', rd(BITER, 2))[0] & 0x7FFF
        src = struct.unpack('>I', rd(SADDR, 4))[0]
        soff = struct.unpack('>h', rd(SOFF, 2))[0]
        nbytes = struct.unpack('>I', rd(NBYTES, 4))[0]
        slast = struct.unpack('>i', rd(SLAST, 4))[0]
        smod = (struct.unpack('>H', rd(ATTR, 2))[0] >> 11) & 0x1F
        mask = (1 << smod) - 1 if smod else 0
        ring = src & ~mask if mask else 0
        out = bytearray()
        for _ in range(citer * nbytes):
            out.append(uc.mem_read(src, 1)[0])
            src += soff
            if mask:
                src = ring | (src & mask)
        src += slast
        if mask:
            src = ring | (src & mask)
        uc.mem_write(base + SADDR, struct.pack('>I', src & 0xFFFFFFFF))
        uc.mem_write(base + CITER, struct.pack('>H', biter))
        csr = struct.unpack('>H', rd(CSR, 2))[0]
        # Major loop done: DONE set, ACTIVE clear. The UART9 ISR reads DONE
        # to decide whether a paused transfer has finished.
        uc.mem_write(base + CSR, struct.pack('>H', (csr | 0x80) & ~0x40))
        self._record(out, 'dma')
        if csr & 0x02:                    # INT_MAJOR
            self.tx_pending += 1
        return len(out)

    # -- input ----------------------------------------------------------

    def feed(self, data):
        """Bytes received on MIDI in: into channel 36's ring, then vector 156.
        -> False when the receive channel is not set up (nothing written)."""
        uc = self.m.uc
        base = _tcd(RX_CHAN)
        if struct.unpack('>I', bytes(uc.mem_read(base + SADDR, 4)))[0] != UDR:
            return False
        daddr = struct.unpack('>I', bytes(uc.mem_read(base + DADDR, 4)))[0]
        dmod = (struct.unpack('>H', bytes(uc.mem_read(base + ATTR, 2)))[0] >> 3) & 0x1F
        mask = (1 << dmod) - 1 if dmod else 0xFFFFFFFF
        ring = daddr & ~mask
        for byte in data:
            uc.mem_write(daddr, bytes([byte & 0xFF]))
            daddr = ring | ((daddr + 1) & mask)
        uc.mem_write(base + DADDR, struct.pack('>I', daddr))
        self.rx_bytes += len(data)
        self.rx_pending = True
        self.deliver()
        return True

    # -- interrupts -----------------------------------------------------

    def _try(self, vector):
        level = interrupt_level(self.m, vector, respect_mask=False)
        if level is None:
            return False
        ipl = (self.m.uc.reg_read(UC_M68K_REG_SR) >> 8) & 7
        if level <= ipl:
            return False
        if self.m.raise_vector(vector, level=level):
            self.taken[vector] += 1
            return True
        return False

    def deliver(self):
        """Raise at most one owed interrupt the IPL allows. -> bool."""
        if self.rx_pending and self._try(RX_VECTOR):
            self.rx_pending = False
            return True
        if self.tx_pending and self._try(TX_VECTOR):
            self.tx_pending -= 1
            return True
        if self.uart_unmasked and self._try(UART_VECTOR):
            return True
        return False

    # async event source protocol (emu.longrun.spin): no deadline of its own
    def step(self, done, remaining=None):
        return remaining

    def service(self, done):
        self.deliver()


def install(machine, events, clock=lambda: 0.0, on_byte=None):
    """-> the DinMidi model, also at events['din_midi']. Refuses (None) when
    the image's UART9 receive channel is not where 1.12 puts it."""
    rx_src = struct.unpack('>I', bytes(machine.uc.mem_read(_tcd(RX_CHAN) + SADDR, 4)))[0]
    tx_dst = struct.unpack('>I', bytes(machine.uc.mem_read(_tcd(TX_CHAN) + DADDR, 4)))[0]
    if rx_src != UDR or tx_dst != UDR:
        return None
    model = DinMidi(machine, clock=clock, on_byte=on_byte)
    events['din_midi'] = model
    resume_in_flight(machine, model)
    sem = receive_semaphore(machine)
    model.rx_semaphore = sem
    if sem is not None and 'unblock_skip' in events:
        # build(unblock=True) force-satisfies any pend whose semaphore has
        # no known poster. The MIDI-in task (FUN_40120a02 on 1.12) pends
        # this one and then polls three queues; once real input has woken
        # it, a faked post made it spin for ever (86% of the CPU, the UI
        # starved). With this model the semaphore has a real poster.
        events['unblock_skip'].add(sem)
    return model


RX_CALLBACK_PTR = 0x4058E5B8      # [ptr] = per-byte callback (1.12)
TX_STATE = 0x4058E578             # 0 idle, 1 DMA running, 2 PIO (1.12)


def resume_in_flight(machine, model):
    """A snapshot can be saved with a transfer the firmware believes is
    running (the model's own owed interrupts are not saved). Owe its
    completion again, or the driver waits for it for ever and MIDI out
    stops once the 4 KB ring is full. -> the state seen."""
    try:
        state = struct.unpack('>I', bytes(machine.uc.mem_read(TX_STATE, 4)))[0]
    except Exception:                             # noqa: BLE001
        return None
    if state == 1:
        model.tx_pending = max(model.tx_pending, 1)
    elif state == 2:
        model.uart_unmasked = True
    return state


def receive_semaphore(machine):
    """The semaphore the MIDI parser posts per message, or None.

    The receive callback (0x4011f3ac on 1.12) is `pea.l <parser>; jsr
    <parse>`, and the parser struct (0x445290c4) holds its output queue at
    +0x24 and the semaphore it posts at +0x28 (FUN_4011ee4a).
    """
    try:
        cb = struct.unpack('>I', bytes(machine.uc.mem_read(RX_CALLBACK_PTR, 4)))[0]
        op, parser = struct.unpack('>HI', bytes(machine.uc.mem_read(cb, 6)))
        if op != 0x4879:                          # pea.l abs.l
            return None
        sem = struct.unpack('>I', bytes(machine.uc.mem_read(parser + 0x28, 4)))[0]
    except Exception:                             # noqa: BLE001
        return None
    return sem or None


# --- MIDI bytes -> messages, and a Standard MIDI File ----------------------

def data_len(status):
    """Data bytes after a status byte (sysex: None, until F7)."""
    hi = status & 0xF0
    if hi in (0xC0, 0xD0):
        return 1
    if hi < 0xF0:
        return 2
    return {0xF1: 1, 0xF2: 2, 0xF3: 1}.get(status, 0 if status != 0xF0 else None)


class Parser:
    """MIDI bytes -> messages, one byte at a time, with running status.
    Realtime bytes (F8-FF) come out as their own messages wherever they
    occur. A message's time is its status (or first data) byte's."""

    def __init__(self):
        self.running = None
        self.cur = bytearray()
        self.t0 = 0.0
        self.need = 0
        self.sysex = None

    def feed(self, t, b):
        """-> a list of (t, message) completed by this byte (often empty)."""
        if b >= 0xF8:
            return [(t, bytes([b]))]
        if self.sysex is not None:
            self.sysex.append(b)
            if b == 0xF7:
                msg, self.sysex = bytes(self.sysex), None
                return [(self.t0, msg)]
            return []
        if b & 0x80:
            if b == 0xF0:
                self.sysex, self.t0 = bytearray([b]), t
                self.running = None
                self.cur = bytearray()
                return []
            n = data_len(b)
            self.running = b if b < 0xF0 else None
            if n == 0:
                self.cur = bytearray()
                return [(t, bytes([b]))]
            self.cur, self.t0, self.need = bytearray([b]), t, n
            return []
        if not self.cur:
            if self.running is None:
                return []                 # stray data byte
            self.cur, self.t0 = bytearray([self.running]), t
            self.need = data_len(self.running)
        self.cur.append(b)
        if len(self.cur) == self.need + 1:
            msg, self.cur = bytes(self.cur), bytearray()
            return [(self.t0, msg)]
        return []


def parse(timed_bytes):
    """[(t, byte)] -> [(t, bytes message)]; see Parser."""
    p = Parser()
    out = []
    for t, b in timed_bytes:
        out.extend(p.feed(t, b))
    return out


def describe(msg):
    """A short readable form of one MIDI message."""
    s = msg[0]
    names = {0xF8: 'clock', 0xFA: 'start', 0xFB: 'continue', 0xFC: 'stop',
             0xFE: 'active-sensing', 0xFF: 'reset'}
    if s in names:
        return names[s]
    hi, ch = s & 0xF0, (s & 0x0F) + 1
    if hi == 0x90 and len(msg) == 3:
        return ('note-on ch%d %d vel %d' if msg[2] else 'note-off ch%d %d (vel 0)') % (
            (ch, msg[1], msg[2]) if msg[2] else (ch, msg[1]))
    if hi == 0x80 and len(msg) == 3:
        return 'note-off ch%d %d vel %d' % (ch, msg[1], msg[2])
    if hi == 0xB0 and len(msg) == 3:
        return 'cc ch%d %d = %d' % (ch, msg[1], msg[2])
    if hi == 0xC0 and len(msg) == 2:
        return 'program ch%d %d' % (ch, msg[1])
    if hi == 0xE0 and len(msg) == 3:
        return 'pitchbend ch%d %d' % (ch, (msg[2] << 7 | msg[1]) - 8192)
    if s == 0xF2 and len(msg) == 3:
        return 'song-position %d' % (msg[2] << 7 | msg[1])
    return msg.hex()


def _vlq(n):
    out = [n & 0x7F]
    n >>= 7
    while n:
        out.append(0x80 | (n & 0x7F))
        n >>= 7
    return bytes(reversed(out))


def smf(messages, bpm=120.0, ppq=960, t0=None, realtime='markers'):
    """[(t, message)] -> a format-0 Standard MIDI File (bytes).

    Times are seconds; tick 0 is `t0` (default: the first message). The
    tempo is `bpm`, so bars line up in a sequencer when it is the project's
    tempo. A MIDI file cannot hold realtime or other system bytes as events:
    `realtime='markers'` (default) writes start/continue/stop as marker meta
    events and drops clock ticks, `'escape'` keeps every such byte as an F7
    escape event (valid, but some readers refuse it), `'drop'` drops them."""
    if t0 is None:
        t0 = messages[0][0] if messages else 0.0
    tick_s = 60.0 / bpm / ppq
    track = bytearray()
    track += b'\x00\xff\x51\x03' + int(round(60e6 / bpm)).to_bytes(3, 'big')
    last = 0
    for t, msg in messages:
        if msg[0] == 0xF0:
            body = b'\xf0' + _vlq(len(msg) - 1) + msg[1:]
        elif msg[0] >= 0xF0:
            if realtime == 'escape':
                body = b'\xf7' + _vlq(len(msg)) + msg
            elif realtime == 'markers' and msg[0] in (0xFA, 0xFB, 0xFC):
                text = describe(msg).encode()
                body = b'\xff\x06' + _vlq(len(text)) + text
            else:
                continue
        else:
            body = msg
        tick = max(last, int(round((t - t0) / tick_s)))
        track += _vlq(tick - last) + body
        last = tick
    track += b'\x00\xff\x2f\x00'
    header = b'MThd' + struct.pack('>IHHH', 6, 0, 1, ppq)
    return header + b'MTrk' + struct.pack('>I', len(track)) + bytes(track)


def write_log(path, log, t0=0.0):
    """The byte log as text: emulated time handed / on the wire, byte, path,
    then the parsed messages."""
    with open(path, 'w') as fh:
        fh.write('# DIN MIDI out (UART9), emulated seconds; t_wire at 31250 baud\n')
        fh.write('# t_handed     t_wire         byte  path\n')
        for t, w, b, p in log:
            fh.write('%12.6f  %12.6f   %02x   %s\n' % (t - t0, w - t0, b, p))
        fh.write('\n# messages (time of the first byte on the wire)\n')
        for t, msg in parse([(w, b) for _, w, b, _ in log]):
            fh.write('%12.6f  %-10s %s\n' % (t - t0, msg.hex(), describe(msg)))
