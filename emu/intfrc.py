"""Software-forced interrupts (INTC0/INTC1 INTFRCH/INTFRCL), opt-in.

The MCF5441x interrupt controllers have force registers: setting bit n of
INTFRCL (sources 0-31) or INTFRCH (sources 32-63) asserts that source until
software clears the bit, regardless of the mask registers. The source is
level-sensitive: it is taken whenever its ICR level is above the CPU's IPL,
so a handler that leaves the bit set re-enters as soon as it returns.

Digitone II 1.12 runs its sequencer on forced sources. The frame handler
(vector 191) sets INTFRCH0 bit 12 at 0x40027afc: source 44, vector 108, the
sequencer clock at 0x400d79c8, which clears it at 0x400d79da and sets
INTFRCH0 bit 25 at 0x400d7d66: source 57, vector 121, the step handler
0x400d99de (El-Fw notes/T2.2-arp-midi-v2.md). Without this model those
writes land in plain memory and PLAY never steps.

INTFRCH1 bit 31 (source 63, vector 191) is left to emu/ssi.py when that
model is installed (`skip`), so the frame handler is not delivered twice.

Delivery happens at two points: every spin() step boundary (this object is
an async event source with no deadline of its own) and right after every
`rte` (Machine.after_rte), which is when the IPL drops and a source raised
inside a handler becomes deliverable on hardware.
"""

# pyright: reportMissingImports=false
from __future__ import annotations

import struct

from unicorn.m68k_const import UC_M68K_REG_SR

from emu.pit import interrupt_level

# (INTFRCH, INTFRCL, first vector) per controller
CONTROLLERS = ((0xFC048010, 0xFC048014, 64), (0xFC04C010, 0xFC04C014, 128))


class ForcedInterrupts:
    def __init__(self, machine, skip=()):
        self.m = machine
        self.skip = set(skip)              # vectors handled elsewhere
        self.taken = {}                    # vector -> deliveries
        # The guest's writes land in plain memory; delivery reads them back.
        machine.after_rte.append(self.deliver)

    def pending(self):
        """-> [(level, vector)] of asserted, unmasked-by-ICR sources."""
        out = []
        uc = self.m.uc
        for high, low, first in CONTROLLERS:
            h, l = struct.unpack('>II', bytes(uc.mem_read(high, 8)))
            bits = (h << 32) | l
            while bits:
                src = bits.bit_length() - 1
                bits &= ~(1 << src)
                vec = first + src
                if vec in self.skip:
                    continue
                level = interrupt_level(self.m, vec, respect_mask=False)
                if level:
                    out.append((level, vec))
        return out

    def deliver(self):
        """Take the highest-level forced source the IPL allows. -> bool."""
        sources = self.pending()
        if not sources:
            return False
        level, vec = max(sources)
        ipl = (self.m.uc.reg_read(UC_M68K_REG_SR) >> 8) & 7
        if level <= ipl and level != 7:
            return False
        if self.m.raise_vector(vec, level=level):
            self.taken[vec] = self.taken.get(vec, 0) + 1
            return True
        return False

    # async event source protocol (emu.longrun.spin)
    def step(self, done, remaining=None):
        return remaining

    def service(self, done):
        self.deliver()


def install(machine, events, skip=()):
    source = ForcedInterrupts(machine, skip=skip)
    events['intfrc'] = source
    return source
