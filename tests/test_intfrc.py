"""Forced-interrupt model and ISA_C reversal helpers; no firmware image required."""

# pyright: reportMissingImports=false

import struct
import unittest

from unicorn.m68k_const import UC_M68K_REG_SR

from emu.harness import _bitrev32, _byterev32
from emu.intfrc import ForcedInterrupts


class FakeUc:
    def __init__(self):
        self.memory = {}
        self.regs = {UC_M68K_REG_SR: 0x2000}

    def hook_add(self, *args, **kwargs):
        return 1

    def mem_read(self, address, size):
        return bytes(self.memory.get(address + i, 0) for i in range(size))

    def mem_write(self, address, data):
        for i, value in enumerate(data):
            self.memory[address + i] = value

    def reg_read(self, register):
        return self.regs.get(register, 0)


class FakeMachine:
    def __init__(self):
        self.uc = FakeUc()
        self.after_rte = []
        self.raised = []

    def raise_vector(self, vector, level=None):
        self.raised.append((vector, level))
        return True


def force(m, controller_high, bit):
    current = struct.unpack('>I', m.uc.mem_read(controller_high, 4))[0]
    m.uc.mem_write(controller_high, struct.pack('>I', current | (1 << bit)))


class ForcedInterruptTests(unittest.TestCase):
    def setUp(self):
        self.m = FakeMachine()
        self.f = ForcedInterrupts(self.m, skip=(191,))
        # ICR0_44 = 5, ICR0_57 = 2, ICR1_63 = 5
        self.m.uc.mem_write(0xFC048040 + 44, b'\x05')
        self.m.uc.mem_write(0xFC048040 + 57, b'\x02')
        self.m.uc.mem_write(0xFC04C040 + 63, b'\x05')

    def test_registers_after_rte(self):
        self.assertEqual(self.m.after_rte, [self.f.deliver])

    def test_nothing_forced(self):
        self.assertFalse(self.f.deliver())

    def test_intfrch0_source_44_is_vector_108(self):
        force(self.m, 0xFC048010, 12)
        self.assertTrue(self.f.deliver())
        self.assertEqual(self.m.raised, [(108, 5)])

    def test_highest_level_first(self):
        force(self.m, 0xFC048010, 12)
        force(self.m, 0xFC048010, 25)
        self.f.deliver()
        self.assertEqual(self.m.raised[0], (108, 5))

    def test_ipl_blocks_lower_levels(self):
        force(self.m, 0xFC048010, 25)          # level 2
        self.m.uc.regs[UC_M68K_REG_SR] = 0x2200
        self.assertFalse(self.f.deliver())
        self.m.uc.regs[UC_M68K_REG_SR] = 0x2100
        self.assertTrue(self.f.deliver())
        self.assertEqual(self.m.raised, [(121, 2)])

    def test_skipped_vector_left_alone(self):
        force(self.m, 0xFC04C010, 31)          # source 63, vector 191
        self.assertFalse(self.f.deliver())

    def test_disabled_icr_not_taken(self):
        force(self.m, 0xFC048014, 3)           # source 3, ICR 0
        self.assertFalse(self.f.deliver())


class ReversalTests(unittest.TestCase):
    def test_bitrev(self):
        self.assertEqual(_bitrev32(1), 0x80000000)
        self.assertEqual(_bitrev32(0x0000000F), 0xF0000000)
        self.assertEqual(_bitrev32(0x12345678), 0x1E6A2C48)

    def test_byterev(self):
        self.assertEqual(_byterev32(0x11223344), 0x44332211)


if __name__ == '__main__':
    unittest.main()
