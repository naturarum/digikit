"""The C block budget (emu/nativehook.py) against the Python one, on real Unicorn."""
# pyright: reportMissingImports=false
import types

import pytest
from unicorn import UC_ARCH_M68K, UC_MODE_BIG_ENDIAN, Uc
from unicorn.m68k_const import UC_CPU_M68K_CFV4E, UC_M68K_REG_D0, UC_M68K_REG_PC

from emu import nativehook
from emu.longrun import _FastStepper

# moveq #0,d0 ; loop: addq.l #1,d0 ; bra.s loop
PROGRAM = bytes.fromhex('7000' '5280' '60fc')


def machine():
    uc = Uc(UC_ARCH_M68K, UC_MODE_BIG_ENDIAN)
    uc.ctl_set_cpu_model(UC_CPU_M68K_CFV4E)
    uc.mem_map(0x1000, 0x1000)
    uc.mem_write(0x1000, PROGRAM)
    return types.SimpleNamespace(uc=uc)


def run(native, steps):
    m = machine()
    st = _FastStepper(m, native=native)
    pc, out = 0x1000, []
    for step in steps:
        executed = st.run(pc, step)
        pc = m.uc.reg_read(UC_M68K_REG_PC)
        out.append((executed, st.blocks, st.left, m.uc.reg_read(UC_M68K_REG_D0), pc))
    return st, out


def test_native_matches_python():
    if nativehook._library() is None:
        pytest.skip('no C compiler for the native hook')
    st_n, native = run(True, (387, 1000, 50, 1))
    st_p, python = run(False, (387, 1000, 50, 1))
    assert st_n.native and not st_p.native
    assert native == python
    # the budget really bounded the loop: 100 blocks for the first step
    assert native[0][1] == 100


def test_blocks_and_left_are_writable():
    """emu.live's idle skip credits the rest of a step by writing them."""
    if nativehook._library() is None:
        pytest.skip('no C compiler for the native hook')
    st = _FastStepper(machine(), native=True)
    st.left, st.blocks = 7, 3
    st.blocks += st.left
    st.left = 0
    assert (st.blocks, st.left) == (10, 0)


def test_disabled_by_environment(monkeypatch):
    monkeypatch.setattr(nativehook, '_TRIED', False)
    monkeypatch.setattr(nativehook, '_LIB', None)
    monkeypatch.setenv('DIGIKIT_NATIVE_HOOKS', '0')
    assert nativehook.block_counter(machine().uc) is None
    monkeypatch.setattr(nativehook, '_TRIED', False)
