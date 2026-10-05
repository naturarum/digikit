# pyright: reportMissingImports=false
"""Run the Digitone II OS 1.12 bootstrap from its reset entry under Unicorn and
show whether the STARTUP menu (FUNC held at power-on) is reached, for several
flash contents.  Task T1.5 (recovery path).  See notes/T1.5-recovery-path.md.

    uv run python tools/recovery_harness.py --stock STOCK.syx [--json OUT] [--art DIR]

The code that runs is the stock bootstrap (section 2) of the unit.  The flash is
a model we assemble: an all-0xFF 16 MiB NOR with a container at 0x80000 and a
plausible serial record at 0x3c0000.  A scripted panel answers the bootstrap's
power-on key query over its UART8 link.  Nothing talks to hardware.

Why from the reset entry: the STARTUP-menu decision and the menu itself live
INSIDE the bootstrap's init routine FUN_800009c6 (called from the reset path at
0x80000ee0), so the only way to exercise the real decision is to run the reset
path.  What we cannot faithfully model under Unicorn is the panel MCU on the far
side of UART8 and the asynchronous eDMA/ISR that fills the UART8 receive ring,
and the ColdFire DDR/clock/interrupt-controller bring-up that spins on hardware
status bits.  The recovery decision does not depend on the wire protocol or the
clock tree; it depends on the *answer* the panel gives (which keys, or a
timeout) and on what the flash holds.  So we stub the hardware bring-up and the
panel at the boundary of the bootstrap's own routines (their documented
contract), check the key result against the real parser's bitmap layout, and
let every decision, every flash read and the menu code itself run for real.
Every stub is listed in STUBS below so the report states exactly what was
replaced and what ran unmodified.
"""
import argparse
import hashlib
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from unicorn import UC_HOOK_CODE, UcError                         # noqa: E402
from unicorn.m68k_const import (UC_M68K_REG_A7, UC_M68K_REG_D0,   # noqa: E402
                                UC_M68K_REG_PC, UC_M68K_REG_SR)

from emu.harness import PAGE, Machine                             # noqa: E402
from dt2.container import decode_syx                              # noqa: E402

BOOT_IMAGE = None               # set in main(): path to the stock bootstrap binary (section 2)
BOOT_IMG = 0x800003FC           # section 2 file offset 0 (a 4-byte header precedes the code)
RESET = 0x80000de8              # reset entry
STACK_TOP = 0x1000FE00          # SSP; away from SRAM code and the 0x8000xxxx data it uses
NOR_SIZE = 0x1000000            # 16 MiB
CONTAINER_AT = 0x80000
SERIAL_AT = 0x3c0000
LIMIT = 60_000_000              # a healthy run reaches the menu well inside this

# --- bootstrap addresses (verified against the bytes; see notes/T1.5) ---------
A = {
    'reset':        0x80000de8,
    'init':         0x800009c6,  # clocks, panel, self-upgrade, STARTUP-menu decision
    'wdebug':       0x80000eb8,  # WDEBUG in the reset path (Unicorn lacks it); 6 bytes
    'boot_flag_t':  0x80000ee4,  # moveq #64,d0; and.l flags,d0  (bit 6 -> run section 4)
    'run_sec4':     0x8000f100,  # SRAM copy of "run section 4"  (alias of 0x80007a00)
    'loader':       0x80000596,  # the normal main-OS loader
    'menu':         0x80002dc4,  # the STARTUP menu
    'menu_keypoll': 0x800013ac,  # the menu's / receive path's panel key-event reader (live loop)
    'os_upgrade':   0x80003e5a,  # STARTUP-menu "4 OS UPGRADE": READY TO RECEIVE / RECEIVING...
    'commit':       0x80003c9c,  # verify-and-commit (checksum + HMAC, then flash erase)
    'abort':        0x80003bfc,  # UPGRADE ABORTED / PLEASE REBOOT (spin)
    'drawflush':    0x80001ab4,  # push the 1024-byte panel buffer out over UART8
    'selfupg':      0x80001c48,  # bootstrap self-upgrade
    'halt_dram':    0x80000d7a,  # halt if the DRAM-OK flag is wrong (hardware gate)
}
# SRAM-copied flash read: the reset path copies 0x80007900.. to 0x8000F000, so the
# real call target is 0x8000F010; its source 0x80007910 is hooked too in case.
FLASH_READ_SRAM = 0x8000F010
FLASH_READ_SRC = 0x80007910

# data the bootstrap keeps in SRAM
BITMAP = 0x80008414             # 16-byte panel key bitmap (FUN_80001276 fills; FUN_80001302 reads)
BOOTFLAGS = 0x80008404          # boot flags word
DRAM_OK = 0x800083FC            # set by DDR training; reset path halts on the wrong value
RUNNING_VER = 0x80000408        # running bootstrap version word (0x0201), compared by self-upgrade

# FUNC: keyid 0x11 -> bit index 0x10 (table at 0x80006dc0) -> bitmap[2] bit 0.
FUNC_KEYID = 0x11
FUNC_BITMAP_BYTE = 2
FUNC_BITMAP_BIT = 0

# heavy hardware bring-up and host-side panel routines, replaced by a stub that
# returns to the caller (caller cleans its own pushed args, as on real hardware).
STUB_RET = {
    0x800057a4: 'interrupt-controller / autovector setup (real INTC regs)',
    0x8000240a: 'misc peripheral config',
    0x800006ac: 'DDR controller bring-up (spins on training status); sets DRAM-OK=0 so the '
                'reset-path DRAM gate at 0x80000d7a is not taken',
    0x800008cc: 'UART8 (panel link) controller init + RX-ISR install',
    0x80000962: 'UART9 (MIDI) controller init',
    0x80005336: 'panel power/GPIO sequencing (returns 1)',
    0x80002742: 'DYB sector protect (the boot-time lock of sectors 0 and 0x40000)',
    0x8000065a: 'busy-wait delay (self-upgrade and timeout paths spin millions here)',
    0x800010e6: 'drain the panel link',
    0x800015bc: '5x panel detect / handshake',
    0x80000ff8: 'panel detect',
    0x80005884: 'read a GPIO strap (0xec094019 & 8) -> 0',
}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def lit_count(buf):
    """Number of set bits in the 1024-byte panel buffer (0 = blank screen)."""
    if not buf:
        return 0
    return sum(bin(x).count('1') for x in buf)


def render(buf):
    """Render the bootstrap's 1024-byte panel buffer as 64 rows x 128 cols of
    '#'/' ', using the exact addressing of the flush routine FUN_80001ab4:

      the buffer is 256 big-endian longs L[0..255]; for panel column C (0..127)
      and page P (0..7), with klo = C>>3, j = C&7, k = klo + (16 if P>=4 else 0),
      the source long is L[k*8 + j] and the page byte is (long >> (24-8*(P&3)))&0xff;
      pixel (C, row=P*8+b) is on iff bit b of that byte is set.
    """
    if not buf or len(buf) < 1024:
        return '(no frame)'
    L = struct.unpack('>256I', buf[:1024])
    rows = []
    for P in range(8):
        for b in range(8):
            row = []
            for C in range(128):
                klo, j = C >> 3, C & 7
                k = klo + (16 if P >= 4 else 0)
                val = (L[k * 8 + j] >> (24 - 8 * (P & 3))) & 0xFF
                row.append('#' if (val >> b) & 1 else ' ')
            rows.append(''.join(row))
    return '\n'.join(rows)


def parse(syx):
    stream = decode_syx(syx)
    size = struct.unpack_from('>I', stream, 0)[0]
    cont = stream[8:8 + size]
    if cont[:4] != b'ELE3':
        raise SystemExit('%s: no ELE3 container' % syx)
    n = struct.unpack_from('>I', cont, 0x1C)[0]
    secs = {}
    for k in range(n):
        sid, off, ln, dest = struct.unpack_from('>IIII', cont, 0x20 + 16 * k)
        secs[sid] = (off, ln, dest)
    return cont, secs


# --------------------------------------------------------------------------
# flash model
# --------------------------------------------------------------------------

def serial_record():
    """A plausible 0x16-byte serial record: "SERI" tag + payload + CRC-32 whose
    residue matches FUN_80001bd0 (0xDEBB20E3).  Only the TEST-MODE path reads it;
    it is here so that path does not see garbage, and is never read on the FUNC
    path."""
    import binascii
    body = b'SERI' + b'400062  ' + b'\x00\x00'   # 14 bytes: tag + 12 payload
    crc = binascii.crc32(body) & 0xFFFFFFFF       # zlib crc32: ~residue over body
    return body + struct.pack('<I', crc)          # 18 bytes; reader takes 0x16, rest 0xFF


def build_flash(container):
    """16 MiB erased NOR with the container at 0x80000 and a serial record at
    0x3c0000.  `container` may be shorter than its header claims (truncated) or
    empty (all-0xFF)."""
    nor = bytearray(b'\xff' * NOR_SIZE)
    if container:
        nor[CONTAINER_AT:CONTAINER_AT + len(container)] = container
    rec = serial_record()
    nor[SERIAL_AT:SERIAL_AT + len(rec)] = rec
    return bytes(nor)


def corrupt_main_os(cont, secs):
    """Flip bytes throughout section 3's stored stream, leaving the ELE3 header
    and the section table intact (a resealed-but-wrong or bit-rotted main OS)."""
    b = bytearray(cont)
    off, ln, _ = secs[3]
    for i in range(off, off + ln, 1021):          # scatter flips across the stream
        b[i] ^= 0xA5
    return bytes(b)


def truncate(cont, keep):
    """Keep only the first `keep` bytes of the container (a power cut partway
    through an upgrade write)."""
    return cont[:keep]


def hostile_sec2(cont):
    """A validly-laid-out container whose section-2 (bootstrap) table entry
    claims a version higher than the running 0x0201.  This is what drives the
    boot-time self-upgrade FUN_80001c48 to consider re-flashing the bootstrap.
    The ELE3 table is 16-byte entries from offset 0x20: {id, offset, length,
    load_address}; for section 2 load_address = 0x02010000 (version 0x0201 in
    the high word).  Bump it to 0x02020000."""
    b = bytearray(cont)
    n = struct.unpack_from('>I', b, 0x1C)[0]
    for k in range(n):
        base = 0x20 + 16 * k
        if struct.unpack_from('>I', b, base)[0] == 2:
            struct.pack_into('>I', b, base + 12, 0x02020000)
            break
    return bytes(b)


# --------------------------------------------------------------------------
# machine + stubs
# --------------------------------------------------------------------------

class Panel:
    """A scripted answer to the bootstrap's power-on panel queries.

    mode 'func'   : FUNC held, panel healthy.
    mode 'none'   : panel healthy, no key held.
    mode 'silent' : panel never answers (key read and version both time out).
    """
    def __init__(self, mode):
        self.mode = mode


RECV_SPIN = 0x80003ede          # FUN_80003e5a: `while (DAT_80008f40 == 0)` after READY TO RECEIVE


def run(container, panel, want=('menu',), limit=LIMIT, trace=None, art_sink=None,
        osupgrade=False):
    """Run the reset path with `container` in flash and `panel` answering.
    -> a dict describing where it ended and which landmarks it passed."""
    boot = open(BOOT_IMAGE, 'rb').read()
    flash = build_flash(container)

    m = Machine()
    # SRAM for the bootstrap (0x8000_0000..) and its data, DDR window touched by
    # the RAM test, and the stack.
    for base in range(0x80000000, 0x80010000, PAGE):
        m.ensure(base)
    m.ensure(STACK_TOP)
    m.ensure(0x48000000)                       # soft-reset magic cell
    m.uc.mem_write(BOOT_IMG, boot)
    # Pre-fill the SRAM-copied routines so a hook on 0x8000F010/0x8000F100 catches
    # the real call even before the reset path's copy loop runs.
    m.uc.mem_write(0x8000F000, boot[0x80007900 - BOOT_IMG:0x80007A56 - BOOT_IMG])
    # WDEBUG at 0x80000eb8 (fbef 0003 fff8) is a ColdFire debug-register load that
    # Unicorn's m68k core cannot translate; QEMU aborts the whole basic block
    # that contains it before any code hook can fire, so it must be neutralised
    # in memory, not skipped from a hook.  Three NOPs, same length (6 bytes).
    m.uc.mem_write(A['wdebug'], b'\x4e\x71\x4e\x71\x4e\x71')
    m.install_isa_patches_scoped(boot, BOOT_IMG)
    # Status bits the reset path polls on real hardware before init.  The clock
    # tree and DDR are not modelled; forcing the "ready" bits lets the polls end.
    # install_mmio forces the 32-bit big-endian value at the address on any read
    # in [addr, addr+3]; a byte read returns the top byte (value >> 24).
    m.mmio = {
        0xFC0C0008: 0x00000010,   # CCM status, LONG read, PLL-lock bit 4 (reset path 0x80000e36)
        0xEC070004: 0x04000000,   # UART8 USR, BYTE read, TX-ready bit 2 (draw flush 0x80000fda)
        0xEC074004: 0x0C000000,   # UART9 USR, BYTE read, TX bits (MIDI path only)
    }
    m.install_mmio(scoped=True)

    st = {'n': 0, 'flash_reads': [], 'landmarks': [], 'order': [],
          'keyread': 0, 'verread': 0, 'stop': None, 'stop_pc': None,
          'redirected': False, 'reads_after_osupgrade': []}

    # ---- flash read HLE: read(off, len, dest) ----------------------------
    def flash_read(uc, addr, size, data):
        sp = uc.reg_read(UC_M68K_REG_A7)
        ret, off, ln, dest = struct.unpack('>IIII', uc.mem_read(sp, 16))
        chunk = flash[off:off + ln]
        chunk += b'\xff' * (ln - len(chunk))
        for p in range(dest & ~(PAGE - 1), dest + ln + 1, PAGE):
            m.ensure(p)
        uc.mem_write(dest, chunk)
        st['flash_reads'].append(('%#x' % off, ln, '%#x' % dest))
        if st['redirected']:
            st['reads_after_osupgrade'].append(('%#x' % off, ln))
        uc.reg_write(UC_M68K_REG_A7, sp + 4)   # pop only the return addr
        uc.reg_write(UC_M68K_REG_PC, ret)
    m.uc.hook_add(UC_HOOK_CODE, flash_read, begin=FLASH_READ_SRAM, end=FLASH_READ_SRAM)
    m.uc.hook_add(UC_HOOK_CODE, flash_read, begin=FLASH_READ_SRC, end=FLASH_READ_SRC)

    # ---- generic "return to caller" stub ---------------------------------
    def make_ret(d0=0):
        def h(uc, addr, size, data):
            sp = uc.reg_read(UC_M68K_REG_A7)
            ret = struct.unpack('>I', uc.mem_read(sp, 4))[0]
            uc.reg_write(UC_M68K_REG_A7, sp + 4)
            uc.reg_write(UC_M68K_REG_D0, d0 & 0xFFFFFFFF)
            uc.reg_write(UC_M68K_REG_PC, ret)
        return h
    for addr, _why in STUB_RET.items():
        d0 = 1 if addr == 0x80005336 else 0
        m.uc.hook_add(UC_HOOK_CODE, make_ret(d0), begin=addr, end=addr)

    # DRAM-OK flag: DDR stub ran, so set the flag the reset path's gate wants
    # (0 -> the halt at 0x80000d7a is skipped).  Also clear the self-upgrade
    # running-version source nothing else writes under emulation.
    def after_dram(uc, addr, size, data):
        uc.mem_write(DRAM_OK, b'\x00\x00\x00\x00')
    m.uc.hook_add(UC_HOOK_CODE, after_dram, begin=0x800006ac, end=0x800006ac)

    # ---- panel version FUN_80001518(param_1, *timeout_flag), a0 = *version --
    from unicorn.m68k_const import UC_M68K_REG_A0
    def panel_version(uc, addr, size, data):
        st['verread'] += 1
        sp = uc.reg_read(UC_M68K_REG_A7)
        ret, _p1, pflag = struct.unpack('>III', uc.mem_read(sp, 12))
        a0 = uc.reg_read(UC_M68K_REG_A0)
        if panel.mode == 'silent':
            uc.mem_write(pflag, b'\x01')        # timeout
        else:
            uc.mem_write(pflag, b'\x00')
            uc.mem_write(a0, b'\x15\x15\x15\x15')  # version >= 0x15, skips 0x800015bc
        uc.reg_write(UC_M68K_REG_A7, sp + 4)
        uc.reg_write(UC_M68K_REG_PC, ret)
    m.uc.hook_add(UC_HOOK_CODE, panel_version, begin=0x80001518, end=0x80001518)

    # ---- panel key read FUN_80001276 -> fill the bitmap, return 0 / -1 ----
    def keyread(uc, addr, size, data):
        st['keyread'] += 1
        sp = uc.reg_read(UC_M68K_REG_A7)
        ret = struct.unpack('>I', uc.mem_read(sp, 4))[0]
        uc.mem_write(BITMAP, b'\x00' * 16)
        if panel.mode == 'silent':
            d0 = 0xFFFFFFFF                     # the real -1 timeout result
        else:
            d0 = 0
            if panel.mode == 'func':
                b = bytearray(uc.mem_read(BITMAP + FUNC_BITMAP_BYTE, 1))
                b[0] |= (1 << FUNC_BITMAP_BIT)
                uc.mem_write(BITMAP + FUNC_BITMAP_BYTE, bytes(b))
        uc.reg_write(UC_M68K_REG_A7, sp + 4)
        uc.reg_write(UC_M68K_REG_D0, d0 & 0xFFFFFFFF)
        uc.reg_write(UC_M68K_REG_PC, ret)
    m.uc.hook_add(UC_HOOK_CODE, keyread, begin=0x80001276, end=0x80001276)

    # ---- landmark + stop hooks -------------------------------------------
    want = set(want)
    def landmark(label, stop):
        def h(uc, addr, size, data):
            if label not in st['landmarks']:
                st['landmarks'].append(label)
                st['order'].append((label, st['n']))
            # osupgrade: when the menu's input loop is reached, redirect into the
            # menu's "4 OS UPGRADE" branch FUN_80003e5a (the key navigation is the
            # only step stubbed; the receive setup, the READY TO RECEIVE draw and
            # the MIDI-driven wait all run unmodified) and watch what it reads.
            if label == 'menu_keypoll' and osupgrade and not st['redirected']:
                st['redirected'] = True
                uc.reg_write(UC_M68K_REG_PC, A['os_upgrade'])
                return
            if stop:
                st['stop'] = label
                st['stop_pc'] = '%#x' % addr
                uc.emu_stop()
        return h
    # Recorded-only landmarks pass through; the run stops at the first outcome.
    # The menu is recorded on entry but the run keeps going into its draw
    # (drawflush) and its live input loop (menu_keypoll), so the stop proves the
    # menu actually drew a screen and is waiting for keys, not merely that the
    # entry was branched to.
    LM = {
        'selfupg':     (A['selfupg'],     False),
        'drawflush':   (A['drawflush'],   False),
        'menu':        (A['menu'],        False),
        'menu_keypoll':(A['menu_keypoll'],'menu_keypoll' in want),
        'os_upgrade':  (A['os_upgrade'],  'os_upgrade' in want),
        'commit':      (A['commit'],      'commit' in want),
        'abort':       (A['abort'],       True),
        'loader':      (A['loader'],      'loader' in want),
        'run_sec4':    (A['run_sec4'],    'run_sec4' in want),
        'halt_dram':   (A['halt_dram'],   True),
        # a bootstrap-region flash write -- only reached if the self-upgrade
        # decides to re-flash 0x40000 (brick-relevant); not reached on any
        # recovery path, so hitting it stops the run as a red flag.
        'flash_erase':   (0x80002896,     True),
        'flash_program': (0x8000282a,     True),
    }
    if osupgrade:
        LM['menu_keypoll'] = (A['menu_keypoll'], False)   # redirect, do not stop
        LM['recv_spin'] = (RECV_SPIN, True)               # stop at the MIDI-wait loop
    for label, (addr, stop) in LM.items():
        m.uc.hook_add(UC_HOOK_CODE, landmark(label, stop), begin=addr, end=addr)

    if os.environ.get('RH_PCTRACE'):
        _fd = os.open(os.environ['RH_PCTRACE'], os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)

        def pctr(uc, addr, size, data):
            os.write(_fd, b'%08x\n' % addr)
        m.uc.hook_add(UC_HOOK_CODE, pctr)
    if trace is not None:
        def tr(uc, addr, size, data):
            st['n'] += 1
            if st['n'] <= trace:
                sys.stderr.write('%#x\n' % addr)
        m.uc.hook_add(UC_HOOK_CODE, tr)
    else:
        def count(uc, addr, size, data):
            st['n'] += 1
        # cheap counter only at the reset entry page boundary -> use a light hook
        m.uc.hook_add(UC_HOOK_CODE, count, begin=RESET, end=RESET)

    # d2 = 0 (normal power-on reset entry); SSP set; supervisor, ints masked.
    m.uc.mem_write(STACK_TOP, struct.pack('>III', 0, 0, 0))
    m.uc.reg_write(UC_M68K_REG_SR, 0x2700)
    m.uc.reg_write(UC_M68K_REG_A7, STACK_TOP)
    try:
        m.uc.emu_start(RESET, 0xFFFFFFFE, count=limit)
        if st['stop'] is None:
            st['stop'] = 'ran-to-limit'
            st['stop_pc'] = '%#x' % (m.uc.reg_read(UC_M68K_REG_PC) & 0xFFFFFFFF)
    except UcError as e:
        if st['stop'] is None:
            st['stop'] = 'error: %s' % e
            st['stop_pc'] = '%#x' % (m.uc.reg_read(UC_M68K_REG_PC) & 0xFFFFFFFF)
    st['boot_flags'] = '%#x' % struct.unpack('>I', m.uc.mem_read(BOOTFLAGS, 4))[0]
    # The bootstrap composes into the 1024-byte buffer at 0x80008624 and, after
    # FUN_80001ab4 flushes it, mirrors it to 0x80008a2c (the last frame actually
    # sent to the panel).  Grab both at the stop point.
    try:
        st['panel_compose'] = bytes(m.uc.mem_read(0x80008624, 1024))
        st['panel_sent'] = bytes(m.uc.mem_read(0x80008a2c, 1024))
    except UcError:
        st['panel_compose'] = st['panel_sent'] = None
    st['_m'] = m
    return st


# --------------------------------------------------------------------------
# scenarios + CLI
# --------------------------------------------------------------------------

def containers(syx):
    """Return the four flash-content cases as {name: container-bytes}."""
    cont, secs = parse(syx)
    return {
        'stock':        cont,
        'corrupt-main': corrupt_main_os(cont, secs),
        'all-0xFF':     b'',                              # nothing at 0x80000
        'truncated':    truncate(cont, 0x40000),          # header + partial table + partial sec3
        'hostile-sec2': hostile_sec2(cont),               # section-2 table version bumped to 0x0202
    }, secs


PANELS = ('func', 'none', 'silent')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stock', required=True, help='stock Digitone II OS 1.12 .syx')
    ap.add_argument('--boot', help='the unpacked bootstrap (section 2) binary; '
                    'default ele3_out/01_type2.unpacked.bin beside the .syx')
    ap.add_argument('--json', help='write the full result matrix here')
    ap.add_argument('--art', help='directory for ASCII-art panel dumps of the menu')
    ap.add_argument('--limit', type=int, default=LIMIT)
    ap.add_argument('--case', help='run only this flash case')
    ap.add_argument('--panel', help='run only this panel mode')
    args = ap.parse_args(argv)

    global BOOT_IMAGE
    here = os.path.dirname(os.path.abspath(args.stock))
    BOOT_IMAGE = args.boot or os.path.join(
        os.path.dirname(here) if os.path.basename(here) == '' else here,
        'ele3_out', '01_type2.unpacked.bin')
    if not os.path.exists(BOOT_IMAGE):
        # fall back: alongside repo root two levels up from a .syx in El-Fw
        alt = os.path.join(os.path.dirname(here), 'ele3_out', '01_type2.unpacked.bin')
        BOOT_IMAGE = alt if os.path.exists(alt) else BOOT_IMAGE
    boot = open(BOOT_IMAGE, 'rb').read()
    print('bootstrap %s  sha256 %s  (%d bytes)' % (BOOT_IMAGE, sha(boot)[:16], len(boot)))

    conts, secs = containers(args.stock)
    print('stock container %d bytes; section 3 stored off=%#x len=%d'
          % (len(conts['stock']), secs[3][0], secs[3][1]))

    want = ('menu_keypoll', 'os_upgrade', 'loader', 'run_sec4')
    matrix = {}
    cases = [args.case] if args.case else list(conts)
    panels = [args.panel] if args.panel else list(PANELS)

    for cname in cases:
        for pmode in panels:
            st = run(conts[cname], Panel(pmode), want=want, limit=args.limit)
            key = '%s / %s' % (cname, pmode)
            matrix[key] = {
                'stop': st['stop'], 'stop_pc': st['stop_pc'],
                'reached_menu': 'menu' in st['landmarks'],
                'reached_keypoll': 'menu_keypoll' in st['landmarks'],
                'reached_os_upgrade': 'os_upgrade' in st['landmarks'],
                'landmarks': st['landmarks'],
                'order': [(l, n) for l, n in st['order']],
                'keyreads': st['keyread'], 'versionreads': st['verread'],
                'boot_flags': st['boot_flags'],
                'flash_reads': st['flash_reads'][:12],
                'panel_lit_compose': lit_count(st.get('panel_compose')),
                'panel_lit_sent': lit_count(st.get('panel_sent')),
            }
            print('\n=== %-22s' % key)
            print('   stop: %s at %s   boot_flags=%s' % (st['stop'], st['stop_pc'], st['boot_flags']))
            print('   landmarks: %s' % ' > '.join(st['landmarks']))
            print('   flash reads: %s' % (', '.join('%s:%d' % (o, l) for o, l, d in st['flash_reads'][:8]) or 'none'))
            print('   panel lit pixels: compose=%d sent=%d'
                  % (lit_count(st.get('panel_compose')), lit_count(st.get('panel_sent'))))
            if args.art and st.get('panel_compose'):
                os.makedirs(args.art, exist_ok=True)
                p = os.path.join(args.art, '%s_%s.txt' % (cname, pmode))
                with open(p, 'w') as fh:
                    fh.write('compose buffer @0x80008624 (%d lit):\n' % lit_count(st['panel_compose']))
                    fh.write(render(st['panel_compose']) + '\n')
                print('   art: %s' % p)

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(matrix, fh, indent=2)
        print('\nwrote %s' % args.json)
    return matrix


if __name__ == '__main__':
    main()
