# pyright: reportMissingImports=false
"""Run the unit's own unpack and acceptance code on .syx files, under Unicorn.

    uv run python tools/acceptoracle.py --stock STOCK.syx [--json OUT] SYX [SYX...]

Written for Digitone II OS 1.12. Every address is found by signature in the
stock file's own bootstrap (section 2) and main OS (section 3), and printed
with how it was found, so the same run tells you when a build moved them.
emu/oracle.py does part of this with fixed 1.15C addresses.

The code that runs is always the STOCK file's: the unit validates and unpacks
an incoming file with the bootstrap and main OS it already has. Each
candidate .syx is only data.

What runs, per candidate:

  boot      The bootstrap's boot path from just after its init and STARTUP
            menu routine returns, through the loader (section lookup, the
            flash read of section 3 to the stage, the depacker writing over
            the load address), up to the first instruction it executes in
            the main OS. Flash is the candidate's container at flash_at, with
            the SPI NOR read served at the routine's entry. The depacked
            image is compared with --want-main (or the stock main OS).
  depack    The bootstrap's depacker on each packed section, in the layout
            its own callers use: section 3 staged at the loader's stage and
            unpacked to its section-table address (in place); section 2 at
            the self-upgrade layout, followed by the self-upgrade's CRC-32
            residue check. Section 7 with the main OS's copy of the
            depacker, in the layout of the main OS's DSP loader (output at
            buffer + 1). Section 8 only as a codec check (no ColdFire code
            unpacks it on the unit).
  bootcheck The bootstrap's receive-side verify-and-commit routine from its
            entry, with the decoded stream where the receive path puts it.
            It never returns: it either calls the abort screen (fail) or
            falls through to UPGRADING and the flash erase (pass). The run
            stops at either point, so no flash code runs.
  hmac      The bootstrap's trailer check, called on its own.
  osvalid   The main OS's upgrade validator (the routine its SysEx, file and
            flash-writer paths all call), and its two callees on their own.

Negative controls (in memory only, never written to disk) are run on the
first candidate: one trailer bit flipped; the same with the preamble checksum
redone; one bit flipped in the main OS stream with checksum and seal redone.
"""
import argparse
import hashlib
import hmac as hmaclib
import json
import os
import re
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ, UcError  # noqa: E402
from unicorn.m68k_const import (UC_M68K_REG_A1, UC_M68K_REG_A6, UC_M68K_REG_A7,  # noqa: E402
                                UC_M68K_REG_D0, UC_M68K_REG_D1,
                                UC_M68K_REG_PC, UC_M68K_REG_SR)

from emu.harness import PAGE, Machine, call  # noqa: E402
from dt2.container import decode_syx  # noqa: E402
from dt2 import elz  # noqa: E402

BOOT_BASE = 0x800003FC      # section 2 file offset 0 (a 4-byte header precedes the code)
MAIN_BASE = 0x40000400
UPD_BASE = 0x80000400        # section 4 loads with its 8-byte header: the bootstrap reads the
                             # stored section to its table address and jumps to its first long
SCRATCH = 0x50000000        # buffers for routines that take a pointer (not a unit address)
STACK = 0x10000000
FILL = 0xA5                 # what the stage and output area hold before a run
LIMIT = 4_000_000_000       # instructions; the trailer check alone is ~49M basic blocks
LOADER_LIMIT = 1_000_000_000  # a good main OS loads in a few seconds of this

# --------------------------------------------------------------------------
# containers
# --------------------------------------------------------------------------


def parse(syx):
    """-> dict: stream (decoded bytes as the receive path stores them),
    container, sections {id: (offset, length, dest)}."""
    stream = decode_syx(syx)
    size, chk = struct.unpack_from('>II', stream, 0)
    cont = stream[8:8 + size]
    if cont[:4] != b'ELE3':
        raise SystemExit('%s: no ELE3 container' % syx)
    n = struct.unpack_from('>I', cont, 0x1C)[0]
    secs = {}
    for k in range(n):
        sid, off, ln, dest = struct.unpack_from('>IIII', cont, 0x20 + 16 * k)
        secs[sid] = (off, ln, dest)
    return {'path': syx, 'stream': stream, 'size': size, 'chk': chk,
            'container': cont, 'sections': secs}


def stored(c, sid):
    off, ln, _ = c['sections'][sid]
    return c['container'][off:off + ln]


def sha(b):
    return hashlib.sha256(b).hexdigest()


def content_checksum(cont):
    s = 0
    for i in range(len(cont) // 4):
        s = (s + ((i + 1) ^ struct.unpack_from('>I', cont, 4 * i)[0])) & 0xFFFFFFFF
    return s


# --------------------------------------------------------------------------
# signatures
# --------------------------------------------------------------------------


def find1(img, base, pattern, what):
    hits = [m for m in re.finditer(pattern, img, re.S)]
    if len(hits) != 1:
        raise SystemExit('%s: %d matches for %s' % (what, len(hits), pattern))
    return hits[0], base + hits[0].start()


def u32(b):
    return struct.unpack('>I', b)[0]


def s16(b):
    return struct.unpack('>h', b)[0]


def resolve_boot(boot):
    """Bootstrap addresses by signature. -> dict name -> (value, how)."""
    r = {}
    # depacker: its whole prologue up to "addq.l #8,a0" (skips the stream's header)
    _, a = find1(boot, BOOT_BASE, re.escape(bytes.fromhex(
        '4fefffec428148d7043c282f001c7601206f001822445088')), 'depacker')
    r['depack'] = (a, 'unique 24-byte prologue lea -20(sp); clr.l d1; movem.l; '
                      'move.l 28(sp),d4; ...; addq.l #8,a0')
    end = boot.find(bytes.fromhex('4cd7043c4fef00144e75'), a - BOOT_BASE)
    r['depack_len'] = (end + 10 - (a - BOOT_BASE), 'to its movem/lea/rts epilogue')
    # loader: pea stage; push len; d0 = off + flash_at; jsr flash_read; push dest;
    # pea stage; jsr depack(pc)
    m, a = find1(boot, BOOT_BASE, rb'\x48\x79(.{4})\x2f\x2e\xff\xf8\x20\x2e\xff\xf4\x06\x80(.{4})'
                 rb'\x2f\x00\x4e\xb9(.{4})\x2f\x2e\xff\xfc\x48\x79(.{4})\x4e\xba(.{2})', 'loader')
    stage, flash_at, fread, stage2 = (u32(m.group(i)) for i in (1, 2, 3, 4))
    bsr_at = a + m.start(5) - m.start() - 2
    target = bsr_at + 2 + s16(m.group(5))
    if stage != stage2 or target != r['depack'][0]:
        raise SystemExit('loader signature: stage %#x/%#x, depack %#x' % (stage, stage2, target))
    start = boot.rfind(b'\x4e\x56', 0, a - BOOT_BASE)
    r['loader'] = (BOOT_BASE + start, 'link a6 before: pea stage; move.l -8(a6),-(sp); '
                   'move.l -12(a6),d0; addi.l #flash_at,d0; jsr flash_read; '
                   'move.l -4(a6),-(sp); pea stage; jsr depack(pc)')
    r['loader_after_depack'] = (bsr_at + 4, 'the instruction after its jsr depack(pc)')
    r['stage'] = (stage, 'pea operand in the loader, twice')
    r['flash_at'] = (flash_at, 'addi.l operand in the loader')
    r['flash_read'] = (fread, 'jsr target in the loader')
    look = boot.rfind(b'\x48\x78\x00\x03\x4e\xb9', start, a - BOOT_BASE)
    r['lookup'] = (u32(boot[look + 6:look + 10]), 'jsr after pea 3 in the loader')
    # the boot path's call of the loader: jsr loader(pc), then a0 = *(d0); jsr (a0)
    lo = r['loader'][0]
    calls = [BOOT_BASE + i for i in range(0, len(boot) - 4, 2)
             if boot[i:i + 2] == b'\x4e\xba' and BOOT_BASE + i + 2 + s16(boot[i + 2:i + 4]) == lo]
    if len(calls) != 1:
        raise SystemExit('loader has %d pc-relative callers' % len(calls))
    r['boot_call'] = (calls[0], 'the only jsr loader(pc)')
    # boot path start: the instruction after the jsr to the init/menu routine before it
    i = calls[0] - BOOT_BASE
    j = boot.rfind(b'\x4e\xba', i - 0x20, i)
    r['boot_from'] = (BOOT_BASE + j + 4, 'after the jsr (pc) just before the boot-flag test')
    r['boot_init'] = (BOOT_BASE + j + 2 + s16(boot[j + 2:j + 4]), 'that jsr target')
    f = BOOT_BASE + j + 4 - BOOT_BASE
    if boot[f:f + 4] != b'\x70\x40\xc0\xb9':
        raise SystemExit('boot path does not start with moveq #64,d0; and.l flags,d0')
    r['boot_flags'] = (u32(boot[f + 4:f + 8]), 'and.l operand: bit 6 runs the updater first')
    # receive verify-and-commit: move.l rx,d2; ... movea.l rx+4,a1; ...
    m, a = find1(boot, BOOT_BASE, rb'\x4f\xef\xff\xe4\x42\x80\x48\xd7\x1c\x3c\x24\x39(.{4})'
                 rb'\x26\x02\xe4\x8b\x22\x79(.{4})', 'verify-and-commit')
    rx = u32(m.group(1))
    r['commit'] = (a, 'lea -28(sp); clr.l d0; movem.l; move.l rx,d2 (length); '
                      'lsr.l #2; movea.l rx+4,a1 (stored checksum)')
    r['rx'] = (rx, 'its move.l abs.l operand')
    m2 = re.compile(rb'\xb2\x89\x67\x04\x4e\xba(.{2})\x2f\x02\x48\x79(.{4})\x4e\xb9(.{4})'
                    rb'\x50\x8f\x4a\x00\x67.', re.S).search(boot, a - BOOT_BASE)
    cmp_at = BOOT_BASE + m2.start()
    r['cmp'] = (cmp_at, 'cmp.l a1,d1 (computed vs stored checksum)')
    r['abort'] = (cmp_at + 6 + s16(m2.group(1)), 'jsr target when the compare fails: '
                  'UPGRADE ABORTED / PLEASE REBOOT, then a spin')
    r['csum_ok'] = (cmp_at + 8, 'first instruction after the checksum passes')
    r['hmac'] = (u32(m2.group(3)), 'jsr target with (rx+8, length)')
    r['commit_pass'] = (BOOT_BASE + m2.end(), 'first instruction after the trailer passes '
                        '(UPGRADING..., then erase and program)')
    # key derivation, the first pc-relative call inside the trailer check
    h = r['hmac'][0] - BOOT_BASE
    k = boot.find(b'\x4e\xba', h, h + 0x40)
    r['hmac_key'] = (BOOT_BASE + k + 2 + s16(boot[k + 2:k + 4]), 'first jsr (pc) in the trailer check')
    # bootstrap self-upgrade: read section 2 to stage, depack to dst, CRC-32 residue
    m, a = find1(boot, BOOT_BASE, rb'\x48\x79(.{4})\x2f\x00\x20\x2e\xff\xf4\x06\x80(.{4})\x2f\x00'
                 rb'\x4e\xb9(.{4})\x48\x79(.{4})\x48\x79(.{4})\x4e\xb9(.{4})', 'self-upgrade')
    if u32(m.group(6)) != r['depack'][0]:
        raise SystemExit('self-upgrade calls %#x, not the depacker' % u32(m.group(6)))
    r['upg_stage'] = (u32(m.group(1)), 'self-upgrade: flash read destination and depack source')
    r['upg_dst'] = (u32(m.group(4)), 'self-upgrade: depack destination')
    m, a = find1(boot, BOOT_BASE, rb'\x4e\xba(.{2})\x4f\xef\x00\x20\x0c\x80\xde\xbb\x20\xe3', 'crc')
    r['crc32'] = (a + 2 + s16(m.group(1)), 'jsr (pc) before cmpi.l #0xdebb20e3,d0')
    # version gate: mvz.w (a6-4),d3; mvz.w running,d0; cmp.l d3,d0; bcc
    m, a = find1(boot, BOOT_BASE, rb'\x77\xee\xff\xfc\x71\xf9(.{4})\xb0\x83\x64\x00', 'gate')
    r['version_gate'] = (a + 10, 'bcc.w after mvz.w -4(a6),d3; mvz.w running,d0; cmp.l d3,d0')
    r['running_version'] = (u32(m.group(1)), 'mvz.w operand')
    return r


def resolve_upd(upd, boot_depack_bytes, boot, R):
    """Section 4 addresses by signature, plus the bootstrap code that runs it."""
    r = {}
    r['entry'] = (u32(upd[0:4]), 'first long of the stored section (the bootstrap jumps to it)')
    k = upd.find(boot_depack_bytes)
    if k < 0 or upd.find(boot_depack_bytes, k + 1) >= 0:
        raise SystemExit('section 4 depacker copy: not exactly one')
    r['depack'] = (UPD_BASE + k, 'byte-identical copy of the bootstrap depacker')
    m, a = find1(upd, UPD_BASE, rb'\x48\x79(.{4})\x2f\x2e\xff\xf8\x22\x2e\xff\xf4\x06\x81(.{4})'
                 rb'\x2f\x01\x4e\xb9(.{4})\x2f\x2e\xff\xfc\x48\x79(.{4})\x4e\xb9(.{4})', 'section 4 loader')
    stage, flash_at, fread, stage2, dep = (u32(m.group(i)) for i in range(1, 6))
    if stage != stage2 or dep != r['depack'][0]:
        raise SystemExit('section 4 loader: stage %#x/%#x depack %#x' % (stage, stage2, dep))
    look = upd.rfind(b'\x48\x6e\xff\xf0\x48\x78\x00\x03\x4e\xb9', 0, a - UPD_BASE)
    st = look - 6 if upd[look - 6:look] == bytes.fromhex('40c246fc2700') else look
    r['loader'] = (UPD_BASE + st, 'move.w sr,d2; move.w #0x2700,sr; pea -16(a6); pea 3; jsr lookup; '
                   'pea stage; ...; jsr flash_read; ...; pea stage; jsr depack')
    r['lookup'] = (u32(upd[look + 10:look + 14]), 'jsr after pea 3')
    r['stage'] = (stage, 'pea operand, twice')
    r['flash_at'] = (flash_at, 'addi.l operand')
    r['flash_read'] = (fread, 'jsr target')
    r['loader_after_depack'] = (a + m.end() - m.start(), 'the instruction after its jsr depack')
    mt = re.search(rb'\x20\x7c\x40\x00\x00\x00\x20\x3c\xcc\xdd\xee\xff.{18}\xb1\xfc(.{4})', upd, re.S)
    r['memtest_end'] = (u32(mt.group(1)) if mt else None, 'cmpa.l operand of the pattern fill '
                        'from 0x40000000 (a DDR test before the loader)')
    # the bootstrap routine that loads section 4 (copied to SRAM at reset): pea buf; pea 4; jsr lookup
    m, a = find1(boot, BOOT_BASE, rb'\x48\x79(.{4})\x48\x78\x00\x04\x24\x39(.{4})\x4e\xb9(.{4})', 'run section 4')
    r['boot_runs_it'] = (a, 'bootstrap: pea buf; pea 4; move.l flags,d2; jsr lookup; read the '
                         'stored section 4 to its table address; jump to its first long (copied '
                         'to SRAM at reset)')
    # the boot-flag bit that selects it: moveq #0x60,d0; or.l d0,flags after jsr X; tst.l d0; bge
    flags = R['boot_flags'][0]
    m, a = find1(boot, BOOT_BASE, rb'\x4e\xb9(.{4})\x4a\x80\x6c.\x70\x60\x81\xb9' + re.escape(struct.pack('>I', flags)),
                 'flag 0x60')
    r['flag_set_at'] = (a + 10, 'or.l #0x60 into the boot flags when the jsr before returns < 0')
    r['flag_cond'] = (u32(m.group(1)), 'that jsr: sends 0x60, 0x01 on UART8 and waits for replies; '
                      '-1 on a timeout')
    return r


def resolve_main(img, boot_depack_bytes):
    """Main OS addresses by signature."""
    r = {}

    def s(text):
        hits = [mm.start() for mm in re.finditer(re.escape(text + b'\0'), img)]
        # the error strings start after a NUL; take the copy whose pointer exists
        return [MAIN_BASE + h for h in hits]

    names = [b'No error', b'Checksum failed', b'Power adapter must be connected',
             b'Unsupported downgrade']
    addr = {}
    for t in names:
        cands = [a for a in s(t) if img.find(struct.pack('>I', a)) >= 0]
        if len(cands) != 1:
            raise SystemExit('error string %r: %d referenced copies' % (t, len(cands)))
        addr[t] = cands[0]
    want = struct.pack('>6I', addr[names[0]], addr[names[1]], addr[names[1]], addr[names[1]],
                       addr[names[2]], addr[names[3]])
    i = img.find(want)
    if i < 0 or img.find(want, i + 1) >= 0:
        raise SystemExit('error table not found once')
    table = MAIN_BASE + i
    r['error_table'] = (table, 'six pointers: No error, Checksum failed x3, Power adapter '
                        'must be connected, Unsupported downgrade')
    ref = img.find(b'\x41\xf9' + struct.pack('>I', table))
    if ref < 0 or img.find(b'\x41\xf9' + struct.pack('>I', table), ref + 1) >= 0:
        raise SystemExit('lea error_table: not exactly one')
    r['error_table_ref'] = (MAIN_BASE + ref, 'the only lea error_table,a0')
    # validator: pea 32(a6); jsr V; addq #4; moveq #1,d1; cmp.l d0,d1 -- just before the lea
    window = img[ref - 0x80:ref]
    m = re.search(rb'\x48\x6e\x00\x20\x4e\xb9(.{4})\x58\x8f\x72\x01\xb2\x80', window, re.S)
    if not m:
        raise SystemExit('validator call not found before the error table reference')
    v = u32(m.group(1))
    r['validator'] = (v, 'jsr target of pea 32(a6); jsr; addq.l #4,sp; moveq #1,d1; '
                      'cmp.l d0,d1, 0x80 bytes before the lea error_table')
    r['validator_call_sysex'] = (MAIN_BASE + ref - 0x80 + m.start() + 4,
                                 'that call site (the SysEx receive state machine)')
    vo = v - MAIN_BASE
    # the body ends at the first rts that follows a "move.l (sp)+,a2" restore
    body_end = img.find(b'\x24\x5f\x4e\x75', vo) + 4
    body = img[vo:body_end]
    r['validator_len'] = (len(body), 'to the movea.l (sp)+,a2; rts epilogue')
    jsrs = [u32(body[k + 2:k + 6]) for k in range(0, len(body) - 5, 2) if body[k:k + 2] == b'\x4e\xb9']
    if len(jsrs) != 2:
        raise SystemExit('validator: %d jsr abs.l, expected 2' % len(jsrs))
    r['os_checksum'] = (jsrs[0], 'first jsr in the validator (arg: the stream preamble)')
    r['os_hmac'] = (jsrs[1], 'second jsr in the validator (args: preamble+8, length)')
    rets = sorted({body[k + 1] for k in range(0, len(body) - 1, 2) if body[k] == 0x70})
    r['validator_returns'] = (rets, 'every moveq #n,d0 in its body')
    floor = [mm for mm in re.finditer(rb'\x22\x3c(.{4})\xb2\xaa\x00\x10', body, re.S)]
    r['build_floor'] = ([f.group(1).decode('latin-1') for f in floor],
                        'move.l #imm,d1; cmp.l 16(a2),d1 inside the validator (Digitakt II 1.16 shape)')
    whole = [MAIN_BASE + mm.start() for mm in re.finditer(rb'\x22\x3c....\xb2\xaa\x00\x10', img, re.S)]
    r['build_floor_anywhere'] = (whole, 'the same signature over the whole main OS')
    callers = [MAIN_BASE + k for k in range(0, len(img) - 6, 2)
               if img[k:k + 2] in (b'\x4e\xb9', b'\x4e\xf9') and u32(img[k + 2:k + 6]) == v]
    r['validator_callers'] = (callers, 'every jsr/jmp abs.l to it')
    # the seed the main OS's trailer check copies
    ho = jsrs[1] - MAIN_BASE
    m = re.compile(rb'\x48\x78\x00(.)\x48\x79(.{4})', re.S).search(img, ho, ho + 0x20)
    seed_at = u32(m.group(2))
    seed = img[seed_at - MAIN_BASE:seed_at - MAIN_BASE + m.group(1)[0]]
    r['os_hmac_seed'] = (seed.rstrip(b'\0').decode('latin-1'), 'pea operand (%#x) of the first '
                         'copy in the trailer check' % seed_at)
    # main OS copy of the depacker: byte-identical to the bootstrap's
    k = img.find(boot_depack_bytes)
    if k < 0 or img.find(boot_depack_bytes, k + 1) >= 0:
        raise SystemExit('main OS depacker copy: not exactly one')
    r['os_depack'] = (MAIN_BASE + k, 'byte-identical copy of the bootstrap depacker (%d bytes)'
                      % len(boot_depack_bytes))
    # the DSP loader: pea 7; jsr lookup; ... addi.l #flash_at,d0; ... jsr os_depack
    tgt = struct.pack('>I', MAIN_BASE + k)
    sites = []
    for mm in re.finditer(rb'\x48\x78\x00\x07\x4e\xb9(.{4})', img, re.S):
        d = img.find(b'\x4e\xb9' + tgt, mm.start(), mm.start() + 0x60)
        if d >= 0:
            sites.append((mm, d))
    if len(sites) != 1:
        raise SystemExit('pea 7; jsr; ... jsr os_depack: %d sites' % len(sites))
    mm, d = sites[0]
    r['os_lookup'] = (u32(mm.group(1)), 'jsr after pea 7 in the DSP loader')
    r['os_dsp_depack_call'] = (MAIN_BASE + d, 'jsr os_depack after pea 7; jsr lookup')
    pre = img[d - 12:d]
    r['os_dsp_layout'] = ('output at buffer+1 after move.b #3,(a0)+'
                          if b'\x10\xfc\x00\x03' in pre else 'unknown', 'bytes before that call')
    return r


# --------------------------------------------------------------------------
# emulation
# --------------------------------------------------------------------------


class Stop(Exception):
    pass


def machine(image, base):
    m = Machine()
    m.load(image, base)
    m.install_isa_patches_scoped(image, base)
    m.ensure(STACK)
    return m


def run_until(m, start, stops, sp, limit=LIMIT):
    """Run from `start` until a code hook in `stops` ({addr: label}) fires.
    -> (label, instructions-or-None)."""
    hit = {}

    def make(label):
        def h(uc, addr, size, data):
            hit['label'] = label
            hit['pc'] = addr
            hit['d0'] = uc.reg_read(UC_M68K_REG_D0) & 0xFFFFFFFF
            hit['d1'] = uc.reg_read(UC_M68K_REG_D1) & 0xFFFFFFFF
            hit['a1'] = uc.reg_read(UC_M68K_REG_A1) & 0xFFFFFFFF
            uc.emu_stop()
        return h

    hs = [m.uc.hook_add(UC_HOOK_CODE, make(lbl), begin=a, end=a) for a, lbl in stops.items()]
    m.uc.reg_write(UC_M68K_REG_SR, 0x2700)
    m.uc.reg_write(UC_M68K_REG_A7, sp)
    try:
        m.uc.emu_start(start, 0xFFFFFFFE, count=limit)
    except UcError as e:
        hit.setdefault('label', 'error: %s' % e)
    for h in hs:
        m.uc.hook_del(h)
    hit.setdefault('label', 'limit')
    return hit


def fill(m, lo, hi, byte=FILL):
    for a in range(lo & ~(PAGE - 1), hi, PAGE):
        m.ensure(a)
    m.uc.mem_write(lo, bytes([byte]) * (hi - lo))


def flash_hle(m, addr, flash, log):
    """Serve the bootstrap's SPI NOR read(off, len, dest) from `flash`."""
    def h(uc, a, size, data):
        sp = uc.reg_read(UC_M68K_REG_A7)
        ret, off, ln, dest = struct.unpack('>IIII', uc.mem_read(sp, 16))
        chunk = flash[off:off + ln]
        chunk += b'\xff' * (ln - len(chunk))        # erased NOR past the image
        for p in range(dest & ~(PAGE - 1), dest + ln + 1, PAGE):
            m.ensure(p)
        uc.mem_write(dest, chunk)
        log.append((off, ln, dest))
        uc.reg_write(UC_M68K_REG_A7, sp + 4)
        uc.reg_write(UC_M68K_REG_PC, ret)
    return m.uc.hook_add(UC_HOOK_CODE, h, begin=addr, end=addr)


def read_extent(m, lo, hi):
    """Record the highest address read in [lo, hi)."""
    st = {'max': None, 'n': 0}

    def h(uc, typ, a, size, val, data):
        st['n'] += 1
        e = a + size - 1
        if st['max'] is None or e > st['max']:
            st['max'] = e
    hk = m.uc.hook_add(UC_HOOK_MEM_READ, h, begin=lo, end=hi - 1)
    return st, hk


def loader_run(image, base, start, flash_read, flash_at, stage, after_depack, cand, want_main,
               a6=None, pre=None):
    """Run a loader from `start` to the first instruction it executes in the
    main OS, with flash = the candidate's container at `flash_at`."""
    m = machine(image, base)
    flash = bytearray(b'\xff' * (flash_at + len(cand['container']) + 0x10000))
    flash[flash_at:flash_at + len(cand['container'])] = cand['container']
    flash = bytes(flash)
    off, ln, dest = cand['sections'][3]
    n = len(want_main)
    fill(m, 0x40000000, max(stage + ln, dest + n) + 0x100000)
    if pre:
        pre(m)
    log = []
    flash_hle(m, flash_read, flash, log)
    disjoint = dest + n <= stage
    ext, _ = read_extent(m, stage, stage + 0x200000) if disjoint else ({'n': None, 'max': None}, None)
    out = {}

    def at_after(uc, a, size, data):
        out['depack_ret'] = uc.reg_read(UC_M68K_REG_D0) & 0xFFFFFFFF
    m.uc.hook_add(UC_HOOK_CODE, at_after, begin=after_depack, end=after_depack)
    entered = {}

    def into_main(uc, a, size, data):
        entered['pc'] = a
        uc.emu_stop()
    m.uc.hook_add(UC_HOOK_CODE, into_main, begin=0x40000000, end=0x47FFFFFF)
    sp = STACK + PAGE - 0x400
    m.uc.reg_write(UC_M68K_REG_SR, 0x2700)
    m.uc.reg_write(UC_M68K_REG_A7, sp)
    if a6:
        m.uc.reg_write(UC_M68K_REG_A6, sp + 0x100)
    t = time.time()
    try:
        m.uc.emu_start(start, 0xFFFFFFFE, count=LOADER_LIMIT)
        err = None
    except UcError as e:
        err = str(e)
    secs = time.time() - t
    final_pc = m.uc.reg_read(UC_M68K_REG_PC) & 0xFFFFFFFF
    got = bytes(m.uc.mem_read(dest, n))
    before = bytes(m.uc.mem_read(0x40000000, dest - 0x40000000))
    res = {
        'flash_reads': [{'flash': '%#x' % o, 'len': l, 'dest': '%#x' % d} for o, l, d in log],
        'section3': {'offset': '%#x' % off, 'length': ln, 'dest': '%#x' % dest},
        'stage': '%#x' % stage,
        'depack_returned': out.get('depack_ret'),
        'entered_main_at': '%#x' % entered['pc'] if 'pc' in entered else None,
        'stopped_at': None if 'pc' in entered else '%#x (budget of %d instructions spent)'
                      % (final_pc, LOADER_LIMIT),
        'main_entry_vector': '%#x' % struct.unpack('>I', got[:4])[0],
        'output_sha256': sha(got),
        'output_matches': got == want_main,
        'output_bytes_differing': sum(1 for x, y in zip(got, want_main) if x != y),
        'first_difference': next(('%#x' % (dest + i) for i, (x, y) in enumerate(zip(got, want_main))
                                  if x != y), None),
        'untouched_below_output': before == bytes([FILL]) * len(before),
        'error': err,
        'seconds': round(secs, 1),
    }
    if disjoint:
        after = bytes(m.uc.mem_read(dest + n, stage - (dest + n)))
        res['untouched_between_output_and_stage'] = after == bytes([FILL]) * len(after)
        res['stage_reads'] = ext['n']
        res['stage_last_byte_read'] = '%#x' % ext['max'] if ext['max'] is not None else None
        res['stage_stream_end'] = '%#x' % (stage + ln - 1)
    return res


def boot_path(boot, R, cand, want_main):
    """The bootstrap from boot_from to the first instruction in the main OS,
    with the boot flags clear (no request to run section 4 first)."""
    def pre(m):
        m.uc.mem_write(R['boot_flags'][0], b'\0\0\0\0')
    return loader_run(boot, BOOT_BASE, R['boot_from'][0], R['flash_read'][0], R['flash_at'][0],
                      R['stage'][0], R['loader_after_depack'][0], cand, want_main, pre=pre)


def updater_path(upd, U, cand, want_main):
    """Section 4's own loader (taken when the bootstrap sets boot flag bit 6),
    from its first instruction to the first instruction in the main OS."""
    return loader_run(upd, UPD_BASE, U['loader'][0], U['flash_read'][0], U['flash_at'][0],
                      U['stage'][0], U['loader_after_depack'][0], cand, want_main, a6=True)


def inplace_gap(stream, stage, load):
    """Smallest distance between the depacker's write pointer and the next
    unread stream byte when the stream sits at `stage` and the output at
    `load` (as elekloader's syx.inplace_depack measures it)."""
    length = struct.unpack_from('>I', stream, 0)[0]
    s = elz.Bits(stream, 8, 8 + length)
    n = 0
    base = stage - load
    gap = None
    last = 1
    while True:
        if s.bit():
            g = base + s.p - n
            gap = g if gap is None or g < gap else gap
            s.byte()
            n += 1
            continue
        g = s.gamma()
        if g == elz.REUSE:
            off = last
        else:
            raw = ((g << 8) + s.byte()) & 0xFFFFFFFF
            if raw == elz.BIAS:
                break
            off = (raw - elz.BIAS) & 0xFFFFFFFF
            last = off
        short = 2 * s.bit() + s.bit()
        k = short if short else s.gamma() + 2
        if off > elz.FAR:
            k += 1
        g = base + s.p - (n + k)
        gap = g if gap is None or g < gap else gap
        n += k + 1
    return gap, n


def depack_call(image, base, depack, stream, src, dst, want, extra_fill=None):
    m = machine(image, base)
    hi = max(src + len(stream), dst + len(want)) + 0x100000
    fill(m, min(src, dst) & ~(PAGE - 1), hi)
    if extra_fill:
        for a, b in extra_fill:
            m.uc.mem_write(a, b)
    m.uc.mem_write(src, stream)
    hi = src + len(stream) + 0x1000
    if src < dst < hi:                 # the depacker reads its own output for matches
        hi = dst
    ext, _ = read_extent(m, src, hi)
    t = time.time()
    n = call(m, depack, [src, dst], stack_top=STACK + PAGE - 0x100, limit=LIMIT)
    got = bytes(m.uc.mem_read(dst, n)) if 0 < n <= 0x800000 else b''
    return m, {
        'returned_len': n,
        'output_sha256': sha(got),
        'matches': got == want,
        'last_stream_byte_read': ext['max'] - src if ext['max'] is not None else None,
        'stream_len': len(stream),
        'seconds': round(time.time() - t, 1),
    }, got


def boot_commit(boot, R, stream):
    m = machine(boot, BOOT_BASE)
    rx = R['rx'][0]
    for a in range(rx, rx + len(stream) + PAGE, PAGE):
        m.ensure(a)
    m.uc.mem_write(rx, stream)
    stops = {R['abort'][0]: 'abort', R['commit_pass'][0]: 'pass'}
    seen = {}

    def at_cmp(uc, a, size, data):
        seen['computed'] = uc.reg_read(UC_M68K_REG_D1) & 0xFFFFFFFF
        seen['stored'] = uc.reg_read(UC_M68K_REG_A1) & 0xFFFFFFFF

    def at_ok(uc, a, size, data):
        seen['checksum_passed'] = True
    m.uc.hook_add(UC_HOOK_CODE, at_cmp, begin=R['cmp'][0], end=R['cmp'][0])
    m.uc.hook_add(UC_HOOK_CODE, at_ok, begin=R['csum_ok'][0], end=R['csum_ok'][0])
    hit = run_until(m, R['commit'][0], stops, STACK + PAGE - 0x200)
    return {
        'verdict': {'pass': 'PASS', 'abort': 'FAIL'}.get(hit['label'], hit['label']),
        'stopped_at': '%#x' % hit.get('pc', 0),
        'checksum_computed': '%#010x' % seen.get('computed', 0),
        'checksum_stored': '%#010x' % seen.get('stored', 0),
        'checksum_passed': seen.get('checksum_passed', False),
        'trailer_passed': hit['label'] == 'pass',
    }


def boot_hmac(boot, R, stream):
    m = machine(boot, BOOT_BASE)
    rx = R['rx'][0]
    for a in range(rx, rx + len(stream) + PAGE, PAGE):
        m.ensure(a)
    m.uc.mem_write(rx, stream)
    size = struct.unpack_from('>I', stream, 0)[0]
    d0 = call(m, R['hmac'][0], [rx + 8, size], stack_top=STACK + PAGE - 0x100, limit=LIMIT)
    return d0 & 0xFF


def os_checks(img, M, stream):
    m = machine(img, MAIN_BASE)
    for a in range(SCRATCH, SCRATCH + len(stream) + PAGE, PAGE):
        m.ensure(a)
    m.uc.mem_write(SCRATCH, stream)
    size = struct.unpack_from('>I', stream, 0)[0]
    top = STACK + PAGE - 0x100
    v = call(m, M['validator'][0], [SCRATCH], stack_top=top, limit=LIMIT)
    c = call(m, M['os_checksum'][0], [SCRATCH], stack_top=top, limit=LIMIT)
    h = call(m, M['os_hmac'][0], [SCRATCH + 8, size], stack_top=top, limit=LIMIT) & 0xFF
    names = {1: 'pass', 3: 'Checksum failed (content checksum)', 4: 'Checksum failed (trailer)',
             6: 'Unsupported downgrade', 5: 'Power adapter must be connected'}
    return {'validator': v, 'validator_means': names.get(v, '?'),
            'checksum_routine': '%#010x' % c, 'hmac_routine': h}


# --------------------------------------------------------------------------
# negative controls
# --------------------------------------------------------------------------


def seal_key(boot):
    """The bootstrap's key derivation, done in Python only to reseal controls."""
    i = boot.find(b'Multiplier\0')
    seed = b'Multiplier'
    const = boot[i + 11:i + 43]
    a = hashlib.sha256(seed).digest()
    b = hashlib.sha256(seed[::-1]).digest()
    return bytes(x ^ y ^ z for x, y, z in zip(const, a, b))


def restream(stream, cont):
    tail = stream[8 + len(cont):]
    return struct.pack('>II', len(cont), content_checksum(cont)) + cont + tail


def controls(boot, cand):
    stream = cand['stream']
    size = cand['size']
    cont = bytearray(cand['container'])
    out = {}
    s1 = bytearray(stream)
    s1[8 + size - 1] ^= 1
    out['trailer bit flipped'] = bytes(s1)
    c2 = bytearray(cont)
    c2[-1] ^= 1
    out['trailer bit flipped, checksum redone'] = restream(stream, bytes(c2))
    off, ln, _ = cand['sections'][3]
    c3 = bytearray(cont)
    c3[off + 8 + ln // 2] ^= 0x10
    key = seal_key(boot)
    c3[-32:] = hmaclib.new(key, bytes(c3[:-32]), hashlib.sha256).digest()
    out['main OS stream bit flipped, resealed'] = restream(stream, bytes(c3))
    return out


# --------------------------------------------------------------------------


def fmt(v):
    if isinstance(v, int) and not isinstance(v, bool) and v > 0xFFFF:
        return '%#x' % v
    if isinstance(v, list):
        return '[' + ', '.join(fmt(x) for x in v) + ']'
    return str(v)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--stock', required=True, help='the stock .syx whose code runs')
    ap.add_argument('--want-main', help='expected depacked main OS (default: stock, depacked)')
    ap.add_argument('--json', help='write every result here')
    ap.add_argument('--no-controls', action='store_true')
    ap.add_argument('--parts', default='boot,updater,depack,bootcheck,os',
                    help='comma list of: boot, updater, depack, bootcheck, os')
    ap.add_argument('syx', nargs='+')
    a = ap.parse_args(argv)

    stock = parse(a.stock)
    boot = elz.depack_section(stored(stock, 2))
    img = elz.depack_section(stored(stock, 3))
    want_main = open(a.want_main, 'rb').read() if a.want_main else img
    print('stock %s\n  sha256 %s' % (a.stock, sha(open(a.stock, 'rb').read())))
    print('  bootstrap sha256 %s (%d bytes, version word %#06x)'
          % (sha(boot), len(boot), struct.unpack_from('>H', boot, 0x0C)[0]))
    print('  main OS   sha256 %s (%d bytes)' % (sha(img), len(img)))
    print('  want main sha256 %s' % sha(want_main))

    R = resolve_boot(boot)
    dep_bytes = boot[R['depack'][0] - BOOT_BASE:R['depack'][0] - BOOT_BASE + R['depack_len'][0]]
    M = resolve_main(img, dep_bytes)
    upd = stored(stock, 4)
    U = resolve_upd(upd, dep_bytes, boot, R)
    print('\nbootstrap (section 2, file offset 0 = %#x)' % BOOT_BASE)
    for k, (v, how) in R.items():
        print('  %-16s %-12s %s' % (k, fmt(v), how))
    print('\nmain OS (section 3, %#x)' % MAIN_BASE)
    for k, (v, how) in M.items():
        print('  %-22s %-14s %s' % (k, fmt(v), how))
    print('\nsection 4 (stored raw, header included, at %#x)' % UPD_BASE)
    for k, (v, how) in U.items():
        print('  %-20s %-12s %s' % (k, fmt(v), how))

    res = {'stock': a.stock, 'bootstrap': {k: v[0] for k, v in R.items()},
           'main_os': {k: v[0] for k, v in M.items()},
           'section4': {k: v[0] for k, v in U.items()}, 'files': {}}
    parts = set(a.parts.split(','))
    stage, load = R['stage'][0], stock['sections'][3][2]

    for path in a.syx:
        c = parse(path)
        f = {'sha256': sha(open(path, 'rb').read()), 'container_len': c['size']}
        print('\n=== %s\n  sha256 %s, container %d bytes' % (path, f['sha256'], c['size']))
        same = {sid: stored(c, sid) == stored(stock, sid) for sid in sorted(c['sections'])}
        f['stored_same_as_stock'] = same
        print('  stored sections identical to stock: %s' % same)
        for st_ in (stage, U['stage'][0]):
            gap, n = inplace_gap(stored(c, 3), st_, load)
            f['inplace_gap_%#x' % st_] = gap
            print('  in-place gap (stage %#x -> %#x): %d bytes, %d bytes out%s'
                  % (st_, load, gap, n, '  ** NEGATIVE: the output overruns unread input **' if gap < 0 else ''))
        if 'updater' in parts:
            up = updater_path(upd, U, c, want_main)
            f['updater_path'] = up
            print('  section 4 loader %#x -> main OS: entered %s, depacker returned %s, output sha256 %s, '
                  'matches=%s (%d bytes differ, first at %s), %s s%s'
                  % (U['loader'][0], up['entered_main_at'], up['depack_returned'], up['output_sha256'][:16],
                     up['output_matches'], up['output_bytes_differing'], up['first_difference'],
                     up['seconds'], ', ERROR ' + up['error'] if up['error'] else
                     (', stopped at ' + up['stopped_at'] if up['stopped_at'] else '')))
            print('    flash reads: %s' % up['flash_reads'])
        if 'boot' in parts:
            bp = boot_path(boot, R, c, want_main)
            f['boot_path'] = bp
            print('  boot path %#x -> main OS: entered %s (vector %s), depacker returned %s, '
                  'output sha256 %s, matches=%s, below/after untouched=%s/%s, stage read up to %s '
                  '(stream ends %s), %s s%s'
                  % (R['boot_from'][0], bp['entered_main_at'], bp['main_entry_vector'],
                     bp['depack_returned'], bp['output_sha256'][:16], bp['output_matches'],
                     bp['untouched_below_output'], bp.get('untouched_between_output_and_stage'),
                     bp.get('stage_last_byte_read'), bp.get('stage_stream_end'), bp['seconds'],
                     ', ERROR ' + bp['error'] if bp['error'] else ''))
            print('    flash reads: %s' % bp['flash_reads'])

        if 'depack' in parts:
            dp = {}
            _, r3, _ = depack_call(boot, BOOT_BASE, R['depack'][0], stored(c, 3), stage, load, want_main)
            dp['3 main OS, bootstrap depacker, stage -> load'] = r3
            boot_want = elz.depack_section(stored(stock, 2))
            m2, r2, got2 = depack_call(boot, BOOT_BASE, R['depack'][0], stored(c, 2),
                                       R['upg_stage'][0], R['upg_dst'][0], boot_want)
            r2['crc32_residue'] = '%#010x' % call(m2, R['crc32'][0], [0xFFFFFFFF, R['upg_dst'][0], len(got2)],
                                                  stack_top=STACK + PAGE - 0x100)
            r2['version_word'] = '%#06x' % struct.unpack_from('>H', got2, 0x0C)[0] if got2 else None
            dp['2 bootstrap, bootstrap depacker, self-upgrade layout'] = r2
            dsp_want = elz.depack_section(stored(stock, 7))
            buf = SCRATCH + 0x200000
            _, r7, _ = depack_call(img, MAIN_BASE, M['os_depack'][0], stored(c, 7), SCRATCH, buf + 1,
                                   dsp_want, extra_fill=[(buf, b'\x03')])
            dp['7 DSP, main OS depacker, output at buffer+1'] = r7
            s8_want = elz.depack_section(stored(stock, 8))
            _, r8, _ = depack_call(boot, BOOT_BASE, R['depack'][0], stored(c, 8), SCRATCH,
                                   SCRATCH + 0x200000, s8_want)
            dp['8, bootstrap depacker (codec check only)'] = r8
            f['depack'] = dp
            for k, r in dp.items():
                extra = ''
                if 'crc32_residue' in r:
                    extra = ', CRC-32 residue %s, version word %s' % (r['crc32_residue'], r['version_word'])
                print('  depack %-52s len %d sha256 %s matches=%s, read %s of %d stream bytes, %s s%s'
                      % (k, r['returned_len'], r['output_sha256'][:16], r['matches'],
                         r['last_stream_byte_read'] + 1 if r['last_stream_byte_read'] is not None else None,
                         r['stream_len'], r['seconds'], extra))

        if 'bootcheck' in parts:
            f['bootcheck'] = boot_commit(boot, R, c['stream'])
            f['boot_hmac'] = boot_hmac(boot, R, c['stream'])
            print('  bootstrap verify-and-commit: %(verdict)s at %(stopped_at)s (checksum computed '
                  '%(checksum_computed)s stored %(checksum_stored)s, checksum passed %(checksum_passed)s, '
                  'trailer passed %(trailer_passed)s)' % f['bootcheck'])
            print('  bootstrap trailer check alone: %d' % f['boot_hmac'])
        if 'os' in parts:
            f['os'] = os_checks(img, M, c['stream'])
            print('  main OS validator: %(validator)d (%(validator_means)s); checksum routine '
                  '%(checksum_routine)s; trailer routine %(hmac_routine)d' % f['os'])
        res['files'][path] = f

    if not a.no_controls:
        c = parse(a.syx[0])
        res['controls'] = {}
        print('\n=== negative controls on %s (in memory only)' % a.syx[0])
        for name, s in controls(boot, c).items():
            r = {'bootcheck': boot_commit(boot, R, s), 'boot_hmac': boot_hmac(boot, R, s),
                 'os': os_checks(img, M, s)}
            if 'main OS' in name:
                cc = parse_stream(s)
                _, r3, _ = depack_call(boot, BOOT_BASE, R['depack'][0], stored(cc, 3), stage,
                                       load, want_main)
                r['depack3'] = r3
                r['boot_path'] = boot_path(boot, R, cc, want_main)
            res['controls'][name] = r
            print('  %-40s bootstrap %s (checksum passed %s), trailer alone %d, main OS validator %d (%s)%s'
                  % (name, r['bootcheck']['verdict'], r['bootcheck']['checksum_passed'], r['boot_hmac'],
                     r['os']['validator'], r['os']['validator_means'],
                     ('; depack matches=%s sha256 %s; boot path matches=%s, entered %s'
                      % (r['depack3']['matches'], r['depack3']['output_sha256'][:16],
                         r['boot_path']['output_matches'], r['boot_path']['entered_main_at']))
                     if 'depack3' in r else ''))
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(res, fh, indent=1, default=str)
        print('\nwrote %s' % a.json)
    return 0


def parse_stream(stream):
    size = struct.unpack_from('>I', stream, 0)[0]
    cont = stream[8:8 + size]
    n = struct.unpack_from('>I', cont, 0x1C)[0]
    secs = {}
    for k in range(n):
        sid, off, ln, dest = struct.unpack_from('>IIII', cont, 0x20 + 16 * k)
        secs[sid] = (off, ln, dest)
    return {'stream': stream, 'size': size, 'container': cont, 'sections': secs}


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
