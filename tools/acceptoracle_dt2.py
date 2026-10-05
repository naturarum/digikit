# pyright: reportMissingImports=false
"""Run the Digitakt II's own main-OS loaders on .syx files, under Unicorn.

    uv run python tools/acceptoracle_dt2.py --stock Digitakt_II_OS1.17.syx \
        [--json OUT] [--parts updater,boot,model] SYX [SYX...]

A Digitakt II companion to tools/acceptoracle.py (written for Digitone II
1.12), reusing its signature resolvers and its loader runs. Every address is
found by signature in the stock file's own bootstrap (section 2) and
section 4, and printed with how it was found.

The code that runs is always the STOCK file's: a unit runs the bootstrap and
section 4 it already has in flash only for the bootstrap; section 4 and the
main OS come from the candidate's own container. Here section 4 is checked
byte-identical to stock in every candidate, so the stock copy is the one the
unit would run.

What runs, per candidate:

  updater  Section 4's own main-OS loader (the bootstrap runs section 4 when
           boot flag bit 6 is set): the section lookup, the flash read of
           section 3 to its stage, and its copy of the depacker writing the
           image over the load address while it reads the stream from the
           stage, in one memory. Run to the first instruction it executes in
           the main OS, or to the instruction budget.
  boot     The bootstrap's boot path (the control): from just after its
           init and STARTUP-menu routine, with the boot flags clear, through
           its loader, to the first instruction in the main OS.
  model    A pure-Python model of the same in-place run (stream and output
           in one memory): where the writer first reaches unread input and
           where the output first differs.

The intended main OS of each candidate is its own section 3 depacked out of
place (dt2/elz.py, not the unit's code). When elekloader's manifest
(<syx>.json) is next to the file, its main_sha256 must match that image;
for the stock file, the known 1.17 image's sha256 must.
"""
import argparse
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import re  # noqa: E402

import acceptoracle as A  # noqa: E402
from unicorn.m68k_const import (UC_M68K_REG_A0, UC_M68K_REG_A1, UC_M68K_REG_D0,  # noqa: E402
                                UC_M68K_REG_D2, UC_M68K_REG_PC)

# Digitakt II 1.17's main OS, as elekloader's profile pins it
# (elekloader/devices/digitakt_mk2.py)
KNOWN_MAIN = {'a1e7b657b705eba1a19d81c33c1e11ba9c409816447ad74005d7bbf36da6d964': '1.17'}
LO = 0x40000000


def memtest(upd):
    """Section 4's DDR test, which runs before its loader: by signature, its
    fill loop (movea.l #lo,a0; move.l #s0,d0; move.l #s1,d1; add.l d0,d1;
    add.l d1,d0; move.l d1,(a0); move.l d0,4(a0); addq.l #8,a0; cmpa.l #hi,a0).
    -> (lo, hi, s0, s1, address)."""
    pat = (rb'\x20\x7c(.{4})\x20\x3c(.{4})\x22\x3c(.{4})\xd2\x80\xd0\x81\x20\x81'
           rb'\x21\x40\x00\x04\x50\x88\xb1\xfc(.{4})')
    hits = list(re.finditer(pat, upd, re.S))
    if len(hits) != 1:
        raise SystemExit('section 4 DDR test fill: %d matches' % len(hits))
    m = hits[0]
    lo, s0, s1, hi = (A.u32(m.group(i)) for i in (1, 2, 3, 4))
    return lo, hi, s0, s1, A.UPD_BASE + m.start()


def memtest_bytes(lo, hi, s0, s1, upto):
    """The DDR contents the test leaves in [lo, min(hi, upto)): pairs (d1, d0)
    with d1 += d0, d0 += d1 before each store, as the loop does."""
    n = (min(hi, upto) - lo) // 8
    out = bytearray(8 * n)
    d0, d1 = s0, s1
    pk = struct.Struct('>II').pack_into
    M = 0xFFFFFFFF
    for i in range(n):
        d1 = (d1 + d0) & M
        d0 = (d0 + d1) & M
        pk(out, 8 * i, d1, d0)
    return bytes(out)


def updater_run(upd, U, c, want, ddr=None, dump=None):
    """acceptoracle.updater_path (the same loader_run call), optionally with
    DDR holding `ddr` (bytes from 0x40000000) instead of the 0xA5 fill:
    written by loader_run's `pre` hook, after its fill and before the run.
    Adds the final registers and, if `dump`, saves the output image."""
    cap = {}
    orig = A.machine

    def mach(image, base):
        m = orig(image, base)
        cap['m'] = m
        return m

    def pre(m):
        for a in range(LO, LO + len(ddr), A.PAGE):
            m.ensure(a)
        m.uc.mem_write(LO, ddr)
        cap['pre'] = len(ddr)
    A.machine = mach
    try:
        r = A.loader_run(upd, A.UPD_BASE, U['loader'][0], U['flash_read'][0], U['flash_at'][0],
                         U['stage'][0], U['loader_after_depack'][0], c, want, a6=True,
                         pre=pre if ddr is not None else None)
    finally:
        A.machine = orig
    if ddr is not None and cap.get('pre') != len(ddr):
        raise SystemExit('the DDR pattern was not written')
    # untouched_below_output compares with the 0xA5 fill; with the pattern it means nothing
    if ddr is not None:
        below = load_below = c['sections'][3][2] - LO
        r['untouched_below_output'] = bytes(cap['m'].uc.mem_read(LO, below)) == ddr[:load_below]
    uc = cap['m'].uc
    r['final_regs'] = {n: '%#010x' % (uc.reg_read(k) & 0xFFFFFFFF) for n, k in
                       (('pc', UC_M68K_REG_PC), ('a0 read', UC_M68K_REG_A0), ('a1 write', UC_M68K_REG_A1),
                        ('d0', UC_M68K_REG_D0), ('d2', UC_M68K_REG_D2))}
    if dump:
        load = c['sections'][3][2]
        with open(dump, 'wb') as fh:
            fh.write(bytes(uc.mem_read(load, len(want))))
        r['dumped_to'] = dump
    return r


def model(stream, stage, load, want, maxsteps=60_000_000, ddr=None):
    """The depacker in place, in Python: one memory for stream and output
    (0xA5 before the run, or `ddr` from 0x40000000).
    -> dict: how it ended, first write over unread input, first wrong byte."""
    size = max(stage + len(stream), load + len(want)) - LO + 0x200000
    mem = bytearray(b'\xa5' * size)
    if ddr is not None:
        mem[:min(size, len(ddr))] = ddr[:size]
    s0 = stage - LO
    mem[s0:s0 + len(stream)] = stream
    st = {'p': s0 + 8, 'tag': 0}
    w0 = w = load - LO
    first = None

    def byte():
        v = mem[st['p']]
        st['p'] += 1
        return v

    def bit():
        st['tag'] = (st['tag'] << 1) & 0x1FF
        if st['tag'] & 0xFF == 0:
            b = byte()
            st['tag'] = (b << 1) | 1
            return b >> 7
        return st['tag'] >> 8

    def gamma():
        v = 1
        while True:
            v = (v << 1) | bit()
            if bit():
                return v
            if v > 0x02000000:
                raise ValueError('gamma overflow')

    last, steps, how = 1, 0, None
    try:
        while True:
            steps += 1
            if steps > maxsteps or w - w0 > len(want) + 0x100000:
                how = 'runaway'
                break
            if bit():
                if first is None and w >= st['p']:
                    first = (w, st['p'])
                mem[w] = byte()
                w += 1
                continue
            g = gamma()
            if g == 2:
                off = last
            else:
                raw = ((g << 8) + byte()) & 0xFFFFFFFF
                if raw == 767:
                    how = 'end marker'
                    break
                off = (raw - 767) & 0xFFFFFFFF
                last = off
            short = 2 * bit() + bit()
            n = short if short else gamma() + 2
            if off > 3328:
                n += 1
            for _ in range(n + 1):
                if first is None and w >= st['p']:
                    first = (w, st['p'])
                if w >= len(mem) or w - off < 0:
                    raise ValueError('ran off memory')
                mem[w] = mem[w - off]
                w += 1
    except (ValueError, IndexError) as e:
        how = '%s (the unit\'s depacker has no such guard)' % e
    out = bytes(mem[w0:w0 + len(want)])
    diff = next((i for i, (x, y) in enumerate(zip(out, want)) if x != y), None)
    return {
        'ended': how,
        'bytes_out': w - w0,
        'stream_bytes_read': st['p'] - s0,
        'first_write_over_unread_input': ('%#x' % (LO + first[0]), 'next unread %#x' % (LO + first[1]))
                                         if first else None,
        'matches': out == want,
        'bytes_differing': sum(1 for x, y in zip(out, want) if x != y),
        'first_difference': '%#x' % (load + diff) if diff is not None else None,
        'stream_end': '%#x' % (stage + len(stream)),
    }


def intended(path, c, stock_img):
    """The candidate's intended main OS: its section 3 depacked out of place,
    checked against elekloader's manifest or the known stock image."""
    img = A.elz.depack_section(A.stored(c, 3))
    h = A.sha(img)
    how = 'section 3 depacked out of place (dt2/elz.py)'
    man = path + '.json'
    if os.path.exists(man):
        with open(man) as fh:
            want = json.load(fh).get('main_sha256')
        if want != h:
            raise SystemExit('%s: depacks to %s, manifest says %s' % (path, h, want))
        how += '; equals the manifest\'s main_sha256 (elekloader\'s linked image)'
    elif h in KNOWN_MAIN:
        how += '; equals the known %s main OS' % KNOWN_MAIN[h]
    elif img == stock_img:
        how += '; equals the stock main OS'
    else:
        how += '; NOT checked against any manifest'
    return img, how


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--stock', required=True, help='the stock .syx whose bootstrap and section 4 run')
    ap.add_argument('--json', help='write every result here')
    ap.add_argument('--parts', default='updater,boot,model', help='comma list of: updater, boot, model')
    ap.add_argument('--ddr', default='fill', choices=('fill', 'memtest', 'both'),
                    help='DDR before section 4\'s loader: the 0xA5 fill (as acceptoracle.py), the '
                         'pattern section 4\'s own DDR test leaves, or a run with each')
    ap.add_argument('--dump', help='directory: save each section 4 loader run\'s output image there')
    ap.add_argument('syx', nargs='+')
    a = ap.parse_args(argv)
    parts = set(a.parts.split(','))

    stock = A.parse(a.stock)
    boot = A.elz.depack_section(A.stored(stock, 2))
    img = A.elz.depack_section(A.stored(stock, 3))
    upd = A.stored(stock, 4)
    print('stock %s\n  sha256 %s' % (a.stock, A.sha(open(a.stock, 'rb').read())))
    print('  bootstrap sha256 %s (%d bytes, version word %#06x)'
          % (A.sha(boot), len(boot), struct.unpack_from('>H', boot, 0x0C)[0]))
    print('  section 4 sha256 %s (%d bytes stored)' % (A.sha(upd), len(upd)))
    print('  main OS   sha256 %s (%d bytes)' % (A.sha(img), len(img)))

    R = A.resolve_boot(boot)
    dep = boot[R['depack'][0] - A.BOOT_BASE:R['depack'][0] - A.BOOT_BASE + R['depack_len'][0]]
    U = A.resolve_upd(upd, dep, boot, R)
    print('\nbootstrap (section 2, file offset 0 = %#x)' % A.BOOT_BASE)
    for k in ('depack', 'depack_len', 'loader', 'loader_after_depack', 'stage', 'flash_at',
              'flash_read', 'lookup', 'boot_call', 'boot_from', 'boot_init', 'boot_flags'):
        v, how = R[k]
        print('  %-20s %-12s %s' % (k, A.fmt(v), how))
    print('\nsection 4 (stored raw, header included, at %#x)' % A.UPD_BASE)
    for k, (v, how) in U.items():
        print('  %-20s %-12s %s' % (k, A.fmt(v), how))
    mt = memtest(upd)
    print('  %-20s %-12s fill loop: %#x-%#x, d0 = %#x, d1 = %#x; d1 += d0, d0 += d1, store d1, d0'
          % ('ddr_test', A.fmt(mt[4]), mt[0], mt[1], mt[2], mt[3]))

    res = {'stock': a.stock,
           'bootstrap': {k: R[k][0] for k in R},
           'section4': {k: v[0] for k, v in U.items()}, 'files': {}}
    load = stock['sections'][3][2]
    stages = (('boot', R['stage'][0]), ('section4', U['stage'][0]))
    cache = {}

    for path in a.syx:
        c = A.parse(path)
        want, how = intended(path, c, img)
        f = {'sha256': A.sha(open(path, 'rb').read()), 'container_len': c['size'],
             'section3_stored': c['sections'][3][1], 'intended_main_sha256': A.sha(want),
             'intended_main_len': len(want), 'intended_from': how}
        print('\n=== %s\n  sha256 %s, container %d bytes, section 3 stored %d bytes'
              % (path, f['sha256'], c['size'], f['section3_stored']))
        print('  intended main OS: %d bytes, sha256 %s (%s)' % (len(want), A.sha(want), how))
        same = {sid: A.stored(c, sid) == A.stored(stock, sid) for sid in sorted(c['sections'])}
        f['stored_same_as_stock'] = same
        print('  stored sections identical to stock: %s' % same)
        if not (same.get(2) and same.get(4)):
            print('  ** sections 2 or 4 differ from stock: the unit would run the candidate\'s own **')
        for name, st_ in stages:
            gap, n = A.inplace_gap(A.stored(c, 3), st_, load)
            room = st_ - load
            f['inplace_gap_%s' % name] = gap
            f['stage_%s' % name] = st_
            print('  in-place margin, %-8s stage %#x -> %#x: %d bytes (%d bytes out; stream ends %#x, '
                  'image ends %#x)%s'
                  % (name, st_, load, gap, n, st_ + f['section3_stored'], load + len(want),
                     '  ** NEGATIVE: the writer overruns unread input **' if gap < 0 else ''))
            f['min_stream_for_%s' % name] = len(want) - room
        if 'model' in parts:
            runs = [(name, st_, 'fill') for name, st_ in stages]
            if a.ddr in ('memtest', 'both'):
                runs.append(('section4', U['stage'][0], 'memtest'))
            for name, st_, mode in runs:
                ddr = None
                if mode == 'memtest':
                    top = max(st_ + f['section3_stored'], load + len(want)) + 0x200000
                    ddr = memtest_bytes(mt[0], mt[1], mt[2], mt[3], top)
                r = model(A.stored(c, 3), st_, load, want, ddr=ddr)
                f['model_%s%s' % (name, '' if mode == 'fill' else '_memtest')] = r
                print('  model, %-8s stage %#x [DDR: %s]: %s, out %d, read %d; first write over unread '
                      'input %s; matches=%s (%d bytes differ, first at %s)'
                      % (name, st_, '0xA5 fill' if mode == 'fill' else 'DDR test pattern', r['ended'],
                         r['bytes_out'], r['stream_bytes_read'], r['first_write_over_unread_input'],
                         r['matches'], r['bytes_differing'], r['first_difference']))
        if 'updater' in parts:
            modes = ('fill', 'memtest') if a.ddr == 'both' else (a.ddr,)
            for mode in modes:
                # section 4 has run its DDR test before its loader: what the
                # test wrote is what DDR holds, up to past the output and stream
                ddr = None
                if mode == 'memtest':
                    if 'mtfull' not in cache:            # the whole tested range, once
                        cache['mtfull'] = memtest_bytes(mt[0], mt[1], mt[2], mt[3], mt[1])
                    ddr = cache['mtfull']
                dump = None
                if a.dump:
                    os.makedirs(a.dump, exist_ok=True)
                    dump = os.path.join(a.dump, '%s.updater-%s.bin' % (os.path.basename(path), mode))
                up = updater_run(upd, U, c, want, ddr, dump)
                key = 'updater_path' if mode == 'fill' else 'updater_path_memtest'
                f[key] = up
                print('  section 4 loader %#x -> main OS [DDR: %s]: entered %s, depacker returned %s, '
                      'output sha256 %s, matches=%s (%d bytes differ, first at %s), %s s%s'
                      % (U['loader'][0], '0xA5 fill' if mode == 'fill' else 'section 4 DDR test pattern',
                         up['entered_main_at'], up['depack_returned'], up['output_sha256'][:16],
                         up['output_matches'], up['output_bytes_differing'], up['first_difference'],
                         up['seconds'], ', ERROR ' + up['error'] if up['error'] else
                         (', stopped at ' + up['stopped_at'] if up['stopped_at'] else '')))
                print('    final registers: %s' % up['final_regs'])
                print('    flash reads: %s' % up['flash_reads'])
        if 'boot' in parts:
            bp = A.boot_path(boot, R, c, want)
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
        res['files'][path] = f

    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(res, fh, indent=1, default=str)
        print('\nwrote %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
