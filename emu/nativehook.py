"""A basic-block budget counter in C, for emu.longrun._FastStepper.

The fast stepper bounds each timer step with a UC_HOOK_BLOCK callback that
counts blocks and stops Unicorn when the step's budget is spent. In Python
that callback costs about 0.4 us a block through the binding's wrappers, and
with the audio clock on (emu/live.py --ssi0-hz) it ran 7.6 million times per
emulated second: about a third of all wall time. This module compiles the
same three lines in C once (with the system C compiler, into a cache
directory) and installs them through Unicorn's own `uc_hook_add`, so the
count costs a few nanoseconds and nothing else changes: same hook type, same
order of registration, same check-before-count rule as the Python version.

If no compiler is found, or DIGIKIT_NATIVE_HOOKS=0, `block_counter()`
returns None and the caller keeps its Python hook.
"""
# pyright: reportMissingImports=false
from __future__ import annotations

import ctypes
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile

SOURCE = r"""
#include <stdint.h>
typedef int (*stop_fn)(void *uc);
struct budget {
    int64_t left;      /* blocks still allowed in this step */
    int64_t blocks;    /* blocks entered in this step */
    stop_fn stop;      /* uc_emu_stop */
};
/* UC_HOOK_BLOCK callback: void (uc_engine*, uint64_t, uint32_t, void*) */
void digikit_block_budget(void *uc, uint64_t address, uint32_t size,
                          struct budget *b)
{
    (void)address; (void)size;
    /* Checked BEFORE counting, like _FastStepper._on_block: the block that
       exhausts the step still runs and is counted, the one after is not. */
    if (b->left <= 0) {
        b->stop(uc);
        return;
    }
    b->blocks += 1;
    b->left -= 1;
}
"""


class Budget(ctypes.Structure):
    _fields_ = [('left', ctypes.c_int64), ('blocks', ctypes.c_int64),
                ('stop', ctypes.c_void_p)]


_LIB = None
_TRIED = False


def _cache_dir():
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(
        os.path.expanduser('~'), '.cache')
    path = os.path.join(base, 'digikit-native')
    try:
        os.makedirs(path, exist_ok=True)
        return path
    except OSError:
        return tempfile.gettempdir()


def _library():
    """-> the compiled CDLL, or None. Built once per source hash."""
    global _LIB, _TRIED
    if _TRIED:
        return _LIB
    _TRIED = True
    if os.environ.get('DIGIKIT_NATIVE_HOOKS', '1') == '0':
        return None
    suffix = '.dylib' if sys.platform == 'darwin' else '.so'
    tag = hashlib.sha256(SOURCE.encode()).hexdigest()[:16]
    path = os.path.join(_cache_dir(), 'blockbudget-%s%s' % (tag, suffix))
    if not os.path.exists(path):
        cc = os.environ.get('CC') or shutil.which('cc') or shutil.which('clang') \
            or shutil.which('gcc')
        if not cc:
            return None
        src = path[:-len(suffix)] + '.c'
        tmp = '%s.%d.tmp' % (path, os.getpid())
        try:
            with open(src, 'w') as fh:
                fh.write(SOURCE)
            subprocess.run([cc, '-O2', '-shared', '-fPIC', '-o', tmp, src],
                           check=True, capture_output=True, timeout=60)
            os.replace(tmp, path)
        except (OSError, subprocess.SubprocessError):
            return None
    try:
        _LIB = ctypes.CDLL(path)
    except OSError:
        _LIB = None
    return _LIB


class NativeBlockCounter:
    """The budget struct and its hook on one Unicorn instance."""

    def __init__(self, uc, lib):
        from unicorn import UC_HOOK_BLOCK
        from unicorn.unicorn_py3 import unicorn as ucmod
        uclib = ucmod.uclib
        self.budget = Budget()
        self.budget.stop = ctypes.cast(uclib.uc_emu_stop, ctypes.c_void_p).value
        fn = ctypes.cast(lib.digikit_block_budget, ctypes.c_void_p)
        handle = ucmod.uc_hook_h()
        # The binding's own prototype and call shape (Uc.__do_hook_add),
        # with our C function and our struct as the user data.
        status = uclib.uc_hook_add(
            uc._uch, ctypes.byref(handle), UC_HOOK_BLOCK, fn,
            ctypes.cast(ctypes.pointer(self.budget), ctypes.c_void_p),
            ctypes.c_uint64(1), ctypes.c_uint64(0))
        if status != 0:
            raise RuntimeError('uc_hook_add failed: %d' % status)
        self.handle = handle.value
        # Keep everything the C side points at alive with the Uc object.
        uc._callbacks[self.handle] = (fn, self.budget)


def block_counter(uc):
    """-> a NativeBlockCounter installed on `uc`, or None (use Python)."""
    lib = _library()
    if lib is None:
        return None
    try:
        return NativeBlockCounter(uc, lib)
    except Exception:            # noqa: BLE001 -- fall back, never fail a run
        return None
