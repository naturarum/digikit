# Live keyboard control (emu/live.py)

Play the emulated Digitone II from the computer keyboard, in real time: a
window shows the 128x64 screen and emu/gui.py's clickable control surface, and
every key on the Mac keyboard maps to a key or encoder of the unit. Holding a
key holds the button, so FUNC and TRK chords work as on the hardware.

Measured on Digitone II OS 1.12 (Apple M2 Pro). Other builds should work where
`emu/symbols.py` resolves them; the encoder step model checks its anchor bytes
and falls back to raw counts when they differ.

## Launch

```sh
DT2_SECTIONS=out/sections/dn2-1.12 uv run python -m emu.live SNAPSHOT --syx Digitone_II_OS1_12.syx
```

`SNAPSHOT` should be taken after boot, with its timers saved, so the window
opens on the main screen in about a second. Make one once (about 3 minutes):

```sh
DT2_SECTIONS=out/sections/dn2-1.12 uv run python tools/guirun.py \
  out/snapshots/dn2-1.12/boot400M.snap --syx Digitone_II_OS1_12.syx \
  --limit 800000000 --save-at 800M:out/snapshots/dn2-1.12/live800M.snap
```

`boot400M.snap` works too, but the intro and the +Drive set-up then take
several minutes. `--save PATH` writes the current state when the run ends,
and that snapshot can be launched again.

Options: `--speed 1` real time (default), `0` as fast as possible;
`--encoder-mode step|raw`; `--keymap FILE`; `--dwell-ms`; `--no-idle-skip`;
`--dtim0-hz`; `--scale N`; `--verbose` prints every delivered input as a
`tools/guirun.py --feed` argument.

## Key map

Press `?` in the window for this table; the on-screen keys also show their
shortcut. Keys are named after a US (ANSI) keyboard, but on macOS they are
matched by physical position (the event's virtual key code, `MAC_VK` in
emu/live.py), so on a Swiss, French or German layout the same physical keys
work: `[` is the key right of P (è on Swiss French), Shift + the 1 key turns
encoder A left whatever it types, and TRIG 6 is the key right of T (Z on a
QWERTZ keyboard). `?` (whichever key types it) shows the map. Elsewhere keys
are matched by keysym; Tk 9's character keysyms (`,` for `comma`) are
accepted. `--keymap FILE` remaps (lines of `KEY KIND CODE [LABEL]`, KIND
`button`, `encoder` or `none`, KEY a name from the table, as on a US keyboard).

| Mac key | Digitone II | on 1.12, pressed alone / with FUNC |
|---|---|---|
| Q W E R T Y U I | TRIG 1-8 | select track 1-8 (trig mode TRACKS) |
| A S D F G H J K | TRIG 9-16 | select track 9-16 |
| 1 2 3 4 5 6 7 8 | encoder A-H, one detent right | parameter step |
| Shift + 1-8 | encoder A-H, one detent left | |
| F1-F8 (fn on laptops) | encoder A-H push (hold) | shows the value; hold and turn = press-and-turn |
| 9 / Shift + 9 | LEVEL/DATA right / left | track level; scrolls menus |
| \\ | LEVEL/DATA push | nothing visible |
| O | TRIG (parameter page) | Trig Parameters / QUANTIZE |
| P | SYN (firmware name SRC) | SYN pages / MACHINE SEL |
| [ | FLTR | Filter / kit SETUP |
| ] | AMP | Amplitude / Euclidean |
| L | FX | FX / Delay |
| ; | MOD | MOD / Compressor |
| ' | PAGE | nothing visible / SEQUENCER PAGE SETUP |
| Space | FUNC (hold) | |
| Tab | TRK (hold) | TRK + TRIG selects a track / FUNC + TRK: header icon |
| Return | YES | nothing visible / "Prj must be re-saved!" |
| Delete, Fwd Del, Esc | NO | closes menus / "Prj must be re-saved!" |
| arrow keys | UP DOWN LEFT RIGHT | move cursors; on the main screen Up/Down page the SYN pages / FUNC + Up or Down: TRIG MODE |
| 0 | RECORD | record icon in the header / "Copy Pattern A01" |
| - | PLAY | nothing visible (see Limits) / "Cleared 1 sequence!" |
| = | STOP | nothing visible / nothing visible |
| Z | KEYBOARD | scale name in the header / keyboard set-up (octave, mode, root, scale) |
| X / C | MINUS / PLUS | nothing visible / -1 / +1 on the track box |
| V | ARP | ARPEGGIATOR / arp on |
| B | VOICE | VOICE SETUP / unison on |
| N | STACK | NOTE EDIT |
| M | PRESET | PRESET / KIT menu / "Perform Kit=ON" |
| , | PTN | select pattern / select bank |
| . | SONG | select song / song to edit |
| / | TEMPO | TEMPO / tap tempo |
| ` | SETTINGS | SETTINGS menu / a list 001-008 |
| ? | (host) | show or hide the key map |

The third column comes from pressing each key from the main screen of a
fresh project and reading the screen (out of the El-Fw workspace:
`notes/T2.4a-live-control.md`). Command-key shortcuts are left to the Mac.

Holding an encoder key repeats it; after 8 repeats each repeat turns 2
detents, then 4, then 8. Losing window focus releases every held key.

## How input reaches the firmware

The same path as emu/gui.py: `KeyInput` (key down/up) -> the worker's inbox
-> `emu/panelin.py` bytes on the UART8 receive ring -> the firmware's own
panel driver. The worker runs one timer step at a time (at most about 17 ms
of emulated time), delivers input between steps, and sleeps off any lead over
the wall clock; a key press wakes it at once.

- **Buttons.** A held key is a held bit in the channel mask. Two button
  transitions reach the firmware at least `--dwell-ms` (25 ms) of emulated
  time apart, so a fast tap is still a press then a release. Key up is
  delayed 40 ms and cancelled by a key down of the same key, which absorbs
  auto-repeat on systems that send up/down pairs.
- **Encoders, `--encoder-mode step` (default).** The unit's encoders send
  many counts per turn and the firmware filters them: the shared accumulator
  `0x4011364e` (1.12) swallows rotation after a pause (an idle meter that
  rises to 48 per handler tick and drops by 3 per count, rotation counts only
  below 25), then takes one step per `divisor` counts from the parameter's
  own config, times a velocity. One raw count per key press therefore did
  nothing: eight single counts left LEV1 at 90. In step mode a hook on the
  accumulator clears its state for keyboard detents and passes exactly
  `divisor` counts per detent, so one key press is one step of that
  parameter (its press-and-turn step while the encoder push is held: LEV1
  +1 normally, +4 pushed). Only calls with keyboard detents pending are
  touched.
- **Encoders, `--encoder-mode raw`.** `--raw-counts N` counts per press,
  filtered by the firmware exactly as panel input is.
- **DTIM0.** The panel driver stamps every key and encoder record with DMA
  timer 0's counter, and tap tempo divides by it, but nothing in the main OS
  or the bootstrap programs DTIM0, so in the emulator it read 0. Here it
  counts at `--dtim0-hz` (default 132 MHz, a guess: the bus clock) of
  emulated time.

## Speed and latency (1.12, warm snapshot, Apple M2 Pro)

| | result |
|---|---|
| snapshot load / first frame | 0.7-0.8 s / 0.7 s |
| main screen, as fast as possible, idle skipping on | 1.95x real time (36.5M instr/s, 25% of instructions not skipped) |
| same, idle skipping off | 0.22x real time (4.2M instr/s) |
| real-time pacing | 1.00-1.01x |
| key press to changed screen (28 presses: arrows, page keys, encoders, trigs) | median 52 ms, 22-146 ms wall (emulated median 35 ms) |
| input queued before delivery | median 0.1 ms, max 23 ms |

"Real time" is the timers' clock, 18.72M instructions per emulated second
after the intro (emu/pit.py: about a tenth of the real CPU), as in emu/gui.py.

## Headless use

```sh
uv run python -m emu.live SNAP --syx FW --script keys.txt --out DIR [--report r.json]
uv run python -m emu.live SNAP --syx FW --bench 20 --speed 0
```

No Tk window is created. A script runs the same `KeyInput` the window uses,
one command per line: `wait S`, `down KEY`, `up KEY`, `tap KEY [HOLD_S]`,
`png NAME` (writes DIR/NAME.png and an ASCII .txt). KEY is a Tk keysym
(`space`, `bracketleft`, `Down`, `F4`), `shift+KEY` for Shift. Timing is
wall-clock, in real time.

## Limits

- **No sound.** The DSP is not emulated; emu/dsp.py only answers its ready
  line.
- **PLAY does not run the sequencer.** After PLAY the screen stayed
  byte-identical for 20 s. The audio-rate SSI0/eDMA interrupt chain that
  paces the ColdFire-DSP exchange (emu/ssi.py, opt-in, board clock unknown)
  is not modelled here, which is the likely reason; not checked.
- **Empty +Drive.** The eSDHC model completes commands without data, so
  projects and presets cannot be loaded, and FUNC+YES says "Prj must be
  re-saved!".
- **No LEDs, no MIDI.** Trig LEDs and key lights are not shown; MIDI in and
  out are not connected.
- The encoder step model is a keyboard convenience, not the feel of the real
  knob; `--encoder-mode raw` is closer to the hardware path.
- Static screens are rendered only when the firmware redraws (once per
  second on the main screen); `emu.live` shows the panel's current buffer at
  start, so a snapshot taken in a menu opens on that menu.
