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
| - | PLAY | runs the sequencer with `--ssi0-hz` (see Audio clock); nothing visible on the SYN page / "Cleared 1 sequence!" |
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
fresh project and reading the screen (from the dn2-cfw project,
https://github.com/naturarum/dn2-cfw, `notes/T2.4a-live-control.md`). Command-key shortcuts are left to the Mac.

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
| main screen, as fast as possible, idle skipping on | 1.95x real time (36.5M instr/s, 25% of instructions not skipped); 8.0x with the C block counter (T2.4c, see Speed below) |
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
one command per line: `wait S`, `wait-emu S`, `down KEY`, `up KEY`,
`tap KEY [HOLD_S]`, `midi-in HEX` (bytes to the DIN MIDI in, e.g.
`midi-in 90 3c 64`), `png NAME` (writes DIR/NAME.png, an ASCII .txt and the
LED state as NAME.leds.json/.txt/.png). KEY is a Tk keysym (`space`,
`bracketleft`, `Down`, `F4`), `shift+KEY` for Shift. `wait` and tap holds
are wall-clock; `wait-emu` waits for emulated time, which is what to use
when the emulator is not at real time (the audio clock, `--speed 0`). With
`--speed 0` a `tap` lasts 0.12 s of *wall* time, which can be seconds of
emulated time; keep scripts that type into menus at the default `--speed 1`.

## Audio clock (`--ssi0-hz`)

Without it, PLAY does nothing: the sequencer is paced by the audio clock,
which the plain emulator does not run. With it:

    uv run python -m emu.live SNAPSHOT --syx FIRMWARE.syx --ssi0-hz --ips 150e6

- `--ssi0-hz` runs emu/ssi.py's SSI0/eDMA48/50 model. Bare, it is 96,000
  requests a second: 64 requests fill one RX bank of 32 rows, so one
  eDMA50 major loop (vector 170) per 32-sample block at 48 kHz, 1,500 a
  second, the SHARC's own block rate. The model serves the 64 requests of a
  major loop in one event, so the emulator stops 1,500 times a second, not
  96,000.
- The SHARC is not emulated, so its sync word `0x007fffff` is planted at
  the head of every row of both RX banks (`0x4e6e7100`, `0x4e6e7900` on
  Digitone II 1.12). The vector-170 handler `0x400d55d4` counts 64 aligned
  blocks, then installs the real channel-50 ISR `0x40025386`, which forces
  vector 191, the frame handler. `--no-dsp-sync` leaves the banks alone
  (the handover then never happens).
- emu/intfrc.py delivers the software-forced interrupts the frame handler
  raises: INTFRCH0 bit 12 (vector 108, the sequencer clock `0x400d79c8`),
  which forces bit 25 (vector 121, the step handler `0x400d99de`). They are
  level-sensitive and are taken after every `rte` and at every step
  boundary when the IPL allows.
- `--ips` sets the emulated CPU rate. The default rate (4x 4.68M) is far
  below the unit's 250 MHz MCF54418; at 1,500 frame interrupts a second it
  leaves the UI task no time and the screen stops redrawing. At 150M the
  emulated CPU load is about 20%, and idle skipping makes the spare time
  free.
- Checked: with the warm snapshot's project at 81.0 BPM (shown on screen),
  the sequencer position (`0x42c4f900`, +900,000 per sub-step, 5,400,000
  per step) advances 5.4 steps a second of emulated audio, which is 81 BPM
  in sixteenths; trig LED messages (`0xB9 index value` on UART8) change
  while playing and stop after STOP.
- The SSI clock starts half a period after the timers' clock. Aligned on
  it, every SSI event at `--ips 150e6` fell a few instructions before a
  PIT/DTIM deadline (a snapshot is saved on a deadline and every timer
  period is a whole number of SSI periods); the timer then found the CPU in
  the channel-50 ISR and the tick was dropped, every time: the RTOS tick
  and the UI loop nearly stopped (no LED refresh, 3 main-loop messages in
  2 s). `spin(timers_first=True)` also takes a timer before an audio
  interrupt that is due on the same instruction.
- Cost: about 0.8x real time on an M2 Pro with the clock on, playing or
  not (8x without the clock). See Speed.
- The harness now emulates BITREV and BYTEREV (ISA_C), which Unicorn lacks;
  1.12 executes BITREV at `0x4002aa80` as soon as the sequencer plays.

## DIN MIDI (`emu/midi.py`)

On by default (`--no-midi` turns it off). UART9 at 31,250 baud is the DIN
port on Digitone II 1.12: out by eDMA channel 37 (a 4 KB ring, completion
vector 157) and, for single realtime bytes, by programmed I/O with the
UART9 interrupt (vector 181); in by eDMA channel 36 into a 32 KB ring
(vector 156). The model runs a transfer when it is started and queues its
completion; bytes are logged with emulated time stamps.

    --midi-log FILE    every byte sent: time handed to the UART, time on the
                       wire (31,250 baud, back to back), path; then the
                       parsed messages
    --midi-file F.mid  the messages as a Standard MIDI File, format 0;
                       time 0 = start of the run; start/continue/stop as
                       markers, clock ticks left out (they are in the log)
    --midi-bpm 81      the tempo written into the file, so bars line up
    --midi-port [NAME] a virtual MIDI output ("Digitone II (emulator)"):
                       plug the emulator into a synth or DAW on the Mac
    --midi-in [NAME]   a virtual MIDI input ("Digitone II (emulator) in")
                       whose messages reach the DIN in

The ports need python-rtmidi: `uv run --with python-rtmidi python -m
emu.live ...`. Only virtual ports are created; no device is opened.

Checked on 1.12 (dn2-cfw `notes/T2.4c-midi-leds-speed.md`): with CLOCK SEND
and TRANSPORT SEND switched on in SETTINGS > MIDI CONFIG > SYNC and track
1 set to the MIDI machine (CHAN enabled with FUNC + encoder A push), PLAY
sends `FA`, clock at 80.9 BPM (24 ppq) for the project's 81.0, and
`90 3C 64` on trigs 1, 5, 9, 13 exactly 4.000 sixteenths apart at 81 BPM,
each ended by `90 3C 00` one sixteenth later; STOP sends `FC`. MIDI in:
`FA` and 24 ppq clock start the sequencer on the external clock (6 clocks
a step), `FC` stops it. Through the virtual ports the timing is the
emulator's: at 0.8x real time the notes come 20% slow.

## Panel LEDs (`emu/leds.py`)

The firmware drives the LEDs through the panel MCU on UART8: `B9 index
value` sets LED `index` to palette entry `value`, `B5 index r g b` sets a
palette entry (6 bits a channel). Palette: 0 off, 1 the dim unlit glow,
then 16 levels each of red, green, blue, yellow, cyan, magenta, white and
orange from 2, six specials at 130-135. Which key each LED belongs to comes
from the factory UI test's own tables. The window draws each key's LED as a
coloured strip along its top, and the 8 round LEDs (the pattern page
indicator on the main screen) under the panel. `png` in a script also saves
the LED state. What lights on 1.12: STOP white when stopped, PLAY green
when playing, RECORD red, the active page key white, the selected track's
trig key dim red, the playhead red on the trig keys, triggering tracks
flashing white, all trig keys green while FUNC is held, TEMPO blinking.

## Speed (1.12, warm snapshot, Apple M2 Pro, `--speed 0`)

| | emulated s per wall s |
|---|---|
| main screen, no audio clock | 8.0 (was 1.95) |
| audio clock on, stopped | 0.81 |
| audio clock on, playing | 0.82 (was 0.10) |

Three changes, none of which changes what the guest sees: the fast
stepper's block budget is counted in C (`emu/nativehook.py`, compiled once
with the system `cc` into `~/.cache/digikit-native/`; `DIGIKIT_NATIVE_HOOKS=0`
keeps the Python hook), the SSI model runs a batch of eDMA minors with one
descriptor read (`Ssi0Dma._run_minors`), and the timer/SSI phase fix above
(which costs a little speed back, because the UI now runs). With the clock
on, most of the remaining time is Unicorn running the guest's own frame
work, about 30M instructions per emulated second (the SHARC frame builder
`0x400db28e` and its helpers are two thirds of it).

## Limits

- **No sound.** The DSP is not emulated; emu/dsp.py only answers its ready
  line.
- **PLAY needs `--ssi0-hz`** (see Audio clock), and then runs at about
  0.8x real time on an M2 Pro.
- **Empty +Drive.** The eSDHC model completes commands without data, so
  projects and presets cannot be loaded, and FUNC+YES says "Prj must be
  re-saved!".
- **MIDI is DIN only.** USB MIDI (the firmware's other port, bit 1 of its
  port mask) is not modelled. The UART is ideal: bytes leave at once, no
  framing errors, no 31,250-baud throttling of what the firmware queues
  (the log computes when each byte would be on the wire).
- **LEDs**: the 10-byte bitmap blocks the firmware also sends the panel MCU
  (heads 0x10-0x16) are not decoded.
- The encoder step model is a keyboard convenience, not the feel of the real
  knob; `--encoder-mode raw` is closer to the hardware path.
- Static screens are rendered only when the firmware redraws (once per
  second on the main screen); `emu.live` shows the panel's current buffer at
  start, so a snapshot taken in a menu opens on that menu.
