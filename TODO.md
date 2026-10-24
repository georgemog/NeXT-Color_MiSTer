# To do

## Audio

- [ ] **OSD audio gain: 0 / +6 / +12 dB.** NeXT's sounds are recorded quiet
  (system sounds peak -8 to -14 dBFS, RMS about -23 to -26 dBFS; measured
  2026-10-24), so over HDMI they sound very low next to other cores. The
  output chain is unity gain and correct (a -6 dBFS 1 kHz tone played at
  normal level; NeXTSTEP volume 43/43, MiSTer Volume.dat 0). Add the gain at
  the end of `rtl/next_sound_output.sv`, after the volume stage, with
  saturation (no wrap-around), plus one OSD option in `NeXT-Color.sv`.

## DSP56001 (FPGA)

- [ ] Timing: `dsp56k-50` seed 4 checked at the slow 100 C corner only; its
  only miss is the framework HDMI scaler clock (-0.093 ns). Check all four
  corners, or take a fully clean seed from the sweep (seeds 1, 5, 6, 7).
- [ ] Heavier Music Kit scores (BachFugue, Twilight, Jungle, CoventryCarol):
  not on the disk; copy them over and run `playscore` (as root over telnet).
- [ ] Finish the full NeXTSTEP boot simulation with the `jsr` forwarding fix
  (the unfixed core failed at about 211 s of machine time).
- [ ] Confirm the licence of apolkosnik/Falcon_MiSTer (no LICENSE file; the
  DSP code follows Hatari, GPL-2) with its author before publishing.
- [ ] DSP at 57 MHz: not possible at today's chip usage (95% ALMs); needs the
  CPU's own 57 MHz timing margin improved first.
- [ ] Delete the failed test cores from the MiSTer when done with them:
  `_Computer/NeXT-Color_20261023dsp57test_ap68040.rbf`,
  `_Computer/NeXT-Color_20261023dsp1x_ap68040.rbf`.
