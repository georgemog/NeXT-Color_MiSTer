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
- [x] Music Kit scores through the FPGA DSP (2026-10-24, dsp50fix build, live
  `playscore` as root): the 16 Nebula SE scores (`/NEBULA_SE/Audio/Score`) are
  in `/LocalLibrary/Music/Scores`.  CoventryCarol played through (52 s, rc 0);
  FugueCMajor played through (rc 0, two "SynthInstrument: Omitting note"
  messages at 64 s and 66 s: no free synth voice then); Pachelbel ran the
  full 5 min test window, stopped by hand.  User: the DSP works.
- [ ] BachFugue, Twilight, Jungle: on the NeXTSTEP-Apps3 CD only (not on the
  MiSTer or this Mac); try them if that image turns up.  They hung on the old
  ARM DSP.
- [ ] FugueCMajor's "Omitting note": check whether a real NeXT drops the same
  notes (DSP voice limit) or whether the FPGA DSP is short of time or memory.
- [ ] Finish the full NeXTSTEP boot simulation with the `jsr` forwarding fix
  (the unfixed core failed at about 211 s of machine time).
- [x] Falcon_MiSTer licence: Adam Polkosnik knows of and agrees to the use of
  his DSP56001 code (2026-10-24).
- [ ] DSP at 57 MHz: not possible at today's chip usage (95% ALMs); needs the
  CPU's own 57 MHz timing margin improved first.
- [x] Delete the failed test cores from the MiSTer:
  `_Computer/NeXT-Color_20261023dsp57test_ap68040.rbf`,
  `_Computer/NeXT-Color_20261023dsp1x_ap68040.rbf` (deleted 2026-10-24;
  copies in the 2x-fl and dsp1x worktrees' `releases/`).
