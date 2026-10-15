# AP68040 in this repository

The pipelined MC68040 by Adam Polkosnik (github.com/apolkosnik/AP68040,
GPL v2 or later). It replaces the older sequential `ap040_core` tree (the
MacQuadra800 / wombat33 copy) and its `wombat_cpu` bundle.

## Where the tree came from

The files are the copy Minimig-AGA's AP68040 build carries (`rtl/ap68040`
on its `ap040-60mhz-racefix` branch, 2026-10-14):

| what | commit |
|---|---|
| AP68040 `ap68040-60mhz` | `f1b0ff8` (2026-10-11) |
| + the multiply/divide `md_fin` fix (`minimig-md-fin`) | `a5279fa` (2026-10-14) |
| + `ap68040.sv` exports `dbg_vbr`, `dbg_cacr`, `dbg_cmaint` | Minimig `9d93ccf` / `68d6000` |

Not copied: Minimig's `ap040_tg68k_compat.v` (the Amiga bus glue) and
`fastram/` (its DDR3 Fast RAM port).

## The NeXT side

`rtl/next_ap68040.sv` puts the core's 68040 bus (BCLK = PCLK = clk_sys,
33 MHz) onto tc_machine's CPU transaction port (the wombat_cpu contract that
wombat_bus32 and the service FSM's fast paths consume), keeps
`rtl/wombat_store_buffer.sv` for posted RAM/VRAM stores, and turns
tc_machine's DMA-write snoop strobes into alternate-master SC = 10 snoops
(the core's own snooper invalidates the cache lines). The MMU's table walks
travel the 68040 bus like any other transfer, so tc_machine's walker port is
tied off.

Area options: `FPU_PIPE = LOOP_PRED = EARLY_RES = 0`, as Minimig's build
(about 22.8K ALMs out of context, against ~25.6K in context for the old
core). `NeXT-Color.qsf` turns register retiming off for the CPU instance and
synthesizes it for area.

## Tests

The core's own regression lives in its repository (`tb/run_tests.sh`, the
cputest corpus). Here: the Verilator full-machine sim (`verilator/`) runs the
v74 ROM to the `NeXT>` prompt.
