//============================================================================
//  next_ap68040 -- the AP68040 (github.com/apolkosnik/AP68040, the pipelined
//  MC68040, branch ap68040-60mhz + the md_fin fix, as in Minimig-AGA's
//  AP68040 build of 2026-10-14) on tc_machine's CPU transaction port.
//
//  Takes wombat_cpu's place: the transaction port below is the one
//  wombat_bus32 and the service FSM's fast paths consume (wombat_cpu
//  contract).  The MMU, both caches and the table walker are inside the
//  core now and run over the 68040 bus like any other transfer, so the
//  wombat walker port, the retained-line sideband and the store buffer are
//  gone.  wombat_store_buffer stays: RAM and VRAM stores are posted.
//
//  The 68040 bus (MC68040UM section 7), BCLK = PCLK = clk:
//   - a transfer is latched on the clock TS is seen; the address and the
//     attributes stay driven until the last TA (the slave advances A3-A2
//     of a line, wrapping), the write data of each beat from TS (first
//     beat) or from the clock after the previous TA
//   - each beat becomes one transaction here: byte/word/long as SIZ says,
//     a line beat as an aligned long word.  Read data is placed on every
//     byte lane (so SIZ/A1-A0 pick the right one); write data is taken off
//     the lanes A1-A0 select and right-aligned
//   - bus_req is level-held; a read line holds it across its four beats
//     with bus_addr advancing on each ack (wombat contract); writes drop it
//     for a clock between beats (the next beat's data appears then)
//   - berr (the service FSM's pulse) ends the transfer in TEA
//   - interrupt acknowledges (TT = 3, TM = level) end in TA + AVEC (the
//     Turbo Color autovectors every level); any other CPU space access
//     (breakpoint acknowledge) ends in TEA
//   - TCI on reads of the device page $02xxxxxx (the ROM's ITT0/DTT0 make
//     it non-cacheable too; this is the belt, as wombat_cpu's NOCACHE_PAGE)
//
//  DMA coherency (SNOOP = 1).  wombat_cpu dropped the D-cache line of every DMA write
//  (snoop_stb).  Here that is the 68040's own snooper: the line addresses
//  wait in a small queue (consecutive beats of one line coalesce); when it
//  is not empty BG is negated, the core gives the bus up at the end of its
//  transfer, and each line is presented as an alternate master's long-word
//  write with SC = 10 (any hit: invalidate, memory responds -- MC68040UM
//  table 4-4).  TA is given once the snooper negates MI.  snoop_full asks
//  tc_machine to hold DMA beats while the queue cannot take another line.
//
//  SNOOP = 0 leaves the snoop queue empty (DMA coherency is then the
//  drivers').  It was tc_machine's setting until the AP68040 fix for a load
//  answered with a discarded slow-path job's data during a snoop freeze
//  (upstream PR #10): NeXTSTEP read a pushed argument as $1112400C, the
//  wrong-path load's value, and panicked ("illegal instruction").
//
//  ce must be 1 (the NeXT-Color top ties it high, as for wombat_cpu).
//
//  NEXT_A040_TRACE (sim only, verilator/Makefile V_DEFINE): a stall watchdog
//  and the traces used to find the two core bugs above.
//============================================================================

module next_ap68040
#(
	parameter [7:0]  FPU_REVISION = 8'h41,         // FSAVE frames (Turbo: $41)
	parameter        FPU_PIPE     = 0,             // AP68040 area options
	parameter        LOOP_PRED    = 0,
	parameter        EARLY_RES    = 0,
	parameter        SNOOP        = 1,             // present DMA writes to the 68040's snooper
	parameter [7:0]  NOCACHE_PAGE = 8'h02,         // physical [31:24] read with TCI
	parameter [31:0] POST_LO      = 32'h0400_0000, // posted stores [POST_LO, POST_HI):
	parameter [31:0] POST_HI      = 32'h0D00_0000  //   NeXT RAM + VRAM
)
(
	input         clk,
	input         nreset,

	input   [2:0] ipl,            // active low
	input         berr,           // single-cycle, the in-flight transaction

	// transaction bus (physical addresses), wombat_cpu contract
	output        bus_req,
	output        bus_write,
	output        bus_instr,
	output  [1:0] bus_size,       // 0 byte, 1 word, 2 long
	output [31:0] bus_addr,
	output [31:0] bus_wdata,      // right-aligned by size
	output  [2:0] bus_fc,
	input         bus_ack,
	input  [31:0] bus_rdata,      // right-aligned by size

	// DMA write snoop
	input         snoop_stb,
	input  [31:0] snoop_addr,
	output        snoop_full,

	output        nresetout,
	output [31:0] cacr_out,
	output [31:0] vbr_out,
	output [31:0] dbg_pc,
	output        debug_fault,
	output        debug_halted,
	output [255:0] debug_status,
	output [127:0] debug_status2
);

localparam [1:0] SIZ_L = 2'b00, SIZ_B = 2'b01, SIZ_W = 2'b10, SIZ_LINE = 2'b11;

//----------------------------------------------------------------------------
// the processor
//----------------------------------------------------------------------------
wire [31:0] a_o, d_o, vbr, cacr;
wire        d_oe, rw_n, ts_n, tip_n, rsto_n, ciout_n, mi_n;
wire  [1:0] siz, tt;
wire  [2:0] tm;
wire        bb_n_o, bb_oe;
wire        dbg_retire, dbg_halted, cmaint;

reg  [31:0] d_i;
reg         ta_n, tea_n, tci_n, avec_n;
reg         bg_n;

// the alternate master (the snoop presenter below)
reg  [31:0] alt_a;
reg         alt_ts_n;
reg         alt_ta_n;

ap68040 #(
	.FPU_REVISION(FPU_REVISION),
	.FPU_PIPE(FPU_PIPE), .LOOP_PRED(LOOP_PRED), .EARLY_RES(EARLY_RES)
) cpu (
	.clk(clk), .bclk_en(1'b1), .rsti_n(nreset),
	.a_o(a_o), .a_oe(),
	.a_i(alt_a), .ts_n_i(alt_ts_n), .rw_n_i(1'b0), .siz_i(SIZ_L), .tt_i(2'b00),
	.sc(2'b10), .mi_n(mi_n), .ta_n_o(), .ta_oe(),
	.d_i(d_i), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .tln(), .upa(),
	.ciout_n(ciout_n), .lock_n(), .locke_n(),
	.ts_n(ts_n), .tip_n(tip_n),
	.ta_n(ta_n & alt_ta_n), .tea_n(tea_n), .tci_n(tci_n), .tbi_n(1'b1),
	.ipl_n(ipl), .avec_n(avec_n),
	.br_n(), .bg_n(bg_n),
	.bb_n_i(bb_oe ? bb_n_o : 1'b1), .bb_n_o(bb_n_o), .bb_oe(bb_oe),
	.rsto_n(rsto_n), .cdis_n(1'b1), .mdis_n(1'b1), .ipend_n(), .pst(),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .dbg_halted(dbg_halted),
	.dbg_vbr(vbr), .dbg_cacr(cacr), .dbg_cmaint(cmaint)
);

assign nresetout    = rsto_n;
assign cacr_out     = cacr;
assign vbr_out      = vbr;
assign debug_halted = dbg_halted;

//----------------------------------------------------------------------------
// the 68040's transfers -> transactions
//----------------------------------------------------------------------------
localparam [2:0] C_IDLE = 3'd0, C_REQ = 3'd1, C_TA = 3'd2;
reg   [2:0] cst;
reg  [31:0] xa;                // the transfer (TS to its last TA)
reg   [1:0] xsiz;
reg         xrd, xcode;
reg   [2:0] xfc;
reg   [1:0] beat;              // line beats done
reg         xlast;             // the beat whose TA is out ends the transfer

wire        x_line = (xsiz == SIZ_LINE);
wire  [1:0] lw     = xa[3:2] + beat;     // a line's beats wrap from A3-A2

wire        c_req, c_write, c_instr, c_ack;
wire  [1:0] c_size;
wire [31:0] c_addr, c_wdata, c_rdata;
wire  [2:0] c_fc;
wire        c_berr = berr;

assign c_req   = (cst == C_REQ);
assign c_write = !xrd;
assign c_instr = xcode;
assign c_fc    = xfc;
assign c_size  = (xsiz == SIZ_B) ? 2'd0 : (xsiz == SIZ_W) ? 2'd1 : 2'd2;
assign c_addr  = x_line ? {xa[31:4], lw, 2'b00} : xa;
// write data off the lanes (bs: D31-D24 is the byte at A1-A0 = 0)
assign c_wdata = (xsiz == SIZ_B) ? {24'd0, d_o[31 - 8 * xa[1:0] -: 8]} :
                   (xsiz == SIZ_W) ? {16'd0, xa[1] ? d_o[15:0] : d_o[31:16]} : d_o;

// read data onto every lane
wire [31:0] rd_lanes = (xsiz == SIZ_B) ? {4{c_rdata[7:0]}} :
                       (xsiz == SIZ_W) ? {2{c_rdata[15:0]}} : c_rdata;

// TM -> function code (MC68040UM table 7-3; TT = 2 is MOVES: TM is the FC)
function [2:0] tm_fc;
	input [1:0] t;
	input [2:0] m;
	begin
		if (t == 2'd2) tm_fc = m;
		else case (m)
			3'd1, 3'd2, 3'd5, 3'd6: tm_fc = m;
			default:                tm_fc = 3'd5;   // push, table search
		endcase
	end
endfunction

always @(posedge clk) begin
	if (!nreset) begin
		cst    <= C_IDLE;
		ta_n   <= 1'b1;
		tea_n  <= 1'b1;
		tci_n  <= 1'b1;
		avec_n <= 1'b1;
		d_i    <= 32'd0;
		xa     <= 32'd0;
		xsiz   <= SIZ_L;
		xrd    <= 1'b1;
		xcode  <= 1'b0;
		xfc    <= 3'd0;
		beat   <= 2'd0;
		xlast  <= 1'b0;
	end
	else begin
		ta_n   <= 1'b1;
		tea_n  <= 1'b1;
		tci_n  <= 1'b1;
		avec_n <= 1'b1;
		case (cst)
			C_IDLE:
				if (!ts_n) begin
					xa    <= a_o;
					xsiz  <= siz;
					xrd   <= rw_n;
					xcode <= (tt != 2'd2) && (tm == 3'd2 || tm == 3'd6);
					xfc   <= tm_fc(tt, tm);
					beat  <= 2'd0;
					if (tt == 2'd3) begin
						// CPU space: an interrupt acknowledge autovectors,
						// anything else (breakpoint acknowledge) is TEA
						if (rw_n && tm != 3'd0) begin
							d_i    <= 32'd0;
							ta_n   <= 1'b0;
							avec_n <= 1'b0;
						end
						else tea_n <= 1'b0;
						xlast <= 1'b1;
						cst   <= C_TA;
					end
					else cst <= C_REQ;
				end
			C_REQ:
				if (c_berr) begin
					tea_n <= 1'b0;
					xlast <= 1'b1;
					cst   <= C_TA;
				end
				else if (c_ack) begin
					ta_n  <= 1'b0;
					tci_n <= !(xrd && xa[31:24] == NOCACHE_PAGE);
					d_i   <= rd_lanes;
					beat  <= beat + 2'd1;
					xlast <= !x_line || beat == 2'd3;
					// a read line keeps the request up, the next beat's
					// address follows the ack; a write line waits for the
					// next beat's data (the clock after this TA)
					if (xrd && x_line && beat != 2'd3) cst <= C_REQ;
					else cst <= C_TA;
				end
			C_TA:
				// TA/TEA is out this clock; the core drives a write line's
				// next beat on D31-D0 from the next
				cst <= xlast ? C_IDLE : C_REQ;
			default: cst <= C_IDLE;
		endcase
	end
end

// the transaction consumer never sees a line read's next beat before the
// ack of the previous one: c_addr moves with the ack's clock edge, which
// is the "req && !ack" re-accept rule of the wombat contract.

// Stores to RAM and VRAM ([POST_LO, POST_HI), never a bus error) are posted
// through wombat_store_buffer as under wombat_cpu: acknowledged the clock
// after capture, drained in order; everything else waits for its own answer.
wombat_store_buffer #(.ENABLE(1), .POST_LO(POST_LO), .POST_HI(POST_HI)) store_buffer (
	.clk(clk),
	.nreset(nreset),
	.ce(1'b1),
	.buffer_writes(1'b1),

	.s_req(c_req),
	.s_posted(1'b0),
	.s_write(c_write),
	.s_instr(c_instr),
	.s_size(c_size),
	.s_addr(c_addr),
	.s_wdata(c_wdata),
	.s_fc(c_fc),
	.s_ack(c_ack),
	.s_rdata(c_rdata),

	.m_req(bus_req),
	.m_write(bus_write),
	.m_instr(bus_instr),
	.m_size(bus_size),
	.m_addr(bus_addr),
	.m_wdata(bus_wdata),
	.m_fc(bus_fc),
	.m_ack(bus_ack),
	.m_rdata(bus_rdata),
	.m_err(berr),
	.pending()
);

//----------------------------------------------------------------------------
// DMA write snoop: a queue of line addresses, presented as alternate-master
// SC = 10 long-word writes while the core has given the bus up
//----------------------------------------------------------------------------
localparam QD = 8;
reg  [31:4] q_line [0:QD-1];
reg   [2:0] q_wp, q_rp;
reg   [3:0] q_n;
reg  [31:4] q_last;            // the line queued last (coalescing)
reg         q_last_v;

localparam [2:0] N_IDLE = 3'd0, N_REL = 3'd1, N_TS = 3'd2, N_MI = 3'd3,
                 N_TA = 3'd4, N_GAP = 3'd5;
reg   [2:0] nst;

wire        q_pop  = (nst == N_TS);
wire        q_dup  = q_last_v && (q_last == snoop_addr[31:4]);
wire        q_push = (SNOOP != 0) && snoop_stb && !q_dup;
assign snoop_full = (q_n >= QD - 1);

always @(posedge clk) begin
	if (!nreset) begin
		q_wp     <= 0;
		q_rp     <= 0;
		q_n      <= 0;
		q_last_v <= 1'b0;
		q_last   <= 0;
	end
	else begin
		if (q_push && q_n != QD) begin
			q_line[q_wp] <= snoop_addr[31:4];
			q_wp         <= q_wp + 1'd1;
			q_last       <= snoop_addr[31:4];
			q_last_v     <= 1'b1;
		end
		if (q_pop) begin
			q_rp <= q_rp + 1'd1;
			// once presented, a later beat of the same line needs another
			if (q_n == 1 && !q_push) q_last_v <= 1'b0;
		end
		q_n <= q_n + (q_push && q_n != QD) - q_pop;
	end
end

always @(posedge clk) begin
	if (!nreset) begin
		nst      <= N_IDLE;
		bg_n     <= 1'b0;
		alt_ts_n <= 1'b1;
		alt_ta_n <= 1'b1;
		alt_a    <= 32'd0;
	end
	else begin
		alt_ts_n <= 1'b1;
		alt_ta_n <= 1'b1;
		case (nst)
			N_IDLE:
				if (q_n != 0) begin
					bg_n <= 1'b1;       // the core finishes its transfer, then lets go
					nst  <= N_REL;
				end
			N_REL:
				// BG has been negated for a clock: BB free means the core is
				// not the master and cannot become it
				if (bb_n_o && tip_n && cst == C_IDLE && ts_n) nst <= N_TS;
			N_TS: begin
				alt_a    <= {q_line[q_rp], 4'h0};
				alt_ts_n <= 1'b0;
				nst      <= N_MI;
			end
			N_MI:
				// the snooper has looked: MI negated, memory may answer
				if (alt_ts_n && mi_n) begin
					alt_ta_n <= 1'b0;
					nst      <= N_TA;
				end
			N_TA:
				nst <= N_GAP;
			N_GAP:
				if (q_n != 0) nst <= N_TS;
				else begin
					bg_n <= 1'b0;
					nst  <= N_IDLE;
				end
			default: nst <= N_IDLE;
		endcase
	end
end

//----------------------------------------------------------------------------
// debug
//----------------------------------------------------------------------------
`ifdef NEXT_A040_TRACE
// sim-only stall watchdog: no retire for 1M clocks -> dump the bus state and
// the last 16 transfers once
reg [63:0] trc_cyc = 0;
reg [31:0] trc_idle = 0;
reg        trc_done = 0;
reg [31:0] h_a [0:15];
reg [15:0] h_info [0:15];
reg [63:0] h_cyc [0:15];
reg  [3:0] h_wp = 0;
integer    k;
// the last 32 retired PCs and the last 16 front-end redirects
reg [31:0] r_pc  [0:255];
reg [63:0] r_cyc [0:255];
reg  [7:0] r_wp = 0;
reg [31:0] d_pc  [0:63];
reg [63:0] d_cyc [0:63];
reg  [7:0] d_inf [0:63];
reg  [5:0] d_wp = 0;
// one stack line (VA $1113DA60-6F): committed loads/stores, and bus
// transfers whose page offset matches (its PA is not known up front)
reg [31:0] ln_n = 0;
always @(posedge clk) if (nreset && trc_cyc > 64'd1119700000 && ln_n < 3000) begin
	if (cpu.be.wb_commit && (cpu.dmu.m4.r.a[31:5] == 27'h0889ed3 || cpu.dmu.m4.r.a[31:5] == 27'h0889ed4) && (cpu.be.wb_st || cpu.be.wb_dwe)) begin
		ln_n <= ln_n + 1;
		$display("[A040] LN @%0d pc=%h va=%h st=%b st_data=%h dwe=%b d_reg=%0d res=%h", trc_cyc, cpu.be.wb_u.pc,
			cpu.dmu.m4.r.a, cpu.be.wb_st, cpu.be.wb_st_data, cpu.be.wb_dwe, cpu.be.wb_u.d_reg, cpu.be.wb_res);
	end
	if (cpu.be.wb_commit && (cpu.be.wb_u.pc == 32'h04067afa || cpu.be.wb_u.pc == 32'h04067b00 || cpu.be.wb_u.pc == 32'h04067b02 ||
	    cpu.be.wb_u.pc == 32'h04061aac)) begin
		ln_n <= ln_n + 1;
		$display("[A040] CALL @%0d pc=%h va=%h st=%b st_data=%h dwe=%b d_reg=%0d res=%h upd=%b upd_reg=%0d upd_val=%h", trc_cyc, cpu.be.wb_u.pc,
			cpu.dmu.m4.r.a, cpu.be.wb_st, cpu.be.wb_st_data, cpu.be.wb_dwe, cpu.be.wb_u.d_reg, cpu.be.wb_res,
			cpu.be.wb_u.upd_v, cpu.be.wb_u.upd_reg, cpu.be.wb_upd_val);
	end
	if (cpu.sn_inv) $display("[A040] SNINV @%0d pa=%h", trc_cyc, cpu.sn_pa);
	if (!alt_ts_n) $display("[A040] SNOOP-TS @%0d a=%h", trc_cyc, alt_a);
	if (!ts_n && (a_o[11:5] == 7'h53 || a_o[11:5] == 7'h54) && a_o[31:26] == 6'b000001) begin
		ln_n <= ln_n + 1;
		$display("[A040] LNBUS @%0d TS a=%h rw=%b siz=%0d tm=%0d d_o=%h", trc_cyc, a_o, rw_n, siz, tm, d_o);
	end
	if (!ta_n && (xa[11:5] == 7'h53 || xa[11:5] == 7'h54) && xa[31:26] == 6'b000001)
		$display("[A040] LNBUS @%0d   TA rd=%b d_i=%h d_o=%h beat=%0d", trc_cyc, xrd, d_i, d_o, beat);
	if (store_buffer.push && (store_buffer.s_addr[11:5] == 7'h53 || store_buffer.s_addr[11:5] == 7'h54) && store_buffer.s_addr[31:26] == 6'b000001)
		$display("[A040] LNSB @%0d push a=%h d=%h sz=%0d", trc_cyc, store_buffer.s_addr, store_buffer.s_wdata, store_buffer.s_size);
	if (bus_req && bus_ack && (bus_addr[11:5] == 7'h53 || bus_addr[11:5] == 7'h54) && bus_addr[31:26] == 6'b000001)
		$display("[A040] LNM @%0d ack a=%h wr=%b wdata=%h rdata=%h", trc_cyc, bus_addr, bus_write, bus_wdata, bus_rdata);
end

// stack_attach ($040A29F6..$040A2A28): every committed uop
reg [31:0] sa_n = 0;
always @(posedge clk) if (nreset) begin
	if (cpu.be.wb_commit && cpu.be.wb_u.pc >= 32'h040a29f6 && cpu.be.wb_u.pc <= 32'h040a2a28 && sa_n < 2000) begin
		sa_n <= sa_n + 1;
		$display("[A040] SA @%0d pc=%h op=%0d dwe=%b d_reg=%0d res=%h st=%b st_data=%h m4a=%h", trc_cyc, cpu.be.wb_u.pc,
			cpu.be.wb_u.op, cpu.be.wb_dwe, cpu.be.wb_u.d_reg, cpu.be.wb_res, cpu.be.wb_st, cpu.be.wb_st_data, cpu.dmu.m4.r.a);
	end
end

// who stores $1112400C; the context switch's register loads
reg [31:0] sv_n = 0;
always @(posedge clk) if (nreset) begin
	if (cpu.be.wb_commit && cpu.be.wb_st && cpu.be.wb_st_data == 32'h1112400C && sv_n < 50) begin
		sv_n <= sv_n + 1;
		$display("[A040] ST1112 @%0d pc=%h va=%h", trc_cyc, cpu.be.wb_u.pc, cpu.dmu.m4.r.a);
	end
	if (cpu.be.wb_commit && cpu.be.wb_dwe && (cpu.be.wb_u.pc == 32'h04001930 || cpu.be.wb_u.pc == 32'h040018c2 ||
	    cpu.be.wb_u.pc == 32'h040018ca || cpu.be.wb_u.pc == 32'h04001918) && sv_n < 400 && trc_cyc > 64'd1119000000) begin
		sv_n <= sv_n + 1;
		$display("[A040] CTX @%0d pc=%h d_reg=%0d res=%h", trc_cyc, cpu.be.wb_u.pc, cpu.be.wb_u.d_reg, cpu.be.wb_res);
	end
end

// the instruction fill of VA $1112xxxx: its physical page, the data it
// returns, and every later bus transfer to that page
reg        pg_v = 0;
reg [19:0] pg_pa = 0;
reg [31:0] pg_n = 0;
always @(posedge clk) if (nreset) begin
	if (cpu.fetch.st != 3'd2 && cpu.fetch.st != 3'd3 && cpu.fetch.f1_v && 0) ;
	if (cpu.fetch.b_req_r && cpu.fetch.m_la[31:16] == 16'h1112 && !pg_v) begin
		pg_v  <= 1;
		pg_pa <= cpu.fetch.m_pa[31:12];
		$display("[A040] PG fetch VA %h -> PA %h st=%0d @%0d", cpu.fetch.m_la, cpu.fetch.m_pa, cpu.fetch.st, trc_cyc);
	end
	if (!ts_n && pg_v && a_o[31:12] == pg_pa && pg_n < 300) begin
		pg_n <= pg_n + 1;
		$display("[A040] PG ts @%0d a=%h rw=%b siz=%0d tt=%0d tm=%0d d_o=%h", trc_cyc, a_o, rw_n, siz, tt, tm, d_o);
	end
	if (!ta_n && pg_v && xa[31:12] == pg_pa && pg_n < 300)
		$display("[A040] PG ta @%0d beat-addr=%h d_i=%h d_o=%h rd=%b", trc_cyc, c_addr, d_i, d_o, xrd);
end

// exceptions other than interrupts, traps and access faults; and the
// decoder's illegal / line A / line F records
reg [31:0] xc_n = 0, dx_n = 0;
always @(posedge clk) if (nreset) begin
	if (cpu.be.x_go && xc_n < 200 && !(cpu.be.x_vec >= 8'd24 && cpu.be.x_vec < 8'd48) && cpu.be.x_vec != 8'd2) begin
		xc_n <= xc_n + 1;
		$display("[A040] EXC @%0d vec=%0d pc=%h sr=%h", trc_cyc, cpu.be.x_vec, cpu.be.x_pc, cpu.sr);
	end
	if (cpu.be.x_go && cpu.be.x_vec == 8'd4 && xc_n < 2) begin
		for (k = 0; k < 256; k = k + 1)
			$display("[A040]   ret4[-%0d] @%0d pc=%h", 256 - k, r_cyc[(r_wp + k) & 255], r_pc[(r_wp + k) & 255]);
		for (k = 0; k < 64; k = k + 1)
			$display("[A040]   redir4[-%0d] @%0d pc=%h wb=%b d1=%b S=%b", 64 - k, d_cyc[(d_wp + k) & 63], d_pc[(d_wp + k) & 63],
				d_inf[(d_wp + k) & 63][7], d_inf[(d_wp + k) & 63][6], d_inf[(d_wp + k) & 63][5]);
	end
	if (cpu.dec.push && cpu.dec.nrec.exc inside {8'd4, 8'd10, 8'd11} && dx_n < 200 && cpu.dec.nrec.pc[31:20] == 12'h111) begin
		dx_n <= dx_n + 1;
		$display("[A040] DEC-ILL @%0d pc=%h opw=%h ext1=%h ext2=%h exc=%0d S=%b", trc_cyc, cpu.dec.nrec.pc,
			cpu.dec.nrec.opw, cpu.dec.nrec.ext1, cpu.dec.nrec.ext2, cpu.dec.nrec.exc, cpu.sr[13]);
	end
	if (cpu.dec.push && dx_n < 300 && ((cpu.dec.nrec.pc >= 32'h040a2a60 && cpu.dec.nrec.pc < 32'h040a2b00) ||
	    (cpu.dec.nrec.pc >= 32'h040692e0 && cpu.dec.nrec.pc < 32'h04069360))) begin
		dx_n <= dx_n + 1;
		$display("[A040] DEC-AT @%0d pc=%h opw=%h ext1=%h ext2=%h exc=%0d rt=%h win=%h %h %h %h %h %h qcnt=%0d flt=%b", trc_cyc, cpu.dec.nrec.pc,
			cpu.dec.nrec.opw, cpu.dec.nrec.ext1, cpu.dec.nrec.ext2, cpu.dec.nrec.exc, cpu.dec.nrec.rt,
			cpu.win[0], cpu.win[1], cpu.win[2], cpu.win[3], cpu.win[4], cpu.win[5], cpu.qcnt, cpu.win_flt);
	end
end
always @(posedge clk) begin
	if (dbg_retire) begin
		r_pc[r_wp] <= dbg_pc; r_cyc[r_wp] <= trc_cyc; r_wp <= r_wp + 1;
	end
	if (cpu.redir_v || cpu.d_redir_v) begin
		d_pc[d_wp]  <= cpu.redir_v ? cpu.redir_pc : cpu.d_redir_pc;
		d_cyc[d_wp] <= trc_cyc;
		d_inf[d_wp] <= {cpu.redir_v, cpu.d_redir_v, cpu.sr[13], cpu.sr[12], cpu.be.wb_sr_we, 3'd0};
		d_wp <= d_wp + 1;
	end
end
always @(posedge clk) begin
	trc_cyc  <= trc_cyc + 1;
	trc_idle <= dbg_retire ? 0 : trc_idle + 1;
	if (!ts_n) begin
		h_a[h_wp]    <= a_o;
		h_info[h_wp] <= {rw_n, siz, tt, tm, ciout_n, 7'd0};
		h_cyc[h_wp]  <= trc_cyc;
		h_wp         <= h_wp + 1;
	end
	if (nreset && !trc_done && trc_idle == 32'd1000000) begin
		trc_done <= 1;
		$display("[A040] STALL @%0d pc=%h cst=%0d xa=%h xsiz=%0d xrd=%b beat=%0d | c_req=%b c_ack=%b bus_req=%b bus_addr=%h bus_ack=%b berr=%b",
			trc_cyc, dbg_pc, cst, xa, xsiz, xrd, beat, c_req, c_ack, bus_req, bus_addr, bus_ack, berr);
		$display("[A040]   ts_n=%b tip_n=%b bb_n_o=%b bb_oe=%b bg_n=%b mi_n=%b | nst=%0d q_n=%0d alt_ts_n=%b alt_ta_n=%b alt_a=%h | ipl=%b halted=%b",
			ts_n, tip_n, bb_n_o, bb_oe, bg_n, mi_n, nst, q_n, alt_ts_n, alt_ta_n, alt_a, ipl, dbg_halted);
		$display("[A040]   biu.st=%0d biu.owner=%b biu.locked=%b req=%b snoop.st=%0d | sb.count=%0d sb.drain=%b sb.direct=%b",
			cpu.biu.st, cpu.biu.owner, cpu.biu.locked, cpu.biu.req, cpu.snoop.st,
			store_buffer.count, store_buffer.drain_active, store_buffer.direct_active);
		$display("[A040]   fetch st=%0d fpc=%h qpc=%h cnt=%0d odd=%b f1_v=%b busy=%b b_req=%b stale=%b inv_busy=%b m_la=%h m_pa=%h m_s=%b m_dem=%b win_flt=%b win_fdem=%b iw_req=%b iw_done=%b",
			cpu.fetch.st, cpu.fetch.fpc, cpu.fetch.qpc_r, cpu.fetch.cnt, cpu.fetch.odd, cpu.fetch.f1_v, cpu.fetch.busy, cpu.fetch.b_req_r,
			cpu.fetch.stale, cpu.fetch.inv_busy, cpu.fetch.m_la, cpu.fetch.m_pa, cpu.fetch.m_s, cpu.fetch.m_dem, cpu.win_flt, cpu.win_fdem, cpu.iw_req, cpu.iw_done);
		$display("[A040]   rq_n=%0d uq_n=%0d qcnt=%0d | be ag=%b dc1=%b dc2=%b ex=%b wb=%b stopped=%b ipend=%b sr=%h | dmu e_st=%0d tw_lvl=%0d dm_req=%b tw_busy=%b",
			cpu.rq_n, cpu.uq_n, cpu.qcnt, cpu.be.ag_v, cpu.be.dc1_v, cpu.be.dc2_v, cpu.be.ex_v, cpu.be.wb_v, cpu.be.stopped, cpu.be.ipend_r, cpu.sr,
			cpu.dmu.e_st, cpu.dmu.tw_lvl, cpu.dm_req, cpu.tw_busy);
		for (k = 224; k < 256; k = k + 1)
			$display("[A040]   ret[-%0d] @%0d pc=%h", 256 - k, r_cyc[(r_wp + k) & 255], r_pc[(r_wp + k) & 255]);
		for (k = 48; k < 64; k = k + 1)
			$display("[A040]   redir[-%0d] @%0d pc=%h wb=%b d1=%b S=%b M=%b srwe=%b", 64 - k, d_cyc[(d_wp + k) & 63], d_pc[(d_wp + k) & 63],
				d_inf[(d_wp + k) & 63][7], d_inf[(d_wp + k) & 63][6], d_inf[(d_wp + k) & 63][5], d_inf[(d_wp + k) & 63][4], d_inf[(d_wp + k) & 63][3]);
		for (k = 0; k < 16; k = k + 1)
			$display("[A040]   ts[-%0d] @%0d a=%h rw=%b siz=%0d tt=%0d tm=%0d ci_n=%b", 16 - k,
				h_cyc[(h_wp + k) & 15], h_a[(h_wp + k) & 15],
				h_info[(h_wp + k) & 15][15], h_info[(h_wp + k) & 15][14:13], h_info[(h_wp + k) & 15][12:11],
				h_info[(h_wp + k) & 15][10:8], h_info[(h_wp + k) & 15][7]);
	end
end
`endif
reg         flt_seen;
always @(posedge clk)
	if (!nreset) flt_seen <= 1'b0;
	else if (!tea_n) flt_seen <= 1'b1;

assign debug_fault   = flt_seen;
assign debug_status  = {24'd0, 5'd0, cst, 192'd0, dbg_pc};
assign debug_status2 = {64'd0, xa, 30'd0, cmaint, dbg_retire};

endmodule
