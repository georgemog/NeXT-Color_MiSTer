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
//  DMA coherency.  wombat_cpu dropped the D-cache line of every DMA write
//  (snoop_stb).  Here that is the 68040's own snooper: the line addresses
//  wait in a small queue (consecutive beats of one line coalesce); when it
//  is not empty BG is negated, the core gives the bus up at the end of its
//  transfer, and each line is presented as an alternate master's long-word
//  write with SC = 10 (any hit: invalidate, memory responds -- MC68040UM
//  table 4-4).  TA is given once the snooper negates MI.  snoop_full asks
//  tc_machine to hold DMA beats while the queue cannot take another line.
//
//  ce must be 1 (the NeXT-Color top ties it high, as for wombat_cpu).
//============================================================================

module next_ap68040
#(
	parameter [7:0]  FPU_REVISION = 8'h41,         // FSAVE frames (Turbo: $41)
	parameter        FPU_PIPE     = 0,             // AP68040 area options
	parameter        LOOP_PRED    = 0,
	parameter        EARLY_RES    = 0,
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
wire        q_push = snoop_stb && !q_dup;
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
reg         flt_seen;
always @(posedge clk)
	if (!nreset) flt_seen <= 1'b0;
	else if (!tea_n) flt_seen <= 1'b1;

assign debug_fault   = flt_seen;
assign debug_status  = {24'd0, 5'd0, cst, 192'd0, dbg_pc};
assign debug_status2 = {64'd0, xa, 30'd0, cmaint, dbg_retire};

endmodule
