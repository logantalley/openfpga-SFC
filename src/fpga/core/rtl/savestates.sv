module savestates
(
	input reset_n,
	input clk,

	input             save,
	input             load,

	input       [3:0] ram_size,
	input       [7:0] rom_type,

	input             sysclkf_ce,
	input             sysclkr_ce,

	input             romsel_n,

	input      [15:0] rom_q,

	input      [23:0] ca,
	input             cpurd_n,
	input             cpuwr_n,

	input       [7:0] pa,
	input             pard_n,
	input             pawr_n,

	input       [7:0] di,
	output reg  [7:0] ss_do,

	output     [23:0] rom_addr,

	output     [19:0] ext_addr,

	input       [7:0] spc_di,

	input      [63:0] ddr_di,
	output reg [63:0] ddr_do,
	input             ddr_ack,
	output     [16:0] ddr_addr,
	output reg        ddr_we,
	output reg  [7:0] ddr_be,
	output reg        ddr_req,

	output            aram_sel,
	output            dsp_regs_sel,
	output            smp_regs_sel,

	input       [7:0] ppu_di,

	output            bsram_sel,
	input       [7:0] bsram_di,

	output            dspn_regs_sel,
	output            dspn_ram_sel,
	input       [7:0] dspn_di,

	output            gsu_regs_sel,
	input       [7:0] gsu_di,

	input             sa1_active,
	input      [23:0] sa1_a,
	input       [7:0] sa1_di,
	input             sa1_rd_n,
	input             sa1_wr_n,
	input             sa1_sa1_romsel,
	input             sa1_sns_romsel,

	input             vblank_n,

	input       [7:0] cpu_di,           // what the CPU is currently reading (for debug snoop)

	output            ss_do_ovr,
	output            ss_rom_ovr,
	output reg        ss_busy,

	// Debug taps — exposed to core_top for on-screen overlay
	output reg [3:0]  dbg_rti_arms,        // count of rti_sel arms during ss_busy
	output reg [3:0]  dbg_vect_reentry,    // count of NMI/IRQ vector reads while ss_busy=1
	output reg [3:0]  dbg_ddr_writes,      // count of WRITE_DATA state entries
	output reg [3:0]  dbg_save_end_writes, // count of sta SS_END
	output reg [3:0]  dbg_fw_entry,        // count of fetches at PC=$00:$8009 (Save_start)
	output reg [3:0]  dbg_fw_nmidis,       // count of fetches at PC=$00:$802C (STA NMITIMEN)
	output reg [3:0]  dbg_fw_at_8000,      // count of fetches at PC=$00:$8000 (JML opcode)
	output reg [3:0]  dbg_fw_at_8003,      // count of fetches at PC=$00:$8003 (JML last byte)
	output reg [7:0]  dbg_byte_at_8000,    // last byte read at PC=$00:$8000 (should be $5C)
	output reg [7:0]  dbg_byte_at_8001,    // last byte read at PC=$00:$8001 (should be $09)
	output reg [7:0]  dbg_load_byte0,      // first byte the load firmware reads from SSDATA (should be 'S' = $53)
	output reg [7:0]  dbg_load_byte1,      // second byte (should be 'N' = $4E)
	// Phase C load-trigger diagnostics
	output reg [3:0]  dbg_load_en_cnt,     // count of load_en rising edges (= ss_load seen)
	output reg [3:0]  dbg_load_vect_cnt,   // count of vblank NMI/IRQ vector reads while load armed
	output reg [3:0]  dbg_load_busy_cnt,   // count of ss_busy rises during a load
	output reg [15:0] dbg_load_stall_cnt   // count of prefetch stalls (load_buf_valid <= 0 at chunk boundary)
);

reg cpurd_n_old, cpuwr_n_old;
reg pawr_n_old, pard_n_old;
reg save_old, load_old;

always @(posedge clk or negedge reset_n) begin
	if (~reset_n) begin
		cpurd_n_old <= 1'b1;
		cpuwr_n_old <= 1'b1;
		pard_n_old <= 1'b1;
		pawr_n_old <= 1'b1;
		save_old <= 0;
		load_old <= 0;
	end else begin
		cpurd_n_old <= cpurd_n;
		cpuwr_n_old <= cpuwr_n;

		pawr_n_old <= pawr_n;
		pard_n_old <= pard_n;

		save_old <= save;
		load_old <= load;
	end
end

wire cpurd_ce   =  cpurd_n_old & ~cpurd_n;
wire cpurd_ce_n = ~cpurd_n_old &  cpurd_n;
wire cpuwr_ce   =  cpuwr_n_old & ~cpuwr_n;
wire cpuwr_ce_n = ~cpuwr_n_old &  cpuwr_n;

wire pard_ce   =  pard_n_old & ~pard_n;
wire pard_ce_n = ~pard_n_old &  pard_n;
wire pawr_ce   =  pawr_n_old & ~pawr_n;
wire pawr_ce_n = ~pawr_n_old &  pawr_n;

reg save_en;
reg load_en;
reg rd_rti;
reg save_end;
// Arms when ss_busy is first set (low-byte cpurd_ce); clears after the
// high-byte read cycle ends so that a re-entrant NMI while the firmware
// runs is NOT redirected to nmi_vect_addr.
reg ss_in_vect;

wire nmi_vect = ({ca[23:1],1'b0} == 24'h00FFEA);
wire nmi_vect_l = nmi_vect & ~ca[0];
wire nmi_vect_h = nmi_vect &  ca[0];

wire irq_vect = ({ca[23:1],1'b0} == 24'h00FFEE);
wire irq_vect_l = irq_vect & ~ca[0];
wire irq_vect_h = irq_vect &  ca[0];

wire ss_reg_sel = (ca[23:16] == 8'hC0);

reg [19:0] ss_data_addr;
reg [19:0] ss_data_size;
reg [19:0] ss_ddr_addr;
reg ss_data_addr_inc;
wire ss_data_sel = ss_reg_sel & (ca[15:0] == 16'h6000);
wire ss_addr_sel = ss_reg_sel & (ca[15:0] == 16'h6001);
wire ss_ext_addr_sel = ss_reg_sel & (ca[15:0] == 16'h6002);
wire ss_ramsize_sel = ss_reg_sel & (ca[15:0] == 16'h6003);
wire ss_romtype_sel = ss_reg_sel & (ca[15:0] == 16'h6004);
wire ss_end_sel = ss_reg_sel & (ca[15:0] == 16'h600E);
wire ss_status_sel = ss_reg_sel & (ca[15:0] == 16'h600F);

assign dspn_regs_sel = ss_reg_sel & (ca[15:8] == 8'h61);
assign gsu_regs_sel = ss_reg_sel & (ca[15:8] == 8'h62);

wire ppu_sel = (ca[23:16] == 8'hC1) & (ca[15:8] == 8'h21);

wire rti_sel = (ca[15:0] == 16'h8008);  // firmware RTI is at $C08008; match only low 16 bits

reg [19:0] ss_ext_addr;
reg ss_ext_addr_inc;

wire spc_sel = (aram_sel | dsp_regs_sel | smp_regs_sel);
wire spc_read = spc_sel & ~pard_n;

wire bsram_read = bsram_sel & ~pard_n;

wire dspn_ram_read = dspn_ram_sel & ~pard_n;

reg [3:0] ddr_state;
reg [7:0] ddr_data;
reg load_ready;
reg [19:0] ss_wr_base_addr; // Chunk-aligned address latched when WRITE_DATA is armed

// 2026-07-06 REAL FIX: ddr_ack (from the controller, clk_sys) crosses into
// this MCLK domain UNSYNCHRONIZED, and it arrives on the SAME edge as the
// ddr_di data bus (both driven by the controller together).  ddr_busy — and
// thus load_fetch_done / load_fetch_done_r, the capture trigger — was derived
// from the RAW ack, so the trigger could fire on the very edge ddr_di is still
// transitioning.  Double-registering ddr_di ALONE did not help because the
// trigger still fired early (the data regs hadn't propagated the transition
// yet).  Synchronize ddr_ack through a 3-FF synch_3 so the trigger is delayed
// by 3 MCLK stages — strictly MORE than the 2-stage ddr_di_r/ddr_di_r2
// pipeline — guaranteeing ddr_di_r2 holds the fully-settled chunk (all 64
// bits, including byte0) before load_fetch_done can assert.  This is the
// proper matched data+control CDC the handshake needed.
wire ddr_ack_sync;
synch_3 sync_ddr_ack (.i(ddr_ack), .o(ddr_ack_sync), .clk(clk));
wire ddr_busy = ddr_req != ddr_ack_sync;

localparam DDR_IDLE = 4'd0, LOAD_DATA = 4'd1, WRITE_DATA = 4'd2,
			DDR_END = 4'd6;

// ===== Load double-buffer =====
// During DMA-based load, the savestates.bin code reads $C06000 continuously
// without polling ddr_busy between 8-byte chunks.  The SRAM fetch via CDC
// takes ~487 ns, but the next DMA byte arrives in ~372 ns, so ddr_di still
// has the OLD chunk when byte 0 of the next chunk is read.
//
// Fix: buffer the current chunk in load_buf; prefetch the next chunk into
// ddr_di in the background.  Swap at each 8-byte boundary.
//
// Stall protocol: load_buf_valid drives STATUS_BUSY (bit 1 = ~load_buf_valid).
// We clear load_buf_valid at the byte-7 boundary so the firmware's existing
// STATUS_BUSY poll loop stalls until the prefetch finishes, at which point
// load_fetch_done copies ddr_di → load_buf and reasserts load_buf_valid.
// This enforces correctness even when DDR latency exceeds the 8-byte window.
reg [63:0] load_buf;        // Current chunk data being read by CPU
reg        load_buf_valid;  // load_buf contains valid data (cleared to stall)
reg        load_pf_ready;   // Prefetch completed, ddr_di has next chunk
reg [19:0] load_pf_addr;    // Address for next LOAD_DATA during prefetch
reg        prev_ddr_busy_r; // For falling-edge detection
reg        load_fetch_done_r; // load_fetch_done delayed 1 cycle (see fix note)
reg        dbg_ddr_di_cap_seen; // one-shot: captured ddr_di at first load_buf write
reg [7:0]  dbg_ddr_di_or_b0;    // OR of raw ddr_di[7:0]  over the whole load
reg [7:0]  dbg_ddr_di_or_b1;    // OR of raw ddr_di[15:8] over the whole load

// 2026-06-30 ROOT-CAUSE FIX (chunk0 byte0 = 0x00).  MCLK = clk_sys & clk_sys_en
// is a real GATED/DERIVED clock (SNES.sv), so its edges are physically skewed
// from clk_sys by the AND-gate + global-buffer insertion delay.  The 64-bit
// ss_dout→ddr_di bus crosses clk_sys→MCLK with NO synchronizer, so sampling it
// on an MCLK edge can catch the bus mid-propagation: byte0 (the only low byte
// that must transition 0→1 from ss_dout's 64'h0 reset for the 'SNES' header)
// latches its stale reset 0 while already-settled higher bytes latch correctly
// → deterministic byte0=0x00.  All prior "add upstream margin" fixes failed
// because the skew is at the MCLK SAMPLE edge, not a clk_sys-cycle alignment.
// Fix: double-register ddr_di into the MCLK domain and consume the twice-
// registered copy.  ss_dout is held stable by the controller from
// SYS_SERVE_RD_NEXT until the next chunk (many MCLK cycles, verified single
// driver, never cleared between serves), so ddr_di_r2 is always past the
// settling window of any single MCLK edge.
// (* preserve, noprune *): ap_core.qsf enables aggressive physical synthesis
// (register retiming, async-signal pipelining, duplication, WYSIWYG remap) and
// this clk_sys→MCLK crossing is UNCONSTRAINED in the SDC.  Without a
// preservation barrier the optimizer constant-folded/retimed byte0 of this bus
// to 0 (ss_dout has an =64'h0 initializer, its source serve_buffer does not, so
// byte0 'S'=0x53 was treated as the reset constant) → ddr_di[7:0] dead in
// silicon while sim passed.  Preserve these FFs so the lane survives.
(* preserve, noprune *) reg [63:0] ddr_di_r, ddr_di_r2;

wire load_fetch_done = load_en & prev_ddr_busy_r & ~ddr_busy;

// Detect if NMI is being used. Some games do not use NMI during game play.
reg [15:0] nmi_cycle_cnt, nmi_read_sr;
wire ss_use_nmi = |nmi_read_sr;
always @(posedge clk) begin
	if (~reset_n) begin
		nmi_cycle_cnt <= 0;
		nmi_read_sr   <= 0;
	end else if (sysclkf_ce) begin
		nmi_cycle_cnt <= nmi_cycle_cnt + 1'b1;
		if (&nmi_cycle_cnt | (~cpurd_n & nmi_vect_l)) begin
			nmi_read_sr <= { nmi_read_sr[14:0], nmi_vect_l };
			nmi_cycle_cnt <= 0;
		end
	end
end


always @(posedge clk) begin
	if (~reset_n) begin
		ss_busy <= 0;
		save_en <= 0;
		load_en <= 0;
		save_end <= 0;
		load_ready <= 0;
		rd_rti <= 0;
		ss_data_addr <= 0;
		ss_data_addr_inc <= 0;
		ss_ext_addr <= 0;
		ss_ext_addr_inc <= 0;
		ss_wr_base_addr <= 0;
		ss_in_vect <= 0;
		ddr_state <= DDR_IDLE;
		ddr_req <= 0;
		ddr_we <= 0;
		dbg_rti_arms <= 0;
		dbg_vect_reentry <= 0;
		dbg_ddr_writes <= 0;
		dbg_save_end_writes <= 0;
		dbg_fw_entry <= 0;
		dbg_fw_nmidis <= 0;
		dbg_fw_at_8000 <= 0;
		dbg_fw_at_8003 <= 0;
		dbg_byte_at_8000 <= 8'h00;
		dbg_byte_at_8001 <= 8'h00;
		dbg_load_byte0 <= 8'h00;
		dbg_load_byte1 <= 8'h00;
		dbg_ddr_di_cap_seen <= 1'b0;
		dbg_load_en_cnt <= 4'h0;
		dbg_load_vect_cnt <= 4'h0;
		dbg_load_busy_cnt <= 4'h0;
		dbg_load_stall_cnt <= 16'h0;
		load_buf_valid <= 0;
		load_pf_ready <= 0;
		load_pf_addr <= 0;
		prev_ddr_busy_r <= 0;
		ddr_di_r <= 64'h0;
		ddr_di_r2 <= 64'h0;
		dbg_ddr_di_or_b0 <= 8'h00;
		dbg_ddr_di_or_b1 <= 8'h00;
	end else begin
		prev_ddr_busy_r <= ddr_busy;
		// Defer the load_buf capture by one MCLK cycle so ddr_di (the
		// controller's registered ss_dout) is sampled a cycle after the
		// ddr_busy fall.  NOTE: the actual first-chunk word-0 zeroing was a
		// MULTI-BIT CDC SKEW in ss_psram_arbiter (burst_rdata vs rd_ack
		// crossed clk_mem→clk_sys on independent synch_3 chains); that is
		// fixed there (SS_RD_HOLD).  This one-cycle defer is retained as
		// harmless extra margin — ddr_di is held stable by the controller
		// until the next request, so a late sample is always safe.
		load_fetch_done_r <= load_fetch_done;

		// Double-register the unsynchronized ddr_di bus into MCLK (see note at
		// the ddr_di_r declaration).  ddr_di_r2 is the settled copy consumed by
		// the load_buf capture and the ss_do read mux.
		ddr_di_r  <= ddr_di;
		ddr_di_r2 <= ddr_di_r;

		// 2026-07-06 DECISIVE TEST: OR-accumulate the RAW ddr_di low two bytes
		// every MCLK cycle during load, ungated by any trigger.  Five capture-
		// timing fixes left byte0=00; if byte0 EVER arrives nonzero at this
		// module input, this OR-accumulator will show it.  If dbg_ddr_di_or_b0
		// stays 00 across the entire load, byte0 physically never reaches
		// ddr_di[7:0] here → the loss is in the port/wire path core_top.ss_dout
		// → ddr_di[7:0], NOT any timing.  (ss_dout[7:0]=53 is proven by GREEN.)
		if (load_en) begin
			dbg_ddr_di_or_b0 <= dbg_ddr_di_or_b0 | ddr_di[7:0];
			dbg_ddr_di_or_b1 <= dbg_ddr_di_or_b1 | ddr_di[15:8];
		end
		// Publish the OR-accumulators to the overlay outputs (YELLOW/CYAN).
		// 2026-07-06: YELLOW was hardwired to 8'h00 at the SNES.sv level since
		// v60 (dbg_load_byte0 hijack leftover) — every prior YELLOW=00 reading
		// measured that constant, not this accumulator.  Re-connected now.
		// YELLOW = OR of every raw ddr_di[7:0]  seen during load  (want nonzero)
		// CYAN   = OR of every raw ddr_di[15:8] seen during load  (control; =FF)
		dbg_load_byte0   <= dbg_ddr_di_or_b0;
		dbg_byte_at_8000 <= dbg_ddr_di_or_b1;

		if (~(load_en | save_en)) begin
			if (~save_old & save) begin
				save_en <= 1;
			end else if (~load_old & load) begin
				load_en <= 1;
				load_ready <= 1; // APF handles file validity, no header check needed
				load_buf_valid <= 0;
				load_pf_ready <= 0;
				dbg_load_en_cnt <= dbg_load_en_cnt + 4'd1;
			end
		end

		if (cpurd_ce) begin
			if (nmi_vect_l | (~ss_use_nmi & irq_vect_l)) begin // Prefer to use NMI
				// Diagnostic: count vector reads seen while a load is armed
				// (whether or not the hijack condition fully passes).
				if (load_en & load_ready) begin
					dbg_load_vect_cnt <= dbg_load_vect_cnt + 4'd1;
				end
				if (~ss_busy & (save_en | (load_en & load_ready)) & ~vblank_n) begin
					ss_busy    <= 1; // Override NMI/IRQ vector only during vblank
					ss_in_vect <= 1; // Arm two-byte vector override
					if (load_en & load_ready) begin
						dbg_load_busy_cnt <= dbg_load_busy_cnt + 4'd1;
					end
				end
				// Debug: count vector reads that happen while ss_busy is already 1
				// (re-entrant NMI/IRQ — should never happen if firmware disables NMI fast enough)
				if (ss_busy) begin
					dbg_vect_reentry <= dbg_vect_reentry + 4'd1;
				end
			end

			if (ss_busy & rti_sel & ~rd_rti) begin
				rd_rti <= 1;
				dbg_rti_arms <= dbg_rti_arms + 4'd1;
			end

			// Debug: count when CPU fetches at firmware entry points (while ss_busy=1)
			// $00:$8009 = Save_start (first instruction after the jml at $8000)
			// $00:$802C = STA NMITIMEN — once we get here, NMI is about to be disabled
			if (ss_busy & (ca[23:0] == 24'h008009)) begin
				dbg_fw_entry <= dbg_fw_entry + 4'd1;
			end
			if (ss_busy & (ca[23:0] == 24'h00802C)) begin
				dbg_fw_nmidis <= dbg_fw_nmidis + 4'd1;
			end
			if (ss_busy & (ca[23:0] == 24'h008000)) begin
				dbg_fw_at_8000 <= dbg_fw_at_8000 + 4'd1;
			end
			if (ss_busy & (ca[23:0] == 24'h008003)) begin
				dbg_fw_at_8003 <= dbg_fw_at_8003 + 4'd1;
			end

			// 2026-06-26 LOAD HEADER CAPTURE — the decisive read-side check.
			// The firmware's FIRST 4 SSDATA reads during a load are the "SNES"
			// magic written at save time.  If these come back $53 $4E $45 $53
			// ('S','N','E','S'), the firmware is reading the SAVED state
			// correctly on hardware (read path perfect → garbage restore is a
			// WRITE-side / restore problem).  If they're wrong, the hardware
			// read path differs from the proven sim round-trip.
			//   dbg_load_byte0 = byte @ stream addr 0  (want $53 'S')
			//   dbg_load_byte1 = byte @ stream addr 1  (want $4E 'N')
			//   dbg_byte_at_8000 = byte @ stream addr 2 (want $45 'E')
			//   dbg_byte_at_8001 = byte @ stream addr 3 (want $53 'S')
			// 2026-06-30: dbg_load_byte0 / dbg_byte_at_8000 are REPURPOSED to
			// capture ddr_di at the load_buf write instant (see prefetch
			// handler).  Keep byte1/byte3 as the firmware-read view for context.
			if (ss_busy & load_en & ss_data_sel) begin
				if (ss_data_addr == 20'd1) dbg_load_byte1   <= ss_do;
				if (ss_data_addr == 20'd3) dbg_byte_at_8001 <= ss_do;
			end
		end

		// (NMITIMEN-snoop diagnostic removed — confirmed NMITIMEN=$A0 on
		// hardware, bit7 set, NMI re-enables fine.  dbg_byte_at_8000/8001 are
		// now reused by the LOAD HEADER CAPTURE below.)

		if (cpurd_ce_n) begin
			if (rd_rti) begin
				ss_busy    <= 0;
				rd_rti     <= 0;
				load_en    <= 0;
				save_en    <= 0;
				save_end   <= 0;
				ss_in_vect <= 0;
			end
			// Disarm after the high-byte read cycle ends; the override has
			// already been latched by the CPU, and any subsequent NMI must
			// not be re-routed to nmi_vect_addr.
			if (ss_in_vect & (nmi_vect_h | (~ss_use_nmi & irq_vect_h))) begin
				ss_in_vect <= 0;
			end
		end

		if (cpuwr_ce & ss_busy) begin
			if (ss_addr_sel) begin // Reset save state address
				ss_data_addr <= 20'd0;
				if (load_en) begin
					// Request first chunk when address is reset
					load_pf_addr <= 20'd0;
					load_buf_valid <= 0;
					load_pf_ready <= 0;
					ddr_state <= LOAD_DATA;
				end
			end

			if (ss_ext_addr_sel) begin
				ss_ext_addr <= 0;
			end

			if (ss_end_sel) begin // Saving finished
				save_end <= 1;
				ss_data_size <= ss_data_addr;
				dbg_save_end_writes <= dbg_save_end_writes + 4'd1;
				if (ss_data_addr[2:0] != 3'd0) begin
					// Write remaining data first; latch chunk-aligned address now
					// because ss_data_addr may advance before WRITE_DATA fires.
					ss_wr_base_addr <= {ss_data_addr[19:3], 3'b000};
					ddr_state <= WRITE_DATA;
				end
			end
		end

		if (cpuwr_ce | cpurd_ce) begin
			if (ss_data_sel & ss_busy) begin
				ss_data_addr_inc <= 1;
			end
		end

		if (cpuwr_ce_n | cpurd_ce_n) begin
			if (ss_data_addr_inc) begin
				ss_data_addr <= ss_data_addr + 1'b1;
				ss_data_addr_inc <= 0;
				if (cpurd_ce_n & (ss_data_addr[2:0] == 3'd7)) begin
					if (load_en) begin
						// End of 8-byte chunk.  If the prefetch is already done,
						// swap immediately and kick off the next one.  Otherwise
						// clear load_buf_valid so STATUS_BUSY stalls the firmware
						// until load_fetch_done completes the swap below.
						if (load_pf_ready) begin
							load_buf       <= ddr_di_r2;  // settled MCLK copy (CDC fix)
							load_buf_valid <= 1;
							load_pf_ready  <= 0;
							ddr_state      <= LOAD_DATA;
						end else begin
							load_buf_valid <= 0; // stall firmware via STATUS_BUSY
							if (dbg_load_stall_cnt != 16'hFFFF)
								dbg_load_stall_cnt <= dbg_load_stall_cnt + 16'd1;
						end
					end else begin
						// Original path (should not occur — reads are load-only)
						ddr_state <= LOAD_DATA;
					end
				end
			end
		end

		if (pawr_ce | pard_ce) begin
			if (spc_sel | bsram_sel | dspn_ram_sel) begin
				ss_ext_addr_inc <= 1;
			end
		end

		if (pawr_ce_n | pard_ce_n) begin
			if (ss_ext_addr_inc) begin
				ss_ext_addr <= ss_ext_addr + 1'b1;
				ss_ext_addr_inc <= 0;
			end
		end

		if (~cpuwr_n & sysclkf_ce & ss_busy & ss_data_sel) begin // Data write
			if (ss_data_addr[2:0] == 3'd0) begin
				ddr_do[63:8] <= 0; // Clear for possible partial last write
				// Latch the chunk-aligned SRAM address now, before ss_data_addr
				// can advance further.  WRITE_DATA may not fire until the bus is
				// idle, by which time ss_data_addr may point to the next chunk.
				ss_wr_base_addr <= {ss_data_addr[19:3], 3'b000};
			end

			ddr_do[ss_data_addr[2:0]*8 +:8] <= ddr_data;

			if (ss_data_addr[2:0] == 3'd7) begin // 8 bytes written
				ddr_state <= WRITE_DATA;
			end
		end

		ddr_be <= 8'hFF;

		// ddr_we is held stable across the entire ddr_req!=ddr_ack window so
		// the controller (which samples ss_rnw = ~ddr_we one clk_sys cycle
		// after the new ddr_req edge) sees the correct direction.  Without
		// this, a one-cycle pulse of ddr_we=1 disappears before the
		// controller can observe it, and save chunks are never serviced.
		if (ddr_req == ddr_ack) begin
			case(ddr_state)
				LOAD_DATA: begin
					if (load_en) begin
						ss_ddr_addr <= load_pf_addr;
						load_pf_addr <= load_pf_addr + 20'd8;
					end else begin
						ss_ddr_addr <= ss_data_addr;
					end
					ddr_req <= ~ddr_req;
					ddr_we <= 0;
					ddr_state <= DDR_END;
				end
				WRITE_DATA: begin
					ss_ddr_addr <= ss_wr_base_addr;
					ddr_req <= ~ddr_req;
					ddr_we <= 1;
					ddr_state <= DDR_END;
					dbg_ddr_writes <= dbg_ddr_writes + 4'd1;
				end

				DDR_END: begin
					ddr_state <= DDR_IDLE;
				end
			endcase

		end

		// ---- Load prefetch completion handler ----
		// 2026-06-29 FIX: fire on load_fetch_done_r (ONE cycle after the
		// combinational ddr_busy fall) instead of load_fetch_done.  By then the
		// controller's registered ss_dout(=ddr_di) has fully settled, so the
		// load_buf capture no longer races the wide-register update edge that
		// zeroed the first chunk's low word.  ss_dout is held stable by the
		// controller until the next request, so the 1-cycle defer is safe.
		if (load_fetch_done_r) begin
			if (~load_buf_valid) begin
				// First chunk arrived (or byte-7 swap stalled): copy the now-
				// settled MCLK copy (ddr_di_r2) → load_buf, prefetch next.
				load_buf       <= ddr_di_r2;  // settled MCLK copy (CDC fix)
				load_buf_valid <= 1;
				load_pf_ready  <= 0;
				ddr_state      <= LOAD_DATA;
			end else begin
				// Buffer still valid (CPU hasn't reached byte-7 yet) — just
				// mark prefetch ready so the byte-7 handler can swap inline.
				load_pf_ready <= 1;
			end
		end
	end
end

wire [15:0] nmi_vect_addr = save_en ? 16'h8000 : 16'h8004;

wire [7:0] ssr_do;
wire ssr_oe;
savestates_regs ss_regs
(
	.reset_n(reset_n),
	.clk(clk),

	.ss_busy(ss_busy),
	.save_en(save_en),

	.ss_reg_sel(ss_reg_sel),

	.sysclkf_ce(sysclkf_ce),
	.sysclkr_ce(sysclkr_ce),

	.romsel_n(romsel_n),

	.ca(ca),
	.cpurd_ce(cpurd_ce),
	.cpurd_ce_n(cpurd_ce_n),
	.cpuwr_ce(cpuwr_ce),

	.pa(pa),

	.pard_ce(pard_ce),
	.pawr_ce(pawr_ce),

	.di(di),
	.ssr_do(ssr_do),
	.ssr_oe(ssr_oe)
);

wire [ 7:0] map_ss_do;
wire        map_ss_oe;
wire [15:0] map_rom_addr;
wire        map_rom_ovr;
wire        map_active;

savestates_map ss_map
(
	.reset_n(reset_n),
	.clk(clk),

	.ss_busy(ss_busy),
	.save_en(save_en),

	.ss_reg_sel(ss_reg_sel),

	.sysclkf_ce(sysclkf_ce),
	.sysclkr_ce(sysclkr_ce),

	.ca(ca),
	.cpurd_n(cpurd_n),
	.cpuwr_n(cpuwr_n),
	.cpuwr_ce(cpuwr_ce),

	.pa(pa),
	.pard_n(pard_n),
	.pawr_n(pawr_n),

	.di(di),

	.sa1_active(sa1_active),

	.sa1_a(sa1_a),
	.sa1_rd_n(sa1_rd_n),
	.sa1_wr_n(sa1_wr_n),
	.sa1_di(sa1_di),
	.sa1_sa1_romsel(sa1_sa1_romsel),
	.sa1_sns_romsel(sa1_sns_romsel),

	.map_active(map_active),

	.rom_addr(map_rom_addr),
	.rom_ovr(map_rom_ovr),

	.ss_do(map_ss_do),
	.ss_oe(map_ss_oe)
);


// NMI/IRQ vector override spans the two-byte vector read ($00FFEAh/$00FFEBh
// or $00FFEEh/$00FFEFh) that kicks off the savestate firmware.
//
// Why ss_in_vect instead of ~ss_busy:
//   ss_busy is set via non-blocking assignment on the same cpurd_ce that fires
//   for the LOW byte, so it is already 1 by INT_CLKF_CE (when the CPU latches
//   data) several master clocks later.  The gate must therefore remain active
//   for both bytes, controlled by ss_in_vect which is cleared only AFTER the
//   high-byte read cycle ends (cpurd_ce_n).  Using ~ss_busy would form the
//   identity ss_busy & ~ss_busy = 0, making the override permanently dead.
// Combinational early-arm: detect the vector read condition before ss_busy/
// ss_in_vect have updated via NBA.  This ensures the override is asserted on
// the very FIRST master clock that ca presents the vector address — required
// for cores where the CPU latches data on the same posedge that cpurd_ce
// fires (i.e. INT_CLKF_CE coincides with cpurd_ce).
wire vect_addr_match = (nmi_vect_l | nmi_vect_h | (~ss_use_nmi & (irq_vect_l | irq_vect_h)));
wire vect_hijack_now = vect_addr_match
                      & (save_en | (load_en & load_ready))
                      & ~vblank_n
                      & ~rd_rti;  // not in the middle of clearing ss_busy

wire ss_vect_ovr = ((nmi_vect | irq_vect) & ss_in_vect) | vect_hijack_now;

wire ss_oe = ss_data_sel | ss_status_sel | ss_vect_ovr |
			ss_ramsize_sel | ss_romtype_sel | ssr_oe | map_ss_oe |
			ppu_sel | dspn_regs_sel | gsu_regs_sel;

// Combinational ss_do — must be valid in the same cycle that CA is
// presented so the CPU (and especially DMA) reads the correct byte.
// A registered version would lag by one cycle, causing every load byte
// to be the *previous* byte's value (first byte = 0x00).
always @(*) begin
	ss_do = 8'h00;
	if (ss_data_sel) begin
		if (load_en)
			ss_do = load_buf[ss_data_addr[2:0]*8 +:8];
		else
			ss_do = ddr_di[ss_data_addr[2:0]*8 +:8];  // (non-load path; unchanged)
	end
	if (ss_status_sel) begin
		if (load_en)
			// During load, report ~load_buf_valid as the busy bit so that
			// the savestates.bin polling loop (AND #$02, BNE) still works:
			// 0 = buffer ready (proceed), 1 = waiting for first chunk.
			ss_do = { 6'd0, ~load_buf_valid, 1'b0 };
		else
			ss_do = { 6'd0, ddr_busy, save_en };
	end
	if (ss_vect_ovr & (nmi_vect_l | irq_vect_l)) ss_do = nmi_vect_addr[7:0];
	if (ss_vect_ovr & (nmi_vect_h | irq_vect_h)) ss_do = nmi_vect_addr[15:8];
	if (ss_ramsize_sel) ss_do = { 4'd0, ram_size };
	if (ss_romtype_sel) ss_do = rom_type;
	if (ssr_oe) ss_do = ssr_do;
	if (map_ss_oe) ss_do = map_ss_do;
	if (ppu_sel) ss_do = ppu_di;
	if (dspn_regs_sel) ss_do = dspn_di;
	if (gsu_regs_sel) ss_do = gsu_di;
end

always @(*) begin
	// savestate.bin ROM
	rom_addr[23:16] = { 2'b11, 6'b11_1111 };
	rom_addr[15: 0] = { ca[16], ca[14:0] };
	if (map_rom_ovr) begin
		rom_addr[15:0] = map_rom_addr;
	end
end

// Data to DDRAM
always @(*) begin
	ddr_data = di;
	if (spc_read) ddr_data = spc_di;
	if (bsram_read) ddr_data = bsram_di;
	if (dspn_ram_read) ddr_data = dspn_di;
end

// Override DI whenever any savestate selector is active.  For the vect path,
// `ss_oe` already includes `vect_hijack_now` (the combinational early-arm),
// so we no longer gate on registered `ss_busy` — that gate was racing the
// first NMI vector read on cores where INT_CLKF_CE coincides with cpurd_ce.
assign ss_do_ovr = (ss_busy | vect_hijack_now) & ss_oe;
// Exclude NMI/IRQ vector addresses from the ROM redirect (even while
// ss_busy=1) so that:
//   (a) During the initial two-byte vector read: ss_do_ovr is 1 (via
//       ss_in_vect → ss_vect_ovr → ss_oe), so DI comes from SS_DO and
//       the ROM address is irrelevant — but keeping ss_rom_ovr=0 here
//       avoids confusing the mapper with a redirect it doesn't expect.
//   (b) If a re-entrant NMI fires after ss_in_vect has cleared: the real
//       NMI vector is read from ROM rather than garbage from savestates.bin.
assign ss_rom_ovr = map_active ? map_rom_ovr : (ss_busy & ~(nmi_vect | irq_vect));

assign aram_sel = ss_busy & (pa == 8'h84);
assign dsp_regs_sel = ss_busy & (pa == 8'h85);
assign smp_regs_sel = ss_busy & (pa == 8'h86);
assign bsram_sel = ss_busy & (pa == 8'h87);
assign dspn_ram_sel = ss_busy & (pa == 8'h88);
assign ext_addr = ss_ext_addr;

assign ddr_addr = ss_ddr_addr[19:3];

endmodule