// Save State Controller
//
// Bridges between the APF bridge (clk_74a, 32-bit) and the core's
// savestates module (clk_sys, 64-bit) using cross-domain FIFOs.
//
// Load path: Bridge writes 32-bit words → load FIFO → core reads 64-bit
// Save path: Core writes 64-bit → save FIFO → bridge reads 32-bit words
//
// Byte-swaps each 32-bit half for APF big-endian ↔ core little-endian.

module save_state_controller #(
	parameter ADDR_PREFIX = 4'h4,    // Bridge address prefix for save state data
	parameter SS_DATA_SIZE = 524288  // 512 KB: covers WRAM(128K)+VRAM(64K)+ARAM(64K)+regs+BSRAM(up to 256K)
) (
	input wire clk_74a,
	input wire clk_sys,
	input wire reset_n,

	// Bridge interface (clk_74a domain)
	input  wire        bridge_wr,
	input  wire        bridge_rd,
	input  wire [31:0] bridge_addr,
	input  wire [31:0] bridge_wr_data,
	output reg  [31:0] bridge_rd_data,

	// Savestate handshake with core_bridge_cmd (clk_74a domain)
	output wire        savestate_supported,
	output wire [31:0] savestate_addr,
	output wire [31:0] savestate_size,
	output wire [31:0] savestate_maxloadsize,

	input  wire        savestate_start,
	output reg         savestate_start_ack,
	output reg         savestate_start_busy,
	output reg         savestate_start_ok,
	output reg         savestate_start_err,

	input  wire        savestate_load,
	output reg         savestate_load_ack,
	output reg         savestate_load_busy,
	output reg         savestate_load_ok,
	output reg         savestate_load_err,

	// Core-side interface (clk_sys domain)
	output wire        ss_save,       // Save trigger (active high, level)
	output wire        ss_load,       // Load trigger (active high, level)

	// FIFO handshake with savestates module (clk_sys domain)
	input  wire [63:0] ss_dout,       // 64-bit data from core (save)
	output reg  [63:0] ss_din,        // 64-bit data to core (load)
	input  wire        ss_req,        // Toggle: core requests FIFO operation
	output reg         ss_ack,        // Toggle: controller acknowledges
	input  wire        ss_we,         // 1=save (write), 0=load (read)

	// Core status (clk_sys domain)
	input  wire        ss_busy        // Active while core is processing
);

// ========================================================================
// Constants
// ========================================================================

assign savestate_supported = 1'b1;
assign savestate_addr      = {ADDR_PREFIX, 28'h0000000};
assign savestate_size      = SS_DATA_SIZE;
assign savestate_maxloadsize = SS_DATA_SIZE;

wire bridge_ss_sel = (bridge_addr[31:28] == ADDR_PREFIX);

// ========================================================================
// Cross-domain synchronizers
// ========================================================================

// Sync save/load triggers: clk_74a → clk_sys
reg save_trigger_74a, load_trigger_74a;
wire save_trigger_sys, load_trigger_sys;

synch_3 save_trig_sync (save_trigger_74a, save_trigger_sys, clk_sys);
synch_3 load_trig_sync (load_trigger_74a, load_trigger_sys, clk_sys);

// Sync ss_busy: clk_sys → clk_74a
wire ss_busy_74a;
synch_3 busy_sync (ss_busy, ss_busy_74a, clk_74a);

assign ss_save = save_trigger_sys;
assign ss_load = load_trigger_sys;

// ========================================================================
// Bridge-side state machine (clk_74a domain)
// ========================================================================

localparam [2:0] BR_IDLE       = 3'd0,
                 BR_SAVE_WAIT  = 3'd1,
                 BR_SAVE_DONE  = 3'd2,
                 BR_LOAD_FILL  = 3'd3,
                 BR_LOAD_WAIT  = 3'd4,
                 BR_LOAD_DONE  = 3'd5;

reg [2:0] br_state;
reg ss_busy_74a_prev;
reg save_started, load_started;

// Load FIFO: bridge writes 32-bit (clk_74a), core reads 32-bit (clk_sys)
reg        load_fifo_wrreq;
reg [31:0] load_fifo_data;
wire       load_fifo_wrfull;
wire       load_fifo_rdempty;
reg        load_fifo_rdreq;
wire [31:0] load_fifo_q;
reg        load_fifo_aclr;

// Save FIFO: core writes 32-bit (clk_sys), bridge reads 32-bit (clk_74a)
reg        save_fifo_wrreq;
reg [31:0] save_fifo_data;
wire       save_fifo_wrfull;
wire       save_fifo_rdempty;
reg        save_fifo_rdreq;
wire [31:0] save_fifo_q;
reg        save_fifo_aclr;

// Byte-swap function: reverse bytes in a 32-bit word
function [31:0] byte_swap;
	input [31:0] d;
	byte_swap = {d[7:0], d[15:8], d[23:16], d[31:24]};
endfunction

always @(posedge clk_74a or negedge reset_n) begin
	if (~reset_n) begin
		br_state <= BR_IDLE;
		save_trigger_74a <= 0;
		load_trigger_74a <= 0;
		savestate_start_ack <= 0;
		savestate_start_busy <= 0;
		savestate_start_ok <= 0;
		savestate_start_err <= 0;
		savestate_load_ack <= 0;
		savestate_load_busy <= 0;
		savestate_load_ok <= 0;
		savestate_load_err <= 0;
		load_fifo_wrreq <= 0;
		save_fifo_rdreq <= 0;
		load_fifo_aclr <= 1;
		save_fifo_aclr <= 1;
		ss_busy_74a_prev <= 0;
		save_started <= 0;
		load_started <= 0;
	end else begin
		load_fifo_wrreq <= 0;
		save_fifo_rdreq <= 0;
		load_fifo_aclr <= 0;
		save_fifo_aclr <= 0;
		ss_busy_74a_prev <= ss_busy_74a;

		case (br_state)
			BR_IDLE: begin
				// Detect save request from APF
				if (savestate_start && !savestate_start_ack) begin
					savestate_start_ack <= 1;
					savestate_start_busy <= 1;
					savestate_start_ok <= 0;
					savestate_start_err <= 0;
					save_fifo_aclr <= 1;
					save_trigger_74a <= 1;
					save_started <= 0;
					br_state <= BR_SAVE_WAIT;
				end
				// Detect load request from APF
				else if (savestate_load && !savestate_load_ack) begin
					savestate_load_ack <= 1;
					savestate_load_busy <= 1;
					savestate_load_ok <= 0;
					savestate_load_err <= 0;
					load_fifo_aclr <= 1;
					load_trigger_74a <= 1;
					load_started <= 0;
					br_state <= BR_LOAD_FILL;
				end
			end

			// ---- Save flow ----
			BR_SAVE_WAIT: begin
				// Detect core entering busy (save state process started)
				if (ss_busy_74a && !ss_busy_74a_prev) begin
					save_started <= 1;
				end
				// Detect core leaving busy (save state process complete)
				if (save_started && !ss_busy_74a && ss_busy_74a_prev) begin
					save_trigger_74a <= 0;
					br_state <= BR_SAVE_DONE;
				end
				// Allow bridge reads from save FIFO during save
			end

			BR_SAVE_DONE: begin
				savestate_start_busy <= 0;
				savestate_start_ok <= 1;
				savestate_start_ack <= 0;
				br_state <= BR_IDLE;
			end

			// ---- Load flow ----
			BR_LOAD_FILL: begin
				// Bridge writes fill the load FIFO
				// Transition to wait when load trigger causes core to go busy
				if (ss_busy_74a && !ss_busy_74a_prev) begin
					load_started <= 1;
				end
				if (load_started) begin
					br_state <= BR_LOAD_WAIT;
				end
			end

			BR_LOAD_WAIT: begin
				// Wait for core to finish processing load
				if (!ss_busy_74a && ss_busy_74a_prev) begin
					load_trigger_74a <= 0;
					br_state <= BR_LOAD_DONE;
				end
			end

			BR_LOAD_DONE: begin
				savestate_load_busy <= 0;
				savestate_load_ok <= 1;
				savestate_load_ack <= 0;
				br_state <= BR_IDLE;
			end
		endcase

		// Bridge write → load FIFO (byte-swapped)
		if (bridge_wr && bridge_ss_sel && !load_fifo_wrfull) begin
			load_fifo_wrreq <= 1;
			load_fifo_data <= byte_swap(bridge_wr_data);
		end

		// Bridge read → save FIFO (byte-swapped)
		if (bridge_rd && bridge_ss_sel && !save_fifo_rdempty) begin
			save_fifo_rdreq <= 1;
		end
	end
end

// Bridge read data: byte-swap from save FIFO
always @(posedge clk_74a) begin
	if (save_fifo_rdreq) begin
		bridge_rd_data <= byte_swap(save_fifo_q);
	end else begin
		bridge_rd_data <= 32'h0;
	end
end

// ========================================================================
// Core-side FIFO interface (clk_sys domain)
// ========================================================================

// Toggle handshake: detect when ss_req changes (core requests operation)
reg ss_req_prev;
reg [2:0] core_state;

localparam [2:0] CORE_IDLE    = 3'd0,
                 CORE_RD_REQ  = 3'd1,
                 CORE_RD_WAIT = 3'd2,
                 CORE_RD_LATCH = 3'd3,
                 CORE_WR_LO   = 3'd4,
                 CORE_WR_HI   = 3'd5,
                 CORE_DONE    = 3'd6;

reg        core_load_rdreq;
reg        core_save_wrreq;
reg [31:0] core_save_data;
reg        core_rd_half; // 0=reading low 32 bits, 1=reading high 32 bits

always @(posedge clk_sys or negedge reset_n) begin
	if (~reset_n) begin
		ss_ack <= 0;
		ss_din <= 0;
		ss_req_prev <= 0;
		core_state <= CORE_IDLE;
		core_load_rdreq <= 0;
		core_save_wrreq <= 0;
		core_save_data <= 0;
		core_rd_half <= 0;
	end else begin
		core_load_rdreq <= 0;
		core_save_wrreq <= 0;

		case (core_state)
			CORE_IDLE: begin
				// Detect toggle on ss_req
				if (ss_req != ss_req_prev) begin
					if (!ss_we) begin
						// Load: read two 32-bit words from load FIFO → 64-bit
						if (!load_fifo_rdempty) begin
							core_load_rdreq <= 1;
							core_rd_half <= 0;
							core_state <= CORE_RD_WAIT;
						end
						// else: FIFO empty, stay in IDLE and retry next cycle
						// (ss_req_prev won't update to match ss_req until next iteration)
					end else begin
						// Save: write 64-bit as two 32-bit words to save FIFO
						if (!save_fifo_wrfull) begin
							core_save_data <= ss_dout[31:0]; // Low 32 bits first
							core_save_wrreq <= 1;
							core_state <= CORE_WR_LO;
						end
						// else: FIFO full, stay in IDLE and retry next cycle
					end
				end
			end

			// ---- Load (read from FIFO) ----
			CORE_RD_WAIT: begin
				// Wait one cycle for FIFO output to be valid
				core_state <= CORE_RD_LATCH;
			end

			CORE_RD_LATCH: begin
				if (!core_rd_half) begin
					// Latch low 32 bits
					ss_din[31:0] <= load_fifo_q;
					core_rd_half <= 1;
					// Request high 32 bits (wait if FIFO is empty)
					if (!load_fifo_rdempty) begin
						core_load_rdreq <= 1;
						core_state <= CORE_RD_WAIT;
					end
					// else: stay in CORE_RD_LATCH, retry reading high half next cycle
				end else begin
					// Latch high 32 bits
					ss_din[63:32] <= load_fifo_q;
					core_state <= CORE_DONE;
				end
			end

			// ---- Save (write to FIFO) ----
			CORE_WR_LO: begin
				// Low 32 bits written, now write high 32 bits (wait if FIFO full)
				if (!save_fifo_wrfull) begin
					core_save_data <= ss_dout[63:32];
					core_save_wrreq <= 1;
					core_state <= CORE_WR_HI;
				end
				// else: stay in CORE_WR_LO, retry next cycle
			end

			CORE_WR_HI: begin
				core_state <= CORE_DONE;
			end

			// ---- Complete ----
			CORE_DONE: begin
				ss_ack <= ss_req; // Match ss_req to acknowledge
				ss_req_prev <= ss_req; // Update after acknowledgment
				core_state <= CORE_IDLE;
			end
		endcase
	end
end

// ========================================================================
// FIFO instances (Altera dcfifo for cross-clock-domain)
// ========================================================================

// Load FIFO: clk_74a (write) → clk_sys (read), 32-bit, 4096 deep
dcfifo load_fifo (
	.data    (load_fifo_data),
	.wrclk   (clk_74a),
	.wrreq   (load_fifo_wrreq),
	.wrfull  (load_fifo_wrfull),
	.rdclk   (clk_sys),
	.rdreq   (core_load_rdreq),
	.q       (load_fifo_q),
	.rdempty (load_fifo_rdempty),
	.aclr    (load_fifo_aclr),
	.eccstatus(),
	.rdfull  (),
	.rdusedw (),
	.wrempty (),
	.wrusedw ()
);
defparam
	load_fifo.intended_device_family = "Cyclone V",
	load_fifo.lpm_numwords = 4096,
	load_fifo.lpm_showahead = "OFF",
	load_fifo.lpm_type = "dcfifo",
	load_fifo.lpm_width = 32,
	load_fifo.lpm_widthu = 12,
	load_fifo.overflow_checking = "ON",
	load_fifo.rdsync_delaypipe = 5,
	load_fifo.underflow_checking = "ON",
	load_fifo.use_eab = "ON",
	load_fifo.wrsync_delaypipe = 5;

// Save FIFO: clk_sys (write) → clk_74a (read), 32-bit, 64 deep
// 64 entries = 32 × 64-bit words; provides headroom when bridge reads are bursty.
dcfifo save_fifo (
	.data    (core_save_data),
	.wrclk   (clk_sys),
	.wrreq   (core_save_wrreq),
	.wrfull  (save_fifo_wrfull),
	.rdclk   (clk_74a),
	.rdreq   (save_fifo_rdreq),
	.q       (save_fifo_q),
	.rdempty (save_fifo_rdempty),
	.aclr    (save_fifo_aclr),
	.eccstatus(),
	.rdfull  (),
	.rdusedw (),
	.wrempty (),
	.wrusedw ()
);
defparam
	save_fifo.intended_device_family = "Cyclone V",
	save_fifo.lpm_numwords = 64,
	save_fifo.lpm_showahead = "OFF",
	save_fifo.lpm_type = "dcfifo",
	save_fifo.lpm_width = 32,
	save_fifo.lpm_widthu = 6,
	save_fifo.overflow_checking = "ON",
	save_fifo.rdsync_delaypipe = 5,
	save_fifo.underflow_checking = "ON",
	save_fifo.use_eab = "ON",
	save_fifo.wrsync_delaypipe = 5;

endmodule
