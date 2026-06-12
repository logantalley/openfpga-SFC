//
// sdram.v
//
// sdram controller implementation
// Copyright (c) 2018 Sorgelig
// 
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 
//

module sdram
(

	// interface to the MT48LC16M16 chip
	inout      [15:0] SDRAM_DQ,   // 16 bit bidirectional data bus
	output reg [12:0] SDRAM_A,    // 13 bit multiplexed address bus
	output reg        SDRAM_DQML, // byte mask
	output reg        SDRAM_DQMH, // byte mask
	output reg  [1:0] SDRAM_BA,   // two banks
	output            SDRAM_nCS,  // a single chip select
	output reg        SDRAM_nWE,  // write enable
	output reg        SDRAM_nRAS, // row address select
	output reg        SDRAM_nCAS, // columns address select
	output            SDRAM_CLK,
	output            SDRAM_CKE,

	// cpu/chipset interface
	input             init,			// init signal after FPGA config to initialize RAM
	input             clk,			// sdram is accessed at up to 128MHz

	input      [24:0] addr,
	input             rd,
	input             wr,
	input             word,
	input      [15:0] din,
	output     [15:0] dout,
	output reg        busy = 1'b0,

	// Diagnostic v60: chip-boundary capture of the FIRST word-2/3-class
	// write in the staging region (v59 lesson: a 17-bit class counter
	// wraps to exactly 0 at the exact 131072 writes a full 512KB load
	// produces, so GREEN/YELLOW=$00 was ambiguous).
	//   dbg_real_writes = first_w2_data[15:0] — the din captured at the
	//     first staging write with addr[2]=1 (chunk word 2 or 3).  For
	//     Super Metroid chunk 0 word 2 expect $532D ("-S" → lo=$2D hi=$53).
	//   dbg_w2_info[15:8] = cnt_wr_hi[17:10] — 18-bit class-2/3 counter,
	//     wraps at 262144 so a full load reads $80 unambiguously.
	//   dbg_w2_info[7:0]  = first_w2_addr[7:0] — byte addr low bits of that
	//     first write (expect $04 = BASE+4).
	output wire [15:0] dbg_real_writes,
	output wire [15:0] dbg_w2_info
);

assign SDRAM_nCS = 0;
assign SDRAM_CKE = 1;
assign {SDRAM_DQMH,SDRAM_DQML} = SDRAM_A[12:11];

localparam RASCAS_DELAY   = 3'd1; // tRCD=20ns -> 2 cycles@85MHz
localparam BURST_LENGTH   = 3'd0; // 0=1, 1=2, 2=4, 3=8, 7=full page
localparam ACCESS_TYPE    = 1'd0; // 0=sequential, 1=interleaved
localparam CAS_LATENCY    = 3'd2; // 2/3 allowed
localparam OP_MODE        = 2'd0; // only 0 (standard operation) allowed
localparam NO_WRITE_BURST = 1'd1; // 0=write burst enabled, 1=only single access write

localparam MODE = { 3'b000, NO_WRITE_BURST, OP_MODE, CAS_LATENCY, ACCESS_TYPE, BURST_LENGTH}; 

localparam STATE_IDLE  = 3'd0;             // state to check the requests
localparam STATE_START = STATE_IDLE+1'd1;  // state in which a new command is started
localparam STATE_CONT  = STATE_START+RASCAS_DELAY;
localparam STATE_READY = STATE_CONT+CAS_LATENCY+1'd1;
localparam STATE_LAST  = STATE_READY;      // last state in cycle

// (declarations hoisted above first use so ModelSim's vlog accepts the
// file; Quartus tolerated the original forward references.  No functional
// change.)
localparam MODE_NORMAL = 2'b00;
localparam MODE_RESET  = 2'b01;
localparam MODE_LDM    = 2'b10;
localparam MODE_PRE    = 2'b11;

// Explicit power-up values matching Quartus's default (registers come up
// 0 on Cyclone V).  Without these, simulation starts them at X and the
// state machine deadlocks — hardware behavior is unchanged.
reg [1:0] mode = 2'b00;
reg [4:0] reset = '1;

reg  [2:0] state = 3'd0;
reg [17:0] cnt_wr_lo = 18'd0;
reg [17:0] cnt_wr_hi = 18'd0;
reg [15:0] first_w2_data    = 16'h0000;
reg  [7:0] first_w2_addr_lo = 8'h00;
reg        first_w2_seen    = 1'b0;
assign dbg_real_writes = first_w2_data;
assign dbg_w2_info     = {cnt_wr_hi[17:10], first_w2_addr_lo};
reg [24:0] a;
reg [15:0] data;
reg        we = 1'b0;
reg        ds;
reg        ram_req=0;
wire       ram_req_test = (we || (a[24:1] != addr[24:1]));
reg [15:0] last_data;

// access manager
always @(posedge clk) begin
	reg old_ref;
	reg old_rd,old_wr;

	old_rd <= old_rd & rd;
	old_wr <= old_wr & wr;

	if(state == STATE_IDLE && mode == MODE_NORMAL) begin
		if((~old_rd & rd) | (~old_wr & wr)) begin
			old_rd <= rd;
			old_wr <= wr;
			we <= wr;
			ds <= word;
			busy <= 1;
			state <= STATE_START;
		end
	end
	
	if(state == STATE_START && busy) begin
		a <= addr;
		data <= word ? din : {din[7:0],din[7:0]};
		ram_req <= ram_req_test;
		// Diagnostic v60: count real writes by word-class (addr[2:1]) and
		// capture data+address of the FIRST class-2/3 write at the chip
		// boundary.  ram_req_test=true AND we=1 = CMD_WRITE fires at
		// STATE_CONT.  Gated to the savestate staging region (byte addr
		// $800000..$FFFFFF, addr[24:23]=01) so cart-download writes don't
		// pollute the capture.
		if (ram_req_test && we && addr[24:23] == 2'b01) begin
			if (addr[2]) begin
				cnt_wr_hi <= cnt_wr_hi + 18'd1;   // word 2 or 3 of a chunk
				if (!first_w2_seen) begin
					first_w2_seen    <= 1'b1;
					first_w2_data    <= word ? din : {din[7:0],din[7:0]};
					first_w2_addr_lo <= addr[7:0];
				end
			end else begin
				cnt_wr_lo <= cnt_wr_lo + 18'd1;   // word 0 or 1 of a chunk
			end
		end
	end

	if(state == STATE_READY && busy) begin
		ram_req <= 0;
		we <= 0;
		busy <= 0;
		if(ram_req) begin
			if(we) begin
				a <= '1;
			end
			else begin
				last_data <= SDRAM_DQ;
			end
		end
	end

	if(mode != MODE_NORMAL || state != STATE_IDLE || reset) begin
		state <= state + 1'd1;
		if(state == STATE_LAST) state <= STATE_IDLE;
	end
end

assign dout = ((~ds & a[0]) ? {last_data[7:0],last_data[15:8]} : last_data);

// initialization
always @(posedge clk) begin
	reg init_old=0;
	init_old <= init;

	if(init_old & ~init) reset <= '1;
	else if(state == STATE_LAST) begin
		if(reset != 0) begin
			reset <= reset - 5'd1;
			if(reset == 14)     mode <= MODE_PRE;
			else if(reset == 3) mode <= MODE_LDM;
			else                mode <= MODE_RESET;
		end
		else mode <= MODE_NORMAL;
	end
end

localparam CMD_NOP             = 3'b111;
localparam CMD_ACTIVE          = 3'b011;
localparam CMD_READ            = 3'b101;
localparam CMD_WRITE           = 3'b100;
localparam CMD_BURST_TERMINATE = 3'b110;
localparam CMD_PRECHARGE       = 3'b010;
localparam CMD_AUTO_REFRESH    = 3'b001;
localparam CMD_LOAD_MODE       = 3'b000;

wire [1:0] dqm = {we & ~ds & ~a[0], we & ~ds & a[0]};

// DQ tristate driver: ModelSim requires the inout port to be a net, so
// drive it from an internal reg via a continuous assign (synthesizes to
// the same tristate buffer Quartus inferred from the old `inout reg`).
reg [15:0] SDRAM_DQ_out;
assign SDRAM_DQ = SDRAM_DQ_out;

// SDRAM state machines
always @(posedge clk) begin
	if(state == STATE_START) SDRAM_BA <= (mode == MODE_NORMAL) ? addr[24:23] : 2'b00;

	SDRAM_DQ_out <= 'Z;
	casex({ram_req,we,mode,state})
		{2'bXX, MODE_NORMAL, STATE_START}: {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= ram_req_test ? CMD_ACTIVE : CMD_AUTO_REFRESH;
		{2'b11, MODE_NORMAL, STATE_CONT }: {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE, SDRAM_DQ_out} <= {CMD_WRITE, data};
		{2'b10, MODE_NORMAL, STATE_CONT }: {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_READ;

		// init
		{2'bXX,    MODE_LDM, STATE_START}: {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_LOAD_MODE;
		{2'bXX,    MODE_PRE, STATE_START}: {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PRECHARGE;

		                          default: {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_NOP;
	endcase

	if(mode == MODE_NORMAL) begin
		casex(state)
			STATE_START: SDRAM_A <= addr[13:1];
			STATE_CONT:  SDRAM_A <= {dqm, 2'b10, a[22:14]};
		endcase;
	end
	else if(mode == MODE_LDM && state == STATE_START) SDRAM_A <= MODE;
	else if(mode == MODE_PRE && state == STATE_START) SDRAM_A <= 13'b0010000000000;
	else SDRAM_A <= 0;
end

altddio_out
#(
	.extend_oe_disable("OFF"),
	.intended_device_family("Cyclone V"),
	.invert_output("OFF"),
	.lpm_hint("UNUSED"),
	.lpm_type("altddio_out"),
	.oe_reg("UNREGISTERED"),
	.power_up_high("OFF"),
	.width(1)
)
sdramclk_ddr
(
	.datain_h(1'b0),
	.datain_l(1'b1),
	.outclock(clk),
	.dataout(SDRAM_CLK),
	.aclr(1'b0),
	.aset(1'b0),
	.oe(1'b1),
	.outclocken(1'b1),
	.sclr(1'b0),
	.sset(1'b0)
);

endmodule
