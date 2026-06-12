// ss_sdram_arbiter — savestate-staging SDRAM CDC + access FSM.
//
// Extracted VERBATIM from SNES.sv (the inline Phase B/C block) so the same
// RTL can be instantiated by both the synthesized core and the focused
// ModelSim testbench (tb_ss_staging.sv).  Behavior must remain bit-identical
// to the previous inline version — do not "improve" logic here without a
// matching hardware test.
//
// The savestate controller (clk_sys, 21.48 MHz) asks SDRAM (clk_mem,
// 85.9 MHz) to write or read a single 16-bit word at a 25-bit address.
// Because clk_mem is ~4× clk_sys, control-toggle CDC via synch_3 is
// adequate: data lines are stable across many clk_mem cycles by the
// time the synchronized toggle edge fires.
//
// Protocol (per direction): the clk_sys side flips a `req` bit when it
// wants the access; the clk_mem side detects the edge of the synced
// toggle, issues exactly one sdram pulse, captures rd_data if it was a
// read, then flips an `ack` bit which goes back through synch_3 to
// clk_sys.  Address/data are sampled when the edge fires.

module ss_sdram_arbiter (
    input  wire        clk_sys,
    input  wire        clk_mem,

    // clk_sys domain — from save_state_controller
    input  wire        ss_sdram_wr_req,   // toggle
    input  wire [24:0] ss_sdram_wr_addr,
    input  wire [15:0] ss_sdram_wr_data,
    output wire        ss_sdram_wr_ack,   // toggle (clk_sys)
    input  wire        ss_sdram_rd_req,   // toggle
    input  wire [24:0] ss_sdram_rd_addr,
    output wire [15:0] ss_sdram_rd_data,  // synch_3'd back to clk_sys
    output wire        ss_sdram_rd_ack,   // toggle (clk_sys)
    input  wire        ss_loading,        // clk_sys level

    // clk_mem domain — to the sdram controller input mux
    output wire        ss_loading_mem,
    output reg         ss_mem_rd,
    output reg         ss_mem_wr,
    output reg  [24:0] ss_mem_addr,
    output reg  [15:0] ss_mem_din,
    input  wire [15:0] sdram_dout,        // = ROM_Q_SDRAM in SNES.sv
    input  wire        sdram_busy
);

  // ----- clk_sys → clk_mem CDC -----
  wire ss_sdram_wr_req_mem;
  wire ss_sdram_rd_req_mem;
  synch_3 sync_ss_wr_req  (.i(ss_sdram_wr_req),  .o(ss_sdram_wr_req_mem),  .clk(clk_mem));
  synch_3 sync_ss_rd_req  (.i(ss_sdram_rd_req),  .o(ss_sdram_rd_req_mem),  .clk(clk_mem));
  synch_3 sync_ss_loading (.i(ss_loading),       .o(ss_loading_mem),       .clk(clk_mem));

  // Edge detection (clk_mem side) — when a toggle edge arrives we kick
  // off the SDRAM access via the FSM below.
  reg prev_ss_sdram_wr_req_mem = 0;
  reg prev_ss_sdram_rd_req_mem = 0;
  wire ss_sdram_wr_edge_mem =
      (ss_sdram_wr_req_mem != prev_ss_sdram_wr_req_mem);
  wire ss_sdram_rd_edge_mem =
      (ss_sdram_rd_req_mem != prev_ss_sdram_rd_req_mem);

  // ----- clk_mem-side SDRAM-access FSM -----
  // States:
  //   SS_IDLE       — no savestate access pending
  //   SS_WR_ISSUE   — drive wr=1 to sdram on this cycle to launch write
  //   SS_WR_WAIT    — wait for sdram.busy to fall, then toggle ack
  //   SS_RD_ISSUE   — drive rd=1 to launch read
  //   SS_RD_WAIT    — wait for sdram.busy to fall, then latch dout + ack
  //
  // ss_loading_mem stays high through the entire access, so the input
  // mux gives us the bus the whole time.  rd/wr go through dedicated
  // FSM-driven regs so they hold for as long as needed.
  localparam SS_IDLE       = 3'd0;
  localparam SS_WR_SETTLE  = 3'd5;  // wait for unsynced addr/data bus to settle
  localparam SS_WR_ISSUE   = 3'd1;
  localparam SS_WR_WAIT    = 3'd2;
  localparam SS_RD_SETTLE  = 3'd6;
  localparam SS_RD_ISSUE   = 3'd3;
  localparam SS_RD_WAIT    = 3'd4;

  reg [2:0]  ss_mem_state = SS_IDLE;
  reg        ss_sdram_wr_ack_mem  = 0;
  reg        ss_sdram_rd_ack_mem  = 0;
  reg [15:0] ss_sdram_rd_data_mem = 16'h0000;
  reg        sdram_busy_seen = 0;
  // Settle counter: after detecting the toggle edge, the unsynchronized
  // multi-bit address/data buses (ss_sdram_wr_addr / ss_sdram_wr_data /
  // ss_sdram_rd_addr) may have per-bit skew across the CDC boundary.
  // Wait a few clk_mem cycles before sampling so all bits settle.  Synch_3
  // takes 3 clk_mem cycles to propagate the toggle edge; pad with 3 more
  // to be safe (total ~70 ns at 85.9 MHz, well within the >180 ns the
  // clk_sys side holds addr/data stable).
  reg [2:0] ss_settle_cnt = 3'd0;

  initial begin
    ss_mem_rd   = 1'b0;
    ss_mem_wr   = 1'b0;
    ss_mem_addr = 25'd0;
    ss_mem_din  = 16'h0000;
  end

  always @(posedge clk_mem) begin
    prev_ss_sdram_wr_req_mem <= ss_sdram_wr_req_mem;
    prev_ss_sdram_rd_req_mem <= ss_sdram_rd_req_mem;

    case (ss_mem_state)
      SS_IDLE: begin
        ss_mem_rd <= 0;
        ss_mem_wr <= 0;
        sdram_busy_seen <= 0;
        // Defer addr/data sampling — go to a SETTLE state and wait a few
        // clk_mem cycles before sampling, so unsynchronized multi-bit
        // buses settle across the CDC.
        if (ss_sdram_wr_edge_mem) begin
          ss_settle_cnt <= 3'd1;
          ss_mem_state  <= SS_WR_SETTLE;
        end else if (ss_sdram_rd_edge_mem) begin
          ss_settle_cnt <= 3'd1;
          ss_mem_state  <= SS_RD_SETTLE;
        end
      end

      SS_WR_SETTLE: begin
        if (ss_settle_cnt == 3'd0) begin
          // All addr/data bits should now be settled across CDC.  Sample.
          ss_mem_addr  <= ss_sdram_wr_addr;
          ss_mem_din   <= ss_sdram_wr_data;
          ss_mem_state <= SS_WR_ISSUE;
        end else begin
          ss_settle_cnt <= ss_settle_cnt - 3'd1;
        end
      end

      SS_WR_ISSUE: begin
        // Gate on ~sdram_busy: sdram.sv only accepts new requests in
        // STATE_IDLE.  Issuing wr=1 while busy is silently ignored, yet
        // our FSM would still see busy fall and toggle ack, dropping the
        // write.  Hold here until sdram is idle.
        if (~sdram_busy) begin
          ss_mem_wr    <= 1;
          ss_mem_state <= SS_WR_WAIT;
        end
      end

      SS_WR_WAIT: begin
        // Hold ss_mem_wr=1 until sdram has latched the request (busy
        // rises one cycle after the rd/wr pulse).  Then drop wr and
        // wait for busy to fall, which signals the access completed.
        if (sdram_busy) begin
          ss_mem_wr        <= 0;
          sdram_busy_seen  <= 1;
        end
        if (sdram_busy_seen && ~sdram_busy) begin
          ss_sdram_wr_ack_mem <= ~ss_sdram_wr_ack_mem;
          ss_mem_state        <= SS_IDLE;
        end
      end

      SS_RD_SETTLE: begin
        if (ss_settle_cnt == 3'd0) begin
          ss_mem_addr  <= ss_sdram_rd_addr;
          ss_mem_state <= SS_RD_ISSUE;
        end else begin
          ss_settle_cnt <= ss_settle_cnt - 3'd1;
        end
      end

      SS_RD_ISSUE: begin
        // Same gate as SS_WR_ISSUE — only assert rd when sdram is idle.
        if (~sdram_busy) begin
          ss_mem_rd    <= 1;
          ss_mem_state <= SS_RD_WAIT;
        end
      end

      SS_RD_WAIT: begin
        if (sdram_busy) begin
          ss_mem_rd        <= 0;
          sdram_busy_seen  <= 1;
        end
        if (sdram_busy_seen && ~sdram_busy) begin
          // sdram drops busy and presents dout from last_data on the
          // same cycle (STATE_READY block in sdram.sv).
          ss_sdram_rd_data_mem <= sdram_dout;
          ss_sdram_rd_ack_mem  <= ~ss_sdram_rd_ack_mem;
          ss_mem_state         <= SS_IDLE;
        end
      end

      default: ss_mem_state <= SS_IDLE;
    endcase
  end

  // ----- clk_mem → clk_sys CDC -----
  synch_3 sync_ss_wr_ack (.i(ss_sdram_wr_ack_mem), .o(ss_sdram_wr_ack), .clk(clk_sys));
  synch_3 sync_ss_rd_ack (.i(ss_sdram_rd_ack_mem), .o(ss_sdram_rd_ack), .clk(clk_sys));
  synch_3 #(.WIDTH(16)) sync_ss_rd_data
      (.i(ss_sdram_rd_data_mem), .o(ss_sdram_rd_data), .clk(clk_sys));

endmodule
