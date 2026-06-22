// ss_psram_arbiter — savestate-side PSRAM CDC + access driver.
//
// Mirrors ss_sdram_arbiter.sv but targets psram_arbiter's Port B instead of
// sdram.sv.  Lets the save_state_controller (clk_sys, 21.48 MHz) issue
// 16-bit reads/writes to a PSRAM region (clk_mem, 85.9 MHz) using the same
// toggle-CDC handshake pattern.
//
// Differences from ss_sdram_arbiter (kept intentionally minimal):
//   - Address width is 19 bits (covers 1 MB at 16-bit granularity, ample
//     for the 512 KB savestate slot — psram itself supports 22-bit, but
//     we use less here so the constant base addr is a clean alignment).
//   - psram has no `busy` deassert / reassert handshake the way sdram.sv
//     does: psram_arbiter exposes `b_busy` which is high from the cycle a
//     transaction kicks off through the cycle it completes.  We wait for
//     b_busy to rise (transaction accepted) then fall (transaction done).
//   - psram_arbiter pulses `b_read_avail` for one clk_mem cycle when read
//     data is valid; we latch on that pulse instead of after busy falls.
//
// Protocol on the clk_sys side is identical to the SDRAM version: flip
// req toggle, wait for ack toggle.  Address/data multi-bit CDC is handled
// the same way (SETTLE state pads several clk_mem cycles after the
// synch_3'd req edge so all bits land).

module ss_psram_arbiter (
    input  wire        clk_sys,
    input  wire        clk_mem,

    // clk_sys domain — from save_state_controller
    input  wire        ss_psram_wr_req,   // toggle
    input  wire [18:0] ss_psram_wr_addr,
    input  wire [15:0] ss_psram_wr_data,
    output wire        ss_psram_wr_ack,   // toggle (clk_sys)
    input  wire        ss_psram_rd_req,   // toggle
    input  wire [18:0] ss_psram_rd_addr,
    output wire [15:0] ss_psram_rd_data,  // synch_3'd back to clk_sys
    output wire        ss_psram_rd_ack,   // toggle (clk_sys)

    // clk_mem domain — to psram_arbiter Port B
    output reg         b_write_en,
    output reg         b_read_en,
    output reg  [21:0] b_addr,
    output reg  [15:0] b_data_in,
    output reg         b_write_high_byte,
    output reg         b_write_low_byte,
    output wire        b_bank_sel,
    input  wire [15:0] b_data_out,
    input  wire        b_read_avail,
    input  wire        b_busy
);

  // Savestate region is fixed to PSRAM bank 1; address is the 19-bit
  // word offset, zero-extended into the 22-bit psram address bus.
  assign b_bank_sel = 1'b1;

  // ----- clk_sys → clk_mem CDC -----
  wire ss_psram_wr_req_mem;
  wire ss_psram_rd_req_mem;
  synch_3 sync_ss_wr_req (.i(ss_psram_wr_req), .o(ss_psram_wr_req_mem), .clk(clk_mem));
  synch_3 sync_ss_rd_req (.i(ss_psram_rd_req), .o(ss_psram_rd_req_mem), .clk(clk_mem));

  reg prev_ss_psram_wr_req_mem = 0;
  reg prev_ss_psram_rd_req_mem = 0;
  wire ss_psram_wr_edge_mem = (ss_psram_wr_req_mem != prev_ss_psram_wr_req_mem);
  wire ss_psram_rd_edge_mem = (ss_psram_rd_req_mem != prev_ss_psram_rd_req_mem);

  // ----- clk_mem-side FSM -----
  localparam SS_IDLE      = 3'd0;
  localparam SS_WR_SETTLE = 3'd1;
  localparam SS_WR_ISSUE  = 3'd2;
  localparam SS_WR_WAIT   = 3'd3;
  localparam SS_RD_SETTLE = 3'd4;
  localparam SS_RD_ISSUE  = 3'd5;
  localparam SS_RD_WAIT   = 3'd6;

  reg [2:0]  ss_mem_state = SS_IDLE;
  reg        ss_psram_wr_ack_mem  = 0;
  reg        ss_psram_rd_ack_mem  = 0;
  reg [15:0] ss_psram_rd_data_mem = 16'h0000;
  reg        busy_seen = 0;

  // Settle counter: same role as ss_sdram_arbiter — pad after the
  // synch_3 edge so multi-bit addr/data settles across CDC before we
  // latch.  3 cycles is plenty at 85.9 MHz vs. the >180 ns the clk_sys
  // side holds addr/data stable.
  reg [2:0] settle_cnt = 3'd0;

  initial begin
    b_write_en        = 1'b0;
    b_read_en         = 1'b0;
    b_addr            = 22'd0;
    b_data_in         = 16'h0000;
    b_write_high_byte = 1'b0;
    b_write_low_byte  = 1'b0;
  end

  always @(posedge clk_mem) begin
    prev_ss_psram_wr_req_mem <= ss_psram_wr_req_mem;
    prev_ss_psram_rd_req_mem <= ss_psram_rd_req_mem;

    case (ss_mem_state)
      SS_IDLE: begin
        b_write_en        <= 1'b0;
        b_read_en         <= 1'b0;
        b_write_high_byte <= 1'b0;
        b_write_low_byte  <= 1'b0;
        busy_seen         <= 1'b0;
        if (ss_psram_wr_edge_mem) begin
          settle_cnt   <= 3'd3;
          ss_mem_state <= SS_WR_SETTLE;
        end else if (ss_psram_rd_edge_mem) begin
          settle_cnt   <= 3'd3;
          ss_mem_state <= SS_RD_SETTLE;
        end
      end

      SS_WR_SETTLE: begin
        if (settle_cnt == 3'd0) begin
          b_addr            <= {3'b000, ss_psram_wr_addr};
          b_data_in         <= ss_psram_wr_data;
          b_write_high_byte <= 1'b1;
          b_write_low_byte  <= 1'b1;
          ss_mem_state      <= SS_WR_ISSUE;
        end else begin
          settle_cnt <= settle_cnt - 3'd1;
        end
      end

      SS_WR_ISSUE: begin
        // Wait until Port B reports the underlying psram core is idle
        // (b_busy reflects either ARAM in flight or a previous request).
        // psram_arbiter accepts our write_en once b_busy is 0.
        if (~b_busy) begin
          b_write_en   <= 1'b1;
          ss_mem_state <= SS_WR_WAIT;
        end
      end

      SS_WR_WAIT: begin
        // Once the arbiter latches the request, b_busy will rise.  After
        // we observe it rise, drop write_en and wait for b_busy to fall —
        // that's transaction complete.  We need write_en held until the
        // arbiter has committed it; psram_arbiter combinationally feeds
        // start_b into the core, so by the next clk after start b_busy=1.
        if (b_busy) begin
          b_write_en        <= 1'b0;
          b_write_high_byte <= 1'b0;
          b_write_low_byte  <= 1'b0;
          busy_seen         <= 1'b1;
        end
        if (busy_seen && ~b_busy) begin
          ss_psram_wr_ack_mem <= ~ss_psram_wr_ack_mem;
          ss_mem_state        <= SS_IDLE;
        end
      end

      SS_RD_SETTLE: begin
        if (settle_cnt == 3'd0) begin
          b_addr       <= {3'b000, ss_psram_rd_addr};
          ss_mem_state <= SS_RD_ISSUE;
        end else begin
          settle_cnt <= settle_cnt - 3'd1;
        end
      end

      SS_RD_ISSUE: begin
        if (~b_busy) begin
          b_read_en    <= 1'b1;
          ss_mem_state <= SS_RD_WAIT;
        end
      end

      SS_RD_WAIT: begin
        if (b_busy) begin
          b_read_en <= 1'b0;
          busy_seen <= 1'b1;
        end
        // b_read_avail pulses the cycle psram presents data_out — same
        // cycle b_busy falls.  Latch the data, toggle ack, return to idle.
        if (b_read_avail) begin
          ss_psram_rd_data_mem <= b_data_out;
        end
        if (busy_seen && ~b_busy) begin
          // Capture data here too in case read_avail and busy-falling
          // arrive on the same cycle (psram.sv asserts both at
          // STATE_READ_DATA_RECEIVED).
          if (b_read_avail) ss_psram_rd_data_mem <= b_data_out;
          ss_psram_rd_ack_mem <= ~ss_psram_rd_ack_mem;
          ss_mem_state        <= SS_IDLE;
        end
      end

      default: ss_mem_state <= SS_IDLE;
    endcase
  end

  // ----- clk_mem → clk_sys CDC -----
  synch_3 sync_ss_wr_ack (.i(ss_psram_wr_ack_mem), .o(ss_psram_wr_ack), .clk(clk_sys));
  synch_3 sync_ss_rd_ack (.i(ss_psram_rd_ack_mem), .o(ss_psram_rd_ack), .clk(clk_sys));
  synch_3 #(.WIDTH(16)) sync_ss_rd_data
      (.i(ss_psram_rd_data_mem), .o(ss_psram_rd_data), .clk(clk_sys));

endmodule
