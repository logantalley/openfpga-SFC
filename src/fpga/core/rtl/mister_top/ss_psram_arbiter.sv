// ss_psram_arbiter — savestate-side PSRAM CDC + 4-word BURST access driver.
//
// Translates the save_state_controller's per-chunk requests (clk_sys,
// 21.48 MHz) into PSRAM Port-B accesses (clk_mem, 85.9 MHz) on
// psram_arbiter.  Mirrors the toggle-CDC handshake of ss_sdram_arbiter but
// transfers a WHOLE 64-bit chunk (4×16-bit PSRAM words) per request.
//
// ## Why burst
//
// The earlier per-word design paid a full clk_sys↔clk_mem toggle-CDC round
// trip (synch_3 each way + settle ≈ 30+ clk_mem cycles) for EACH of the 4
// words in a chunk.  That made a chunk so slow that the SNES DMA feeding
// SSDATA (no flow control) outran staging and the save died ~8 KB in.
//
// Burst pays the CDC round trip ONCE per chunk: the controller hands over
// the 64-bit chunk + base word address with a single req toggle; this FSM
// writes (or reads) the 4 consecutive PSRAM words back-to-back, then
// toggles ack once.  ~3× faster per chunk — comfortably ahead of DMA even
// under ARAM contention on the shared CRAM1 chip.
//
// Savestate uses PSRAM bank 1 (private; ARAM uses bank 0).  PSRAM has no
// skip-stale quirk, so the 4 words step the word address by 1 (base+0..+3).

module ss_psram_arbiter (
    input  wire        clk_sys,
    input  wire        clk_mem,

    // clk_sys domain — from save_state_controller (one req per 64-bit chunk)
    input  wire        ss_psram_wr_req,   // toggle
    input  wire [18:0] ss_psram_wr_addr,  // chunk-base word address
    input  wire [63:0] ss_psram_wr_data,  // full 64-bit chunk
    output wire        ss_psram_wr_ack,   // toggle (clk_sys)
    input  wire        ss_psram_rd_req,   // toggle
    input  wire [18:0] ss_psram_rd_addr,  // chunk-base word address
    output wire [63:0] ss_psram_rd_data,  // assembled 64-bit chunk (synch_3'd)
    output wire        ss_psram_rd_ack,   // toggle (clk_sys)

    // clk_mem domain — to psram_arbiter Port B (single-word interface)
    output reg         b_write_en,
    output reg         b_read_en,
    output reg  [21:0] b_addr,
    output reg  [15:0] b_data_in,
    output reg         b_write_high_byte,
    output reg         b_write_low_byte,
    output wire        b_bank_sel,
    input  wire [15:0] b_data_out,
    input  wire        b_read_avail,
    input  wire        b_busy,
    input  wire        b_grant       // 1-cycle: our request was accepted by the core
);

  // Savestate region is fixed to PSRAM bank 1.
  assign b_bank_sel = 1'b1;

  // ----- clk_sys → clk_mem CDC of the request toggles -----
  wire ss_psram_wr_req_mem;
  wire ss_psram_rd_req_mem;
  synch_3 sync_ss_wr_req (.i(ss_psram_wr_req), .o(ss_psram_wr_req_mem), .clk(clk_mem));
  synch_3 sync_ss_rd_req (.i(ss_psram_rd_req), .o(ss_psram_rd_req_mem), .clk(clk_mem));

  reg prev_ss_psram_wr_req_mem = 0;
  reg prev_ss_psram_rd_req_mem = 0;
  wire ss_psram_wr_edge_mem = (ss_psram_wr_req_mem != prev_ss_psram_wr_req_mem);
  wire ss_psram_rd_edge_mem = (ss_psram_rd_req_mem != prev_ss_psram_rd_req_mem);

  // ----- clk_mem-side burst FSM -----
  localparam SS_IDLE       = 4'd0;
  localparam SS_WR_SETTLE  = 4'd1;   // wait for CDC addr/data to settle
  localparam SS_WR_ISSUE   = 4'd2;   // assert write_en for word[widx]
  localparam SS_WR_WAIT    = 4'd3;   // wait for that word's transaction to finish
  localparam SS_WR_DONE    = 4'd4;   // all 4 words done → ack
  localparam SS_RD_SETTLE  = 4'd5;
  localparam SS_RD_ISSUE   = 4'd6;
  localparam SS_RD_WAIT    = 4'd7;
  localparam SS_RD_DONE    = 4'd8;

  reg [3:0]  ss_mem_state = SS_IDLE;
  reg        ss_psram_wr_ack_mem  = 0;
  reg        ss_psram_rd_ack_mem  = 0;
  reg        busy_seen = 0;
  reg [2:0]  settle_cnt = 3'd0;
  reg [1:0]  widx = 2'd0;            // which of the 4 words this burst is on

  // Latched burst parameters (captured once after CDC settle).
  reg [18:0] burst_addr  = 19'd0;
  reg [63:0] burst_wdata = 64'd0;
  reg [63:0] burst_rdata = 64'd0;

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
        widx              <= 2'd0;
        if (ss_psram_wr_edge_mem) begin
          settle_cnt   <= 3'd3;
          ss_mem_state <= SS_WR_SETTLE;
        end else if (ss_psram_rd_edge_mem) begin
          settle_cnt   <= 3'd3;
          ss_mem_state <= SS_RD_SETTLE;
        end
      end

      // ---- WRITE burst ----
      SS_WR_SETTLE: begin
        if (settle_cnt == 3'd0) begin
          burst_addr   <= ss_psram_wr_addr;
          burst_wdata  <= ss_psram_wr_data;
          ss_mem_state <= SS_WR_ISSUE;
        end else begin
          settle_cnt <= settle_cnt - 3'd1;
        end
      end

      // DETERMINISTIC write handshake (2026-06-26): hold b_write_en until the
      // arbiter ACCEPTS it (b_grant pulse), then deassert and advance.  The
      // next word's grant cannot fire until the core is free again (the
      // arbiter's start_b requires ~core_busy), so words serialize correctly
      // with no reliance on b_busy edge inference.
      SS_WR_ISSUE: begin
        b_addr            <= {3'b000, burst_addr} + {20'b0, widx};
        b_data_in         <= burst_wdata[widx*16 +: 16];
        b_write_high_byte <= 1'b1;
        b_write_low_byte  <= 1'b1;
        b_write_en        <= 1'b1;
        if (b_grant) begin       // our write launched into the core
          b_write_en        <= 1'b0;
          b_write_high_byte <= 1'b0;
          b_write_low_byte  <= 1'b0;
          if (widx == 2'd3) begin
            ss_mem_state <= SS_WR_DONE;
          end else begin
            widx         <= widx + 2'd1;
            ss_mem_state <= SS_WR_ISSUE;
          end
        end
      end

      // SS_WR_WAIT retained as a no-op safety state (unused by the new
      // b_grant-driven write path).
      SS_WR_WAIT: begin
        ss_mem_state <= SS_WR_ISSUE;
      end

      SS_WR_DONE: begin
        ss_psram_wr_ack_mem <= ~ss_psram_wr_ack_mem;
        ss_mem_state        <= SS_IDLE;
      end

      // ---- READ burst ----
      SS_RD_SETTLE: begin
        if (settle_cnt == 3'd0) begin
          burst_addr   <= ss_psram_rd_addr;
          ss_mem_state <= SS_RD_ISSUE;
        end else begin
          settle_cnt <= settle_cnt - 3'd1;
        end
      end

      // DETERMINISTIC read handshake (2026-06-26 race fix): hold b_read_en
      // until the arbiter ACCEPTS it (b_grant pulse).  Do NOT gate on ~b_busy
      // — under Port A contention b_busy reads 0 while Port A is in flight, so
      // the old gate fired b_read_en at the wrong time and the first word's
      // capture was lost.  b_grant fires for exactly one cycle when this read
      // actually launches into the core.
      SS_RD_ISSUE: begin
        b_addr    <= {3'b000, burst_addr} + {20'b0, widx};
        b_read_en <= 1'b1;
        if (b_grant) begin       // our read launched
          b_read_en    <= 1'b0;
          ss_mem_state <= SS_RD_WAIT;
        end
      end

      SS_RD_WAIT: begin
        // After grant, the core drives b_read_avail for exactly one cycle when
        // the data is valid (psram STATE_READ_DATA_RECEIVED).  Capture it then.
        if (b_read_avail) begin
          burst_rdata[widx*16 +: 16] <= b_data_out;
          if (widx == 2'd3) begin
            ss_mem_state <= SS_RD_DONE;
          end else begin
            widx         <= widx + 2'd1;
            ss_mem_state <= SS_RD_ISSUE;
          end
        end
      end

      SS_RD_DONE: begin
        ss_psram_rd_ack_mem <= ~ss_psram_rd_ack_mem;
        ss_mem_state        <= SS_IDLE;
      end

      default: ss_mem_state <= SS_IDLE;
    endcase
  end

  // ----- clk_mem → clk_sys CDC of acks + read data -----
  synch_3 sync_ss_wr_ack (.i(ss_psram_wr_ack_mem), .o(ss_psram_wr_ack), .clk(clk_sys));
  synch_3 sync_ss_rd_ack (.i(ss_psram_rd_ack_mem), .o(ss_psram_rd_ack), .clk(clk_sys));
  // burst_rdata is fully assembled and stable by the time the rd_ack toggle
  // propagates (3 clk_sys cycles), so a plain synch_3 of the 64-bit value is
  // safe — the controller samples it only after seeing the ack edge.
  synch_3 #(.WIDTH(64)) sync_ss_rd_data
      (.i(burst_rdata), .o(ss_psram_rd_data), .clk(clk_sys));

endmodule
