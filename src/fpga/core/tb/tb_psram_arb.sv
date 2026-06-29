// tb_psram_arb — focused test of psram_arbiter's loss behavior under
// simultaneous Port A (ARAM-restore writes) + Port B (savestate reads).
//
// Hypothesis (2026-06-26): during a LOAD restore, the firmware writes
// restored ARAM via Port A at DMA rate WHILE reading the savestate via
// Port B.  psram_arbiter is drop-on-busy for Port A, so Port A writes that
// collide with an in-flight Port B transaction are SILENTLY DROPPED →
// corrupt ARAM → no audio / hang.  This TB drives that exact collision and
// counts how many Port A writes actually reach the chip.

`timescale 1ps / 1ps

module tb_psram_arb;
  reg clk = 0;
  always #5815 clk = ~clk;   // ~85.9 MHz

  // Port A (ARAM restore writes)
  reg        a_bank_sel = 1'b0;
  reg [21:0] a_addr = 0;
  reg        a_write_en = 0;
  reg [15:0] a_data_in = 0;
  reg        a_write_high_byte = 0;
  reg        a_write_low_byte = 0;
  reg        a_read_en = 0;
  wire       a_read_avail, a_busy;
  wire [15:0] a_data_out;

  // Port B (savestate reads)
  reg        b_bank_sel = 1'b1;
  reg [21:0] b_addr = 0;
  reg        b_write_en = 0;
  reg [15:0] b_data_in = 0;
  reg        b_write_high_byte = 0;
  reg        b_write_low_byte = 0;
  reg        b_read_en = 0;
  wire       b_read_avail, b_busy;
  wire [15:0] b_data_out;

  // CRAM pins
  wire [21:16] cram_a;
  wire [15:0]  cram_dq;
  wire cram_wait, cram_clk, cram_adv_n, cram_cre;
  wire cram_ce0_n, cram_ce1_n, cram_oe_n, cram_we_n, cram_ub_n, cram_lb_n;

  psram_arbiter #(.CLOCK_SPEED(85.9)) dut (
      .clk(clk),
      .a_bank_sel(a_bank_sel), .a_addr(a_addr), .a_write_en(a_write_en),
      .a_data_in(a_data_in), .a_write_high_byte(a_write_high_byte),
      .a_write_low_byte(a_write_low_byte), .a_read_en(a_read_en),
      .a_read_avail(a_read_avail), .a_data_out(a_data_out), .a_busy(a_busy),
      .b_bank_sel(b_bank_sel), .b_addr(b_addr), .b_write_en(b_write_en),
      .b_data_in(b_data_in), .b_write_high_byte(b_write_high_byte),
      .b_write_low_byte(b_write_low_byte), .b_read_en(b_read_en),
      .b_read_avail(b_read_avail), .b_data_out(b_data_out), .b_busy(b_busy),
      .b_grant(),
      .cram_a(cram_a), .cram_dq(cram_dq), .cram_wait(cram_wait),
      .cram_clk(cram_clk), .cram_adv_n(cram_adv_n), .cram_cre(cram_cre),
      .cram_ce0_n(cram_ce0_n), .cram_ce1_n(cram_ce1_n), .cram_oe_n(cram_oe_n),
      .cram_we_n(cram_we_n), .cram_ub_n(cram_ub_n), .cram_lb_n(cram_lb_n)
  );

  psram_chip_model chip (
      .clk(clk), .cram_a(cram_a), .cram_dq(cram_dq), .cram_wait(cram_wait),
      .cram_clk(cram_clk), .cram_adv_n(cram_adv_n), .cram_cre(cram_cre),
      .cram_ce0_n(cram_ce0_n), .cram_ce1_n(cram_ce1_n), .cram_oe_n(cram_oe_n),
      .cram_we_n(cram_we_n), .cram_ub_n(cram_ub_n), .cram_lb_n(cram_lb_n)
  );

  // Single clocked driver: emulate a LOAD restore where the firmware writes
  // ARAM (Port A) at the DMA byte cadence while it ALSO reads the staged
  // savestate (Port B).  Port A writes pulse a_write_en for one cycle every
  // AW_PERIOD cycles (mimics WE_N pulses ~32 clk_mem apart at DMA rate).
  // Port B reads are issued whenever Port B is free (continuous demand).
  // The firmware holds each write's addr/data stable through its access — we
  // model that by keeping a_addr/a_data valid until the next write is set up.
  localparam int AW_PERIOD = 16;   // clk_mem cycles between ARAM writes
  int aw_idx = 0;                  // next ARAM byte index to write
  int aw_timer = 0;
  int br_idx = 0;
  reg b_inflight = 0;

  always @(posedge clk) begin
    // ----- Port A write generator (one pulse per AW_PERIOD) -----
    a_write_en <= 1'b0;
    if (aw_idx < 256) begin
      if (aw_timer == 0) begin
        a_addr            <= aw_idx[21:0];
        a_data_in         <= 16'h1100 + aw_idx[7:0];
        a_write_high_byte <= 1'b1;
        a_write_low_byte  <= 1'b1;
        a_write_en        <= 1'b1;     // 1-cycle request pulse
        aw_idx            <= aw_idx + 1;
        aw_timer          <= AW_PERIOD;
      end else begin
        aw_timer <= aw_timer - 1;
      end
    end

    // ----- Port B read generator (continuous demand) -----
    // Keep b_read_en asserted until accepted, then move to next address.
    if (~b_inflight) begin
      b_addr     <= (br_idx % 64);
      b_read_en  <= 1'b1;
      b_inflight <= 1'b1;
    end else if (b_read_avail) begin
      b_read_en  <= 1'b0;
      b_inflight <= 1'b0;
      br_idx     <= br_idx + 1;
    end
  end

  initial begin
    a_write_en = 0; b_read_en = 0;
    // Wait until all 256 ARAM writes have been issued + drained.
    wait (aw_idx == 256);
    repeat (400) @(posedge clk);

    // Verify: did all 256 Port A writes reach bank 0?
    begin
      int miss; int j;
      logic [15:0] got;
      miss = 0;
      for (j = 0; j < 256; j++) begin
        got = chip.peek(1'b0, j[21:0]);
        if (got !== (16'h1100 + j[7:0])) begin
          miss++;
          if (miss <= 16)
            $display("MISS: ARAM byte %0d: got %h want %h", j, got, 16'h1100 + j[7:0]);
        end
      end
      $display("[tb_arb] Port A writes landed: %0d / 256 (missed %0d)", 256 - miss, miss);
      $display("[tb_arb] chip bank0 writes=%0d bank1 reads=%0d",
               chip.total_writes_bank0, chip.total_reads_bank1);
      if (miss == 0)
        $display("*** ARB PASS — no Port A writes dropped under contention ***");
      else
        $display("*** ARB FAIL — %0d Port A (ARAM restore) writes DROPPED ***", miss);
    end
    $finish;
  end

  initial begin
    #200_000_000;
    $display("WATCHDOG timeout (aw_idx=%0d)", aw_idx);
    $finish;
  end
endmodule
