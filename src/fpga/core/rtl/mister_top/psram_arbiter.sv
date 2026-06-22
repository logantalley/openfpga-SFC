// psram_arbiter — 2-master wrapper around one psram core.
//
// One physical PSRAM chip (cram1) was previously owned exclusively by the
// `aram` psram instance, which forced any new master onto a different chip
// or onto SDRAM.  This wrapper lets two masters share the same psram core:
//
//   Port A (priority): ARAM (SPC700 audio RAM) — bank_sel=0, the original
//     consumer.  Hit by SMP at ~1.024 MHz, so accesses are sparse.
//   Port B: savestate — bank_sel=1, a private 8 MB region with no other
//     consumer.  Savestate stage+serve land here.
//
// Priority: when both ports have a pending request and the core is idle,
// Port A wins.  One psram transaction is ~70 ns; SMP cycle is ~977 ns, so
// Port A is never delayed enough to miss.  bank_sel is sampled by the psram
// core only on transaction start, and this arbiter only commits a new
// transaction when busy=0, so cross-bank switching is always safe.
//
// The arbiter is combinational on the inputs to the psram core (gated by
// `granted_port`) and registers the in-flight grant + a read-routing
// pointer so data_out / read_avail go back to the master that asked.

module psram_arbiter #(
    parameter CLOCK_SPEED = 85.9  // MHz — passed through to psram core
) (
    input wire clk,

    // ----- Port A (priority) — typically ARAM, bank 0 -----
    input  wire        a_bank_sel,
    input  wire [21:0] a_addr,
    input  wire        a_write_en,
    input  wire [15:0] a_data_in,
    input  wire        a_write_high_byte,
    input  wire        a_write_low_byte,
    input  wire        a_read_en,
    output wire        a_read_avail,
    output wire [15:0] a_data_out,
    output wire        a_busy,

    // ----- Port B — typically savestate, bank 1 -----
    input  wire        b_bank_sel,
    input  wire [21:0] b_addr,
    input  wire        b_write_en,
    input  wire [15:0] b_data_in,
    input  wire        b_write_high_byte,
    input  wire        b_write_low_byte,
    input  wire        b_read_en,
    output wire        b_read_avail,
    output wire [15:0] b_data_out,
    output wire        b_busy,

    // ----- Physical PSRAM pins -----
    output wire [21:16] cram_a,
    inout  wire [15:0]  cram_dq,
    input  wire         cram_wait,
    output wire         cram_clk,
    output wire         cram_adv_n,
    output wire         cram_cre,
    output wire         cram_ce0_n,
    output wire         cram_ce1_n,
    output wire         cram_oe_n,
    output wire         cram_we_n,
    output wire         cram_ub_n,
    output wire         cram_lb_n
);

  // ----- Grant register -----
  // grant_valid=0 → no transaction in flight, ready to pick.
  // grant_valid=1 → a transaction is in flight; grant_port records who owns
  // the upcoming data_out / read_avail (1 = A, 0 = B).
  reg grant_valid = 1'b0;
  reg grant_is_a  = 1'b0;
  reg grant_was_read = 1'b0;

  // Aggregate request flags from each port.
  wire a_req = a_write_en | a_read_en;
  wire b_req = b_write_en | b_read_en;

  // Underlying psram core's busy signal — see psram.sv: busy goes high one
  // clk after a transaction starts and falls when state returns to NONE.
  wire core_busy;

  // Edge-detect A's request so a continuously-asserted ARAM CE/OE pair
  // (which on hardware can stay high across many clk_mem cycles for a
  // single SMP access) doesn't starve Port B.  Treat A as wanting service
  // only on the transition from idle to request — that's exactly one
  // transaction per ARAM access, which matches psram.sv semantics
  // (psram only acts on the first edge after STATE_NONE).
  reg  prev_a_req = 1'b0;
  wire a_req_edge = a_req & ~prev_a_req;
  always @(posedge clk) prev_a_req <= a_req;

  // Track whether A is currently being serviced (grant_valid & grant_is_a)
  // OR has a fresh edge this cycle.  Either way, A "wins" priority and
  // B must wait.
  wire a_active = (grant_valid & grant_is_a) | a_req_edge;

  // ----- Decide whether to start a new transaction this cycle -----
  // Only when no grant is currently in flight AND the psram core is idle.
  // A wins on edge; B wins whenever the core is free and A has no fresh
  // edge.  This guarantees A is never delayed (its edge starts immediately)
  // and B fills every otherwise-idle slot.
  wire can_start  = ~grant_valid & ~core_busy;
  wire start_a    = can_start &  a_req_edge;
  wire start_b    = can_start & ~a_req_edge & b_req;

  // Selected master's inputs to the core.
  wire        sel_bank_sel        = start_a ? a_bank_sel        : b_bank_sel;
  wire [21:0] sel_addr            = start_a ? a_addr            : b_addr;
  wire        sel_write_en        = start_a ? a_write_en        : (start_b & b_write_en);
  wire        sel_read_en         = start_a ? a_read_en         : (start_b & b_read_en);
  wire [15:0] sel_data_in         = start_a ? a_data_in         : b_data_in;
  wire        sel_write_high_byte = start_a ? a_write_high_byte : b_write_high_byte;
  wire        sel_write_low_byte  = start_a ? a_write_low_byte  : b_write_low_byte;

  // Final gated drives to the core.  When can_start is false (or no port
  // requested), write_en / read_en are held low so the core stays idle.
  wire core_write_en        = (start_a & a_write_en) | (start_b & b_write_en);
  wire core_read_en         = (start_a & a_read_en)  | (start_b & b_read_en);

  // ----- Grant tracking -----
  // Latch ownership on the cycle we kick a new transaction.  The core's
  // busy will rise on the next cycle (per psram.sv:294/313 it actually rises
  // the same cycle, but we depend on the latched grant for routing anyway).
  // Clear when the core returns to idle AND we've seen busy go high for
  // this transaction — guards against the immediate-NONE-after-start case
  // where busy might briefly flicker.
  reg saw_busy = 1'b0;
  always @(posedge clk) begin
    if (start_a) begin
      grant_valid    <= 1'b1;
      grant_is_a     <= 1'b1;
      grant_was_read <= a_read_en;
      saw_busy       <= 1'b0;
    end else if (start_b) begin
      grant_valid    <= 1'b1;
      grant_is_a     <= 1'b0;
      grant_was_read <= b_read_en;
      saw_busy       <= 1'b0;
    end else if (grant_valid) begin
      if (core_busy) saw_busy <= 1'b1;
      // Release the grant the cycle after busy falls.  read_avail (pulsed
      // by psram.sv at STATE_READ_DATA_RECEIVED) coincides with busy falling
      // on the read path, so we keep the grant flag valid that cycle to
      // route data_out / read_avail correctly, then drop it next cycle.
      if (saw_busy & ~core_busy) begin
        grant_valid <= 1'b0;
      end
    end
  end

  // ----- Per-port read responses -----
  // The psram core drives data_out + read_avail on the cycle the transaction
  // completes.  We hand it to whichever port currently holds the grant.
  wire        core_read_avail;
  wire [15:0] core_data_out;
  assign a_read_avail = grant_valid &  grant_is_a & grant_was_read & core_read_avail;
  assign b_read_avail = grant_valid & ~grant_is_a & grant_was_read & core_read_avail;
  assign a_data_out   = core_data_out;
  assign b_data_out   = core_data_out;

  // ----- Per-port busy -----
  // A port reports busy from the moment its transaction is granted through
  // the end of that transaction.  Callers use this the same way they would
  // poll the bare psram core's busy: low = "core is free, you can request",
  // high = "transaction in progress for you, wait".
  //
  // Note: we do NOT report busy on a port simply because the OTHER port is
  // in flight — that would cause ss_psram_arbiter (which polls b_busy in
  // SS_WR_ISSUE/SS_RD_ISSUE) to think its own request was accepted while
  // really Port A was being serviced, dropping the write.  The grant-based
  // formulation guarantees b_busy goes high only when our request is the
  // one in flight.
  assign a_busy = (grant_valid &  grant_is_a) | start_a;
  assign b_busy = (grant_valid & ~grant_is_a) | start_b;

  // ----- Underlying psram core -----
  psram #(
      .CLOCK_SPEED(CLOCK_SPEED)
  ) core (
      .clk(clk),

      .bank_sel(sel_bank_sel),
      .addr(sel_addr),

      .write_en(core_write_en),
      .data_in(sel_data_in),
      .write_high_byte(sel_write_high_byte),
      .write_low_byte(sel_write_low_byte),

      .read_en(core_read_en),
      .read_avail(core_read_avail),
      .data_out(core_data_out),

      .busy(core_busy),

      .cram_a(cram_a),
      .cram_dq(cram_dq),
      .cram_wait(cram_wait),
      .cram_clk(cram_clk),
      .cram_adv_n(cram_adv_n),
      .cram_cre(cram_cre),
      .cram_ce0_n(cram_ce0_n),
      .cram_ce1_n(cram_ce1_n),
      .cram_oe_n(cram_oe_n),
      .cram_we_n(cram_we_n),
      .cram_ub_n(cram_ub_n),
      .cram_lb_n(cram_lb_n)
  );

endmodule
