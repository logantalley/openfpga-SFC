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
// Port A is never delayed enough to miss.
//
// ## Why edge-capture, not live-level forwarding
//
// The original (broken) design routed each port's live signals into the
// psram core when the arbiter granted it.  That worked when only one port
// was present (psram's old solo instance) because the consumer (e.g.,
// ARAM) holds CE/OE active across the entire access.  With two masters
// sharing one controller, an arbiter may DEFER a master's request by a
// few clk_mem cycles (waiting for the other master's transaction to
// finish).  If the deferred master has already dropped its level signal
// by then, the core sees no live request — the transaction is lost.
//
// On hardware this manifested as: audio broken (every other ARAM access
// dropped because the SPC700 pulses CE_N briefly, not continuously, and
// the arbiter was occasionally busy on Port B during the pulse).
//
// Fix: on each port's rising-edge, latch its parameters (addr, data_in,
// byte enables, bank, direction) into capture registers.  When the
// arbiter grants the port, drive psram from the captured registers — not
// from the live signals.  The consumer is then free to drop its level
// signal the very next cycle; the captured parameters survive until
// service.  Both ports get edge-capture for symmetry (Port B's
// ss_psram_arbiter already holds its lines stable but capturing costs
// nothing and removes a dependency).

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

  // ----- Edge detection on each port's request line -----
  wire a_req_raw = a_write_en | a_read_en;
  wire b_req_raw = b_write_en | b_read_en;
  reg  prev_a_req_raw = 1'b0;
  reg  prev_b_req_raw = 1'b0;
  wire a_edge = a_req_raw & ~prev_a_req_raw;
  wire b_edge = b_req_raw & ~prev_b_req_raw;

  // ----- Per-port pending bits + parameter capture registers -----
  // The capture registers hold the consumer's request parameters from the
  // edge cycle so the arbiter can faithfully replay them when the port is
  // eventually granted, even if the consumer has already moved on.
  reg        a_pending      = 1'b0;
  reg        a_cap_bank_sel = 1'b0;
  reg [21:0] a_cap_addr     = 22'd0;
  reg        a_cap_write_en = 1'b0;
  reg        a_cap_read_en  = 1'b0;
  reg [15:0] a_cap_data_in  = 16'h0000;
  reg        a_cap_ub       = 1'b0;
  reg        a_cap_lb       = 1'b0;

  reg        b_pending      = 1'b0;
  reg        b_cap_bank_sel = 1'b0;
  reg [21:0] b_cap_addr     = 22'd0;
  reg        b_cap_write_en = 1'b0;
  reg        b_cap_read_en  = 1'b0;
  reg [15:0] b_cap_data_in  = 16'h0000;
  reg        b_cap_ub       = 1'b0;
  reg        b_cap_lb       = 1'b0;

  // psram core's busy signal.
  wire core_busy;

  // ----- Grant tracking (declarations hoisted above first use) -----
  reg grant_valid    = 1'b0;
  reg grant_is_a     = 1'b0;
  reg grant_was_read = 1'b0;
  reg saw_busy       = 1'b0;

  // Arbitration: only start a new transaction when no grant is in flight
  // and the core is idle.  A wins ties.
  wire can_start = ~grant_valid & ~core_busy;
  wire start_a   = can_start &  a_pending;
  wire start_b   = can_start & ~a_pending & b_pending;

  // Drive psram core from the captured registers of whichever port we're
  // starting this cycle.  When neither starts, write_en/read_en are 0 so
  // the core stays idle.
  wire        sel_bank_sel = start_a ? a_cap_bank_sel : b_cap_bank_sel;
  wire [21:0] sel_addr     = start_a ? a_cap_addr     : b_cap_addr;
  wire [15:0] sel_data_in  = start_a ? a_cap_data_in  : b_cap_data_in;
  wire        sel_ub       = start_a ? a_cap_ub       : b_cap_ub;
  wire        sel_lb       = start_a ? a_cap_lb       : b_cap_lb;
  wire        core_write_en = (start_a & a_cap_write_en) | (start_b & b_cap_write_en);
  wire        core_read_en  = (start_a & a_cap_read_en)  | (start_b & b_cap_read_en);

  // ----- Edge / pending / capture / grant clocked block -----
  always @(posedge clk) begin
    prev_a_req_raw <= a_req_raw;
    prev_b_req_raw <= b_req_raw;

    // Capture A's parameters at the rising edge of its request.  We do
    // NOT capture continuously while pending — that would change the
    // captured value if the consumer is still driving live signals.  A
    // single capture at the edge is exactly the consumer's intent.
    if (a_edge) begin
      a_pending      <= 1'b1;
      a_cap_bank_sel <= a_bank_sel;
      a_cap_addr     <= a_addr;
      a_cap_write_en <= a_write_en;
      a_cap_read_en  <= a_read_en;
      a_cap_data_in  <= a_data_in;
      a_cap_ub       <= a_write_high_byte;
      a_cap_lb       <= a_write_low_byte;
    end
    if (start_a) a_pending <= 1'b0;

    if (b_edge) begin
      b_pending      <= 1'b1;
      b_cap_bank_sel <= b_bank_sel;
      b_cap_addr     <= b_addr;
      b_cap_write_en <= b_write_en;
      b_cap_read_en  <= b_read_en;
      b_cap_data_in  <= b_data_in;
      b_cap_ub       <= b_write_high_byte;
      b_cap_lb       <= b_write_low_byte;
    end
    if (start_b) b_pending <= 1'b0;

    // Latch grant on the cycle we kick a new transaction.
    if (start_a) begin
      grant_valid    <= 1'b1;
      grant_is_a     <= 1'b1;
      grant_was_read <= a_cap_read_en;
      saw_busy       <= 1'b0;
    end else if (start_b) begin
      grant_valid    <= 1'b1;
      grant_is_a     <= 1'b0;
      grant_was_read <= b_cap_read_en;
      saw_busy       <= 1'b0;
    end else if (grant_valid) begin
      if (core_busy) saw_busy <= 1'b1;
      // Release the grant the cycle after busy falls.  read_avail (pulsed
      // by psram.sv at STATE_READ_DATA_RECEIVED) coincides with busy
      // falling on reads, so we keep the grant flag valid that cycle to
      // route data_out / read_avail correctly, then drop it next cycle.
      if (saw_busy & ~core_busy) grant_valid <= 1'b0;
    end
  end

  // ----- Per-port read responses -----
  wire        core_read_avail;
  wire [15:0] core_data_out;
  assign a_read_avail = grant_valid &  grant_is_a & grant_was_read & core_read_avail;
  assign b_read_avail = grant_valid & ~grant_is_a & grant_was_read & core_read_avail;
  assign a_data_out   = core_data_out;
  assign b_data_out   = core_data_out;

  // ----- Per-port busy -----
  // Busy reflects only THIS port's transaction state, not the other's.
  // - If the port has a pending request OR is currently being serviced, busy=1.
  // - Otherwise busy=0 (consumer can poll and expect "free" to mean free).
  assign a_busy = a_pending | (grant_valid &  grant_is_a);
  assign b_busy = b_pending | (grant_valid & ~grant_is_a);

  // ----- Underlying psram core -----
  psram #(
      .CLOCK_SPEED(CLOCK_SPEED)
  ) core (
      .clk(clk),

      .bank_sel(sel_bank_sel),
      .addr(sel_addr),

      .write_en(core_write_en),
      .data_in(sel_data_in),
      .write_high_byte(sel_ub),
      .write_low_byte(sel_lb),

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
