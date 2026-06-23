// psram_arbiter — 2-master wrapper around one psram core.
//
// One physical PSRAM chip (cram1) was previously owned exclusively by the
// `aram` psram instance, which forced any new master onto a different chip
// or onto SDRAM.  This wrapper lets two masters share the same psram core:
//
//   Port A (priority): ARAM (SPC700 audio RAM) — bank_sel=0, the original
//     consumer.  Hit by SMP at ~1.024 MHz and the DSP at clk_sys rate.
//   Port B: savestate — bank_sel=1, a private 8 MB region with no other
//     consumer.  Savestate stage+serve land here.
//
// ## Why drop-on-busy and not queue
//
// The ORIGINAL standalone psram core (psram.sv) drops any rd/wr request
// that arrives while the core is busy with a previous one — it only
// samples write_en/read_en when state==STATE_NONE.  Sustained access
// from the DSP (which cycles RAM access at clk_sys rate, ~4 clk_mem
// cycles per state) was already faster than psram's 7-cycle transactions
// even without an arbiter.  Audio works in the original config because
// the consumer (DSP) tolerates the occasional dropped access — psram's
// `data_out` is a latched register that retains the last value, so the
// DSP just gets the previous read result occasionally.
//
// This arbiter preserves that same contract for Port A: if the core is
// busy when A's request edge arrives, A is silently dropped — exactly
// as the original psram would have done.  This keeps the DSP/ARAM
// timing contract intact while giving Port B (savestate) its own
// dedicated transaction stream.
//
// Earlier attempts queued A's request with a "pending" latch + edge
// capture so no access was lost.  That broke audio: by the time the
// arbiter eventually serviced the queued A request, the DSP had moved
// on to a different state with different RAM_A / RAM_DO values, and
// the captured-at-edge values landed in psram with wrong data or were
// sampled too late relative to the DSP's expected window.
//
// Port B requests are also drop-on-busy at the arbiter level, but
// ss_psram_arbiter (on the clk_sys side) uses a toggle-CDC handshake
// and retries via the SS_WR_ISSUE / SS_RD_ISSUE state's busy gate, so
// no savestate data is ever lost.

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

  // ----- Direct request flags (no edge detection, no queueing) -----
  wire a_req = a_write_en | a_read_en;
  wire b_req = b_write_en | b_read_en;

  // psram core's busy signal.
  wire core_busy;

  // ----- Grant tracking (declarations hoisted above first use) -----
  reg grant_valid    = 1'b0;
  reg grant_is_a     = 1'b0;
  reg grant_was_read = 1'b0;
  reg saw_busy       = 1'b0;

  // ----- Arbitration -----
  // Two asymmetric contracts:
  //   Port A (DSP/ARAM): drop-on-busy.  If the core is busy when A's
  //     request fires, A is silently dropped — matching the original
  //     psram's behavior.  DSP tolerates this; data_out is a latched
  //     register that retains the previous read value.
  //   Port B (savestate): NEVER dropped.  Savestate cannot lose writes
  //     or reads — every byte matters.
  //
  // To honor both, we use round-robin-ish fairness: when the core is
  // idle and BOTH ports want service, alternate between them via a
  // last_served flag.  When only one wants service, it gets it.
  //
  // The OLD logic was "A wins ties + drop A on busy" which permanently
  // starved B whenever A's level signal stayed asserted across many
  // clk_mem cycles (the DSP holds RAM_CE_N low across all states),
  // causing the APF "Save Failed" timeout when ss_psram_arbiter could
  // never get a transaction through.
  reg last_served_a = 1'b0;  // 1 = A was last serviced; prefer B next
  wire can_start = ~core_busy;
  // If both ports want service, alternate based on who went last.
  // If only one wants service, give it to that one.
  wire prefer_a = a_req & (~b_req | last_served_a == 1'b0);
  wire prefer_b = b_req & (~a_req | last_served_a == 1'b1);
  wire start_a  = can_start &  prefer_a;
  wire start_b  = can_start & ~prefer_a & prefer_b;

  always @(posedge clk) begin
    if (start_a) last_served_a <= 1'b1;
    else if (start_b) last_served_a <= 1'b0;
  end

  // ----- Drive psram core directly from live port signals -----
  // The selected port's signals flow straight into the core — same
  // semantics as the original solo psram instance.  No capture register
  // is needed because the consumer is expected to hold its lines stable
  // for the entire access (psram.sv samples data_in at state 3, ~35 ns
  // after the request edge; ARAM holds the lines longer than that).
  wire        sel_bank_sel        = start_a ? a_bank_sel        : b_bank_sel;
  wire [21:0] sel_addr            = start_a ? a_addr            : b_addr;
  wire [15:0] sel_data_in         = start_a ? a_data_in         : b_data_in;
  wire        sel_write_high_byte = start_a ? a_write_high_byte : b_write_high_byte;
  wire        sel_write_low_byte  = start_a ? a_write_low_byte  : b_write_low_byte;
  wire        core_write_en       = (start_a & a_write_en) | (start_b & b_write_en);
  wire        core_read_en        = (start_a & a_read_en)  | (start_b & b_read_en);

  // ----- Grant clocked block -----
  // Grant tracking serves two purposes:
  //  1) Route read_avail / data_out to the requesting port (the consumer
  //     sees its read result with no extra latency vs solo psram).
  //  2) Prevent a fresh request from interrupting an in-flight one.
  //
  // To match original psram's timing (no dead cycle between accesses), we
  // clear grant_valid the SAME cycle core_busy falls, not the cycle after.
  // saw_busy is still required to handle the cycle just after start_X
  // when core_busy hasn't yet risen — without it, ~core_busy would be
  // true the cycle right after start and grant_valid would immediately
  // drop.
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
  // Each port's busy reflects whether THAT port's transaction is in
  // flight or starting this cycle.  ss_psram_arbiter polls b_busy in
  // SS_WR_ISSUE / SS_RD_ISSUE to gate the write_en/read_en pulse; if
  // b_busy is high (Port B in flight), it stalls.  Doesn't include the
  // OTHER port's transaction state.
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
