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
  // ----- Lossless Port A WRITES (2026-06-26 fix) -----
  // Port A WRITES must never be dropped: besides live SMP writes, Port A
  // carries the savestate LOAD restore writing ARAM at DMA rate.  Under
  // contention with Port B (the firmware simultaneously reading the staged
  // savestate), drop-on-busy silently discarded ~95% of ARAM restore writes
  // → corrupt APU state → black screen + no audio.  So we LATCH each Port A
  // write request (rising edge of a_write_en) with its parameters and hold
  // it pending until serviced — guaranteeing zero loss.
  //
  // Port A READS keep drop-on-busy (the live SMP/DSP tolerates dropped reads
  // — latching a read late just yields stale data anyway, and the previous
  // pending-read attempt broke audio).  Port B keeps its round-robin slot
  // (its own ss_psram_arbiter retry handshake makes it lossless end-to-end).
  reg        prev_a_write_en = 1'b0;
  reg        aw_pending  = 1'b0;
  reg        aw_cap_bank = 1'b0;
  reg [21:0] aw_cap_addr = 22'd0;
  reg [15:0] aw_cap_data = 16'd0;
  reg        aw_cap_ub   = 1'b0;
  reg        aw_cap_lb   = 1'b0;
  wire a_write_edge = a_write_en & ~prev_a_write_en;

  reg last_served_a = 1'b0;  // 1 = A was last serviced; prefer B next

  // Aggregate "wants service" per port.  Port A's write is represented by
  // the pending latch (lossless); Port A's read + Port B stay live-level.
  wire a_wants = aw_pending | a_read_en;
  wire b_wants = b_req;

  wire can_start = ~core_busy & ~grant_valid;
  // Pending Port A WRITE has top priority (it cannot be dropped and must
  // not be starved).  Otherwise round-robin between A-read and B.
  wire start_aw = can_start & aw_pending;
  wire prefer_a = a_read_en & (~b_wants | last_served_a == 1'b0);
  wire prefer_b = b_wants   & (~a_read_en | last_served_a == 1'b1);
  wire start_ar = can_start & ~aw_pending &  prefer_a;
  wire start_b  = can_start & ~aw_pending & ~prefer_a & prefer_b;
  wire start_a  = start_aw | start_ar;   // any Port A transaction this cycle

  always @(posedge clk) begin
    prev_a_write_en <= a_write_en;
    // Capture a Port A write at its request edge; hold pending until served.
    // (Edge-detect so a sustained a_write_en across consecutive same-cycle
    // writes still produces exactly one pending request per distinct write —
    // the ARAM DMA pulses WE_N per byte, giving one edge per byte.)
    if (a_write_edge) begin
      aw_pending  <= 1'b1;
      aw_cap_bank <= a_bank_sel;
      aw_cap_addr <= a_addr;
      aw_cap_data <= a_data_in;
      aw_cap_ub   <= a_write_high_byte;
      aw_cap_lb   <= a_write_low_byte;
    end else if (start_aw) begin
      aw_pending <= 1'b0;     // consumed (only clear if no new edge same cycle)
    end
    if (start_a)  last_served_a <= 1'b1;
    else if (start_b) last_served_a <= 1'b0;
  end

  // ----- Drive psram core -----
  // A pending write replays from its capture registers; an A read and a B
  // access drive from live signals (their consumers hold lines stable for
  // the access window, or tolerate drops).
  wire        sel_bank_sel        = start_aw ? aw_cap_bank
                                  : start_ar ? a_bank_sel : b_bank_sel;
  wire [21:0] sel_addr            = start_aw ? aw_cap_addr
                                  : start_ar ? a_addr     : b_addr;
  wire [15:0] sel_data_in         = start_aw ? aw_cap_data : b_data_in;
  wire        sel_write_high_byte = start_aw ? aw_cap_ub   : b_write_high_byte;
  wire        sel_write_low_byte  = start_aw ? aw_cap_lb   : b_write_low_byte;
  wire        core_write_en       = start_aw | (start_b & b_write_en);
  wire        core_read_en        = (start_ar & a_read_en) | (start_b & b_read_en);

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
      grant_was_read <= start_ar & a_read_en;  // a pending WRITE is never a read
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
  // CRITICAL: the psram core has a SINGLE data_out register that holds the
  // result of the most recent read regardless of which port issued it.
  // The original solo psram had one consumer, so that was always its data.
  // With two masters sharing the core, a Port B (savestate) read would
  // overwrite core_data_out — and ARAM (Port A) reads core_data_out
  // CONTINUOUSLY and ungated (ARAM_Q = aram_16_out = a_data_out in
  // SNES.sv).  So a savestate serve-read would feed savestate bytes into
  // ARAM_Q → the SMP/DSP reads garbage from "ARAM" → audio corruption that
  // persists (the SMP writes the garbage back into ARAM).
  //
  // Fix: give each port its OWN latched data register.  When a port's read
  // completes (its grant + read_avail), capture core_data_out into that
  // port's latch.  Each consumer then only ever sees its own last read.
  wire        core_read_avail;
  wire [15:0] core_data_out;
  wire        a_read_complete = grant_valid &  grant_is_a & grant_was_read & core_read_avail;
  wire        b_read_complete = grant_valid & ~grant_is_a & grant_was_read & core_read_avail;

  // Port A (ARAM) reads core_data_out CONTINUOUSLY and ungated in SNES.sv
  // (ARAM_Q = aram_16_out = a_data_out).  It must therefore be insulated
  // from Port B's reads, which also land on the shared core_data_out
  // register.  Latch A's own last-read value so a savestate serve-read
  // can never bleed into ARAM_Q.  The +1 clk_mem latency (~11 ns) is far
  // inside the consumer's clk_sys sample window (~46 ns).
  reg  [15:0] a_data_latched = 16'h0000;
  always @(posedge clk) begin
    if (a_read_complete) a_data_latched <= core_data_out;
  end
  assign a_read_avail = a_read_complete;
  assign a_data_out   = a_data_latched;

  // Port B (savestate) consumer (ss_psram_arbiter) captures b_data_out on
  // the same cycle b_read_avail pulses, so it must see core_data_out
  // DIRECTLY (no extra latch cycle) — a latch here shifts every served
  // word by one, scrambling the serve stream.  B never reads ungated, so
  // it doesn't need the protection A does.
  assign b_read_avail = b_read_complete;
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
