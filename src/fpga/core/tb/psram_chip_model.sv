// psram_chip_model — behavioral model of one CRAM chip, sized to match
// the openFPGA Pocket's 64Mb×2 dual-die parts.
//
// Models the async cellular-RAM interface that psram.sv drives:
//   1. Host pulls ce*_n low (bank select via ce0_n vs ce1_n).
//   2. Host pulls adv_n low while presenting upper-6 addr bits on cram_a
//      and lower-16 addr bits on cram_dq.  Host then raises adv_n; we
//      latch the full 22-bit address on the rising edge of adv_n.
//   3. WRITE: host stops driving dq (we tri-state too), then drives data
//      with we_n low + ub_n/lb_n low for the bytes to commit.  We sample
//      data on the rising edge of we_n.
//      READ: host pulls oe_n low; we drive the latched word onto dq.
//
// Sparse storage keyed by linear word index = {bank_sel, addr[21:0]} so
// each die's 8 MB lives in its own keyspace.  cram_wait is tied low (we
// never insert wait states — real psram supports it but psram.sv doesn't
// drive accesses that need it).
//
// Per-bank write counters help the TB assert priority and bank separation.

`timescale 1ps / 1ps

module psram_chip_model (
    input  wire        clk,         // the controller's clock (matches psram.sv)

    // PSRAM pins driven by psram.sv (host side):
    input  wire [21:16] cram_a,
    inout  wire [15:0]  cram_dq,
    output wire         cram_wait,  // we never stall
    input  wire         cram_clk,   // unused (async core)
    input  wire         cram_adv_n,
    input  wire         cram_cre,
    input  wire         cram_ce0_n,
    input  wire         cram_ce1_n,
    input  wire         cram_oe_n,
    input  wire         cram_we_n,
    input  wire         cram_ub_n,
    input  wire         cram_lb_n
);

  assign cram_wait = 1'b0;

  // -------------------------------------------------------------------
  // Sparse memory: linear key = {bank_sel, addr[21:0]} (23 bits).
  // bank_sel is derived from which ce*_n is asserted at adv_n's rising edge.
  // -------------------------------------------------------------------
  logic [15:0] mem [longint unsigned];

  // Latched per-transaction state.
  reg        bank_latched      = 1'b0;
  reg [21:0] addr_latched      = 22'h0;
  reg        addr_valid        = 1'b0;
  reg        prev_adv_n        = 1'b1;
  reg        prev_we_n         = 1'b1;
  reg        prev_ce0_n        = 1'b1;
  reg        prev_ce1_n        = 1'b1;

  // Read drive: when oe_n is low and ce is asserted, present data on dq.
  // psram.sv reads back the value at STATE_READ_DATA_RECEIVED; we hold
  // the data for as long as oe_n is low, which is plenty.
  wire bank_active = ~cram_ce0_n | ~cram_ce1_n;
  wire bank_sel_now = ~cram_ce1_n;  // 1 if bank 1 (ce1_n low), else 0
  wire drive_read   = ~cram_oe_n & bank_active & addr_valid;

  reg [15:0] read_data = 16'h0000;
  always_comb begin
    longint unsigned key;
    if (addr_valid) begin
      key = {1'b0, bank_latched, addr_latched};
      read_data = mem.exists(key) ? mem[key] : 16'hDEAD;
    end else begin
      read_data = 16'hDEAD;
    end
  end

  assign cram_dq = drive_read ? read_data : 16'hzzzz;

  // -------------------------------------------------------------------
  // Instrumentation
  // -------------------------------------------------------------------
  int total_writes_bank0 = 0;
  int total_writes_bank1 = 0;
  int total_reads_bank0  = 0;
  int total_reads_bank1  = 0;
  int log_remaining = 16;

  // -------------------------------------------------------------------
  // Behavior: react to control-line edges.  We poll on the controller's
  // clock since psram.sv's FSM ticks per-clk.  Capture at the rising
  // edge of adv_n (address latch) and rising edge of we_n (data commit).
  // -------------------------------------------------------------------
  always @(posedge clk) begin
    longint unsigned key;

    // Address latch: rising edge of adv_n while a bank is selected.
    if (cram_adv_n & ~prev_adv_n & bank_active) begin
      bank_latched <= bank_sel_now;
      addr_latched <= {cram_a, cram_dq};
      addr_valid   <= 1'b1;
    end

    // Write commit: rising edge of we_n while addr is latched.  ub_n/lb_n
    // pick which byte(s) actually update.  Note: psram.sv asserts we_n
    // low BEFORE the data is on dq (it drives addr-on-dq during adv), so
    // we cannot sample data on we_n's falling edge — we sample on rising.
    if (cram_we_n & ~prev_we_n & addr_valid) begin
      key = {1'b0, bank_latched, addr_latched};
      if (!mem.exists(key)) mem[key] = 16'h0000;
      if (~cram_ub_n) mem[key][15:8] = cram_dq[15:8];
      if (~cram_lb_n) mem[key][7:0]  = cram_dq[7:0];
      if (bank_latched) total_writes_bank1++;
      else              total_writes_bank0++;
      if (log_remaining > 0) begin
        $display("[psram_chip] WR bank=%0d addr=%h data=%h (ub=%b lb=%b) t=%0t",
                 bank_latched, addr_latched, cram_dq, ~cram_ub_n, ~cram_lb_n, $time);
        log_remaining--;
      end
    end

    // Read accounting: count one read per access (deasserting ce).
    // Detected when both ce lines rise — chip is being released.
    if ((cram_ce0_n & ~prev_ce0_n) | (cram_ce1_n & ~prev_ce1_n)) begin
      if (addr_valid & ~cram_oe_n) begin
        if (bank_latched) total_reads_bank1++;
        else              total_reads_bank0++;
      end
    end

    // When the chip is fully deselected (both ce lines high), invalidate
    // the latch so a stale addr can't bleed into the next access.
    if (cram_ce0_n & cram_ce1_n) addr_valid <= 1'b0;

    prev_adv_n <= cram_adv_n;
    prev_we_n  <= cram_we_n;
    prev_ce0_n <= cram_ce0_n;
    prev_ce1_n <= cram_ce1_n;
  end

  // -------------------------------------------------------------------
  // Public helper for the TB: peek a word at (bank, addr) without going
  // through the bus.
  // -------------------------------------------------------------------
  function automatic logic [15:0] peek(input bit bank, input [21:0] addr);
    longint unsigned key;
    key = {1'b0, bank, addr};
    peek = mem.exists(key) ? mem[key] : 16'hDEAD;
  endfunction

endmodule
