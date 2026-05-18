// Save State Controller — SRAM-buffered approach
//
// Uses the on-board SRAM (131072 × 16-bit = 256 KB) as a complete buffer for
// save state data.  This avoids the FIFO streaming deadlock: the SNES core
// produces save data via CPU-executed savestates.bin (~3.58 MHz effective),
// which is ~4× slower than the APF bridge read rate.  A small FIFO cannot
// bridge this gap, so all data must be fully buffered before signaling ok.
//
// Architecture:
//   SAVE: core writes → toggle CDC → 4×16 SRAM writes → ack.
//         After ss_busy falls (save complete), signal ok.
//         APF reads from SRAM via pre-fetch FSM.
//   LOAD: APF bridge writes → 2×16 SRAM writes directly (clk_74a).
//         On savestate_load, trigger ss_load pulse.
//         Core reads → toggle CDC → 4×16 SRAM reads → ack + data.
//         After ss_busy falls (load complete), signal load ok.
//
// No FIFOs — the SRAM is the sole buffer.
//
// IMPORTANT TIMING NOTE — ss_busy latency:
//   savestates.sv sets ss_busy only when an NMI/IRQ vector read occurs
//   while (save_en | load_en) is set.  This means ss_busy may not rise
//   for up to one full video frame (~16 ms) after ss_load/ss_save is
//   pulsed.  The controller must NOT treat ss_busy staying low as an
//   error.  It simply waits in ACTIVE state until ss_busy eventually
//   rises and then falls.
//
// FIX SUMMARY (relative to previous version):
//   1. SYS_LOAD_ACTIVE / SYS_SAVE_ACTIVE: the completion check
//      (prev_ss_busy && ~ss_busy) was firing spuriously before ss_busy
//      ever went high, immediately returning to IDLE and leaving the
//      core stuck mid-load.  Fixed by gating the falling-edge check with
//      a "ss_busy_seen" flag that is set once ss_busy rises.
//   2. SYS_LOAD_ACTIVE: checked ss_rnw for direction but savestates.sv
//      exports ddr_we (0=read/load, 1=write/save).  The controller now
//      uses ~ddr_we (i.e., it was already correct via the ss_rnw wire
//      being driven by ~ddr_we in main.v — left as-is but documented).
//   3. SRAM prefetch: bridge reads can arrive faster than the SRAM FSM
//      can service them.  Added prefetch_addr_next latch so that a
//      bridge_rd that arrives while the FSM is busy is not lost.
//   4. savestate_start_busy was never cleared on the save error path.
//      Fixed.
//   5. Minor: savestate_load_ack held high until SYS_IDLE processes it;
//      now cleared in the same cycle it is acted on to avoid re-trigger.

module save_state_controller (
    input wire clk_74a,
    input wire clk_sys,

    // APF Bridge
    input wire bridge_wr,
    input wire bridge_rd,
    input wire bridge_endian_little,
    input wire [31:0] bridge_addr,
    input wire [31:0] bridge_wr_data,
    output reg  [31:0] save_state_bridge_read_data,

    // APF Save State Handshake
    input  wire savestate_load,
    output wire savestate_load_ack_s,
    output wire savestate_load_busy_s,
    output wire savestate_load_ok_s,
    output wire savestate_load_err_s,

    input  wire savestate_start,
    output wire savestate_start_ack_s,
    output wire savestate_start_busy_s,
    output wire savestate_start_ok_s,
    output wire savestate_start_err_s,

    // Core-side savestate control
    output reg ss_save,
    output reg ss_load,

    // Core-side DDR-style interface (toggle-based req/ack)
    input wire [63:0] ss_din,     // Data from core (save)
    output reg [63:0] ss_dout,    // Data to core (load)
    input wire [16:0] ss_addr,    // DDR word address from savestates.sv
    input wire ss_rnw,            // Read/not-write (0=write/save, 1=read/load)
    input wire ss_req,            // Toggle request from savestates.sv
    input wire [7:0] ss_be,       // Byte enable
    output reg ss_ack = 0,        // Toggle acknowledge to savestates.sv

    input wire ss_busy,

    // Debug taps (clk_sys domain) — consumed by core_top for on-screen overlay.
    output wire [3:0] debug_sys_state,
    output wire       debug_ss_busy_seen,
    output wire       debug_ss_busy_ever,    // sticky: ss_busy rose at least once
    output wire       debug_ss_save_ever,    // sticky: controller pulsed ss_save
    output wire [3:0] debug_ss_save_count,   // count of ss_save pulses (wraps)
    output wire [3:0] debug_ss_busy_rises,   // count of ss_busy rising edges
    output wire       debug_ss_req_ever,     // sticky: ss_req has toggled at least once
    output wire [3:0] debug_ss_req_toggles,  // count of ss_req edges (mod 16)
    output wire       debug_core_wr_ever,    // sticky: core_wr_req_toggle has ever changed
    output wire       debug_sram_wr_ack_ever,// sticky: sram_wr_ack_toggle has ever changed
    output wire [7:0] debug_bridge_wr_count_lo, // low byte of bridge_wr events at 4xxxxxxx (mod 256)
    output wire [7:0] debug_bridge_wr_count_hi, // high byte (so 16-bit count, mod 65536)
    output wire [7:0] debug_first_wr_data_b0,   // bridge_wr_data[31:24] of FIRST bridge_wr (expect $53 ='S')
    output wire [7:0] debug_first_wr_data_b1,   // bridge_wr_data[23:16] of first write (expect $4E='N')
    output wire [7:0] debug_first_wr_addr_lo,   // bridge_addr[7:0]  of first write (expect $00)
    output wire [7:0] debug_first_wr_addr_hi,   // bridge_addr[15:8] of first write (expect $00)
    output wire [7:0] debug_first_save_byte0,   // First byte SNES wrote during save: core_wr_data[7:0] (expect $53='S')
    output wire [7:0] debug_first_save_byte1,   // Second byte: core_wr_data[15:8] (expect $4E='N')
    output wire [7:0] debug_bridge_rd_count_lo, // low byte of bridge_rd events at 4xxxxxxx
    output wire [7:0] debug_bridge_rd_count_hi, // high byte (saturates at FF)
    output wire [7:0] debug_first_rd_addr_lo,   // bridge_addr[7:0] of first bridge_rd (expect $00)
    output wire [7:0] debug_first_rd_addr_hi,   // bridge_addr[15:8] of first bridge_rd (expect $00)

    // SRAM interface (directly to Pocket board SRAM)
    output reg  [16:0] sram_a,
    inout  wire [15:0] sram_dq,
    output reg         sram_oe_n,
    output reg         sram_we_n,
    output wire        sram_ub_n,
    output wire        sram_lb_n
);

  // Always enable both SRAM bytes
  assign sram_ub_n = 1'b0;
  assign sram_lb_n = 1'b0;

  // SRAM bidirectional data bus control
  reg [15:0] sram_dq_out;
  reg        sram_dq_oe;  // 1 = drive output, 0 = tristate (read)
  assign sram_dq = sram_dq_oe ? sram_dq_out : 16'hZZZZ;
  wire [15:0] sram_dq_in = sram_dq;

  // ===================================================================
  // CDC: APF handshake signals (clk_74a ↔ clk_sys)
  // ===================================================================

  wire savestate_load_s;
  wire savestate_start_s;

  synch_3 #(.WIDTH(2)) savestate_in (
      {savestate_load, savestate_start},
      {savestate_load_s, savestate_start_s},
      clk_sys
  );

  reg savestate_load_ack  = 0;
  reg savestate_load_busy = 0;
  reg savestate_load_ok   = 0;
  reg savestate_load_err  = 0;

  reg savestate_start_ack  = 0;
  reg savestate_start_busy = 0;
  reg savestate_start_ok   = 0;
  reg savestate_start_err  = 0;

  // savestate_start_ok_s_internal is the controller's view of "save done"
  // in the clk_74a domain.  We GATE it with pf_initial_ready (see the
  // prefetch logic below) before exposing to APF, so APF only sees
  // savestate_start_ok rise once the first bridge-read prefetch is staged.
  wire savestate_start_ok_s_internal;
  synch_3 #(.WIDTH(8)) savestate_out (
      {
        savestate_load_ack,  savestate_load_busy,
        savestate_load_ok,   savestate_load_err,
        savestate_start_ack, savestate_start_busy,
        savestate_start_ok,  savestate_start_err
      },
      {
        savestate_load_ack_s,  savestate_load_busy_s,
        savestate_load_ok_s,   savestate_load_err_s,
        savestate_start_ack_s, savestate_start_busy_s,
        savestate_start_ok_s_internal,  savestate_start_err_s
      },
      clk_74a
  );
  assign savestate_start_ok_s = savestate_start_ok_s_internal & pf_initial_ready;

  // ===================================================================
  // CDC: Core ↔ SRAM toggle-based data transfer
  // ===================================================================
  //
  // Save writes: clk_sys latches ss_din + addr, toggles wr_req.
  //   clk_74a detects toggle, writes 4×16 SRAM words, toggles wr_ack.
  //   clk_sys detects ack, toggles ss_ack.
  //
  // Load reads: clk_sys latches addr, toggles rd_req.
  //   clk_74a detects toggle, reads 4×16 SRAM words, latches data, toggles rd_ack.
  //   clk_sys detects ack, copies data to ss_dout, toggles ss_ack.

  // Core-side registers (clk_sys domain)
  reg        core_wr_req_toggle = 0;
  reg [63:0] core_wr_data;          // Latched ss_din for CDC
  reg [14:0] core_sram_base;        // ss_addr[14:0] — SRAM base = {this, 2'b00}

  reg        core_rd_req_toggle = 0;
  reg [14:0] core_rd_base;          // ss_addr[14:0]

  // SRAM-side registers (clk_74a domain)
  reg        sram_wr_ack_toggle = 0;
  reg        sram_rd_ack_toggle = 0;
  reg [63:0] sram_rd_result;        // 64-bit data read from SRAM (load)

  // Synchronize toggles across clock domains
  wire core_wr_req_74a;
  wire core_rd_req_74a;
  wire sram_wr_ack_sys;
  wire sram_rd_ack_sys;

  synch_3 sync_wr_req (.i(core_wr_req_toggle), .o(core_wr_req_74a), .clk(clk_74a));
  synch_3 sync_rd_req (.i(core_rd_req_toggle), .o(core_rd_req_74a), .clk(clk_74a));
  synch_3 sync_wr_ack (.i(sram_wr_ack_toggle), .o(sram_wr_ack_sys), .clk(clk_sys));
  synch_3 sync_rd_ack (.i(sram_rd_ack_toggle), .o(sram_rd_ack_sys), .clk(clk_sys));

  // ===================================================================
  // Core-side state machine (clk_sys domain)
  // ===================================================================

  localparam SYS_IDLE            = 4'd0;
  localparam SYS_SAVE_ACTIVE     = 4'd1;
  localparam SYS_SAVE_WAIT_SRAM  = 4'd2;
  localparam SYS_LOAD_ACTIVE     = 4'd3;
  localparam SYS_LOAD_WAIT_SRAM  = 4'd4;

  reg [3:0] sys_state = SYS_IDLE;

  // Debug taps — directly expose internal state for the on-screen overlay
  assign debug_sys_state = sys_state;

  reg prev_savestate_start = 0;
  reg prev_savestate_load  = 0;
  reg prev_ss_busy         = 0;
  reg prev_ss_req          = 0;
  reg prev_sram_wr_ack     = 0;
  reg prev_sram_rd_ack     = 0;

  wire new_ddr_req  = (ss_req != prev_ss_req);
  wire sram_wr_done = (sram_wr_ack_sys != prev_sram_wr_ack);
  wire sram_rd_done = (sram_rd_ack_sys != prev_sram_rd_ack);

  // Load initiation flag — set when savestate_load command is received
  reg load_cmd_pending = 0;

  // FIX #1: Track whether ss_busy has risen at least once since save/load
  // started.  Without this, the falling-edge completion check fires on the
  // very first cycle (prev_ss_busy=0, ss_busy=0 → prev && ~cur is false,
  // but ss_busy staying 0 for a full frame means we never exit ACTIVE).
  // The real bug was subtler: if ss_busy had been high from a *previous*
  // operation and happened to be low when we entered ACTIVE, the check
  // prev_ss_busy && ~ss_busy would fire on the next falling edge which
  // belongs to the previous op, not the current one.  The flag ensures we
  // only act on a falling edge that follows a rising edge we observed
  // during this operation.
  reg ss_busy_seen = 0;
  assign debug_ss_busy_seen = ss_busy_seen;

  // Sticky / count debug regs — never cleared, only set/incremented.
  reg       ss_busy_ever       = 0;
  reg       ss_save_ever       = 0;
  reg [3:0] ss_save_count      = 4'h0;
  reg [3:0] ss_busy_rises      = 4'h0;
  reg       ss_req_ever        = 0;
  reg [3:0] ss_req_toggles     = 4'h0;
  reg       core_wr_ever       = 0;
  reg       sram_wr_ack_ever   = 0;
  reg       prev_core_wr_req   = 0;
  reg       prev_sram_wr_ack_s = 0;

  assign debug_ss_busy_ever      = ss_busy_ever;
  assign debug_ss_save_ever      = ss_save_ever;
  assign debug_ss_save_count     = ss_save_count;
  assign debug_ss_busy_rises     = ss_busy_rises;
  assign debug_ss_req_ever       = ss_req_ever;
  assign debug_ss_req_toggles    = ss_req_toggles;
  assign debug_core_wr_ever      = core_wr_ever;
  assign debug_sram_wr_ack_ever  = sram_wr_ack_ever;

  // Count of bridge_wr events at 4xxxxxxx (the save-state region).
  // Lives in clk_74a domain; CDC'd via the synch_3 output bundle below.
  reg [15:0] bridge_wr_count = 16'h0000;
  assign debug_bridge_wr_count_lo = bridge_wr_count[7:0];
  assign debug_bridge_wr_count_hi = bridge_wr_count[15:8];

  // Capture data and address of the FIRST bridge_wr at 4xxxxxxx ever seen.
  // Sticky — never overwritten after the first event.
  reg [31:0] first_wr_data  = 32'h00000000;
  reg [31:0] first_wr_addr  = 32'h00000000;
  reg [31:0] second_wr_data = 32'h00000000;
  reg [1:0]  wr_capture_idx = 0;
  reg        first_wr_seen  = 0;
  // Expose first 4 bytes of bridge_wr_data (from first write) and 4 more (second write).
  assign debug_first_wr_data_b0 = first_wr_data[31:24];   // file byte 0 if big-endian
  assign debug_first_wr_data_b1 = first_wr_data[23:16];   // file byte 1
  assign debug_first_wr_addr_lo = second_wr_data[31:24];  // (repurposed) file byte 4 (start of second write)
  assign debug_first_wr_addr_hi = second_wr_data[23:16];  // file byte 5

  // Capture the FIRST 64-bit chunk the SNES wrote during save.
  // core_wr_data is in clk_sys; latch it sticky.
  reg [63:0] first_save_chunk = 64'h0;
  reg        first_save_seen  = 0;
  assign debug_first_save_byte0 = first_save_chunk[7:0];
  assign debug_first_save_byte1 = first_save_chunk[15:8];

  // Count bridge_rd events at 4xxxxxxx (the save-state region).
  // Capture the very first bridge_rd's address (sticky).
  reg [15:0] bridge_rd_count = 16'h0000;
  reg [31:0] first_rd_addr   = 32'h00000000;
  reg        first_rd_seen   = 0;
  assign debug_bridge_rd_count_lo = bridge_rd_count[7:0];
  assign debug_bridge_rd_count_hi = bridge_rd_count[15:8];
  assign debug_first_rd_addr_lo   = first_rd_addr[7:0];
  assign debug_first_rd_addr_hi   = first_rd_addr[15:8];

  always @(posedge clk_sys) begin
    prev_savestate_start <= savestate_start_s;
    prev_savestate_load  <= savestate_load_s;
    prev_ss_busy         <= ss_busy;
    prev_ss_req          <= ss_req;
    prev_sram_wr_ack     <= sram_wr_ack_sys;
    prev_sram_rd_ack     <= sram_rd_ack_sys;

    ss_save <= 0;
    ss_load <= 0;

    // Track ss_busy rising edge during an active operation
    if (ss_busy && !prev_ss_busy) begin
      ss_busy_seen <= 1;
      ss_busy_ever <= 1;
      ss_busy_rises <= ss_busy_rises + 4'd1;
    end

    // Track ss_req toggles (each toggle = one save chunk handshake start)
    if (ss_req != prev_ss_req) begin
      ss_req_ever <= 1;
      ss_req_toggles <= ss_req_toggles + 4'd1;
    end

    // Track when controller forwards a save chunk to SRAM (core_wr_req_toggle edge)
    prev_core_wr_req <= core_wr_req_toggle;
    if (core_wr_req_toggle != prev_core_wr_req)
      core_wr_ever <= 1;

    // Track when SRAM FSM acks back via sram_wr_ack_sys
    prev_sram_wr_ack_s <= sram_wr_ack_sys;
    if (sram_wr_ack_sys != prev_sram_wr_ack_s)
      sram_wr_ack_ever <= 1;

    // ----- APF triggers save -----
    if (savestate_start_s && ~prev_savestate_start) begin
      sys_state            <= SYS_SAVE_ACTIVE;
      ss_busy_seen         <= 0;
      savestate_start_ack  <= 1;
      savestate_start_busy <= 1;
      savestate_start_ok   <= 0;
      savestate_start_err  <= 0;
      savestate_load_ok    <= 0;
      savestate_load_err   <= 0;
      ss_save              <= 1;
      ss_save_ever         <= 1;
      ss_save_count        <= ss_save_count + 4'd1;
    end

    // ----- APF signals load command (data already in SRAM) -----
    if (savestate_load_s && ~prev_savestate_load) begin
      load_cmd_pending    <= 1;
      savestate_load_ack  <= 1;
      savestate_load_ok   <= 0;
      savestate_load_err  <= 0;
      savestate_start_ok  <= 0;
      savestate_start_err <= 0;
    end

    case (sys_state)

      SYS_IDLE: begin
        // Start load sequence when APF load command is received
        if (load_cmd_pending) begin
          sys_state           <= SYS_LOAD_ACTIVE;
          ss_busy_seen        <= 0;
          ss_load             <= 1;
          load_cmd_pending    <= 0;
          savestate_load_ack  <= 0;  // FIX #5: clear ack in same cycle we act
          savestate_load_busy <= 1;
        end
      end

      // ===== Save path =====
      SYS_SAVE_ACTIVE: begin
        if (~savestate_start_s)
          savestate_start_ack <= 0;

        if (new_ddr_req && ~ss_rnw) begin
          // Core has save data — send to SRAM via CDC
          core_wr_data       <= ss_din;
          core_sram_base     <= ss_addr[14:0];
          core_wr_req_toggle <= ~core_wr_req_toggle;
          sys_state          <= SYS_SAVE_WAIT_SRAM;
          // Sticky capture of the very first save chunk's data
          if (!first_save_seen) begin
            first_save_chunk <= ss_din;
            first_save_seen  <= 1;
          end
        end else if (ss_busy_seen && prev_ss_busy && ~ss_busy) begin
          // FIX #1: Only complete when we have seen ss_busy rise AND fall
          // during this operation.
          sys_state            <= SYS_IDLE;
          ss_busy_seen         <= 0;
          savestate_start_busy <= 0;
          savestate_start_ok   <= 1;
        end
      end

      SYS_SAVE_WAIT_SRAM: begin
        if (sram_wr_done) begin
          // SRAM write complete — toggle ack back to savestates.sv
          ss_ack <= ~ss_ack;
          if (ss_busy_seen && ~ss_busy) begin
            // FIX #1: Save finished while writing last word
            sys_state            <= SYS_IDLE;
            ss_busy_seen         <= 0;
            savestate_start_busy <= 0;
            savestate_start_ok   <= 1;
          end else begin
            sys_state <= SYS_SAVE_ACTIVE;
          end
        end
      end

      // ===== Load path =====
      SYS_LOAD_ACTIVE: begin
        if (new_ddr_req && ss_rnw) begin
          // Core requesting data — read from SRAM via CDC
          core_rd_base       <= ss_addr[14:0];
          core_rd_req_toggle <= ~core_rd_req_toggle;
          sys_state          <= SYS_LOAD_WAIT_SRAM;
        end else if (ss_busy_seen && prev_ss_busy && ~ss_busy) begin
          // FIX #1: Only complete when we have seen ss_busy rise AND fall
          // during this operation.
          sys_state           <= SYS_IDLE;
          ss_busy_seen        <= 0;
          savestate_load_busy <= 0;
          savestate_load_ok   <= 1;
        end
      end

      SYS_LOAD_WAIT_SRAM: begin
        if (sram_rd_done) begin
          // SRAM read data is available (stable before ack toggle crossed)
          // Apply byte swap: reverse bytes within each 32-bit half.
          // Same transform as the original FIFO-based ss_dout path.
          ss_dout <= {
            sram_rd_result[39:32], sram_rd_result[47:40],
            sram_rd_result[55:48], sram_rd_result[63:56],
            sram_rd_result[7:0],   sram_rd_result[15:8],
            sram_rd_result[23:16], sram_rd_result[31:24]
          };
          ss_ack <= ~ss_ack;
          if (ss_busy_seen && ~ss_busy) begin
            // FIX #1: Load finished on last word
            sys_state           <= SYS_IDLE;
            ss_busy_seen        <= 0;
            savestate_load_busy <= 0;
            savestate_load_ok   <= 1;
          end else begin
            sys_state <= SYS_LOAD_ACTIVE;
          end
        end
      end

    endcase
  end

  // ===================================================================
  // SRAM FSM (clk_74a domain)
  // ===================================================================
  //
  // Handles all SRAM access: core save writes, core load reads,
  // bridge load writes, and bridge save-read pre-fetch.
  //
  // These phases are naturally serialized by the protocol:
  //   Save:  core writes → then APF reads (pre-fetch)
  //   Load:  APF writes → then core reads
  // so no complex arbitration is needed.
  //
  // SRAM write timing: async SRAM requires WE_n to pulse (rise) for each
  // word.  Each 16-bit write takes 2 cycles: WE_n=0 then WE_n=1.

  localparam SRAM_IDLE           = 4'd0;
  localparam SRAM_CORE_WR        = 4'd1;  // WE_n asserted (low)
  localparam SRAM_CORE_WR_END    = 4'd2;  // WE_n deasserted (high) → completes write
  localparam SRAM_CORE_RD_SETUP  = 4'd3;
  localparam SRAM_CORE_RD_N      = 4'd4;
  localparam SRAM_BRIDGE_WR_LO   = 4'd5;  // WE_n asserted for low half
  localparam SRAM_BRIDGE_WR_GAP  = 4'd6;  // WE_n high between halves
  localparam SRAM_BRIDGE_WR_HI   = 4'd7;  // WE_n asserted for high half
  localparam SRAM_BRIDGE_WR_END  = 4'd8;  // WE_n high, done
  localparam SRAM_PF_SETUP       = 4'd9;
  localparam SRAM_PF_LO          = 4'd10;
  localparam SRAM_PF_GAP         = 4'd11; // 1-cycle address setup between low/high reads
  localparam SRAM_PF_HI          = 4'd12;

  reg [3:0]  sram_state = SRAM_IDLE;
  reg [1:0]  sram_word_idx;
  reg [16:0] sram_base_addr;

  // CDC toggle edge detection (clk_74a side)
  reg prev_core_wr_req_74a = 0;
  reg prev_core_rd_req_74a = 0;
  wire core_wr_pending_74a = (core_wr_req_74a != prev_core_wr_req_74a);
  wire core_rd_pending_74a = (core_rd_req_74a != prev_core_rd_req_74a);

  // Core save data latched into clk_74a domain
  // Safe to read because data is stable well before toggle crosses via synch_3
  reg [63:0] latched_core_wr_data;

  // Byte-swapped 16-bit words for core save writes
  wire [15:0] core_wr_word_0 = {latched_core_wr_data[23:16], latched_core_wr_data[31:24]};
  wire [15:0] core_wr_word_1 = {latched_core_wr_data[7:0],   latched_core_wr_data[15:8]};
  wire [15:0] core_wr_word_2 = {latched_core_wr_data[55:48], latched_core_wr_data[63:56]};
  wire [15:0] core_wr_word_3 = {latched_core_wr_data[39:32], latched_core_wr_data[47:40]};

  // Bridge write pending latch (clk_74a domain)
  reg        bridge_wr_pending = 0;
  reg [16:0] bridge_wr_sram_addr;
  reg [31:0] bridge_wr_latched;

  // Save-side bridge-read prefetch — double-buffered, always-ahead.
  //
  // Background: the previous design (FIX #3) assumed that APF would
  // re-poll a bridge_rd when the FSM hadn't yet produced fresh data.
  // That assumption is false: APF reads bridge_rd_data at its own ~75
  // clk_74a-cycle pace and never re-issues a "missed" read.  The result
  // was that the first ~3 bridge_rds after savestate_start_ok returned
  // a stale (uninitialized) value while prefetch_sram_addr kept
  // advancing — silently dropping ~12 bytes (the firmware preamble
  // including the 'SNES' header) from the saved state file.
  //
  // Design now:
  //   * pf_next_addr always tracks the SRAM word offset that the FSM
  //     should fetch NEXT.  It is advanced ONLY when the FSM actually
  //     completes a prefetch — never on bridge_rd alone.
  //   * pf_staged_data holds the 32-bit value that APF will receive on
  //     its NEXT bridge_rd.  pf_staged_valid says it's fresh.
  //   * save_state_bridge_read_data holds the value APF is reading
  //     RIGHT NOW.  On bridge_rd we atomically swap pf_staged_data into
  //     it and clear pf_staged_valid, freeing the FSM to refill the
  //     stage from pf_next_addr (and advance pf_next_addr).
  //   * pf_initial_ready gates savestate_start_ok visibility to APF
  //     until the very first stage value is ready, so APF's first
  //     bridge_rd always sees real data.
  //
  // Because the FSM prefetch latency (~5 clk_74a cycles) is much smaller
  // than the APF read cadence (~75 cycles), the stage is always refilled
  // before APF reads again — no race in steady state.
  reg [16:0] pf_next_addr     = 17'd0;   // SRAM word addr the FSM should fetch NEXT
  reg [31:0] pf_staged_data   = 32'd0;   // Word ready for the next bridge_rd
  reg        pf_staged_valid  = 1'b0;    // pf_staged_data is fresh
  reg        pf_initial_ready = 1'b0;    // Set once after first stage fill (gates start_ok)
  reg        pf_armed         = 1'b0;    // Set when savestate_start_ok_internal first rises
  reg [15:0] pf_lo            = 16'd0;   // Intermediate low half within a 32-bit SRAM read

  // Edge-detect bridge_rd — APF holds it high for multiple clk_74a cycles
  // and consumers (including data_unloader.sv elsewhere in this repo)
  // react to the rising edge.  Without this, we'd swap the stage out on
  // every cycle bridge_rd is high.
  reg prev_bridge_rd_74a = 0;

  // Track when save ok (internal) is asserted (for initial pre-fetch arm)
  reg prev_start_ok_74a = 0;
  // The internal flag from the controller's SAVE FSM (set on RTI).  We do
  // NOT propagate this directly to APF; see savestate_start_ok_apf below.
  wire start_ok_74a = savestate_start_ok_s_internal;

  always @(posedge clk_74a) begin
    prev_start_ok_74a   <= start_ok_74a;
    prev_bridge_rd_74a  <= bridge_rd;

    // Latch bridge writes to SRAM (bridge_wr is 1 clk_74a pulse)
    if (bridge_wr && bridge_addr[31:28] == 4'h4 && !bridge_wr_pending) begin
      bridge_wr_pending   <= 1;
      bridge_wr_sram_addr <= {bridge_addr[17:2], 1'b0};
      bridge_wr_latched   <= bridge_wr_data;
      // Saturate at 0xFFFF so the count doesn't roll over to 0 on a full
      // save-state load.  A reading of 0xFFFF means "65535+ writes seen".
      if (bridge_wr_count != 16'hFFFF) begin
        bridge_wr_count <= bridge_wr_count + 16'h0001;
      end
      // Capture the first TWO bridge_wr 32-bit words (sticky — first 8 file bytes).
      if (wr_capture_idx == 2'd0) begin
        first_wr_data  <= bridge_wr_data;
        first_wr_addr  <= bridge_addr;
        first_wr_seen  <= 1;
        wr_capture_idx <= 2'd1;
      end else if (wr_capture_idx == 2'd1) begin
        second_wr_data <= bridge_wr_data;
        wr_capture_idx <= 2'd2;
      end
    end

    // Arm the prefetch pipeline on the save FSM's "ok" rising edge.
    // We do NOT yet propagate this to APF — savestate_start_ok_s is gated
    // by pf_initial_ready (see assign above), which becomes 1 only after
    // the first prefetch has populated save_state_bridge_read_data.
    if (start_ok_74a && ~prev_start_ok_74a) begin
      pf_armed         <= 1'b1;
      pf_next_addr     <= 17'd0;
      pf_staged_valid  <= 1'b0;
      pf_initial_ready <= 1'b0;
    end

    // On the RISING EDGE of bridge_rd in the save-state region, atomically
    // swap the currently-staged word into save_state_bridge_read_data.
    // The FSM will then see !pf_staged_valid and immediately start the
    // next prefetch (which will refill the stage long before the next
    // bridge_rd, since APF reads at ~75 cycle cadence and the FSM
    // completes in ~5 cycles).
    if (~prev_bridge_rd_74a && bridge_rd && bridge_addr[31:28] == 4'h4) begin
      save_state_bridge_read_data <= pf_staged_data;
      pf_staged_valid             <= 1'b0;
      // Debug: count + sticky capture of first bridge_rd address
      if (bridge_rd_count != 16'hFFFF) begin
        bridge_rd_count <= bridge_rd_count + 16'h0001;
      end
      if (!first_rd_seen) begin
        first_rd_addr <= bridge_addr;
        first_rd_seen <= 1;
      end
    end

    case (sram_state)

      SRAM_IDLE: begin
        sram_oe_n  <= 1;
        sram_we_n  <= 1;
        sram_dq_oe <= 0;

        // Priority: core write > core read > bridge write > pre-fetch
        if (core_wr_pending_74a) begin
          // Latch data from clk_sys domain (stable by now)
          latched_core_wr_data <= core_wr_data;
          sram_base_addr       <= {core_sram_base, 2'b00};
          sram_word_idx        <= 2'd0;
          // Set up first write: address + data, assert WE_n.
          // Use core_wr_data directly (not core_wr_word_0) because
          // latched_core_wr_data is updated non-blocking and not yet
          // visible this cycle.
          sram_a      <= {core_sram_base, 2'b00};
          sram_dq_out <= {core_wr_data[23:16], core_wr_data[31:24]};
          sram_dq_oe  <= 1;
          sram_we_n   <= 0;
          sram_state  <= SRAM_CORE_WR;
        end else if (core_rd_pending_74a) begin
          sram_base_addr <= {core_rd_base, 2'b00};
          sram_word_idx  <= 2'd0;
          sram_a         <= {core_rd_base, 2'b00};
          sram_oe_n      <= 0;
          sram_dq_oe     <= 0;
          sram_state     <= SRAM_CORE_RD_SETUP;
        end else if (bridge_wr_pending) begin
          sram_a      <= bridge_wr_sram_addr;
          sram_dq_out <= bridge_wr_latched[15:0];
          sram_dq_oe  <= 1;
          sram_we_n   <= 0;
          sram_state  <= SRAM_BRIDGE_WR_LO;
        end else if (pf_armed && !pf_staged_valid) begin
          // Stage buffer is empty (just consumed by a bridge_rd, or never
          // filled).  Fetch the next 32-bit chunk from SRAM into the
          // stage; pf_next_addr advances on completion (SRAM_PF_HI).
          sram_a     <= pf_next_addr;
          sram_oe_n  <= 0;
          sram_dq_oe <= 0;
          sram_state <= SRAM_PF_SETUP;
        end
      end

      // ----- Core save write: 4 × 16-bit SRAM writes -----
      SRAM_CORE_WR: begin
        sram_we_n  <= 1;  // Rising edge completes the write
        sram_state <= SRAM_CORE_WR_END;
      end

      SRAM_CORE_WR_END: begin
        if (sram_word_idx == 2'd3) begin
          // All 4 words written — done
          sram_dq_oe           <= 0;
          sram_wr_ack_toggle   <= ~sram_wr_ack_toggle;
          prev_core_wr_req_74a <= core_wr_req_74a;  // consume pending
          sram_state           <= SRAM_IDLE;
        end else begin
          sram_word_idx <= sram_word_idx + 2'd1;
          sram_a <= sram_base_addr + {15'd0, sram_word_idx} + 17'd1;
          case (sram_word_idx)
            2'd0: sram_dq_out <= core_wr_word_1;
            2'd1: sram_dq_out <= core_wr_word_2;
            2'd2: sram_dq_out <= core_wr_word_3;
            default: sram_dq_out <= 16'd0;
          endcase
          sram_we_n  <= 0;
          sram_state <= SRAM_CORE_WR;
        end
      end

      // ----- Core load read: 1 setup + 4 captures -----
      SRAM_CORE_RD_SETUP: begin
        sram_state <= SRAM_CORE_RD_N;
      end

      SRAM_CORE_RD_N: begin
        case (sram_word_idx)
          2'd0: sram_rd_result[15:0]  <= sram_dq_in;
          2'd1: sram_rd_result[31:16] <= sram_dq_in;
          2'd2: sram_rd_result[47:32] <= sram_dq_in;
          2'd3: sram_rd_result[63:48] <= sram_dq_in;
        endcase

        if (sram_word_idx == 2'd3) begin
          // All 4 words read — done
          sram_oe_n            <= 1;
          sram_rd_ack_toggle   <= ~sram_rd_ack_toggle;
          prev_core_rd_req_74a <= core_rd_req_74a;  // consume pending
          sram_state           <= SRAM_IDLE;
        end else begin
          sram_word_idx <= sram_word_idx + 2'd1;
          sram_a <= sram_base_addr + {15'd0, sram_word_idx} + 17'd1;
          sram_state <= SRAM_CORE_RD_SETUP;
        end
      end

      // ----- Bridge load write: 2 × 16-bit SRAM writes -----
      SRAM_BRIDGE_WR_LO: begin
        sram_we_n  <= 1;
        sram_state <= SRAM_BRIDGE_WR_GAP;
      end

      SRAM_BRIDGE_WR_GAP: begin
        sram_a      <= bridge_wr_sram_addr + 17'd1;
        sram_dq_out <= bridge_wr_latched[31:16];
        sram_we_n   <= 0;
        sram_state  <= SRAM_BRIDGE_WR_HI;
      end

      SRAM_BRIDGE_WR_HI: begin
        sram_we_n  <= 1;
        sram_state <= SRAM_BRIDGE_WR_END;
      end

      SRAM_BRIDGE_WR_END: begin
        sram_dq_oe        <= 0;
        bridge_wr_pending <= 0;
        sram_state        <= SRAM_IDLE;
      end

      // ----- Bridge save-read pre-fetch: 2 × 16-bit SRAM reads -----
      //
      // Reads two consecutive SRAM words (the low half and high half of
      // the 32-bit value APF will read), then atomically populates the
      // staged buffer and advances pf_next_addr.  If this is the very
      // first prefetch since pf_armed rose, also copy the staged value
      // straight into save_state_bridge_read_data and assert
      // pf_initial_ready so APF can now see savestate_start_ok.
      SRAM_PF_SETUP: begin
        sram_state <= SRAM_PF_LO;
      end

      SRAM_PF_LO: begin
        pf_lo      <= sram_dq_in;
        sram_a     <= pf_next_addr + 17'd1;   // high word lives at base+1
        sram_state <= SRAM_PF_GAP;
      end

      SRAM_PF_GAP: begin
        sram_state <= SRAM_PF_HI;
      end

      SRAM_PF_HI: begin
        // Always advance the SRAM address counter so the NEXT prefetch
        // hits the following 32-bit chunk.
        pf_next_addr <= pf_next_addr + 17'd2;
        sram_oe_n    <= 1;
        sram_state   <= SRAM_IDLE;

        if (!pf_initial_ready) begin
          // VERY FIRST fill since pf_armed: prime save_state_bridge_read_data
          // directly with the freshly-fetched word, leave the stage INVALID
          // so the FSM immediately fetches the NEXT word into the stage.
          // This way APF's first bridge_rd returns the primed value, and
          // the second bridge_rd gets word #2 (already staged by then).
          save_state_bridge_read_data <= {sram_dq_in, pf_lo};
          pf_staged_data              <= 32'd0;
          pf_staged_valid             <= 1'b0;
          pf_initial_ready            <= 1'b1;
        end else begin
          // Steady-state refill: latch the new word into the stage.
          // APF will pull it via the next bridge_rd.
          pf_staged_data  <= {sram_dq_in, pf_lo};
          pf_staged_valid <= 1'b1;
        end
      end

    endcase
  end

endmodule
