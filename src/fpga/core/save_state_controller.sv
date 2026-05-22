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
    //  - SAVE path: bridge_rd at 0x4xxxxxxx pops the save FIFO and presents
    //    the next 32-bit chunk on save_state_bridge_read_data.  The FIFO is
    //    fed by ss_din from savestates.sv (64-bit chunks become 2x 32-bit
    //    FIFO entries on the read side).
    //  - LOAD path: bridge_wr at 0x4xxxxxxx still goes into SRAM (Phase A:
    //    load path unchanged from the SRAM-based architecture).
    input wire bridge_wr,
    input wire bridge_rd,
    input wire bridge_endian_little,
    input wire [31:0] bridge_addr,
    input wire [31:0] bridge_wr_data,
    output wire [31:0] save_state_bridge_read_data,

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
    output wire [7:0] debug_first_save_addr_lo, // First save chunk's SRAM base (low byte of core_sram_base)
    output wire [7:0] debug_first_save_addr_hi, // First save chunk's SRAM base (high byte)
    output wire [7:0] debug_first_pf_addr_lo,   // First prefetched SRAM word address (low byte)
    output wire [7:0] debug_first_pf_addr_hi,   // First prefetched SRAM word address (high byte)
    output wire [7:0] debug_first_sram_w0_lo,   // First SRAM read: word 0 low byte (expect $53='S')
    output wire [7:0] debug_first_sram_w0_hi,   // First SRAM read: word 0 high byte (expect $45='E')
    output wire [7:0] debug_first_sram_w1_lo,   // First SRAM read: word 1 low byte (expect $4E='N')
    output wire [7:0] debug_first_sram_w1_hi,   // First SRAM read: word 1 high byte (expect $53='S')
    output wire [7:0] debug_max_sram_base_lo,   // Highest SRAM word base seen during save (low byte)
    output wire [7:0] debug_max_sram_base_hi,   // Highest SRAM word base seen during save (high byte)
    output wire [7:0] debug_save_wr_count_lo,   // # of SRAM core writes completed (low byte)
    output wire [7:0] debug_save_wr_count_hi,   // # of SRAM core writes completed (high byte)
    output wire [7:0] debug_ss_addr_overflow,   // sticky: ss_addr ever had bit 15 or 16 set (= overflowed SRAM)
    output wire [7:0] debug_ss_addr_max_hi,     // max value of ss_addr[16:8] seen
    output wire [7:0] debug_pf_at_first_rd_lo,  // pf_next_addr captured at first bridge_rd (low byte)
    output wire [7:0] debug_pf_at_first_rd_hi,  // pf_next_addr captured at first bridge_rd (high byte)
    output wire [7:0] debug_bridge_rd_count_lo, // low byte of bridge_rd events at 4xxxxxxx
    output wire [7:0] debug_bridge_rd_count_hi, // high byte (saturates at FF)
    output wire [7:0] debug_first_rd_addr_lo,   // bridge_addr[7:0] of first bridge_rd (expect $00)
    output wire [7:0] debug_first_rd_addr_hi,   // bridge_addr[15:8] of first bridge_rd (expect $00)
    // Sticky-update: last 16-bit value written by ANY bridge_wr that targeted
    // SRAM word 0.  Updated on every such write, so after load completes this
    // holds the FINAL value in SRAM word 0 (= what load firmware will read).
    output wire [7:0] debug_last_w0_data_lo,    // low byte of last bridge_wr to SRAM word 0
    output wire [7:0] debug_last_w0_data_hi,    // high byte of last bridge_wr to SRAM word 0
    // Count of bridge_wr events that targeted SRAM word 0.  If 1, only one
    // write hit addr 0 (probably correct).  If >1, APF wrote to addr 0
    // multiple times and a later one overwrote 'SNES'.
    output wire [7:0] debug_w0_wr_count,

    // SRAM interface (directly to Pocket board SRAM)
    output reg  [16:0] sram_a,
    inout  wire [15:0] sram_dq,
    output reg         sram_oe_n,
    output reg         sram_we_n,
    output wire        sram_ub_n,
    output wire        sram_lb_n,

    // Phase B: SDRAM staging interface (currently no-op; clk_sys domain).
    // Will be driven by Phase C load FSM.  CDC into clk_mem happens in SNES.sv.
    output wire        ss_sdram_wr_req,
    output wire [24:0] ss_sdram_wr_addr,
    output wire [15:0] ss_sdram_wr_data,
    input  wire        ss_sdram_wr_ack,
    output wire        ss_sdram_rd_req,
    output wire [24:0] ss_sdram_rd_addr,
    input  wire [15:0] ss_sdram_rd_data,
    input  wire        ss_sdram_rd_ack,
    output wire        ss_loading
);

  // Phase B: tie all SDRAM-staging outputs low.  Phase C wires them up.
  assign ss_sdram_wr_req  = 1'b0;
  assign ss_sdram_wr_addr = 25'd0;
  assign ss_sdram_wr_data = 16'd0;
  assign ss_sdram_rd_req  = 1'b0;
  assign ss_sdram_rd_addr = 25'd0;
  assign ss_loading       = 1'b0;

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
        savestate_start_ok_s,   savestate_start_err_s
      },
      clk_74a
  );

  // ===================================================================
  // Save FIFO (Phase A): pure streaming, no SRAM buffer
  // ===================================================================
  //
  // savestates.sv produces 64-bit chunks (ss_din + ss_req toggle).  Push
  // each chunk into a 64-wide → 32-wide dcfifo whose read side feeds the
  // APF bridge_rd path on clk_74a.  Flow control is automatic: when the
  // FIFO is full, we stall ss_ack — the SNES CPU sleeps because the
  // SSDATA poll loop spins on STATUS_BUSY.
  //
  // The FIFO is small (16 64-bit entries = 64 32-bit entries) because APF
  // is much faster than the SNES at draining.  Filling-stall only happens
  // briefly during the initial back-to-back chunks.

  reg         fifo_save_write_req = 0;
  reg         fifo_save_read_req  = 0;
  wire        fifo_save_rd_empty;
  wire        fifo_save_wr_empty;
  wire        fifo_save_wr_full;

  dcfifo_mixed_widths fifo_save (
      .data(ss_din),
      .rdclk(clk_74a),
      .rdreq(fifo_save_read_req),
      .wrclk(clk_sys),
      .wrreq(fifo_save_write_req),
      // Byte-swap to match the APF wire format (big-endian on the bridge).
      .q({
          save_state_bridge_read_data[7:0],
          save_state_bridge_read_data[15:8],
          save_state_bridge_read_data[23:16],
          save_state_bridge_read_data[31:24]
      }),
      .rdempty(fifo_save_rd_empty),
      .wrempty(fifo_save_wr_empty),
      .wrfull(fifo_save_wr_full),
      .aclr(1'b0)
  );
  defparam fifo_save.intended_device_family = "Cyclone V",
      fifo_save.lpm_numwords  = 16,
      fifo_save.lpm_showahead = "OFF",
      fifo_save.lpm_type      = "dcfifo_mixed_widths",
      fifo_save.lpm_width     = 64,
      fifo_save.lpm_widthu    = 4,
      fifo_save.lpm_widthu_r  = 5,
      fifo_save.lpm_width_r   = 32,
      fifo_save.overflow_checking  = "ON",
      fifo_save.underflow_checking = "ON",
      fifo_save.rdsync_delaypipe = 5,
      fifo_save.wrsync_delaypipe = 5,
      fifo_save.use_eab = "ON";

  // Bridge-read handler (clk_74a): each rising edge of bridge_rd at
  // 0x4xxxxxxx pops one 32-bit entry from the FIFO.
  reg prev_bridge_rd_save = 0;
  reg [1:0] save_rd_state = 0;
  localparam SAVE_RD_NONE = 2'd0;
  localparam SAVE_RD_REQ  = 2'd1;

  always @(posedge clk_74a) begin
    prev_bridge_rd_save <= bridge_rd;
    fifo_save_read_req  <= 0;

    if (bridge_rd && ~prev_bridge_rd_save && bridge_addr[31:28] == 4'h4) begin
      if (~fifo_save_rd_empty) begin
        fifo_save_read_req <= 1;
        save_rd_state      <= SAVE_RD_REQ;
      end
    end

    case (save_rd_state)
      SAVE_RD_REQ: save_rd_state <= SAVE_RD_NONE;
    endcase
  end

  // ===================================================================
  // CDC: Core ↔ SRAM toggle-based data transfer (LOAD path only)
  // ===================================================================
  //
  // Load reads: clk_sys latches addr, toggles rd_req.
  //   clk_74a detects toggle, reads 4×16 SRAM words, latches data, toggles rd_ack.
  //   clk_sys detects ack, copies data to ss_dout, toggles ss_ack.
  //
  // (The save CDC has been removed — see save FIFO above.)

  // Core-side registers (clk_sys domain) — LOAD path only
  reg        core_rd_req_toggle = 0;
  reg [14:0] core_rd_base;          // ss_addr[14:0]

  // SRAM-side registers (clk_74a domain)
  reg        sram_rd_ack_toggle = 0;
  reg [63:0] sram_rd_result;        // 64-bit data read from SRAM (load)

  // Synchronize toggles across clock domains
  wire core_rd_req_74a;
  wire sram_rd_ack_sys;

  synch_3 sync_rd_req (.i(core_rd_req_toggle), .o(core_rd_req_74a), .clk(clk_74a));
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
  reg prev_sram_rd_ack     = 0;

  wire new_ddr_req  = (ss_req != prev_ss_req);
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
  assign debug_first_wr_addr_lo = first_wr_addr[7:0];
  assign debug_first_wr_addr_hi = first_wr_addr[15:8];

  // Capture the FIRST 64-bit chunk the SNES wrote during save.
  // core_wr_data is in clk_sys; latch it sticky.  Also capture the
  // chunk's SRAM word base (core_sram_base) so we can tell whether
  // it lands at SRAM offset 0 or somewhere else.
  reg [63:0] first_save_chunk = 64'h0;
  reg [14:0] first_save_base  = 15'h0000;
  reg        first_save_seen  = 0;
  assign debug_first_save_byte0   = first_save_chunk[7:0];
  assign debug_first_save_byte1   = first_save_chunk[15:8];
  // Expose first_save_base as two bytes. It's the SRAM WORD base (15 bits);
  // multiply by 4 to get the SRAM byte address of the 8-byte chunk.
  assign debug_first_save_addr_lo = {1'b0, first_save_base[6:0]};
  assign debug_first_save_addr_hi = {1'b0, first_save_base[14:7]};

  // Capture the first prefetched SRAM word address (= what we ask the
  // FSM to read first for serving APF bridge_rds).  If save wrote 'SNES'
  // at SRAM word 0 and we ALSO start prefetching at word 0, the bytes
  // should round-trip.
  reg [16:0] first_pf_addr      = 17'd0;
  reg        first_pf_seen      = 0;
  assign debug_first_pf_addr_lo = first_pf_addr[7:0];
  assign debug_first_pf_addr_hi = {7'b0, first_pf_addr[9:8]};   // bits [9:8] of 17-bit addr

  // Capture the SRAM data on the first prefetch (1st word at SRAM_PF_LO,
  // 2nd word at SRAM_PF_HI).  These should be the bytes the save firmware
  // wrote: word 0 = $4553 (= bytes 'E' high, 'S' low), word 1 = $534E.
  reg [15:0] first_sram_w0 = 16'h0000;
  reg [15:0] first_sram_w1 = 16'h0000;
  reg        first_sram_w0_seen = 0;
  reg        first_sram_w1_seen = 0;
  assign debug_first_sram_w0_lo = first_sram_w0[7:0];
  assign debug_first_sram_w0_hi = first_sram_w0[15:8];
  assign debug_first_sram_w1_lo = first_sram_w1[7:0];
  assign debug_first_sram_w1_hi = first_sram_w1[15:8];

  // Track the maximum SRAM word base address seen across all save chunks
  // (in clk_sys, where ss_addr is sampled).
  reg [14:0] max_sram_base = 15'h0000;
  assign debug_max_sram_base_lo = max_sram_base[7:0];
  assign debug_max_sram_base_hi = {1'b0, max_sram_base[14:8]};

  // Count of fully-completed core SRAM writes (clk_74a, set in SRAM FSM).
  // A healthy 256KB save produces ~32768 writes (saturates at 0xFFFF).
  reg [15:0] save_wr_count = 16'h0000;
  assign debug_save_wr_count_lo = save_wr_count[7:0];
  assign debug_save_wr_count_hi = save_wr_count[15:8];

  // Detect whether ss_addr ever exceeded 15 bits during a save — i.e.,
  // the save state is larger than the 256KB SRAM and chunks are wrapping.
  reg [8:0] ss_addr_max_hi = 9'h000;  // captures the maximum of ss_addr[16:8] seen
  assign debug_ss_addr_overflow = {7'b0, |ss_addr_max_hi[8:7], 1'b0};
  assign debug_ss_addr_max_hi   = ss_addr_max_hi[7:0];

  // Capture pf_next_addr value at the moment of the FIRST bridge_rd
  // (rising edge).  This tells us what SRAM word address the FSM was
  // about to fetch when APF made its first read.  If pf_next_addr=2 we
  // know APF got SRAM word 0 first (= 'SNES').  If non-zero/non-2 something
  // advanced pf_next_addr before APF's first real read.
  reg [16:0] pf_at_first_rd = 17'd0;
  reg        pf_at_first_rd_seen = 0;
  assign debug_pf_at_first_rd_lo = pf_at_first_rd[7:0];
  assign debug_pf_at_first_rd_hi = {7'b0, pf_at_first_rd[9:8]};

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
    prev_sram_rd_ack     <= sram_rd_ack_sys;

    ss_save             <= 0;
    ss_load             <= 0;
    fifo_save_write_req <= 0;  // 1-cycle pulse when we push to save FIFO

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

    // (Old core_wr/sram_wr_ack debug taps removed — save path no longer
    // uses SRAM CDC.  core_wr_ever / sram_wr_ack_ever stay at their reset
    // value, which is fine — those overlay rows will read 0.)

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

      // ===== Save path (FIFO streaming) =====
      //
      // When savestates.sv toggles ss_req with ~ss_rnw (= save chunk
      // ready), push ss_din into the save FIFO.  If the FIFO is full,
      // stall — don't ack — and the SNES CPU will spin on STATUS_BUSY.
      // After pushing, ack with the toggle.  No intermediate buffer.
      SYS_SAVE_ACTIVE: begin
        if (~savestate_start_s)
          savestate_start_ack <= 0;

        if (new_ddr_req && ~ss_rnw && ~fifo_save_wr_full) begin
          // Push 64-bit chunk into FIFO; ack savestates.sv same cycle.
          fifo_save_write_req <= 1;
          ss_ack              <= ~ss_ack;

          // Debug taps (kept for overlay continuity)
          if (!first_save_seen) begin
            first_save_chunk <= ss_din;
            first_save_base  <= ss_addr[14:0];
            first_save_seen  <= 1;
          end
          if (ss_addr[14:0] > max_sram_base) begin
            max_sram_base <= ss_addr[14:0];
          end
          if (ss_addr[16:8] > ss_addr_max_hi) begin
            ss_addr_max_hi <= ss_addr[16:8];
          end
        end else if (ss_busy_seen && prev_ss_busy && ~ss_busy) begin
          // ss_busy just fell after rising → savestates.sv finished
          // producing the save state.  Signal start_ok to APF.
          sys_state            <= SYS_IDLE;
          ss_busy_seen         <= 0;
          savestate_start_busy <= 0;
          savestate_start_ok   <= 1;
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
          // Canonical layout: sram_rd_result already has firmware bytes in
          // their original order (sram_rd_result[8K +: 8] = firmware byte
          // K).  No byte swap needed — pass through directly.
          ss_dout <= sram_rd_result;
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
  localparam SRAM_CORE_RD_HOLD   = 4'd12; // extra cycle for SRAM access time
  localparam SRAM_CORE_RD_N      = 4'd4;
  localparam SRAM_BRIDGE_WR_LO   = 4'd5;  // WE_n asserted for low half
  localparam SRAM_BRIDGE_WR_GAP  = 4'd6;  // WE_n high between halves
  localparam SRAM_BRIDGE_WR_HI   = 4'd7;  // WE_n asserted for high half
  localparam SRAM_BRIDGE_WR_END  = 4'd8;  // WE_n high, done
  // Memory-adapter read state for data_unloader.  When bridge_rd_en pulses
  // (it's a single clk_74a high signal from data_unloader for the duration
  // of the read), we set the SRAM address, assert OE, wait for the read to
  // settle, and latch into bridge_rd_data.  The data_unloader's
  // READ_MEM_CLOCK_DELAY parameter must match the total cycles.
  localparam SRAM_BRIDGE_RD_SETUP = 4'd9;
  localparam SRAM_BRIDGE_RD_HOLD  = 4'd10; // address setup + data settle
  localparam SRAM_BRIDGE_RD_LATCH = 4'd11; // sample sram_dq_in

  reg [3:0]  sram_state = SRAM_IDLE;
  reg [1:0]  sram_word_idx;
  reg [16:0] sram_base_addr;

  // CDC toggle edge detection (clk_74a side) — load path only
  reg prev_core_rd_req_74a = 0;
  wire core_rd_pending_74a = (core_rd_req_74a != prev_core_rd_req_74a);

  // Bridge write pending latch (clk_74a domain)
  reg        bridge_wr_pending = 0;
  reg [16:0] bridge_wr_sram_addr;
  reg [31:0] bridge_wr_latched;

  // Save-side bridge READS are now serviced by a data_unloader instance in
  // core_top.sv (the same proven pattern that the cart-save read path uses).
  // The data_unloader exposes the APF bridge protocol on one side and a
  // simple "read-this-address" memory-adapter interface on the other.  We
  // implement that adapter here as a small state machine in the SRAM FSM:
  // on bridge_rd_en, set the SRAM address, wait for data to settle, latch
  // into bridge_rd_data.  No prefetching, no FIFO management on our end —
  // data_unloader handles all of that with its internal FIFOs.

  // Sticky-update: last data written to SRAM word 0, plus count of writes
  // that targeted SRAM word 0.  Updated whenever bridge_wr_sram_addr == 0.
  reg [15:0] last_w0_data  = 16'h0000;
  reg [7:0]  w0_wr_count   = 8'h00;
  assign debug_last_w0_data_lo = last_w0_data[7:0];
  assign debug_last_w0_data_hi = last_w0_data[15:8];
  assign debug_w0_wr_count     = w0_wr_count;

  // Track whether we've already serviced the CURRENT bridge_rd_en pulse.
  // data_unloader holds bridge_rd_en high for ~7 clk_74a cycles per read;
  // we want to do exactly one SRAM read per such pulse, regardless of
  // when in that window we get to IDLE.  Flag is set on LATCH (read done)
  // and cleared when bridge_rd_en falls.
  reg        bridge_rd_serviced = 0;
  reg [17:0] bridge_rd_addr_latched = 18'd0;

  always @(posedge clk_74a) begin
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
      // Track the LAST data written to SRAM word 0 (= bridge_addr[17:2]==0).
      // First SRAM word of any bridge_wr lands at {bridge_addr[17:2], 1'b0}.
      // Data on first SRAM word is {bridge_wr_data[23:16], bridge_wr_data[31:24]}.
      if (bridge_addr[17:2] == 16'h0000) begin
        last_w0_data <= {bridge_wr_data[23:16], bridge_wr_data[31:24]};
        if (w0_wr_count != 8'hFF) begin
          w0_wr_count <= w0_wr_count + 8'd1;
        end
      end
    end

    case (sram_state)

      SRAM_IDLE: begin
        sram_oe_n  <= 1;
        sram_we_n  <= 1;
        sram_dq_oe <= 0;

        // Phase A: SAVE is now streamed via FIFO, no SRAM writes for save.
        // SRAM is used only for LOAD: bridge_wr (APF → SRAM) and core_rd
        // (SRAM → SNES firmware).  Priority: core read > bridge write.
        if (core_rd_pending_74a) begin
          sram_base_addr <= {core_rd_base, 2'b00};
          sram_word_idx  <= 2'd0;
          sram_a         <= {core_rd_base, 2'b00};
          sram_oe_n      <= 0;
          sram_dq_oe     <= 0;
          sram_state     <= SRAM_CORE_RD_SETUP;
        end else if (bridge_wr_pending) begin
          // Canonical packing for bridge_wr (APF → SRAM during load):
          // bridge_wr_data has file bytes in big-endian wire order, so
          // [31:24]=file byte 0, [23:16]=byte 1, [15:8]=byte 2, [7:0]=byte 3.
          // The first SRAM word (at bridge_wr_sram_addr) holds file bytes
          // 0 and 1 (low byte 0, high byte 1).  The second SRAM word
          // (at +1) holds file bytes 2 and 3.
          sram_a      <= bridge_wr_sram_addr;
          sram_dq_out <= {bridge_wr_latched[23:16], bridge_wr_latched[31:24]};
          sram_dq_oe  <= 1;
          sram_we_n   <= 0;
          sram_state  <= SRAM_BRIDGE_WR_LO;
        end
      end

      // (SRAM_CORE_WR / SRAM_CORE_WR_END removed — save path is now FIFO-based.)

      // ----- Core load read: 1 setup + 1 hold + sample, per word -----
      // The on-board SRAM has ~10ns access time.  A single SETUP cycle
      // (~13.5ns) is marginal; add a HOLD cycle to guarantee the data
      // bus has settled before we sample sram_dq_in.
      SRAM_CORE_RD_SETUP: begin
        sram_state <= SRAM_CORE_RD_HOLD;
      end

      SRAM_CORE_RD_HOLD: begin
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
        // Second SRAM word: file bytes 2 (low) and 3 (high).
        sram_dq_out <= {bridge_wr_latched[7:0], bridge_wr_latched[15:8]};
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

      // (SRAM_BRIDGE_RD_* removed — save reads are now served from the
      // FIFO, not from SRAM.)

    endcase
  end

endmodule
