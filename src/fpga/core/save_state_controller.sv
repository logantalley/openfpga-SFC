// Save State Controller — Phase C: streaming save + SDRAM-staged load.
//
// Architecture:
//
//   SAVE (unchanged from Phase A):
//     SNES firmware writes 64-bit chunks via ss_req/ss_din.  Each chunk is
//     pushed into a 64→32 dcfifo.  APF drains via bridge_rd at 0x4xxxxxxx.
//     Flow control is automatic — when the FIFO is full we stall ss_ack
//     and the SNES CPU sleeps on STATUS_BUSY.
//
//   LOAD (Phase C):
//     APF bridge writes at 0x4xxxxxxx push 32-bit words into a 32→64
//     dcfifo.  A staging FSM drains the FIFO and writes the data into
//     SDRAM at STAGING_BASE_WORD onward (one 64-bit FIFO entry = 4
//     16-bit SDRAM words).  When APF signals savestate_load, the serve
//     FSM pulses ss_load and then services ss_req's from savestates.sv
//     by reading the corresponding 4 SDRAM words and packing them into
//     ss_dout.
//
//     ss_loading is held high for the entire load (staging + serve)
//     phase so SNES.sv routes its `sdram` instance through the
//     savestate mux instead of cart-ROM reads.
//
//   The SRAM (on-board 256 KB) is no longer used at all; the pin
//   outputs in this module's port list have been removed.

module save_state_controller (
    input wire clk_74a,
    input wire clk_sys,

    // APF Bridge
    //   SAVE path: bridge_rd at 0x4xxxxxxx pops the save FIFO.
    //   LOAD path: bridge_wr at 0x4xxxxxxx pushes into the load FIFO.
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

    // Debug taps (clk_sys / clk_74a) — consumed by core_top for on-screen overlay.
    // Many are stale from the SRAM-buffer era and are tied to 0 in this Phase C
    // build; cleanup of unused taps deferred per plan.
    output wire [3:0] debug_sys_state,
    output wire       debug_ss_busy_seen,
    output wire       debug_ss_busy_ever,
    output wire       debug_ss_save_ever,
    output wire [3:0] debug_ss_save_count,
    output wire [3:0] debug_ss_busy_rises,
    output wire       debug_ss_req_ever,
    output wire [3:0] debug_ss_req_toggles,
    output wire       debug_core_wr_ever,
    output wire       debug_sram_wr_ack_ever,
    output wire [7:0] debug_bridge_wr_count_lo,
    output wire [7:0] debug_bridge_wr_count_hi,
    output wire [7:0] debug_first_wr_data_b0,
    output wire [7:0] debug_first_wr_data_b1,
    output wire [7:0] debug_first_wr_addr_lo,
    output wire [7:0] debug_first_wr_addr_hi,
    output wire [7:0] debug_first_save_byte0,
    output wire [7:0] debug_first_save_byte1,
    output wire [7:0] debug_first_save_addr_lo,
    output wire [7:0] debug_first_save_addr_hi,
    output wire [7:0] debug_first_pf_addr_lo,
    output wire [7:0] debug_first_pf_addr_hi,
    output wire [7:0] debug_first_sram_w0_lo,
    output wire [7:0] debug_first_sram_w0_hi,
    output wire [7:0] debug_first_sram_w1_lo,
    output wire [7:0] debug_first_sram_w1_hi,
    output wire [7:0] debug_max_sram_base_lo,
    output wire [7:0] debug_max_sram_base_hi,
    output wire [7:0] debug_save_wr_count_lo,
    output wire [7:0] debug_save_wr_count_hi,
    output wire [7:0] debug_ss_addr_overflow,
    output wire [7:0] debug_ss_addr_max_hi,
    output wire [7:0] debug_pf_at_first_rd_lo,
    output wire [7:0] debug_pf_at_first_rd_hi,
    output wire [7:0] debug_bridge_rd_count_lo,
    output wire [7:0] debug_bridge_rd_count_hi,
    output wire [7:0] debug_first_rd_addr_lo,
    output wire [7:0] debug_first_rd_addr_hi,
    output wire [7:0] debug_last_w0_data_lo,
    output wire [7:0] debug_last_w0_data_hi,
    output wire [7:0] debug_w0_wr_count,

    // SDRAM staging interface — CDC'd into clk_mem by SNES.sv.
    output reg         ss_sdram_wr_req,
    output reg  [24:0] ss_sdram_wr_addr,
    output reg  [15:0] ss_sdram_wr_data,
    input  wire        ss_sdram_wr_ack,
    output reg         ss_sdram_rd_req,
    output reg  [24:0] ss_sdram_rd_addr,
    input  wire [15:0] ss_sdram_rd_data,
    input  wire        ss_sdram_rd_ack,
    output reg         ss_loading,
    output wire        ss_pause_cpu    // high during STAGING only — gates SNES MCLK
);

  // The CPU must be frozen only while we're actively hijacking SDRAM for
  // staging (the serve phase needs the CPU running to execute firmware).
  // Staging states are SYS_STAGE_* (5'd10..5'd15) and the brief STAGING
  // start in SYS_IDLE.  Simplest robust definition: pause whenever the
  // FSM is in any staging-related state.
  assign ss_pause_cpu = (sys_state == SYS_STAGE_FIFO_RD)    ||
                        (sys_state == SYS_STAGE_FIFO_WAIT)  ||
                        (sys_state == SYS_STAGE_FIFO_LATCH) ||
                        (sys_state == SYS_STAGE_WR_REQ)     ||
                        (sys_state == SYS_STAGE_WR_WAIT)    ||
                        (sys_state == SYS_STAGE_IDLE);

  // SDRAM staging base address.  Cart ROMs are at SDRAM word offset 0 and
  // SNES ROMs max out around 8 MB (= 4 Mwords = 25'h400000).  Place the
  // savestate staging area at 8 MB to leave headroom; 307 KB save ≈ 154 K
  // words, fits well below the 64 MB SDRAM limit.
  localparam [24:0] STAGING_BASE_WORD = 25'h800000;

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
  // 0x4xxxxxxx pops one 32-bit entry from the save FIFO.
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
  // Load FIFO (Phase C): 32→64 dcfifo.  Written on clk_74a by bridge_wr,
  // read on clk_sys by the staging FSM.  Capacity 512 entries × 32-bit =
  // 16 KB raw, comfortable buffer between the bursty bridge and the
  // 4-cycle-per-word SDRAM staging path.
  // ===================================================================

  // Byte-swap incoming bridge_wr_data into the canonical layout the
  // firmware will read back.  bridge_wr_data is big-endian on the wire:
  //   [31:24] = file byte 0, [23:16] = byte 1, [15:8] = byte 2, [7:0] = byte 3
  // Once the SDRAM stride bug was fixed (stride 2 per 16-bit word), the
  // FIFO maps bridge_wr_swapped[15:0] -> stage_buffer[15:0] directly (no
  // half-word swap).  We need stage_buffer[15:0]=word0={byte1,byte0} and
  // [31:16]=word1={byte3,byte2} so the firmware reads byte0,1,2,3 in order.
  wire [31:0] bridge_wr_swapped = {
      bridge_wr_data[7:0],   bridge_wr_data[15:8],  // [31:16] = {byte3, byte2}
      bridge_wr_data[23:16], bridge_wr_data[31:24]  // [15:0]  = {byte1, byte0}
  };

  reg         fifo_load_read_req = 0;
  wire        fifo_load_empty;
  wire        fifo_load_full;
  wire [63:0] fifo_load_dout;

  // bridge_wr at 4xxxxxxx is a 1-cycle pulse on clk_74a — feed it directly
  // as wrreq.  The FIFO ignores wrreq when full (overflow_checking=ON).
  wire fifo_load_write = bridge_wr && (bridge_addr[31:28] == 4'h4);

  dcfifo_mixed_widths fifo_load (
      .data(bridge_wr_swapped),
      .rdclk(clk_sys),
      .rdreq(fifo_load_read_req),
      .wrclk(clk_74a),
      .wrreq(fifo_load_write),
      .q(fifo_load_dout),
      .rdempty(fifo_load_empty),
      .wrfull(fifo_load_full),
      .aclr(1'b0)
  );
  defparam fifo_load.intended_device_family = "Cyclone V",
      fifo_load.lpm_numwords  = 512,
      fifo_load.lpm_showahead = "OFF",
      fifo_load.lpm_type      = "dcfifo_mixed_widths",
      fifo_load.lpm_width     = 32,
      fifo_load.lpm_widthu    = 9,
      fifo_load.lpm_widthu_r  = 8,
      fifo_load.lpm_width_r   = 64,
      fifo_load.overflow_checking  = "ON",
      fifo_load.underflow_checking = "ON",
      fifo_load.rdsync_delaypipe = 5,
      fifo_load.wrsync_delaypipe = 5,
      fifo_load.use_eab = "ON";

  // ===================================================================
  // Main FSM (clk_sys domain)
  // ===================================================================

  localparam SYS_IDLE             = 5'd0;
  localparam SYS_SAVE_ACTIVE      = 5'd1;

  // Staging: drain load FIFO into SDRAM
  localparam SYS_STAGE_FIFO_RD    = 5'd10;
  localparam SYS_STAGE_FIFO_WAIT  = 5'd11;
  localparam SYS_STAGE_FIFO_LATCH = 5'd12;
  localparam SYS_STAGE_WR_REQ     = 5'd13;
  localparam SYS_STAGE_WR_WAIT    = 5'd14;
  localparam SYS_STAGE_IDLE       = 5'd15;

  // Serve: respond to SNES firmware load reads from SDRAM
  localparam SYS_SERVE_WAIT_REQ   = 5'd20;
  localparam SYS_SERVE_RD_REQ     = 5'd21;
  localparam SYS_SERVE_RD_WAIT    = 5'd22;
  localparam SYS_SERVE_RD_NEXT    = 5'd23;
  localparam SYS_SERVE_ACK        = 5'd24;
  localparam SYS_SERVE_COMPLETE   = 5'd25;
  // Bridge state between staging and serve: ss_pause_cpu is already LOW
  // here (it's not a SYS_STAGE_* state) so MCLK is running again.  We wait
  // a few cycles for MCLK/CPU to come back to life, THEN pulse ss_load, so
  // the savestates module (clocked by the gated MCLK) actually sees it.
  localparam SYS_SERVE_KICK       = 5'd26;
  localparam SYS_SERVE_KICK_WAIT  = 5'd27;

  reg [4:0] sys_state = SYS_IDLE;
  assign debug_sys_state = sys_state[3:0];

  // Edge / handshake tracking
  reg prev_savestate_start = 0;
  reg prev_savestate_load  = 0;
  reg prev_ss_busy         = 0;
  reg prev_ss_req          = 0;
  reg prev_ss_sdram_wr_ack = 0;
  reg prev_ss_sdram_rd_ack = 0;

  wire new_ddr_req      = (ss_req != prev_ss_req);
  wire sdram_wr_done    = (ss_sdram_wr_ack != prev_ss_sdram_wr_ack);
  wire sdram_rd_done    = (ss_sdram_rd_ack != prev_ss_sdram_rd_ack);

  // ss_busy_seen — same anti-glitch pattern as Phase A: only act on a
  // falling edge of ss_busy if we've already observed it rise during the
  // current operation.
  reg ss_busy_seen = 0;
  assign debug_ss_busy_seen = ss_busy_seen;

  // Load command pending (set on savestate_load rising edge, cleared
  // when serve phase kicks off).
  reg load_cmd_pending = 0;

  // Countdown after un-pausing the CPU before we pulse ss_load, to let
  // the gated MCLK and CPU pipeline come back to life.
  reg [7:0] kick_wait = 8'd0;

  // Staging book-keeping
  reg [1:0]  stage_word_idx;       // which of the 4 SDRAM words within the chunk
  reg [63:0] stage_buffer;
  reg [24:0] stage_addr;

  // Serve book-keeping
  reg [1:0]  serve_word_idx;
  reg [63:0] serve_buffer;
  reg [24:0] serve_addr;

  // Diagnostic: count ALL new_ddr_req edges seen while in SERVE_WAIT_REQ
  // (regardless of ss_rnw), and capture ss_rnw at the first such edge.
  // If ddr_req_in_wait > 0 but cnt_serve_rd_entries == 0, then requests
  // ARE arriving but ss_rnw is 0 (wrong direction) when they do.
  reg [7:0] cnt_ddr_req_in_wait = 8'h00;
  reg       ss_rnw_at_first_wait_req = 0;
  reg       ss_rnw_at_first_wait_req_seen = 0;

  // ----- Debug taps: most kept zero for now (no-op overlay rows) -----
  reg       ss_save_ever     = 0;
  reg [3:0] ss_save_count    = 4'h0;
  reg       ss_busy_ever     = 0;
  reg [3:0] ss_busy_rises    = 4'h0;
  reg       ss_req_ever      = 0;
  reg [3:0] ss_req_toggles   = 4'h0;

  assign debug_ss_busy_ever     = ss_busy_ever;
  assign debug_ss_save_ever     = ss_save_ever;
  assign debug_ss_save_count    = ss_save_count;
  assign debug_ss_busy_rises    = ss_busy_rises;
  assign debug_ss_req_ever      = ss_req_ever;
  assign debug_ss_req_toggles   = ss_req_toggles;
  assign debug_core_wr_ever     = 1'b0;
  assign debug_sram_wr_ack_ever = 1'b0;

  // Bridge-wr count (clk_74a) — useful to confirm APF wrote all chunks.
  reg [15:0] bridge_wr_count = 16'h0000;
  assign debug_bridge_wr_count_lo = bridge_wr_count[7:0];
  assign debug_bridge_wr_count_hi = bridge_wr_count[15:8];

  // First bridge_wr data (sticky) — should be 'SNES' (0x53 0x4E 0x45 0x53)
  reg [31:0] first_wr_data = 32'h00000000;
  reg [31:0] first_wr_addr = 32'h00000000;
  reg        first_wr_seen = 0;
  // Repurposed for Phase C: expose serve-FSM action counters.
  //   data_b0 → cnt_serve_wait_entries  (# entries to SERVE_WAIT_REQ)
  //   data_b1 → cnt_serve_rd_entries    (# entries to SERVE_RD_REQ)
  //   addr_lo → cnt_serve_ack_entries   (# entries to SERVE_ACK)
  //   addr_hi → cnt_ss_load_pulses      (# ss_load pulses we emitted)
  assign debug_first_wr_data_b0 = cnt_serve_wait_entries;
  assign debug_first_wr_data_b1 = cnt_serve_rd_entries;
  assign debug_first_wr_addr_lo = cnt_serve_ack_entries;
  assign debug_first_wr_addr_hi = cnt_ss_load_pulses;

  // SAVE-side first chunk debug (kept from Phase A)
  reg [63:0] first_save_chunk = 64'h0;
  reg [16:0] first_save_addr  = 17'd0;
  reg        first_save_seen  = 0;
  // Repurposed for Phase C diagnostics:
  //   debug_first_save_byte0 → serve_rd_count[7:0]
  //   debug_first_save_byte1 → serve_rd_count[15:8] (progress high byte)
  assign debug_first_save_byte0   = serve_rd_count[7:0];
  assign debug_first_save_byte1   = serve_rd_count[15:8];
  // Repurposed: expose ddr-req-in-wait diagnostics.
  //   addr_lo → cnt_ddr_req_in_wait (# ddr_req edges seen in SERVE_WAIT)
  //   addr_hi → {7'b0, ss_rnw_at_first_wait_req} (direction of first edge)
  assign debug_first_save_addr_lo = cnt_ddr_req_in_wait;
  assign debug_first_save_addr_hi = {7'b0, ss_rnw_at_first_wait_req};

  // Stage-write debug: first SDRAM word written + its address
  reg [15:0] first_stage_word = 16'h0000;
  reg [24:0] first_stage_addr = 25'h0;
  reg        first_stage_seen = 0;
  assign debug_first_pf_addr_lo = first_stage_addr[7:0];
  assign debug_first_pf_addr_hi = first_stage_addr[15:8];
  // Repurposed: expose serve chunk word2 (file bytes 4,5) for mapping check.
  //   debug_first_sram_w0_lo = first_serve_chunk[39:32] (= word2 lo = fb4, want $2D)
  //   debug_first_sram_w0_hi = first_serve_chunk[47:40] (= word2 hi = fb5, want $53)
  // Controller-side view of served chunk0 word0 (bytes 0,1).
  assign debug_first_sram_w0_lo = first_serve_chunk[7:0];
  assign debug_first_sram_w0_hi = first_serve_chunk[15:8];

  // Serve-side first SDRAM read result (= first chunk byte 0..1)
  reg [15:0] first_serve_word = 16'h0000;
  reg        first_serve_seen = 0;
  // Repurposed: expose serve chunk word1 (file bytes 2,3) for mapping check.
  //   debug_first_sram_w1_lo = first_serve_chunk[23:16] (= word1 lo = fb2, want $45)
  //   debug_first_sram_w1_hi = first_serve_chunk[31:24] (= word1 hi = fb3, want $53)
  assign debug_first_sram_w1_lo = first_serve_chunk[23:16];
  assign debug_first_sram_w1_hi = first_serve_chunk[31:24];

  // Capture the FULL first served chunk (all 4 SDRAM words) so we can
  // verify the complete byte mapping against the known .sta payload:
  //   chunk0 file bytes = 53 4E 45 53 2D 53 53 00  ("SNES-SS\0")
  //   correct words: w0=$4E53 w1=$5345 w2=$532D w3=$0053
  reg [63:0] first_serve_chunk = 64'h0;
  reg        first_serve_chunk_seen = 0;

  // Phase C diagnostic: count of completed SDRAM reads during the serve
  // phase.  Increments on every sdram_rd_done in SYS_SERVE_RD_WAIT.
  // Saturates at $FFFF.  If this stays $00 even though red shows ss_addr
  // advancing, the SDRAM read handshake is broken end-to-end.
  reg [15:0] serve_rd_count = 16'h0000;

  // Serve-FSM action counters (saturating).  All four increment on entry
  // to / completion of specific serve states so we can prove which paths
  // ran on hardware.
  reg [7:0] cnt_serve_wait_entries = 8'h00;  // entered SERVE_WAIT_REQ
  reg [7:0] cnt_serve_rd_entries   = 8'h00;  // entered SERVE_RD_REQ
  reg [7:0] cnt_serve_ack_entries  = 8'h00;  // entered SERVE_ACK
  reg [7:0] cnt_ss_load_pulses     = 8'h00;  // ss_load asserted

  // Count of staging-FIFO entries written to SDRAM (saturating)
  reg [15:0] stage_entry_count = 16'h0000;
  assign debug_save_wr_count_lo = stage_entry_count[7:0];
  assign debug_save_wr_count_hi = stage_entry_count[15:8];

  // ss_addr max (load side) — sanity check on how many chunks firmware reads
  reg [16:0] ss_addr_max = 17'd0;
  assign debug_max_sram_base_lo = ss_addr_max[7:0];
  // Repurposed: high byte = full sys_state (5-bit FSM state) + flags so we can
  // tell where the FSM ended up.  bit7=ss_busy_ever, bit6=load_cmd_pending,
  // bit5=ss_req_ever, bit4:0 = sys_state.
  assign debug_max_sram_base_hi = {ss_busy_ever, load_cmd_pending, ss_req_ever, sys_state[4:0]};
  assign debug_ss_addr_overflow = 8'h00;
  assign debug_ss_addr_max_hi   = ss_addr_max[16:9];

  // Bridge_rd count for save side
  reg [15:0] bridge_rd_count = 16'h0000;
  reg [31:0] first_rd_addr   = 32'h00000000;
  reg        first_rd_seen   = 0;
  assign debug_bridge_rd_count_lo = bridge_rd_count[7:0];
  assign debug_bridge_rd_count_hi = bridge_rd_count[15:8];
  assign debug_first_rd_addr_lo   = first_rd_addr[7:0];
  assign debug_first_rd_addr_hi   = first_rd_addr[15:8];

  // pf_at_first_rd debug — repurposed: count of FIFO drains
  reg [15:0] fifo_drain_count = 16'h0000;
  assign debug_pf_at_first_rd_lo = fifo_drain_count[7:0];
  assign debug_pf_at_first_rd_hi = fifo_drain_count[15:8];

  // last_w0_data / w0_wr_count — Phase A SRAM debug, now stale
  assign debug_last_w0_data_lo = 8'h00;
  assign debug_last_w0_data_hi = 8'h00;
  assign debug_w0_wr_count     = 8'h00;

  // ----- bridge-wr counters (clk_74a) -----
  always @(posedge clk_74a) begin
    if (fifo_load_write) begin
      if (bridge_wr_count != 16'hFFFF) begin
        bridge_wr_count <= bridge_wr_count + 16'h0001;
      end
      if (!first_wr_seen) begin
        first_wr_data <= bridge_wr_data;
        first_wr_addr <= bridge_addr;
        first_wr_seen <= 1;
      end
    end
    if (bridge_rd && ~prev_bridge_rd_save && bridge_addr[31:28] == 4'h4) begin
      if (bridge_rd_count != 16'hFFFF) bridge_rd_count <= bridge_rd_count + 16'h0001;
      if (!first_rd_seen) begin
        first_rd_addr <= bridge_addr;
        first_rd_seen <= 1;
      end
    end
  end

  // ----- Main FSM (clk_sys) -----
  always @(posedge clk_sys) begin
    prev_savestate_start <= savestate_start_s;
    prev_savestate_load  <= savestate_load_s;
    prev_ss_busy         <= ss_busy;
    prev_ss_req          <= ss_req;
    prev_ss_sdram_wr_ack <= ss_sdram_wr_ack;
    prev_ss_sdram_rd_ack <= ss_sdram_rd_ack;

    // 1-cycle pulses
    ss_save             <= 0;
    ss_load             <= 0;
    fifo_save_write_req <= 0;
    fifo_load_read_req  <= 0;
    // NOTE: ss_sdram_wr_req / ss_sdram_rd_req are TOGGLE bits, not
    // pulses — the SNES.sv clk_mem-side FSM edge-detects them via
    // synch_3.  Don't default them to 0 here.

    // Sticky debug taps
    if (ss_busy && !prev_ss_busy) begin
      ss_busy_seen  <= 1;
      ss_busy_ever  <= 1;
      ss_busy_rises <= ss_busy_rises + 4'd1;
    end
    if (ss_req != prev_ss_req) begin
      ss_req_ever    <= 1;
      ss_req_toggles <= ss_req_toggles + 4'd1;
      if (ss_addr > ss_addr_max) ss_addr_max <= ss_addr;
    end

    // APF save trigger
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

    // APF load command — note the data is already streaming into the
    // load FIFO via bridge_wr.  Staging may already be in progress.
    if (savestate_load_s && ~prev_savestate_load) begin
      load_cmd_pending    <= 1;
      savestate_load_ack  <= 1;
      savestate_load_ok   <= 0;
      savestate_load_err  <= 0;
      savestate_start_ok  <= 0;
      savestate_start_err <= 0;
      ss_loading          <= 1;
    end

    case (sys_state)

      // ============================================================
      // IDLE: opportunistically drain the load FIFO as APF fills it
      // ============================================================
      SYS_IDLE: begin
        if (~savestate_load_s) savestate_load_ack <= 0;
        if (~fifo_load_empty) begin
          // Begin staging.  stage_addr starts at STAGING_BASE_WORD on
          // the first FIFO entry of a transfer; once running, it just
          // keeps incrementing through the whole load.
          if (stage_entry_count == 16'd0) begin
            stage_addr <= STAGING_BASE_WORD;
            ss_loading <= 1;
          end
          sys_state <= SYS_STAGE_FIFO_RD;
        end else if (load_cmd_pending) begin
          // APF has commanded load but the FIFO is dry (either all
          // chunks already staged, or none have arrived yet).  Park in
          // STAGE_IDLE so the load_cmd_pending → serve transition can
          // fire there once the FIFO is fully drained.
          sys_state <= SYS_STAGE_IDLE;
        end
      end

      // ============================================================
      // SAVE path — streaming via save FIFO (unchanged from Phase A)
      // ============================================================
      SYS_SAVE_ACTIVE: begin
        if (~savestate_start_s) savestate_start_ack <= 0;

        if (new_ddr_req && ~ss_rnw && ~fifo_save_wr_full) begin
          fifo_save_write_req <= 1;
          ss_ack              <= ~ss_ack;
          if (!first_save_seen) begin
            first_save_chunk <= ss_din;
            first_save_addr  <= ss_addr;
            first_save_seen  <= 1;
          end
        end else if (ss_busy_seen && prev_ss_busy && ~ss_busy) begin
          sys_state            <= SYS_IDLE;
          ss_busy_seen         <= 0;
          savestate_start_busy <= 0;
          savestate_start_ok   <= 1;
        end
      end

      // ============================================================
      // STAGING: drain load FIFO 64-bit chunks into SDRAM, 4 words each
      // ============================================================
      SYS_STAGE_FIFO_RD: begin
        if (~fifo_load_empty) begin
          fifo_load_read_req <= 1;
          sys_state          <= SYS_STAGE_FIFO_WAIT;
        end else begin
          // FIFO momentarily empty.  If APF says go, switch to serve;
          // otherwise wait for more bridge writes.
          sys_state <= SYS_STAGE_IDLE;
        end
      end

      SYS_STAGE_FIFO_WAIT: begin
        // dcfifo with showahead=OFF: q is valid 2 cycles after rdreq.
        sys_state <= SYS_STAGE_FIFO_LATCH;
      end

      SYS_STAGE_FIFO_LATCH: begin
        stage_buffer     <= fifo_load_dout;
        stage_word_idx   <= 2'd0;
        sys_state        <= SYS_STAGE_WR_REQ;
        if (fifo_drain_count != 16'hFFFF) fifo_drain_count <= fifo_drain_count + 16'd1;
      end

      SYS_STAGE_WR_REQ: begin
        ss_sdram_wr_req  <= ~ss_sdram_wr_req;  // toggle to request a write
        ss_sdram_wr_addr <= stage_addr;
        case (stage_word_idx)
          2'd0: ss_sdram_wr_data <= stage_buffer[15:0];
          2'd1: ss_sdram_wr_data <= stage_buffer[31:16];
          2'd2: ss_sdram_wr_data <= stage_buffer[47:32];
          2'd3: ss_sdram_wr_data <= stage_buffer[63:48];
        endcase
        if (!first_stage_seen) begin
          first_stage_word <= stage_buffer[15:0];
          first_stage_addr <= stage_addr;
          first_stage_seen <= 1;
        end
        sys_state <= SYS_STAGE_WR_WAIT;
      end

      SYS_STAGE_WR_WAIT: begin
        if (sdram_wr_done) begin
          // sdram.sv treats addr[24:1] as the 16-bit word and addr[0] as
          // byte-within-word, AND skips re-access when addr[24:1] is
          // unchanged.  So distinct 16-bit words must step addr by 2.
          stage_addr <= stage_addr + 25'd2;
          if (stage_word_idx == 2'd3) begin
            if (stage_entry_count != 16'hFFFF)
              stage_entry_count <= stage_entry_count + 16'd1;
            sys_state <= SYS_STAGE_FIFO_RD;
          end else begin
            stage_word_idx <= stage_word_idx + 2'd1;
            sys_state      <= SYS_STAGE_WR_REQ;
          end
        end
      end

      SYS_STAGE_IDLE: begin
        // Load FIFO drained.  If APF has signaled load_cmd_pending, move
        // to the KICK state (which has ss_pause_cpu=0, so MCLK resumes)
        // before pulsing ss_load.  Otherwise wait for more bridge writes.
        if (~fifo_load_empty) begin
          sys_state <= SYS_STAGE_FIFO_RD;
        end else if (load_cmd_pending) begin
          savestate_load_busy <= 1;
          kick_wait           <= 8'd64;  // let CPU/MCLK spin up
          sys_state           <= SYS_SERVE_KICK_WAIT;
        end
      end

      // CPU un-paused (ss_pause_cpu now 0 since this isn't a STAGE state).
      // Wait for MCLK to be running for a while before kicking the load.
      SYS_SERVE_KICK_WAIT: begin
        if (kick_wait != 8'd0) begin
          kick_wait <= kick_wait - 8'd1;
        end else begin
          sys_state <= SYS_SERVE_KICK;
        end
      end

      // Pulse ss_load now that MCLK is alive; savestates module will see it.
      SYS_SERVE_KICK: begin
        load_cmd_pending   <= 0;
        savestate_load_ack <= 0;
        ss_busy_seen       <= 0;
        ss_load            <= 1;
        if (cnt_ss_load_pulses != 8'hFF)
          cnt_ss_load_pulses <= cnt_ss_load_pulses + 8'd1;
        if (cnt_serve_wait_entries != 8'hFF)
          cnt_serve_wait_entries <= cnt_serve_wait_entries + 8'd1;
        sys_state <= SYS_SERVE_WAIT_REQ;
      end

      // ============================================================
      // SERVE: respond to firmware ss_req's by reading SDRAM
      // ============================================================
      SYS_SERVE_WAIT_REQ: begin
        // Diagnostic: log any ddr_req edge that arrives while we wait.
        if (new_ddr_req) begin
          if (cnt_ddr_req_in_wait != 8'hFF)
            cnt_ddr_req_in_wait <= cnt_ddr_req_in_wait + 8'd1;
          if (!ss_rnw_at_first_wait_req_seen) begin
            ss_rnw_at_first_wait_req      <= ss_rnw;
            ss_rnw_at_first_wait_req_seen <= 1;
          end
        end
        if (new_ddr_req && ss_rnw) begin
          // ss_addr = 64-bit chunk index.  Each chunk = 4 SDRAM 16-bit
          // words at byte-stride 2 = 8 bytes of SDRAM addr space.  Staging
          // wrote chunk N at STAGING_BASE_WORD + N*8.  Match here.
          serve_addr     <= STAGING_BASE_WORD + ({5'd0, ss_addr, 3'b000});
          serve_word_idx <= 2'd0;
          if (cnt_serve_rd_entries != 8'hFF)
            cnt_serve_rd_entries <= cnt_serve_rd_entries + 8'd1;
          sys_state      <= SYS_SERVE_RD_REQ;
        end else if (ss_busy_seen && prev_ss_busy && ~ss_busy) begin
          sys_state <= SYS_SERVE_COMPLETE;
        end
      end

      SYS_SERVE_RD_REQ: begin
        ss_sdram_rd_req  <= ~ss_sdram_rd_req;  // toggle to request a read
        ss_sdram_rd_addr <= serve_addr;
        sys_state        <= SYS_SERVE_RD_WAIT;
      end

      SYS_SERVE_RD_WAIT: begin
        if (sdram_rd_done) begin
          case (serve_word_idx)
            2'd0: serve_buffer[15:0]  <= ss_sdram_rd_data;
            2'd1: serve_buffer[31:16] <= ss_sdram_rd_data;
            2'd2: serve_buffer[47:32] <= ss_sdram_rd_data;
            2'd3: serve_buffer[63:48] <= ss_sdram_rd_data;
          endcase
          if (!first_serve_seen && serve_word_idx == 2'd0) begin
            first_serve_word <= ss_sdram_rd_data;
            first_serve_seen <= 1;
          end
          if (serve_rd_count != 16'hFFFF)
            serve_rd_count <= serve_rd_count + 16'd1;
          sys_state <= SYS_SERVE_RD_NEXT;
        end
      end

      SYS_SERVE_RD_NEXT: begin
        if (serve_word_idx == 2'd3) begin
          if (cnt_serve_ack_entries != 8'hFF)
            cnt_serve_ack_entries <= cnt_serve_ack_entries + 8'd1;
          sys_state <= SYS_SERVE_ACK;
        end else begin
          serve_word_idx <= serve_word_idx + 2'd1;
          serve_addr     <= serve_addr + 25'd2;  // stride 2 (see staging note)
          sys_state      <= SYS_SERVE_RD_REQ;
        end
      end

      SYS_SERVE_ACK: begin
        ss_dout   <= serve_buffer;
        ss_ack    <= ~ss_ack;
        if (!first_serve_chunk_seen) begin
          first_serve_chunk      <= serve_buffer;
          first_serve_chunk_seen <= 1;
        end
        if (ss_busy_seen && ~ss_busy) begin
          sys_state <= SYS_SERVE_COMPLETE;
        end else begin
          sys_state <= SYS_SERVE_WAIT_REQ;
        end
      end

      SYS_SERVE_COMPLETE: begin
        sys_state           <= SYS_IDLE;
        ss_busy_seen        <= 0;
        savestate_load_busy <= 0;
        savestate_load_ok   <= 1;
        ss_loading          <= 0;
        // Reset the staging counter so the next load starts fresh from
        // STAGING_BASE_WORD.  bridge_wr_count etc. stay sticky for debug.
        stage_entry_count   <= 16'h0000;
      end

      default: sys_state <= SYS_IDLE;

    endcase
  end

endmodule
