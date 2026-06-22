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

module save_state_controller #(
    // clk_sys cycles APF must be idle (not reading the slot) during the SAVE
    // serve before we declare the read-back complete and release the core.
    // Overridable so the testbench can exercise idle-completion quickly.
    parameter [20:0] SAVE_SERVE_IDLE_MAX = 21'd300_000   // ~14 ms @ 21.48 MHz
) (
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
    // (= initializers on output regs mirror Quartus's power-up-to-0
    // default; required so simulation doesn't start them at X)
    output reg ss_save = 0,
    output reg ss_load = 0,

    // Core-side DDR-style interface (toggle-based req/ack)
    input wire [63:0] ss_din,     // Data from core (save)
    output reg [63:0] ss_dout = 64'h0,  // Data to core (load)
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
    output reg         ss_sdram_wr_req  = 0,
    output reg  [24:0] ss_sdram_wr_addr = 25'h0,
    output reg  [15:0] ss_sdram_wr_data = 16'h0,
    input  wire        ss_sdram_wr_ack,
    output reg         ss_sdram_rd_req  = 0,
    output reg  [24:0] ss_sdram_rd_addr = 25'h0,
    input  wire [15:0] ss_sdram_rd_data,
    input  wire        ss_sdram_rd_ack,
    output reg         ss_loading = 0,
    output wire        ss_pause_cpu,   // high during STAGING only — gates SNES MCLK

    // PSRAM staging interface (CRAM1 bank 1) — CDC'd into clk_mem by
    // ss_psram_arbiter (instantiated in core_top).  Drives Port B of
    // psram_arbiter.  In this Step-4 baseline build the toggles never
    // fire (SAVE still goes through SDRAM), so Port B stays idle.
    output reg         ss_psram_wr_req  = 0,
    output reg  [18:0] ss_psram_wr_addr = 19'h0,
    output reg  [15:0] ss_psram_wr_data = 16'h0,
    input  wire        ss_psram_wr_ack,
    output reg         ss_psram_rd_req  = 0,
    output reg  [18:0] ss_psram_rd_addr = 19'h0,
    input  wire [15:0] ss_psram_rd_data,
    input  wire        ss_psram_rd_ack
);

  // (The ss_pause_cpu assign moved below the sys_state declaration so
  // ModelSim's vlog accepts the forward references; Quartus tolerated
  // them.  No functional change.)

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

  // SAVE-path book-keeping (declared here, before fifo_save, because the
  // serve FSM's assembled chunk feeds fifo_save.data).  Firmware chunk →
  // SDRAM staging, then SDRAM → fifo_save → APF serve; mirrors the load
  // staging/serve with firmware as producer and APF as consumer.
  reg [63:0] save_buffer;        // 64-bit chunk latched from ss_din
  reg [1:0]  save_word_idx;      // which of 4 SDRAM words being written
  reg [24:0] save_addr;          // SDRAM byte addr base for this chunk
  reg [16:0] save_serve_idx;     // chunk index being served back to APF
  reg [1:0]  save_serve_widx;    // which of 4 SDRAM words being read
  reg [63:0] save_serve_buf;     // assembled 64-bit chunk to push to fifo_save
  // Full declared slot = savestate_size (512KB) / 8 bytes = 65536 chunks.
  localparam [16:0] SAVE_SERVE_LAST = 17'd65535;

  // Serve completion watchdog + pre-fill gate — fix the old serve's two bugs:
  //  (1) FREEZE — SRV_PUSH stalled on a full FIFO (or SRV_DONE waiting on
  //      empty) could hang forever if APF stopped reading, leaving
  //      ss_pause_cpu stuck.  An APF-read-activity watchdog forces completion
  //      (ss_loading<=0 → IDLE) after a bounded idle, so the core can NEVER
  //      permanently freeze on save regardless of how much APF read.
  //  (2) MID-STREAM DROPS — pre-fill fifo_save to full BEFORE asserting
  //      savestate_start_ok, so APF begins reading with a full buffer
  //      head-start and is far less likely to out-pace the SDRAM refill.
  reg [20:0] save_serve_idle = 21'd0;
  reg        serve_prefilled = 1'b0;
  // SAVE_SERVE_IDLE_MAX is a module parameter (see header) so the TB can
  // shorten it; default ~14 ms @ 21.48 MHz.

  // Save serve FIFO: written by the SDRAM-read serve FSM (save_serve_buf,
  // clk_sys), drained by APF bridge_rd (clk_74a).  Deep (512 entries) to
  // absorb APF read bursts while the serve refills from SDRAM.  showahead=OFF
  // matches the ORIGINAL streaming-save read timing that delivered the
  // header correctly: the rising-edge bridge_rd pop issues rdreq and q
  // presents the popped word a couple cycles later (within APF's read
  // latency).  showahead=ON dropped the first 32-bit word (the pop advanced
  // q before APF latched the head), which corrupted the header AND left the
  // FIFO one word short so SRV_DONE never saw it empty → save freeze.
  dcfifo_mixed_widths fifo_save (
      .data(save_serve_buf),
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
      fifo_save.lpm_numwords  = 1024,
      fifo_save.lpm_showahead = "OFF",
      fifo_save.lpm_type      = "dcfifo_mixed_widths",
      fifo_save.lpm_width     = 64,
      fifo_save.lpm_widthu    = 10,
      fifo_save.lpm_widthu_r  = 11,
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
  wire  [8:0] fifo_load_rdusedw;  // read-side used count (0..511 words)
  wire [31:0] fifo_load_dout;

  // v65 FIX (the actual root cause): APF holds bridge_wr HIGH for ~2
  // clk_74a cycles per 32-bit word, with bridge_wr_data valid only on the
  // first (rising-edge) cycle and the bus cleared to 0 on the rest.  The
  // old LEVEL-sensitive trigger below pushed each word TWICE — once with
  // real data, once with $0000 — so the load FIFO filled as
  // word0,0,word1,0,... and every staged chunk got correct lower 32 bits
  // and zero upper 32 bits (= "words 2/3 of every chunk are $0000").  The
  // MiSTer data_loader.sv avoids this by EDGE-detecting bridge_wr (see its
  // `~prev_bridge_wr && bridge_wr`); the ROM path worked for exactly that
  // reason.  Match it: push on the rising edge only.
  reg prev_bridge_wr_fifo = 1'b0;
  always @(posedge clk_74a) prev_bridge_wr_fifo <= bridge_wr;
  wire fifo_load_write =
      ~prev_bridge_wr_fifo && bridge_wr && (bridge_addr[31:28] == 4'h4);

  // Overflow detector (clk_74a): if a bridge_wr arrives while the FIFO is
  // full, that write is DROPPED -> staged data has a hole.  Latch a sticky
  // flag and count the drops (saturating).  Hypothesis: APF streams faster
  // than the 4-SDRAM-write-per-chunk staging can drain, so the 512-entry
  // FIFO overflows on large saves -> deterministic per-game corruption.
  reg        fifo_load_overflow = 0;
  reg [15:0] fifo_load_drop_cnt = 16'h0000;
  always @(posedge clk_74a) begin
    if (fifo_load_write && fifo_load_full) begin
      fifo_load_overflow <= 1;
      if (fifo_load_drop_cnt != 16'hFFFF)
        fifo_load_drop_cnt <= fifo_load_drop_cnt + 16'd1;
    end
  end

  // v61 FIX: was dcfifo_mixed_widths 32→64.  v59/v60 hardware evidence
  // (FIFO upper byte lanes carry data at SOME latches, yet chunk 0's
  // word-2/3 writes commit at the correct address with data $0000) points
  // at the mixed-width read side popping a 64-bit word when only its
  // first 32-bit half has been written (upper half reads as 0; M10K
  // powers up zeroed).  The RTL sim model doesn't do this, which is why
  // tb_ss_staging passed.  Chunk 0 holds the "SNES-SS" magic the load
  // firmware validates, so a single half-pop at stream start kills every
  // load while leaving all later chunks perfectly aligned.
  // Fix: plain 32-bit dcfifo (rdempty is exact per 32-bit word) and
  // explicit two-pop 64-bit assembly in the staging FSM (stage_half).
  dcfifo fifo_load (
      .data(bridge_wr_swapped),
      .rdclk(clk_sys),
      .rdreq(fifo_load_read_req),
      .wrclk(clk_74a),
      .wrreq(fifo_load_write),
      .q(fifo_load_dout),
      .rdempty(fifo_load_empty),
      .rdusedw(fifo_load_rdusedw),
      .wrfull(fifo_load_full),
      .aclr(1'b0)
  );
  defparam fifo_load.intended_device_family = "Cyclone V",
      fifo_load.lpm_numwords  = 512,
      fifo_load.lpm_showahead = "OFF",
      fifo_load.lpm_type      = "dcfifo",
      fifo_load.lpm_width     = 32,
      fifo_load.lpm_widthu    = 9,
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

  // SAVE staging: write firmware's 64-bit chunks into SDRAM (4×16-bit words)
  localparam SYS_SAVE_WR_REQ      = 5'd2;
  localparam SYS_SAVE_WR_WAIT     = 5'd3;
  // SAVE serve: read the staged SDRAM back out and feed fifo_save for APF
  localparam SYS_SAVE_SRV_RD_REQ  = 5'd4;
  localparam SYS_SAVE_SRV_RD_WAIT = 5'd5;
  localparam SYS_SAVE_SRV_RD_NEXT = 5'd6;
  localparam SYS_SAVE_SRV_PUSH    = 5'd7;
  localparam SYS_SAVE_SRV_DONE    = 5'd8;

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
  // Probe: after staging completes, read 4 known SDRAM locations directly
  // (no firmware involvement) and latch the results.  Isolates whether
  // staged data survives in SDRAM vs whether the serve/firmware path is
  // the bug.
  localparam SYS_PROBE_REQ        = 5'd28;
  localparam SYS_PROBE_WAIT       = 5'd29;
  localparam SYS_PROBE_NEXT       = 5'd30;

  reg [4:0] sys_state = SYS_IDLE;
  reg [4:0] prev_sys_state = SYS_IDLE;
  assign debug_sys_state = sys_state[3:0];

  // The CPU must be frozen only while we're actively hijacking SDRAM for
  // staging (the serve phase needs the CPU running to execute firmware).
  // Staging states are SYS_STAGE_* (5'd10..5'd15) and the brief STAGING
  // start in SYS_IDLE.  Simplest robust definition: pause whenever the
  // FSM is in any staging-related state.
  // Pause the SNES CPU during LOAD staging (game would otherwise corrupt the
  // hijacked SDRAM) AND during the SAVE serve.  The save firmware has RTI'd
  // back to the running game by the time APF reads the slot, but the serve
  // reads the staged state from SDRAM with ss_loading hijacking the cart-ROM
  // path — so the game must be frozen for the serve window or it reads
  // garbage cart-ROM (the observed save freeze).  Audio (SMP on ACLK) keeps
  // running; this is a brief hitch that resumes when the serve completes.
  // NOT paused during SAVE staging (SYS_SAVE_ACTIVE / SYS_SAVE_WR_*): the CPU
  // must run there to execute the save firmware producing the chunks.
  // SAVE no longer pauses the CPU (2026-06-22): stage and serve are on
  // CRAM1 bank 1 (PSRAM), which is a different physical chip from cart-ROM
  // SDRAM, so the running game's instruction fetch is undisturbed.  Only
  // LOAD staging (SDRAM staging hijack) still needs MCLK paused.
  assign ss_pause_cpu = (sys_state == SYS_STAGE_FIFO_RD)     ||
                        (sys_state == SYS_STAGE_FIFO_WAIT)   ||
                        (sys_state == SYS_STAGE_FIFO_LATCH)  ||
                        (sys_state == SYS_STAGE_WR_REQ)      ||
                        (sys_state == SYS_STAGE_WR_WAIT)     ||
                        (sys_state == SYS_STAGE_IDLE);

  // Edge / handshake tracking
  reg prev_savestate_start = 0;
  reg prev_savestate_load  = 0;
  reg prev_ss_busy         = 0;
  reg prev_ss_req          = 0;
  reg prev_ss_sdram_wr_ack = 0;
  reg prev_ss_sdram_rd_ack = 0;
  reg prev_ss_psram_wr_ack = 0;
  reg prev_ss_psram_rd_ack = 0;

  wire new_ddr_req      = (ss_req != prev_ss_req);
  wire sdram_wr_done    = (ss_sdram_wr_ack != prev_ss_sdram_wr_ack);
  wire sdram_rd_done    = (ss_sdram_rd_ack != prev_ss_sdram_rd_ack);
  wire psram_wr_done    = (ss_psram_wr_ack != prev_ss_psram_wr_ack);
  wire psram_rd_done    = (ss_psram_rd_ack != prev_ss_psram_rd_ack);

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

  // Staging-quiet detector: count clk_sys cycles since the last bridge_wr
  // activity.  When this exceeds a threshold AND FIFO is empty, we
  // consider staging done.  Avoids fragile exact-count gate.
  reg [19:0] stage_quiet_cnt = 20'd0;
  // Bridge-wr count (clk_74a) — useful to confirm APF wrote all chunks.
  // (declaration hoisted above its first use for ModelSim)
  reg [15:0] bridge_wr_count = 16'h0000;
  // Sync bridge_wr_count[7:0] (clk_74a) to clk_sys, then detect changes.
  wire [7:0] bridge_wr_lo_sys;
  synch_3 #(.WIDTH(8)) sync_bridge_wr_lo (
      .i(bridge_wr_count[7:0]), .o(bridge_wr_lo_sys), .clk(clk_sys));
  reg [7:0] prev_bridge_wr_lo_sys = 8'd0;

  // Staging book-keeping
  reg [1:0]  stage_word_idx;       // which of the 4 SDRAM words within the chunk
  reg        stage_half = 1'b0;    // v61: which 32-bit half of the chunk the
                                   // next FIFO pop fills (0=lower, 1=upper)
  reg [15:0] dbg_w2_src      = 16'h0000;  // v62: stage_buffer[47:32] @ word2 WR_REQ
  reg        dbg_w2_src_seen = 1'b0;
  reg [15:0] dbg_pop2        = 16'h0000;  // v63: raw FIFO 2nd-pop[15:0], chunk0
  reg        dbg_pop2_seen   = 1'b0;
  reg [3:0]  wr_gap_cnt = 4'd0;    // v58: cooldown after each word write
  reg [63:0] stage_buffer;
  reg [24:0] stage_addr;

  // Serve book-keeping
  reg [1:0]  serve_word_idx;
  reg [63:0] serve_buffer;
  reg [24:0] serve_addr;

  // Probe book-keeping
  reg  [1:0]  probe_idx;        // which of 4 probe addresses we're reading
  reg  [15:0] probe_result_0;   // SDRAM word at STAGING_BASE_WORD+0    (chunk 0 word 0, want $4E53)
  reg  [15:0] probe_result_1;   // SDRAM word at STAGING_BASE_WORD+2    (chunk 0 word 1, want $5345)
  reg  [15:0] probe_result_2;   // SDRAM word at STAGING_BASE_WORD+0x200 (chunk 64 word 0)
  reg  [15:0] probe_result_3;   // SDRAM word at STAGING_BASE_WORD+0x2000 (chunk 1024 word 0)
  reg         probe_done = 0;

  // Sticky snapshot of stage_addr taken when STAGE_IDLE guard fires (right
  // before entering PROBE).  Tells us how far staging advanced — if this
  // equals BASE+8, only chunk 0 staged.  If BASE+0x20, only 4 chunks staged.
  // If BASE+0x12345, many chunks but ended at addr 0x12345.
  reg  [24:0] stage_addr_at_done = 25'd0;

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

  assign debug_bridge_wr_count_lo = bridge_wr_count[7:0];
  assign debug_bridge_wr_count_hi = bridge_wr_count[15:8];

  // First bridge_wr data (sticky) — should be 'SNES' (0x53 0x4E 0x45 0x53)
  reg [31:0] first_wr_data = 32'h00000000;
  reg [31:0] first_wr_addr = 32'h00000000;
  reg        first_wr_seen = 0;
  // v64/v67: swapped [15:0] of the first FOUR words pushed into the load
  // FIFO.  v67 uses wr3/wr4 to test the "drops odd words" hypothesis at a
  // file position that is genuinely NONZERO (payload words 2,3), since
  // wr2 was zero in the file itself and so couldn't disprove a drop.
  reg [15:0] dbg_wr1 = 16'h0000;
  reg [15:0] dbg_wr2 = 16'h0000;
  reg [15:0] dbg_wr3 = 16'h0000;
  reg [15:0] dbg_wr4 = 16'h0000;
  reg [2:0]  dbg_wr_cap = 3'd0;
  // v66: RAW (level, every clk_74a cycle) capture of bridge_addr[7:0] for
  // the first four cycles where bridge_wr is high to the 0x4xxxxxxx region.
  // Reveals the true APF write pattern that all the downstream symptoms
  // derive from:
  //   $00,$00,$04,$04  → bridge_wr is a MULTI-cycle strobe, stride 4
  //                      (edge-detect is the right fix; if v65 didn't help,
  //                       the fix didn't reach the build)
  //   $00,$04,$08,$0C  → 1-cycle pulses, contiguous 32-bit — then odd words
  //                      genuinely carry $0000 (endianness / data-slot issue)
  //   $00,$08,$10,$18  → APF strides by 8, only writing every other 32-bit
  //                      slot (we must reconstruct the skipped words)
  reg [7:0] dbg_a1 = 8'h00, dbg_a2 = 8'h00, dbg_a3 = 8'h00, dbg_a4 = 8'h00;
  reg [2:0] dbg_a_cap = 3'd0;
  // Repurposed for Phase C: expose serve-FSM action counters.
  //   data_b0 → cnt_serve_wait_entries  (# entries to SERVE_WAIT_REQ)
  //   data_b1 → cnt_serve_rd_entries    (# entries to SERVE_RD_REQ)
  //   addr_lo → cnt_serve_ack_entries   (# entries to SERVE_ACK)
  //   addr_hi → cnt_ss_load_pulses      (# ss_load pulses we emitted)
  // (assigns moved below the counter declarations for ModelSim — see
  // "deferred debug assigns" block after cnt_stage_wr_done_wide.)

  // SAVE-side first chunk debug (kept from Phase A)
  reg [63:0] first_save_chunk = 64'h0;
  reg [16:0] first_save_addr  = 17'd0;
  reg        first_save_seen  = 0;
  // Repurposed for Phase C diagnostics:
  // Repurposed v33: chunk-4 and chunk-40 first-byte samples for data check.
  //   sample_chunk4  = serve_buffer[7:0] at ACK #2  (= chunk 1 byte 0)
  //   sample_chunk40 = serve_buffer[7:0] at ACK #16 (= chunk 15? wait, ACK #N → chunk N)
  // Actually rechecked: ack_idx_count starts at 0, first chunk's ACK is at
  // count=0, so ack_idx_count==1 captures chunk 1 (file offset 0x08, all zeros).
  // sample_chunk8 at ack_idx==4 captures chunk 4 (file +0x20 = "0.0.1\0").
  // sample_chunk40 at ack_idx==8 captures chunk 8 (file +0x40).
  // sample_chunk64 at ack_idx==40 captures chunk 40 (file +0x140).
  // (debug_first_save_byte0/1 assigns moved below sample_chunk8/40
  // declarations for ModelSim — see "deferred debug assigns" block.)
  // Repurposed: expose ddr-req-in-wait diagnostics.
  //   addr_lo → cnt_ddr_req_in_wait (# ddr_req edges seen in SERVE_WAIT)
  //   addr_hi → {7'b0, ss_rnw_at_first_wait_req} (direction of first edge)
  // Repurposed v33: chunk-1 and chunk-40 first-byte samples.
  // v57: probe all 4 words of chunk 0 with settle=1 (reverted from 7).
  // v67: swapped [15:0] of the 3rd word pushed into the FIFO (payload
  // word 2 = 0x0d1feb00, NONZERO in the file).  Tests whether the bridge
  // delivers later words faithfully.  Want $00 / $EB.
  assign debug_first_save_addr_lo = dbg_wr3[7:0];
  assign debug_first_save_addr_hi = dbg_wr3[15:8];

  // Stage-write debug: first SDRAM word written + its address
  reg [15:0] first_stage_word = 16'h0000;
  reg [24:0] first_stage_addr = 25'h0;
  reg        first_stage_seen = 0;
  // Repurposed v42: stage_addr RAW bytes (no offset math) to verify it
  // ever got assigned STAGING_BASE_WORD = $800000 (bit 23 = 1).
  //   sram_w0_lo → stage_addr_at_done[15:8]  (mid byte; for $800000 it's $00)
  //   sram_w0_hi → stage_addr_at_done[23:16] (high byte; for $800000 it's $80)
  // If high byte is NOT $80, stage_addr never got the BASE assignment!
  assign debug_first_sram_w0_lo = stage_addr_at_done[15:8];
  assign debug_first_sram_w0_hi = stage_addr_at_done[23:16];
  // The previously-unassigned debug_first_pf_addr_lo/hi outputs were
  // dangling (default 0) — that's why v28/v29 readings of "cnt_stage_*"
  // via dbg_first_pf_addr_*_video always showed $00.  Wire them up here.
  // v49: 16-bit true count of sdram_wr_done events.  Expected ~262144 if
  // each chunk had 4 distinct writes (saturates 16-bit at $FFFF = 65535).
  // If MUCH LOWER, sdram_wr_done isn't firing as often as we thought.
  // (debug_first_pf_addr_lo/hi assigns moved below the
  // cnt_stage_wr_done_wide declaration for ModelSim — see below.)

  // Serve-side first SDRAM read result (= first chunk byte 0..1)
  reg [15:0] first_serve_word = 16'h0000;
  reg        first_serve_seen = 0;
  // Repurposed: expose serve chunk word1 (file bytes 2,3) for mapping check.
  //   debug_first_sram_w1_lo = first_serve_chunk[23:16] (= word1 lo = fb2, want $45)
  //   debug_first_sram_w1_hi = first_serve_chunk[31:24] (= word1 hi = fb3, want $53)
  // Repurposed v37: SDRAM probe results.
  //   sram_w1_lo → probe_result_2[7:0]  (chunk 64 word 0, low byte)
  //   sram_w1_hi → probe_result_3[7:0]  (chunk 1024 word 0, low byte)
  // v53: nonzero-data writes per top-nibble.  Expose nibble $0 and nibble $4
  // low byte counts (saturated 8 bit) — if nibble $0 (i.e. low addresses
  // like 0x0xxxxxxx ROM area) has MORE nonzero writes than nibble $4
  // (savestate area), then APF is putting the data somewhere unexpected.
  // v54: first nonzero-data 0x4xxxxxxx write — low 2 bytes of bridge_addr.
  // Reveals the starting OFFSET APF uses within savestate region.
  // v55b: re-wire to deep SDRAM probes after the ~sdram_busy gate fix.
  assign debug_first_sram_w1_lo = probe_result_2[7:0];  // 32 KB in
  assign debug_first_sram_w1_hi = probe_result_3[7:0];  // last chunk

  // Capture the FULL first served chunk (all 4 SDRAM words) so we can
  // verify the complete byte mapping against the known .sta payload:
  //   chunk0 file bytes = 53 4E 45 53 2D 53 53 00  ("SNES-SS\0")
  //   correct words: w0=$4E53 w1=$5345 w2=$532D w3=$0053
  reg [63:0] first_serve_chunk = 64'h0;
  reg        first_serve_chunk_seen = 0;

  // Diagnostic: capture serve_addr's low byte at the 2nd, 5th, 16th, 64th
  // SERVE_ACK entries.  If chunks are served sequentially with stride 8
  // bytes, serve_addr at the Nth ACK = STAGING_BASE_WORD + 8*(N-1) + 6,
  // so low byte = 8*(N-1)+6.
  //   2nd ACK: low byte = $0E
  //   5th ACK: low byte = $26 (matches chunk 4!)
  //   16th ACK: low byte = $7E
  //   64th ACK: low byte = $F6
  // If the low bytes don't match this pattern, the serve isn't iterating
  // through sequential chunks as I expect.
  reg [7:0] sample_chunk4  = 8'h00;  // captured at ACK #2
  reg [7:0] sample_chunk8  = 8'h00;  // captured at ACK #5
  reg [7:0] sample_chunk40 = 8'h00;  // captured at ACK #16
  reg [7:0] sample_chunk64 = 8'h00;  // captured at ACK #64
  reg       sample4_seen   = 0;
  reg       sample8_seen   = 0;
  reg       sample40_seen  = 0;
  reg       sample64_seen  = 0;
  reg [7:0] ack_idx_count  = 8'h00;  // counts SERVE_ACK entries (saturating)

  // Capture stage_buffer at chunk 4 staging time — proves whether the
  // FIFO output is correct.  Want $00_00_00_31_2E_30_2E_30 (file +0x20).
  reg [63:0] sample_stage_buf4 = 64'h0;
  reg        sample_buf4_seen  = 0;

  // v44: ungated capture of EVERY stage_buffer load.
  reg [63:0] latest_stage_buf  = 64'h0;
  reg [63:0] second_stage_buf  = 64'h0;
  reg        first_buf_seen    = 0;
  reg        second_buf_seen   = 0;
  // v46: count how many FIFO_LATCH events have non-zero data.
  reg [15:0] cnt_nonzero_fifo_dout = 16'h0000;

  // v48: confidence-gated drain.  Hypothesis: dcfifo's rdempty has CDC lag,
  // so we sometimes see "non-empty" briefly even when the data hasn't fully
  // settled into the read side.  Track how long rdempty has been LOW
  // continuously.  Only drain when it's been LOW for several cycles AND
  // rdusedw indicates real depth.
  reg [3:0] fifo_nonempty_settled = 4'd0;
  always @(posedge clk_sys) begin
    if (fifo_load_empty)
      fifo_nonempty_settled <= 4'd0;
    else if (fifo_nonempty_settled != 4'hF)
      fifo_nonempty_settled <= fifo_nonempty_settled + 4'd1;
  end
  // Considered "safe to read" only after rdempty has been low for several
  // consecutive cycles.  Filters CDC glitches.
  wire fifo_drain_ok = (fifo_nonempty_settled >= 4'd4);
  // OR-checksum: bitwise OR of every fifo_load_dout latched.  Each bit is
  // sticky-set if any FIFO read had that bit high.  Almost-all-bits-high
  // means FIFO is delivering varied data; mostly-zero means FIFO is dry.
  reg [63:0] fifo_or_checksum = 64'h0;

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

  // Staging-FSM action counters (saturating).
  reg [7:0] cnt_stage_fifo_latch = 8'h00;  // entered STAGE_FIFO_LATCH
  reg [7:0] cnt_stage_wr_done    = 8'h00;  // sdram_wr_done fired in WR_WAIT
  reg [15:0] cnt_stage_wr_done_wide = 16'h0000;  // 16-bit version (true count up to 65535)

  // ----- Deferred debug assigns (moved here so every referenced reg is
  // declared above; Quartus tolerated the forward references, ModelSim's
  // vlog does not.  No functional change.) -----
  //   data_b0 → cnt_serve_wait_entries  (# entries to SERVE_WAIT_REQ)
  //   data_b1 → cnt_serve_rd_entries    (# entries to SERVE_RD_REQ)
  //   addr_lo → cnt_serve_ack_entries   (# entries to SERVE_ACK)
  //   addr_hi → cnt_ss_load_pulses      (# ss_load pulses we emitted)
  assign debug_first_wr_data_b0 = cnt_serve_wait_entries;
  assign debug_first_wr_data_b1 = cnt_serve_rd_entries;
  assign debug_first_wr_addr_lo = cnt_serve_ack_entries;
  assign debug_first_wr_addr_hi = cnt_ss_load_pulses;
  // v67: swapped [15:0] of the 4th word pushed into the FIFO (payload
  // word 3 = 0x0000d1e2, NONZERO in the file).  This is a "word 2/3 of a
  // chunk" position — if the bridge dropped odd words it would read $0000.
  // Want $E2 / $D1.
  assign debug_first_save_byte0 = dbg_wr4[7:0];
  assign debug_first_save_byte1 = dbg_wr4[15:8];
  // v49: 16-bit true count of sdram_wr_done events (see comment at the
  // original site above).
  assign debug_first_pf_addr_lo = cnt_stage_wr_done_wide[7:0];
  assign debug_first_pf_addr_hi = cnt_stage_wr_done_wide[15:8];
  reg [7:0] cnt_stage_idle_enter = 8'h00;  // entered SYS_STAGE_IDLE
  reg [7:0] cnt_guard_pass       = 8'h00;  // STAGE_IDLE guard passed -> SERVE_KICK_WAIT
  reg [7:0] cnt_fifo_nonempty    = 8'h00;  // # clk_sys cycles where fifo_load_empty=0 (saturating)
  reg       fifo_nonempty_ever   = 0;
  reg       have_staged_any      = 0;  // sticky: at least one chunk has been staged

  // Count of staging-FIFO entries written to SDRAM (saturating)
  // 17-bit so it can count up to 65536 (savestate_size 512KB / 8) without
  // saturating.  Reset at SERVE_COMPLETE.
  reg [16:0] stage_entry_count = 17'h00000;
  // STICKY max: tracks the highest stage_entry_count ever reached; NOT
  // reset at SERVE_COMPLETE so we can read it via overlay after a load.
  // Reveals how many chunks APF actually streamed.
  reg [16:0] stage_max_count = 17'h00000;
  assign debug_save_wr_count_lo = stage_max_count[7:0];
  assign debug_save_wr_count_hi = stage_max_count[15:8];

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

  // Sync bridge_rd_count[7:0] (clk_74a) to clk_sys so the serve watchdog can
  // tell whether APF is still actively reading the slot.  A change between
  // consecutive clk_sys cycles = APF popped at least one word recently.
  wire [7:0] bridge_rd_lo_sys;
  synch_3 #(.WIDTH(8)) sync_bridge_rd_lo (
      .i(bridge_rd_count[7:0]), .o(bridge_rd_lo_sys), .clk(clk_sys));
  reg [7:0] prev_bridge_rd_lo_sys = 8'd0;
  wire apf_reading = (bridge_rd_lo_sys != prev_bridge_rd_lo_sys);

  // pf_at_first_rd debug — repurposed: stage-FSM action counters
  //   debug_pf_at_first_rd_lo → cnt_stage_fifo_latch (# FIFO entries drained)
  //   debug_pf_at_first_rd_hi → cnt_stage_wr_done    (# SDRAM writes completed)
  reg [15:0] fifo_drain_count = 16'h0000;  // kept for backcompat, not on overlay
  assign debug_pf_at_first_rd_lo = cnt_stage_fifo_latch;
  assign debug_pf_at_first_rd_hi = cnt_stage_wr_done;

  // last_w0_data / w0_wr_count — Phase A SRAM debug, now stale
  // Repurposed: FIFO-overflow diagnostics.
  //   last_w0_data_lo → fifo_load_drop_cnt[7:0]
  //   last_w0_data_hi → fifo_load_drop_cnt[15:8]
  //   w0_wr_count     → {7'b0, fifo_load_overflow}  ($01 = overflowed)
  // v47: chunk 4 word 0 / word 1 low bytes.  Want $30 / $30 for SMW.
  // v57: probe_result_2 low (word 2 want $2D) + probe_result_3 low (word 3 want $53)
  assign debug_last_w0_data_lo = probe_result_2[7:0];
  assign debug_last_w0_data_hi = probe_result_3[7:0];
  assign debug_w0_wr_count     = {7'b0, fifo_load_overflow};

  // Count of fifo_load_write events with non-zero bridge_wr_data.  Lives
  // entirely on clk_74a domain — bypasses FIFO, CDC, and FSM entirely.
  reg [15:0] bridge_wr_nonzero_count = 16'h0000;
  reg [15:0] bridge_wr_any_nonzero = 16'h0000;
  reg [15:0] bridge_wr_4xxx_count  = 16'h0000;
  // v53: capture bridge_addr at the FIRST and a LATE nonzero-data write.
  reg [31:0] first_nonzero_addr  = 32'h0;
  reg        first_nonzero_seen  = 0;
  // v54: track the FIRST nonzero-data write that ALSO targets 0x4xxxxxxx
  // (the savestate region we care about).  Captures the actual addr bits
  // [27:16] so we can see if APF strides through chunks or what.
  reg [31:0] first_nz_4xxx_addr  = 32'h0;
  reg        first_nz_4xxx_seen  = 0;
  // Count nonzero-data writes to ANY 0x4xxxxxxx, full 16-bit.
  reg [15:0] nz_4xxx_count_wide  = 16'h0000;
  // Histogram: count nonzero-data writes per top-nibble of bridge_addr.
  // 16 buckets.  Reveals where APF's nonzero writes actually go.
  reg [15:0] nz_writes_per_top_nibble [0:15];
  integer i_init;
  initial begin
    for (i_init = 0; i_init < 16; i_init = i_init + 1)
      nz_writes_per_top_nibble[i_init] = 16'h0000;
  end

  // ----- bridge-wr counters (clk_74a) -----
  always @(posedge clk_74a) begin
    // v66: RAW level capture of the first four 0x4xxxxxxx write-cycle
    // addresses (low byte).  Uses bridge_wr directly (NOT the edge-detected
    // fifo_load_write) so a multi-cycle strobe shows up as repeated values.
    if (bridge_wr && bridge_addr[31:28] == 4'h4 && dbg_a_cap != 3'd4) begin
      case (dbg_a_cap)
        3'd0: dbg_a1 <= bridge_addr[7:0];
        3'd1: dbg_a2 <= bridge_addr[7:0];
        3'd2: dbg_a3 <= bridge_addr[7:0];
        3'd3: dbg_a4 <= bridge_addr[7:0];
      endcase
      dbg_a_cap <= dbg_a_cap + 3'd1;
    end
    // v52 broad-net counters: count ALL bridge_wr regardless of address.
    if (bridge_wr) begin
      if (bridge_wr_data != 32'h0 && bridge_wr_any_nonzero != 16'hFFFF) begin
        bridge_wr_any_nonzero <= bridge_wr_any_nonzero + 16'h0001;
      end
      if (bridge_addr[31:28] == 4'h4 && bridge_wr_4xxx_count != 16'hFFFF) begin
        bridge_wr_4xxx_count <= bridge_wr_4xxx_count + 16'h0001;
      end
      // v53: capture first nonzero-data write's address; histogram top nibble.
      if (bridge_wr_data != 32'h0) begin
        if (!first_nonzero_seen) begin
          first_nonzero_addr <= bridge_addr;
          first_nonzero_seen <= 1;
        end
        if (nz_writes_per_top_nibble[bridge_addr[31:28]] != 16'hFFFF) begin
          nz_writes_per_top_nibble[bridge_addr[31:28]] <=
              nz_writes_per_top_nibble[bridge_addr[31:28]] + 16'h0001;
        end
      end
      // v54: 16-bit precise count of nonzero-data writes to 0x4xxxxxxx only
      // (the savestate region we care about).  And first such address.
      if (bridge_wr_data != 32'h0 && bridge_addr[31:28] == 4'h4) begin
        if (nz_4xxx_count_wide != 16'hFFFF)
          nz_4xxx_count_wide <= nz_4xxx_count_wide + 16'h0001;
        if (!first_nz_4xxx_seen) begin
          first_nz_4xxx_addr <= bridge_addr;
          first_nz_4xxx_seen <= 1;
        end
      end
    end
    if (fifo_load_write) begin
      if (bridge_wr_count != 16'hFFFF) begin
        bridge_wr_count <= bridge_wr_count + 16'h0001;
      end
      if (bridge_wr_data != 32'h0 && bridge_wr_nonzero_count != 16'hFFFF) begin
        bridge_wr_nonzero_count <= bridge_wr_nonzero_count + 16'h0001;
      end
      if (!first_wr_seen) begin
        first_wr_data <= bridge_wr_data;
        first_wr_addr <= bridge_addr;
        first_wr_seen <= 1;
      end
      // v64: capture the swapped [15:0] of the FIRST and SECOND words
      // pushed into the load FIFO (clk_74a, write side, BEFORE the FIFO).
      // chunk 0 = bytes 0-7 = "SNES" + "-SS\0":
      //   1st FIFO entry [15:0] = $4E53 ("SN")
      //   2nd FIFO entry [15:0] = $532D ("-S")  <- the word that reads $00
      // If dbg_wr2 is $532D here but dbg_pop2 (read side) is $00, the FIFO
      // dropped/zeroed the second word on read.  If dbg_wr2 is $00, APF/
      // bridge never delivered the chunk's second word to us.
      if (dbg_wr_cap != 3'd4) begin
        case (dbg_wr_cap)
          3'd0: dbg_wr1 <= bridge_wr_swapped[15:0];
          3'd1: dbg_wr2 <= bridge_wr_swapped[15:0];
          3'd2: dbg_wr3 <= bridge_wr_swapped[15:0];
          3'd3: dbg_wr4 <= bridge_wr_swapped[15:0];
        endcase
        dbg_wr_cap <= dbg_wr_cap + 3'd1;
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
    prev_sys_state       <= sys_state;
    prev_savestate_start <= savestate_start_s;
    prev_savestate_load  <= savestate_load_s;
    prev_ss_busy         <= ss_busy;
    prev_ss_req          <= ss_req;
    prev_ss_sdram_wr_ack <= ss_sdram_wr_ack;
    prev_ss_sdram_rd_ack <= ss_sdram_rd_ack;
    prev_ss_psram_wr_ack <= ss_psram_wr_ack;
    prev_ss_psram_rd_ack <= ss_psram_rd_ack;

    // 1-cycle pulses
    ss_save             <= 0;
    ss_load             <= 0;
    fifo_save_write_req <= 0;
    fifo_load_read_req  <= 0;

    if (wr_gap_cnt != 4'd0) wr_gap_cnt <= wr_gap_cnt - 4'd1;
    // NOTE: ss_sdram_wr_req / ss_sdram_rd_req are TOGGLE bits, not
    // pulses — the SNES.sv clk_mem-side FSM edge-detects them via
    // synch_3.  Don't default them to 0 here.

    // Sticky debug taps
    if (ss_busy && !prev_ss_busy) begin
      ss_busy_seen  <= 1;
      ss_busy_ever  <= 1;
      ss_busy_rises <= ss_busy_rises + 4'd1;
    end

    // Track whether the load FIFO was ever observed non-empty on the
    // clk_sys side.  If this stays 0 despite bridge_wr_count incrementing,
    // the FIFO's CDC isn't propagating writes to the read side.
    if (~fifo_load_empty) begin
      fifo_nonempty_ever <= 1;
      if (cnt_fifo_nonempty != 8'hFF)
        cnt_fifo_nonempty <= cnt_fifo_nonempty + 8'd1;
    end

    // Staging-quiet tracker: reset to 0 whenever new bridge_wr activity
    // is detected (synced bridge_wr_count low byte changes) OR FIFO is
    // non-empty.  Otherwise increment, saturating.
    prev_bridge_wr_lo_sys <= bridge_wr_lo_sys;
    if ((bridge_wr_lo_sys != prev_bridge_wr_lo_sys) || ~fifo_load_empty) begin
      stage_quiet_cnt <= 20'd0;
    end else if (stage_quiet_cnt != 20'hFFFFF) begin
      stage_quiet_cnt <= stage_quiet_cnt + 20'd1;
    end

    // Serve watchdog clock-edge bookkeeping (see SAVE serve states below).
    prev_bridge_rd_lo_sys <= bridge_rd_lo_sys;
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
      // 2026-06-22: SAVE no longer hijacks cart-ROM SDRAM.  Stage + serve
      // both target CRAM1 bank 1 (PSRAM) — a separate physical chip — so
      // the running game's cart-ROM bus is untouched throughout the save.
      // ss_loading therefore stays low for SAVE; only LOAD raises it.
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
        if (fifo_drain_ok) begin
          // Begin staging.  stage_addr starts at STAGING_BASE_WORD on
          // the first FIFO entry of a transfer; once running, it just
          // keeps incrementing through the whole load.
          if (stage_entry_count == 17'd0) begin
            stage_addr <= STAGING_BASE_WORD;
            stage_half <= 1'b0;
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
      // SAVE path — stage firmware's 64-bit chunks into SDRAM, then serve
      // the complete staged buffer to APF (symmetric with the load path).
      // The previous streaming-FIFO approach raced APF (fast consumer) vs
      // the firmware (slow producer) through a 16-deep FIFO → underrun →
      // the saved file was a constant fill.  Pre-staging in SDRAM removes
      // the race: APF reads a complete, stable buffer.
      // ============================================================
      SYS_SAVE_ACTIVE: begin
        if (~savestate_start_s) savestate_start_ack <= 0;

        if (new_ddr_req && ~ss_rnw) begin
          // Firmware wrote a 64-bit chunk for byte-chunk index ss_addr.
          // Latch it and write it to PSRAM as 4×16-bit words.  Do NOT ack
          // yet — the firmware's ddr_req==ddr_ack backpressure holds it
          // until the chunk has fully committed (ack toggles in WR_WAIT).
          //
          // 2026-06-22: SAVE moved off cart-ROM SDRAM onto CRAM1 bank 1.
          // PSRAM has no skip-stale quirk; addresses step by 1 word, not 2.
          // Bank 1 is private to savestate, so base = 0.  save_addr keeps
          // its 25-bit width but only the low 19 bits are meaningful here.
          save_buffer    <= ss_din;
          save_addr      <= {6'b0, ss_addr[16:0], 2'b00};
          save_word_idx  <= 2'd0;
          sys_state      <= SYS_SAVE_WR_REQ;
          if (!first_save_seen) begin
            first_save_chunk <= ss_din;
            first_save_addr  <= ss_addr;
            first_save_seen  <= 1;
          end
        end else if (ss_busy_seen && prev_ss_busy && ~ss_busy) begin
          // Firmware finished producing the state.  Staging complete; announce
          // OK immediately so APF starts reading, and serve the staged SDRAM
          // through fifo_save concurrently.
          ss_busy_seen         <= 0;
          savestate_start_busy <= 0;
          savestate_start_ok   <= 1;
          save_serve_idx       <= 17'd0;
          save_serve_widx      <= 2'd0;
          serve_prefilled      <= 1'b1;
          save_serve_idle      <= 21'd0;
          sys_state            <= SYS_SAVE_SRV_RD_REQ;
        end
      end

      // ----- SAVE staging: write the latched chunk's 4 words to PSRAM -----
      // CRAM1 bank 1 (private to savestate), word-addressed; step by 1 per
      // word (no stride-by-2 like the SDRAM controller required).
      SYS_SAVE_WR_REQ: begin
        ss_psram_wr_req  <= ~ss_psram_wr_req;  // toggle to request a write
        ss_psram_wr_addr <= save_addr[18:0] + {17'b0, save_word_idx};
        case (save_word_idx)
          2'd0: ss_psram_wr_data <= save_buffer[15:0];
          2'd1: ss_psram_wr_data <= save_buffer[31:16];
          2'd2: ss_psram_wr_data <= save_buffer[47:32];
          2'd3: ss_psram_wr_data <= save_buffer[63:48];
        endcase
        sys_state <= SYS_SAVE_WR_WAIT;
      end

      SYS_SAVE_WR_WAIT: begin
        if (psram_wr_done) begin
          if (save_word_idx == 2'd3) begin
            // Chunk fully committed → release the firmware for the next one.
            ss_ack    <= ~ss_ack;
            sys_state <= SYS_SAVE_ACTIVE;
          end else begin
            save_word_idx <= save_word_idx + 2'd1;
            sys_state     <= SYS_SAVE_WR_REQ;
          end
        end
      end

      // ----- SAVE serve: read staged PSRAM (4 words/chunk) → fifo_save -----
      // 2026-06-22: SAVE serve moved off cart-ROM SDRAM onto CRAM1 bank 1.
      // Address = chunk_idx*4 + word_idx (PSRAM word-stride is 1, not 2).
      SYS_SAVE_SRV_RD_REQ: begin
        ss_psram_rd_req  <= ~ss_psram_rd_req;
        ss_psram_rd_addr <= {save_serve_idx[16:0], 2'b00} + {17'b0, save_serve_widx};
        sys_state <= SYS_SAVE_SRV_RD_WAIT;
      end

      SYS_SAVE_SRV_RD_WAIT: begin
        if (psram_rd_done) begin
          case (save_serve_widx)
            2'd0: save_serve_buf[15:0]  <= ss_psram_rd_data;
            2'd1: save_serve_buf[31:16] <= ss_psram_rd_data;
            2'd2: save_serve_buf[47:32] <= ss_psram_rd_data;
            2'd3: save_serve_buf[63:48] <= ss_psram_rd_data;
          endcase
          sys_state <= SYS_SAVE_SRV_RD_NEXT;
        end
      end

      SYS_SAVE_SRV_RD_NEXT: begin
        if (save_serve_widx == 2'd3) begin
          sys_state <= SYS_SAVE_SRV_PUSH;
        end else begin
          save_serve_widx <= save_serve_widx + 2'd1;
          sys_state       <= SYS_SAVE_SRV_RD_REQ;
        end
      end

      SYS_SAVE_SRV_PUSH: begin
        // Push the assembled 64-bit chunk into fifo_save when there's room,
        // then fetch the next chunk.  When the FIFO is FULL = APF backpressure:
        // if APF is still reading, hold (reset the watchdog) and wait for room;
        // if APF has STOPPED reading for SAVE_SERVE_IDLE_MAX, the read-back is
        // over → force completion so ss_loading/pause can never stick (the old
        // serve hung here forever on a full FIFO when APF stopped early).
        if (~fifo_save_wr_full) begin
          fifo_save_write_req <= 1;
          save_serve_widx     <= 2'd0;
          if (save_serve_idx == SAVE_SERVE_LAST) begin
            sys_state <= SYS_SAVE_SRV_DONE;
          end else begin
            save_serve_idx <= save_serve_idx + 17'd1;
            sys_state      <= SYS_SAVE_SRV_RD_REQ;
          end
        end else if (apf_reading) begin
          save_serve_idle <= 21'd0;            // APF still draining — wait.
        end else if (save_serve_idle >= SAVE_SERVE_IDLE_MAX) begin
          ss_loading <= 0;                     // APF stopped → done, no freeze.
          sys_state  <= SYS_IDLE;
        end else begin
          save_serve_idle <= save_serve_idle + 21'd1;
        end
      end

      SYS_SAVE_SRV_DONE: begin
        // All chunks pushed; once APF has drained the FIFO the save read is
        // complete → release the SDRAM mux and return to idle.  Guard with the
        // same APF-read-activity watchdog: if APF stops before the FIFO
        // reports empty, force completion rather than hang forever.
        if (fifo_save_rd_empty) begin
          ss_loading <= 0;
          sys_state  <= SYS_IDLE;
        end else if (apf_reading) begin
          save_serve_idle <= 21'd0;
        end else if (save_serve_idle >= SAVE_SERVE_IDLE_MAX) begin
          ss_loading <= 0;
          sys_state  <= SYS_IDLE;
        end else begin
          save_serve_idle <= save_serve_idle + 21'd1;
        end
      end

      // ============================================================
      // STAGING: assemble each 64-bit chunk from TWO 32-bit FIFO pops
      // (stage_half selects the destination half), then 4 SDRAM writes.
      // v61: plain 32-bit dcfifo — each pop is gated by fifo_drain_ok
      // individually, so a chunk's upper half can never be popped before
      // it has actually been written (the mixed-width half-pop bug).
      // ============================================================
      SYS_STAGE_FIFO_RD: begin
        // Require fifo_drain_ok: rdempty must have been low for several
        // cycles continuously.  This filters CDC-lag glitches where rdempty
        // briefly deasserts before the read-side has latched real data.
        if (fifo_drain_ok) begin
          fifo_load_read_req <= 1;
          sys_state          <= SYS_STAGE_FIFO_WAIT;
        end else begin
          // FIFO empty or not yet settled.  Park in STAGE_IDLE; we'll come
          // back when data has been present long enough to trust.  A
          // half-assembled chunk (stage_half=1) is preserved — stage_half
          // only resets after the chunk completes its 4 SDRAM writes.
          sys_state <= SYS_STAGE_IDLE;
        end
      end

      SYS_STAGE_FIFO_WAIT: begin
        // dcfifo with showahead=OFF: q is valid 2 cycles after rdreq.
        sys_state <= SYS_STAGE_FIFO_LATCH;
      end

      SYS_STAGE_FIFO_LATCH: begin
        if (!stage_half) begin
          // First 32-bit word of the chunk → lower half; go pop the second.
          stage_buffer[31:0] <= fifo_load_dout;
          stage_half         <= 1'b1;
          sys_state          <= SYS_STAGE_FIFO_RD;
          fifo_or_checksum[31:0] <= fifo_or_checksum[31:0] | fifo_load_dout;
        end else begin
          // Second word → upper half; chunk complete, start SDRAM writes.
          stage_buffer[63:32] <= fifo_load_dout;
          stage_half          <= 1'b0;
          stage_word_idx      <= 2'd0;
          sys_state           <= SYS_STAGE_WR_REQ;
          fifo_or_checksum[63:32] <= fifo_or_checksum[63:32] | fifo_load_dout;
          // v63: capture the RAW FIFO output feeding the upper half of the
          // FIRST chunk, in clk_sys, BEFORE it is stored into stage_buffer.
          // Compared on the overlay against dbg_w2_src (the readback of
          // stage_buffer[47:32]).  If this is good ($532D) but dbg_w2_src
          // is $0000, the stage_buffer register's upper half is the bug;
          // if this is also $0000, the FIFO delivered zero for the second
          // pop (FIFO-read / bridge-fill problem).
          if (!dbg_pop2_seen) begin
            dbg_pop2      <= fifo_load_dout[15:0];
            dbg_pop2_seen <= 1'b1;
          end
          // Per-chunk debug counters/captures (once per assembled chunk).
          if (cnt_stage_fifo_latch != 8'hFF) cnt_stage_fifo_latch <= cnt_stage_fifo_latch + 8'd1;
          if (fifo_drain_count != 16'hFFFF) fifo_drain_count <= fifo_drain_count + 16'd1;
          latest_stage_buf <= {fifo_load_dout, stage_buffer[31:0]};
          if (!first_buf_seen) begin
            first_buf_seen <= 1;
          end else if (!second_buf_seen) begin
            second_stage_buf <= {fifo_load_dout, stage_buffer[31:0]};
            second_buf_seen  <= 1;
          end
          if ({fifo_load_dout, stage_buffer[31:0]} != 64'h0 && cnt_nonzero_fifo_dout != 16'hFFFF)
            cnt_nonzero_fifo_dout <= cnt_nonzero_fifo_dout + 16'd1;
        end
        // Capture stage_buffer at chunk index 4 (= staging chunk 4 starts
        // when stage_entry_count is currently 4, about to become 5).
        // Compare against expected SMW .sta payload at chunk 4:
        //   bytes: 30 2E 30 2E 31 00 00 00  ("0.0.1\0\0\0")
        //   little-endian 64-bit: 64'h0000_0031_2E30_2E30
        // Capture at the 5th FIFO_LATCH (= chunk 4) using cnt_stage_fifo_latch
        // because stage_entry_count is unreliable in this build.
        if (stage_half && !sample_buf4_seen && cnt_stage_fifo_latch == 8'd4) begin
          sample_stage_buf4  <= {fifo_load_dout, stage_buffer[31:0]};
          sample_buf4_seen   <= 1;
        end
      end

      SYS_STAGE_WR_REQ: begin
        // v58: hold off until the wr_gap_cnt cooldown elapses (set in
        // WR_WAIT after previous word's ack).  Tests if rapid-fire
        // back-to-back writes are dropping words 2,3.
        if (wr_gap_cnt == 4'd0) begin
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
          // v62: capture the upper-half slice we feed for word 2, in
          // clk_sys, BEFORE any CDC.  Bisects controller-source vs CDC.
          if (stage_word_idx == 2'd2 && !dbg_w2_src_seen) begin
            dbg_w2_src      <= stage_buffer[47:32];
            dbg_w2_src_seen <= 1'b1;
          end
          sys_state <= SYS_STAGE_WR_WAIT;
        end
      end

      SYS_STAGE_WR_WAIT: begin
        if (sdram_wr_done) begin
          if (cnt_stage_wr_done != 8'hFF) cnt_stage_wr_done <= cnt_stage_wr_done + 8'd1;
          if (cnt_stage_wr_done_wide != 16'hFFFF) cnt_stage_wr_done_wide <= cnt_stage_wr_done_wide + 16'd1;
          // sdram.sv treats addr[24:1] as the 16-bit word and addr[0] as
          // byte-within-word, AND skips re-access when addr[24:1] is
          // unchanged.  So distinct 16-bit words must step addr by 2.
          stage_addr <= stage_addr + 25'd2;
          // v58 test: add a settle gap before next word's WR_REQ to see
          // if rapid-fire writes were dropping words 2,3.
          wr_gap_cnt <= 4'd8;
          if (stage_word_idx == 2'd3) begin
            if (stage_entry_count != 17'h1FFFF) begin
              stage_entry_count <= stage_entry_count + 17'd1;
              // Unconditionally update max — stage_entry_count is strictly
              // increasing within a single load (only reset at COMPLETE).
              stage_max_count <= stage_entry_count + 17'd1;
            end
            have_staged_any <= 1;
            sys_state <= SYS_STAGE_FIFO_RD;
          end else begin
            stage_word_idx <= stage_word_idx + 2'd1;
            sys_state      <= SYS_STAGE_WR_REQ;
          end
        end
      end

      SYS_STAGE_IDLE: begin
        if (prev_sys_state != SYS_STAGE_IDLE && cnt_stage_idle_enter != 8'hFF)
          cnt_stage_idle_enter <= cnt_stage_idle_enter + 8'd1;
        // Load FIFO drained.  Kick serve only when:
        //   1. APF signaled load command
        //   2. We've actually staged at least one chunk (stage_entry_count > 0)
        //   3. Bridge has been quiet long enough that we're sure APF stopped
        //      streaming (stage_quiet_cnt reached threshold)
        // Without (2), if savestate_load_s rises before any FIFO writes drain,
        // we'd skip staging entirely and serve from uninitialized SDRAM.
        // Without (3), a momentary FIFO drain mid-stream would kick serve
        // prematurely.
        if (fifo_drain_ok) begin
          sys_state <= SYS_STAGE_FIFO_RD;
        end else if (load_cmd_pending && (stage_entry_count != 17'd0)
                                       && (stage_quiet_cnt >= 20'h00400)) begin
          if (cnt_guard_pass != 8'hFF) cnt_guard_pass <= cnt_guard_pass + 8'd1;
          savestate_load_busy <= 1;
          // Run the SDRAM read-back probe first to capture ground truth
          // about what is actually staged.  After 4 probe reads, fall
          // through to SERVE_KICK_WAIT.
          if (!probe_done) begin
            probe_idx          <= 2'd0;
            stage_addr_at_done <= stage_addr;  // sticky snapshot
            sys_state          <= SYS_PROBE_REQ;
          end else begin
            kick_wait <= 8'd64;
            sys_state <= SYS_SERVE_KICK_WAIT;
          end
        end
      end

      // ------------------------------------------------------------
      // PROBE: read 4 known SDRAM locations directly (no firmware) to
      // verify staged data integrity.  Targets:
      //   probe 0: STAGING_BASE_WORD + 0       (chunk 0 word 0, want $4E53)
      //   probe 1: STAGING_BASE_WORD + 2       (chunk 0 word 1, want $5345)
      //   probe 2: STAGING_BASE_WORD + 16'h200 (chunk 64 word 0)
      //   probe 3: STAGING_BASE_WORD + 16'h2000(chunk 1024 word 0)
      // ------------------------------------------------------------
      SYS_PROBE_REQ: begin
        ss_sdram_rd_req  <= ~ss_sdram_rd_req;
        case (probe_idx)
          // v56b: probe ALL 4 SDRAM words of chunk 0.  If only word 0
          // ($800000) has data and words 1/2/3 (+2/+4/+6) are zero, that
          // proves each chunk's 4 word writes are landing at the SAME
          // SDRAM address (clk_mem-side captured addr is sticky).
          2'd0: ss_sdram_rd_addr <= STAGING_BASE_WORD + 25'h0000000;  // chunk 0 word 0 (want $53)
          2'd1: ss_sdram_rd_addr <= STAGING_BASE_WORD + 25'h0000002;  // chunk 0 word 1 (want $45 'E')
          2'd2: ss_sdram_rd_addr <= STAGING_BASE_WORD + 25'h0000004;  // chunk 0 word 2 (want $2D '-')
          2'd3: ss_sdram_rd_addr <= STAGING_BASE_WORD + 25'h0000006;  // chunk 0 word 3 (want $53 'S')
        endcase
        sys_state <= SYS_PROBE_WAIT;
      end

      SYS_PROBE_WAIT: begin
        if (sdram_rd_done) begin
          case (probe_idx)
            2'd0: probe_result_0 <= ss_sdram_rd_data;
            2'd1: probe_result_1 <= ss_sdram_rd_data;
            2'd2: probe_result_2 <= ss_sdram_rd_data;
            2'd3: probe_result_3 <= ss_sdram_rd_data;
          endcase
          sys_state <= SYS_PROBE_NEXT;
        end
      end

      SYS_PROBE_NEXT: begin
        if (probe_idx == 2'd3) begin
          probe_done <= 1;
          kick_wait  <= 8'd64;
          sys_state  <= SYS_SERVE_KICK_WAIT;
        end else begin
          probe_idx <= probe_idx + 2'd1;
          sys_state <= SYS_PROBE_REQ;
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
        // Now we know chunks ARE iterated sequentially.  Capture the
        // FIRST BYTE served at specific ACK indices to verify the data.
        // Expected (SMW .sta):
        //   ACK #2  (chunk  1): byte 0 = $00 (file +0x08, all zeros)
        //   ACK #5  (chunk  4): byte 0 = $30 ('0' from "0.0.1")
        //   ACK #9  (chunk  8): byte 0 = $53 ('S' from "Super")
        //   ACK #41 (chunk 40): byte 0 = $73 ('s' from "snes")
        if (ack_idx_count != 8'hFF) ack_idx_count <= ack_idx_count + 8'd1;
        if (!sample4_seen  && ack_idx_count == 8'd1)  begin sample_chunk4  <= serve_buffer[7:0]; sample4_seen  <= 1; end
        if (!sample8_seen  && ack_idx_count == 8'd4)  begin sample_chunk8  <= serve_buffer[7:0]; sample8_seen  <= 1; end
        if (!sample40_seen && ack_idx_count == 8'd8)  begin sample_chunk40 <= serve_buffer[7:0]; sample40_seen <= 1; end
        if (!sample64_seen && ack_idx_count == 8'd40) begin sample_chunk64 <= serve_buffer[7:0]; sample64_seen <= 1; end
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
        stage_entry_count   <= 17'h00000;
        stage_half          <= 1'b0;
      end

      default: sys_state <= SYS_IDLE;

    endcase
  end

endmodule
