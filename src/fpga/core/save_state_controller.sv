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
//         On savestate_load, trigger ss_load.
//         Core reads → toggle CDC → 4×16 SRAM reads → ack + data.
//         After ss_busy falls (load complete), signal load ok.
//
// No FIFOs — the SRAM is the sole buffer.
//
// Reference: mincer-ray/openfpga-GBA uses SDRAM staging for loads (state
//            ~389 KB > 256 KB SRAM); our SNES state ≈ 200 KB fits in SRAM.

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
    output reg ss_ack,            // Toggle acknowledge to savestates.sv

    input wire ss_busy,

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
        savestate_start_ok_s,  savestate_start_err_s
      },
      clk_74a
  );

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

  reg prev_savestate_start = 0;
  reg prev_savestate_load  = 0;
  reg prev_ss_busy         = 0;
  reg prev_ss_req          = 0;
  reg prev_sram_wr_ack     = 0;
  reg prev_sram_rd_ack     = 0;

  wire new_ddr_req = (ss_req != prev_ss_req);
  wire sram_wr_done = (sram_wr_ack_sys != prev_sram_wr_ack);
  wire sram_rd_done = (sram_rd_ack_sys != prev_sram_rd_ack);

  // Load initiation flag — set when savestate_load command is received
  reg load_cmd_pending = 0;

  always @(posedge clk_sys) begin
    prev_savestate_start <= savestate_start_s;
    prev_savestate_load  <= savestate_load_s;
    prev_ss_busy         <= ss_busy;
    prev_ss_req          <= ss_req;
    prev_sram_wr_ack     <= sram_wr_ack_sys;
    prev_sram_rd_ack     <= sram_rd_ack_sys;

    ss_save <= 0;
    ss_load <= 0;

    // ----- APF triggers save -----
    if (savestate_start_s && ~prev_savestate_start) begin
      sys_state            <= SYS_SAVE_ACTIVE;
      savestate_start_ack  <= 1;
      savestate_start_busy <= 1;
      savestate_start_ok   <= 0;
      savestate_start_err  <= 0;
      savestate_load_ok    <= 0;
      savestate_load_err   <= 0;
      ss_save              <= 1;
    end

    // ----- APF signals load command (data already in SRAM) -----
    if (savestate_load_s && ~prev_savestate_load) begin
      load_cmd_pending     <= 1;
      savestate_load_ack   <= 1;
      savestate_load_ok    <= 0;
      savestate_load_err   <= 0;
      savestate_start_ok   <= 0;
      savestate_start_err  <= 0;
    end

    case (sys_state)

      SYS_IDLE: begin
        // Start load sequence when APF load command is received
        if (load_cmd_pending) begin
          sys_state            <= SYS_LOAD_ACTIVE;
          ss_load              <= 1;
          load_cmd_pending     <= 0;
          savestate_load_ack   <= 0;
          savestate_load_busy  <= 1;
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
        end else if (prev_ss_busy && ~ss_busy) begin
          // Save complete with no pending data
          sys_state            <= SYS_IDLE;
          savestate_start_busy <= 0;
          savestate_start_ok   <= 1;
        end
      end

      SYS_SAVE_WAIT_SRAM: begin
        if (sram_wr_done) begin
          // SRAM write complete — toggle ack back to savestates.sv
          ss_ack <= ~ss_ack;
          if (~ss_busy) begin
            // Save finished while writing last word
            sys_state            <= SYS_IDLE;
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
        end else if (prev_ss_busy && ~ss_busy) begin
          // Load complete
          sys_state           <= SYS_IDLE;
          savestate_load_busy <= 0;
          savestate_load_ok   <= 1;
        end
      end

      SYS_LOAD_WAIT_SRAM: begin
        if (sram_rd_done) begin
          // SRAM read data is available (stable before ack toggle crossed)
          // Apply byte swap: reverse bytes within each 32-bit half
          // Same transform as the original FIFO-based ss_dout path
          ss_dout <= {
            sram_rd_result[39:32], sram_rd_result[47:40],
            sram_rd_result[55:48], sram_rd_result[63:56],
            sram_rd_result[7:0],   sram_rd_result[15:8],
            sram_rd_result[23:16], sram_rd_result[31:24]
          };
          ss_ack <= ~ss_ack;
          if (~ss_busy) begin
            sys_state           <= SYS_IDLE;
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
  localparam SRAM_CORE_WR       = 4'd1;  // WE_n asserted (low)
  localparam SRAM_CORE_WR_END   = 4'd2;  // WE_n deasserted (high) → completes write
  localparam SRAM_CORE_RD_SETUP = 4'd3;
  localparam SRAM_CORE_RD_N     = 4'd4;
  localparam SRAM_BRIDGE_WR_LO  = 4'd5;  // WE_n asserted for low half
  localparam SRAM_BRIDGE_WR_GAP = 4'd6;  // WE_n high between halves
  localparam SRAM_BRIDGE_WR_HI  = 4'd7;  // WE_n asserted for high half
  localparam SRAM_BRIDGE_WR_END = 4'd8;  // WE_n high, done
  localparam SRAM_PF_SETUP      = 4'd9;
  localparam SRAM_PF_LO         = 4'd10;
  localparam SRAM_PF_GAP        = 4'd11; // 1-cycle address setup between low/high reads
  localparam SRAM_PF_HI         = 4'd12;

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
  // Bridge expects byte_swap_32(ss_din[31:0]) for first 32-bit read,
  //                byte_swap_32(ss_din[63:32]) for second.
  // Each 32-bit half is split into two 16-bit SRAM words.
  wire [15:0] core_wr_word_0 = {latched_core_wr_data[23:16], latched_core_wr_data[31:24]};
  wire [15:0] core_wr_word_1 = {latched_core_wr_data[7:0],   latched_core_wr_data[15:8]};
  wire [15:0] core_wr_word_2 = {latched_core_wr_data[55:48], latched_core_wr_data[63:56]};
  wire [15:0] core_wr_word_3 = {latched_core_wr_data[39:32], latched_core_wr_data[47:40]};

  // Bridge write pending latch (clk_74a domain)
  reg        bridge_wr_pending = 0;
  reg [16:0] bridge_wr_sram_addr;
  reg [31:0] bridge_wr_latched;

  // Pre-fetch state for bridge save reads
  reg [16:0] prefetch_sram_addr;  // Next SRAM word address to pre-fetch
  reg        prefetch_pending = 0;
  reg [15:0] prefetch_lo;         // Low 16 bits of current pre-fetch

  // Track when save ok is asserted (for initial pre-fetch trigger)
  reg prev_start_ok_74a = 0;
  wire start_ok_74a = savestate_start_ok_s;

  always @(posedge clk_74a) begin
    // NOTE: prev_core_wr_req_74a and prev_core_rd_req_74a are NOT updated
    // unconditionally here. They are only consumed in the SRAM FSM completion
    // states (SRAM_CORE_WR_END, SRAM_CORE_RD_N) so that pending signals
    // persist until the FSM processes them. This prevents lost toggles if the
    // FSM happens to be busy when the CDC toggle arrives.
    prev_start_ok_74a    <= start_ok_74a;

    // Latch bridge writes to SRAM (bridge_wr is 1 clk_74a pulse)
    if (bridge_wr && bridge_addr[31:28] == 4'h4 && !bridge_wr_pending) begin
      bridge_wr_pending   <= 1;
      bridge_wr_sram_addr <= {bridge_addr[17:2], 1'b0};
      bridge_wr_latched   <= bridge_wr_data;
    end

    // Trigger initial pre-fetch on save ok rising edge
    if (start_ok_74a && ~prev_start_ok_74a) begin
      prefetch_sram_addr <= 17'd0;
      prefetch_pending   <= 1;
    end

    // Trigger next pre-fetch after each bridge read
    if (bridge_rd && bridge_addr[31:28] == 4'h4) begin
      prefetch_sram_addr <= prefetch_sram_addr + 17'd2;
      prefetch_pending   <= 1;
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
          // Set up first write: address + data, assert WE_n
          sram_a       <= {core_sram_base, 2'b00};
          // Use core_wr_data directly (not core_wr_word_0) because
          // latched_core_wr_data is non-blocking and not yet updated this cycle
          sram_dq_out  <= {core_wr_data[23:16], core_wr_data[31:24]}; // word 0
          sram_dq_oe   <= 1;
          sram_we_n    <= 0;
          sram_state   <= SRAM_CORE_WR;
        end else if (core_rd_pending_74a) begin
          sram_base_addr <= {core_rd_base, 2'b00};
          sram_word_idx  <= 2'd0;
          // Set up first read address
          sram_a     <= {core_rd_base, 2'b00};
          sram_oe_n  <= 0;
          sram_dq_oe <= 0;
          sram_state <= SRAM_CORE_RD_SETUP;
        end else if (bridge_wr_pending) begin
          // Set up low-half write: address + data, assert WE_n
          sram_a       <= bridge_wr_sram_addr;
          sram_dq_out  <= bridge_wr_latched[15:0];
          sram_dq_oe   <= 1;
          sram_we_n    <= 0;
          sram_state   <= SRAM_BRIDGE_WR_LO;
        end else if (prefetch_pending) begin
          sram_a     <= prefetch_sram_addr;
          sram_oe_n  <= 0;
          sram_dq_oe <= 0;
          sram_state <= SRAM_PF_SETUP;
        end
      end

      // ----- Core save write: 4 × 16-bit SRAM writes -----
      // Each write: WE_n=0 (SRAM_CORE_WR) → WE_n=1 (SRAM_CORE_WR_END)
      SRAM_CORE_WR: begin
        // WE_n is low — write is active. Deassert to complete it.
        sram_we_n <= 1;  // Rising edge completes the write
        sram_state <= SRAM_CORE_WR_END;
      end

      SRAM_CORE_WR_END: begin
        // Write completed. Advance to next word or finish.
        if (sram_word_idx == 2'd3) begin
          // All 4 words written — done
          sram_dq_oe         <= 0;
          sram_wr_ack_toggle <= ~sram_wr_ack_toggle;
          prev_core_wr_req_74a <= core_wr_req_74a;  // consume pending
          sram_state         <= SRAM_IDLE;
        end else begin
          // Set up next word write
          sram_word_idx <= sram_word_idx + 2'd1;
          sram_a <= sram_base_addr + {15'd0, sram_word_idx} + 17'd1;
          case (sram_word_idx)
            2'd0: sram_dq_out <= core_wr_word_1;
            2'd1: sram_dq_out <= core_wr_word_2;
            2'd2: sram_dq_out <= core_wr_word_3;
            default: sram_dq_out <= 16'd0;
          endcase
          sram_we_n  <= 0;  // Assert WE_n for next write
          sram_state <= SRAM_CORE_WR;
        end
      end

      // ----- Core load read: 1 setup + 4 captures -----
      SRAM_CORE_RD_SETUP: begin
        // 1-cycle address setup delay for SRAM access time
        sram_state <= SRAM_CORE_RD_N;
      end

      SRAM_CORE_RD_N: begin
        // Capture current SRAM word
        case (sram_word_idx)
          2'd0: sram_rd_result[15:0]  <= sram_dq_in;
          2'd1: sram_rd_result[31:16] <= sram_dq_in;
          2'd2: sram_rd_result[47:32] <= sram_dq_in;
          2'd3: sram_rd_result[63:48] <= sram_dq_in;
        endcase

        if (sram_word_idx == 2'd3) begin
          // All 4 words read — done
          sram_oe_n          <= 1;
          sram_rd_ack_toggle <= ~sram_rd_ack_toggle;
          prev_core_rd_req_74a <= core_rd_req_74a;  // consume pending
          sram_state         <= SRAM_IDLE;
        end else begin
          // Advance to next SRAM address — go through RD_SETUP for
          // address setup time (async SRAM needs ~15ns tCO+tAA > 13.47ns period)
          sram_word_idx <= sram_word_idx + 2'd1;
          sram_a <= sram_base_addr + {15'd0, sram_word_idx} + 17'd1;
          sram_state <= SRAM_CORE_RD_SETUP;
        end
      end

      // ----- Bridge load write: 2 × 16-bit SRAM writes -----
      SRAM_BRIDGE_WR_LO: begin
        // WE_n was asserted — deassert to complete low-half write
        sram_we_n <= 1;
        sram_state <= SRAM_BRIDGE_WR_GAP;
      end

      SRAM_BRIDGE_WR_GAP: begin
        // Set up high-half write: new address + data, assert WE_n
        sram_a      <= bridge_wr_sram_addr + 17'd1;
        sram_dq_out <= bridge_wr_latched[31:16];
        sram_we_n   <= 0;
        sram_state  <= SRAM_BRIDGE_WR_HI;
      end

      SRAM_BRIDGE_WR_HI: begin
        // WE_n was asserted — deassert to complete high-half write
        sram_we_n  <= 1;
        sram_state <= SRAM_BRIDGE_WR_END;
      end

      SRAM_BRIDGE_WR_END: begin
        // Both halves written — done
        sram_dq_oe        <= 0;
        bridge_wr_pending <= 0;
        sram_state        <= SRAM_IDLE;
      end

      // ----- Bridge save-read pre-fetch: 2 × 16-bit SRAM reads -----
      SRAM_PF_SETUP: begin
        // Address driven in IDLE; SRAM needs 1 cycle for data to appear
        sram_state <= SRAM_PF_LO;
      end

      SRAM_PF_LO: begin
        // Capture low 16 bits, set up high address
        prefetch_lo <= sram_dq_in;
        sram_a      <= prefetch_sram_addr + 17'd1;
        sram_state  <= SRAM_PF_GAP;
      end

      SRAM_PF_GAP: begin
        // 1-cycle address setup delay for SRAM access time
        // (async SRAM needs ~15ns tCO+tAA > 13.47ns clk_74a period)
        sram_state <= SRAM_PF_HI;
      end

      SRAM_PF_HI: begin
        // Capture high 16 bits, combine into 32-bit result
        save_state_bridge_read_data <= {sram_dq_in, prefetch_lo};
        prefetch_pending            <= 0;
        sram_oe_n                   <= 1;
        sram_state                  <= SRAM_IDLE;
      end

    endcase
  end

endmodule
