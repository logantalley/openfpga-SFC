// Save State Stream — direct APF↔engine streaming through an SRAM ring.
//
// Successor to the SRAM-buffered controller (362ea8e, hardware-proven
// restore) and the PSRAM/SDRAM staging that replaced it.  The Pocket has no
// DDR (MiSTer keeps the blob in HPS DDR3), so the engine's ddr-style
// req/ack interface is serviced from the on-board async SRAM
// (131072 × 16 = 256 KB), used as a RING, not a store:
//
//   LOAD: APF bridge writes stream in at ~3.96 MB/s (one 32-bit word per
//         ~75 clk_74a cycles, no backpressure possible).  The load walk is
//         armed on the FIRST data word — not on the load command — so the
//         firmware consumes concurrently at the DMA rate (2.68 MB/s) and
//         ring occupancy peaks well under capacity.  Engine chunk reads are
//         held (ack deferred) until the stream has covered the chunk.
//   SAVE: engine chunk writes land in the ring; APF readout is released
//         (start_ok) once SAVE_OK_THRESHOLD words are buffered — early
//         enough to overlap, late enough that APF (faster reader) can
//         never catch the writer.  Walk-end also releases it (small saves).
//
// All ring addressing is unwrapped-count compared, 17-bit truncated for the
// SRAM — wraparound is free.  Occupancy correctness is by rate/threshold
// analysis (see SAVESTATE_FINDINGS §18) and checked in tb_ss_stream.
//
// No PSRAM, no SDRAM, no arbiters, no console pause: SRAM is single-master
// and otherwise unused.  The SRAM FSM and byte-lane ordering are carried
// verbatim from the proven 362ea8e implementation.

module save_state_stream #(
    // 16-bit SRAM words buffered before releasing APF save readout.
    // APF drains at ~3.96 MB/s while the engine still fills at ~2.68 MB/s;
    // the reader must not catch the writer before the walk ends:
    // B0 >= S * (1 - 2.68/3.96) ≈ 84 KB for S ≈ 260 KB.  Default 100 KB.
    parameter [17:0] SAVE_OK_THRESHOLD_WORDS = 18'd51200
) (
    input wire clk_74a,
    input wire clk_sys,

    // APF Bridge
    input wire bridge_wr,
    input wire bridge_rd,
    input wire bridge_endian_little,
    input wire [31:0] bridge_addr,
    input wire [31:0] bridge_wr_data,
    output reg [31:0] save_state_bridge_read_data,

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
    output reg ss_save = 0,
    output reg ss_load = 0,

    // Core-side ddr-style interface (toggle-based req/ack).  ss_ack MUST
    // power up 0: the engine gates its request FSM on ddr_req == ddr_ack,
    // and an X here poisons that compare forever (sim) / randomizes it (hw).
    input wire [63:0] ss_din,       // Data from engine (save)
    output reg [63:0] ss_dout = 0,  // Data to engine (load)
    input wire [16:0] ss_addr,      // Chunk index from savestates.sv
    input wire ss_rnw,              // 1 = read (load), 0 = write (save)
    input wire ss_req,              // Toggle request
    input wire [7:0] ss_be,         // Byte enable (engine drives 8'hFF)
    output reg ss_ack = 0,          // Toggle acknowledge

    input wire ss_busy,

    // SRAM (directly to Pocket board SRAM)
    output reg  [16:0] sram_a = 0,
    inout  wire [15:0] sram_dq,
    output reg         sram_oe_n = 1,
    output reg         sram_we_n = 1,
    output wire        sram_ub_n,
    output wire        sram_lb_n
);

  assign sram_ub_n = 1'b0;
  assign sram_lb_n = 1'b0;

  reg [15:0] sram_dq_out;
  reg        sram_dq_oe;  // 1 = drive (write), 0 = tristate (read)
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

  reg savestate_load_ack = 0;
  reg savestate_load_busy = 0;
  reg savestate_load_ok = 0;
  reg savestate_load_err = 0;

  reg savestate_start_ack = 0;
  reg savestate_start_busy = 0;
  reg savestate_start_ok = 0;
  reg savestate_start_err = 0;

  synch_3 #(.WIDTH(8)) savestate_out (
      {
        savestate_load_ack, savestate_load_busy, savestate_load_ok, savestate_load_err,
        savestate_start_ack, savestate_start_busy, savestate_start_ok, savestate_start_err
      },
      {
        savestate_load_ack_s, savestate_load_busy_s, savestate_load_ok_s, savestate_load_err_s,
        savestate_start_ack_s, savestate_start_busy_s, savestate_start_ok_s, savestate_start_err_s
      },
      clk_74a
  );

  // ===================================================================
  // Stream progress tracking (clk_74a domain, unwrapped counts)
  // ===================================================================

  // LOAD: 16-bit words received from the bridge (2 per 32-bit write).
  // 512 KB max stream = 2^18 words; 19 bits headroom.
  reg [18:0] load_words_in = 0;
  // A load stream has begun (first write to region 4 offset 0).  Cleared
  // when the load walk completes so a following save/load starts clean.
  reg        load_stream_active = 0;

  // SAVE: 16-bit words the engine has committed to the ring (4 per chunk).
  reg [18:0] save_words_in = 0;
  wire       save_threshold_hit = save_words_in >= {1'b0, SAVE_OK_THRESHOLD_WORDS};

  // Sync toward clk_sys
  wire load_stream_active_s;
  wire save_threshold_hit_s;
  synch_3 #(.WIDTH(2)) progress_sync (
      {load_stream_active, save_threshold_hit},
      {load_stream_active_s, save_threshold_hit_s},
      clk_sys
  );

  // Walk-completion flags back toward clk_74a
  reg  load_walk_done = 0;  // clk_sys
  wire load_walk_done_74a;
  synch_3 sync_walk_done (
      load_walk_done,
      load_walk_done_74a,
      clk_74a
  );

  // ===================================================================
  // CDC: engine ↔ SRAM toggle-based transfer (pattern from 362ea8e)
  // ===================================================================

  reg        core_wr_req_toggle = 0;
  reg [63:0] core_wr_data;
  reg [16:0] core_wr_chunk;  // full chunk index (unwrapped, for counting)

  reg        core_rd_req_toggle = 0;
  reg [16:0] core_rd_chunk;

  reg sram_wr_ack_toggle = 0;
  reg sram_rd_ack_toggle = 0;
  reg [63:0] sram_rd_result;

  wire core_wr_req_74a;
  wire core_rd_req_74a;
  wire sram_wr_ack_sys;
  wire sram_rd_ack_sys;

  synch_3 sync_wr_req (.i(core_wr_req_toggle), .o(core_wr_req_74a), .clk(clk_74a));
  synch_3 sync_rd_req (.i(core_rd_req_toggle), .o(core_rd_req_74a), .clk(clk_74a));
  synch_3 sync_wr_ack (.i(sram_wr_ack_toggle), .o(sram_wr_ack_sys), .clk(clk_sys));
  synch_3 sync_rd_ack (.i(sram_rd_ack_toggle), .o(sram_rd_ack_sys), .clk(clk_sys));

  // ===================================================================
  // Engine-side state machine (clk_sys)
  // ===================================================================

  localparam SYS_IDLE = 3'd0;
  localparam SYS_SAVE_ACTIVE = 3'd1;
  localparam SYS_SAVE_WAIT_SRAM = 3'd2;
  localparam SYS_LOAD_ACTIVE = 3'd3;
  localparam SYS_LOAD_WAIT_SRAM = 3'd4;

  reg [2:0] sys_state = SYS_IDLE;

  reg prev_savestate_start = 0;
  reg prev_savestate_load = 0;
  reg prev_load_stream = 0;
  reg prev_ss_busy = 0;
  reg prev_ss_req = 0;
  reg prev_sram_wr_ack = 0;
  reg prev_sram_rd_ack = 0;

  wire new_ddr_req = (ss_req != prev_ss_req);
  wire sram_wr_done = (sram_wr_ack_sys != prev_sram_wr_ack);
  wire sram_rd_done = (sram_rd_ack_sys != prev_sram_rd_ack);

  // The load command may arrive before or after the walk finishes (data
  // streams first, command last — Tamagotchi choreography).  Stickies.
  reg load_cmd_seen = 0;

  always @(posedge clk_sys) begin
    prev_savestate_start <= savestate_start_s;
    prev_savestate_load <= savestate_load_s;
    prev_load_stream <= load_stream_active_s;
    prev_ss_busy <= ss_busy;
    prev_ss_req <= ss_req;
    prev_sram_wr_ack <= sram_wr_ack_sys;
    prev_sram_rd_ack <= sram_rd_ack_sys;

    ss_save <= 0;
    ss_load <= 0;

    // ----- APF triggers save -----
    if (savestate_start_s && ~prev_savestate_start) begin
      sys_state <= SYS_SAVE_ACTIVE;
      savestate_start_ack <= 1;
      savestate_start_busy <= 1;
      savestate_start_ok <= 0;
      savestate_start_err <= 0;
      savestate_load_ok <= 0;
      savestate_load_err <= 0;
      load_cmd_seen <= 0;
      ss_save <= 1;
    end

    // ----- Load stream begins: arm the walk NOW (concurrent consumption).
    // The APF command arrives later and only drives the ack/ok choreography.
    if (load_stream_active_s && ~prev_load_stream && sys_state == SYS_IDLE) begin
      sys_state <= SYS_LOAD_ACTIVE;
      ss_load <= 1;
      load_walk_done <= 0;
      load_cmd_seen <= 0;
    end

    // ----- APF load command: bookkeeping only -----
    if (savestate_load_s && ~prev_savestate_load) begin
      load_cmd_seen <= 1;
      savestate_load_ack <= 1;
      savestate_load_busy <= 1;
      savestate_load_ok <= 0;
      savestate_load_err <= 0;
      savestate_start_ok <= 0;
      savestate_start_err <= 0;
    end
    if (savestate_load_ack && ~savestate_load_s) savestate_load_ack <= 0;

    // Load ok requires both: walk finished AND command received.
    if (load_walk_done && load_cmd_seen) begin
      savestate_load_busy <= 0;
      savestate_load_ok <= 1;
    end

    // Save ok gate: release APF readout at the ring threshold (overlap)
    // or at walk end, whichever first.  busy stays until walk end.
    if (sys_state == SYS_SAVE_ACTIVE || sys_state == SYS_SAVE_WAIT_SRAM) begin
      if (save_threshold_hit_s) savestate_start_ok <= 1;
    end

    case (sys_state)

      SYS_IDLE: ;  // transitions handled above

      // ===== Save path =====
      SYS_SAVE_ACTIVE: begin
        if (~savestate_start_s) savestate_start_ack <= 0;

        if (new_ddr_req && ~ss_rnw) begin
          core_wr_data <= ss_din;
          core_wr_chunk <= ss_addr;
          core_wr_req_toggle <= ~core_wr_req_toggle;
          sys_state <= SYS_SAVE_WAIT_SRAM;
        end else if (prev_ss_busy && ~ss_busy) begin
          sys_state <= SYS_IDLE;
          savestate_start_busy <= 0;
          savestate_start_ok <= 1;
        end
      end

      SYS_SAVE_WAIT_SRAM: begin
        if (sram_wr_done) begin
          ss_ack <= ~ss_ack;
          if (~ss_busy) begin
            sys_state <= SYS_IDLE;
            savestate_start_busy <= 0;
            savestate_start_ok <= 1;
          end else begin
            sys_state <= SYS_SAVE_ACTIVE;
          end
        end
      end

      // ===== Load path =====
      SYS_LOAD_ACTIVE: begin
        if (new_ddr_req && ss_rnw) begin
          core_rd_chunk <= ss_addr;
          core_rd_req_toggle <= ~core_rd_req_toggle;
          sys_state <= SYS_LOAD_WAIT_SRAM;
        end else if (prev_ss_busy && ~ss_busy) begin
          sys_state <= SYS_IDLE;
          load_walk_done <= 1;
        end
      end

      SYS_LOAD_WAIT_SRAM: begin
        // The SRAM side holds this request until the stream has covered the
        // chunk, so a deferred ack stalls the engine (and, through
        // load_buf/STATUS, the firmware) exactly when data isn't there yet.
        //
        // Walk end with the read still outstanding: the engine ALWAYS
        // prefetches one chunk past the last one consumed, and past the
        // stream end that read can never be served.  Dummy-ack it (load_en
        // is already 0; the data is never used) so the engine's req==ack
        // gate isn't wedged for the next save/load, and let the walk-done
        // edge cancel the SRAM-side pending.
        if (prev_ss_busy && ~ss_busy) begin
          ss_ack <= ~ss_ack;
          sys_state <= SYS_IDLE;
          load_walk_done <= 1;
        end else if (sram_rd_done) begin
          // Byte order carried verbatim from the proven 362ea8e path.
          ss_dout <= {
            sram_rd_result[39:32],
            sram_rd_result[47:40],
            sram_rd_result[55:48],
            sram_rd_result[63:56],
            sram_rd_result[7:0],
            sram_rd_result[15:8],
            sram_rd_result[23:16],
            sram_rd_result[31:24]
          };
          ss_ack <= ~ss_ack;
          if (~ss_busy) begin
            sys_state <= SYS_IDLE;
            load_walk_done <= 1;
          end else begin
            sys_state <= SYS_LOAD_ACTIVE;
          end
        end
      end

      default: sys_state <= SYS_IDLE;

    endcase
  end

  // ===================================================================
  // SRAM FSM (clk_74a) — access pattern verbatim from 362ea8e
  // ===================================================================

  localparam SRAM_IDLE = 4'd0;
  localparam SRAM_CORE_WR = 4'd1;
  localparam SRAM_CORE_WR_END = 4'd2;
  localparam SRAM_CORE_RD_SETUP = 4'd3;
  localparam SRAM_CORE_RD_N = 4'd4;
  localparam SRAM_BRIDGE_WR_LO = 4'd5;
  localparam SRAM_BRIDGE_WR_GAP = 4'd6;
  localparam SRAM_BRIDGE_WR_HI = 4'd7;
  localparam SRAM_BRIDGE_WR_END = 4'd8;
  localparam SRAM_PF_SETUP = 4'd9;
  localparam SRAM_PF_LO = 4'd10;
  localparam SRAM_PF_HI = 4'd11;

  reg [3:0] sram_state = SRAM_IDLE;
  reg [1:0] sram_word_idx;
  reg [16:0] sram_base_addr;

  // STICKY pendings.  The 362ea8e edge-pulse pattern was safe only because
  // its phases were serialized (the FSM was always idle when a request
  // toggled).  Here bridge writes stream CONCURRENTLY with engine reads, so
  // a 1-cycle pulse landing mid bridge-write service would be lost forever
  // (found in tb_ss_stream).  Set on toggle edge, cleared at service end.
  reg prev_core_wr_req_74a = 0;
  reg prev_core_rd_req_74a = 0;
  reg core_wr_pending_74a = 0;
  reg core_rd_pending_74a = 0;

  reg [63:0] latched_core_wr_data;

  // Byte-swapped 16-bit words for engine save chunks (proven ordering).
  wire [15:0] core_wr_word_1 = {latched_core_wr_data[7:0], latched_core_wr_data[15:8]};
  wire [15:0] core_wr_word_2 = {latched_core_wr_data[55:48], latched_core_wr_data[63:56]};
  wire [15:0] core_wr_word_3 = {latched_core_wr_data[39:32], latched_core_wr_data[47:40]};

  // Bridge write latch
  reg bridge_wr_pending = 0;
  reg [16:0] bridge_wr_sram_addr;  // 17-bit truncation = ring wrap
  reg [31:0] bridge_wr_latched;

  // LOAD availability: engine chunk N needs words (N*4 + 4) received.
  // Compare on unwrapped counts; +1 chunk of margin is NOT taken — the
  // engine's own load_buf prefetch provides the pipeline slack.
  wire [18:0] rd_words_needed = {core_rd_chunk, 2'b00} + 19'd4;
  wire        rd_available = load_words_in >= rd_words_needed;

  // Save-read prefetch for APF readout
  reg [16:0] prefetch_sram_addr;
  reg prefetch_pending = 0;
  reg [15:0] prefetch_lo;

  reg prev_start_ok_74a = 0;
  wire start_ok_74a = savestate_start_ok_s;

  reg prev_load_walk_done_74a = 0;
  reg prev_bridge_rd_74a = 0;  // edge-detect: one prefetch per read, not one per cycle held high

  always @(posedge clk_74a) begin
    prev_core_wr_req_74a <= core_wr_req_74a;
    prev_core_rd_req_74a <= core_rd_req_74a;
    if (core_wr_req_74a != prev_core_wr_req_74a) core_wr_pending_74a <= 1;
    if (core_rd_req_74a != prev_core_rd_req_74a) core_rd_pending_74a <= 1;
    prev_start_ok_74a <= start_ok_74a;
    prev_load_walk_done_74a <= load_walk_done_74a;
    prev_bridge_rd_74a <= bridge_rd;

    // Latch bridge writes (1-cycle pulse).  Region 4 = savestate data.
    if (bridge_wr && bridge_addr[31:28] == 4'h4 && !bridge_wr_pending) begin
      bridge_wr_pending <= 1;
      bridge_wr_sram_addr <= {bridge_addr[17:2], 1'b0};
      bridge_wr_latched <= bridge_wr_data;

      if (bridge_addr[27:0] == 28'h0) begin
        // Stream (re)start: reset progress, arm the walk via CDC level.
        load_words_in <= 0;
        load_stream_active <= 1;
      end
    end

    // Stream bookkeeping resets once the walk has finished.  Edge-triggered:
    // a stale done LEVEL from the previous load must not kill a stream that
    // just re-armed (the clk_sys side clears done a few cycles after arming).
    // Also cancels the engine's overhanging final prefetch (dummy-acked on
    // the clk_sys side) so it is never served against a stale chunk index.
    if (load_walk_done_74a && ~prev_load_walk_done_74a) begin
      load_stream_active <= 0;
      core_rd_pending_74a <= 0;
    end

    // Save progress reset on a new save command (ack rising via sync).
    if (savestate_start_ack_s) save_words_in <= 0;

    // Initial APF prefetch when save readout is released.
    if (start_ok_74a && ~prev_start_ok_74a) begin
      prefetch_sram_addr <= 17'd0;
      prefetch_pending <= 1;
    end

    // Next prefetch after each APF read — rising edge only so a multi-cycle
    // bridge_rd doesn't fire multiple prefetch advances for the same word.
    if (bridge_rd && ~prev_bridge_rd_74a && bridge_addr[31:28] == 4'h4) begin
      prefetch_sram_addr <= prefetch_sram_addr + 17'd2;
      prefetch_pending <= 1;
    end

    case (sram_state)

      SRAM_IDLE: begin
        sram_oe_n  <= 1;
        sram_we_n  <= 1;
        sram_dq_oe <= 0;

        // Priority: engine write > bridge write > engine read (when
        // available) > prefetch.  Bridge writes outrank engine reads so an
        // unavailable read can never starve the stream that unblocks it.
        if (core_wr_pending_74a) begin
          latched_core_wr_data <= core_wr_data;
          sram_base_addr <= {core_wr_chunk[14:0], 2'b00};
          sram_word_idx <= 2'd0;
          sram_a <= {core_wr_chunk[14:0], 2'b00};
          sram_dq_out <= {core_wr_data[23:16], core_wr_data[31:24]};  // word 0
          sram_dq_oe <= 1;
          sram_we_n <= 0;
          sram_state <= SRAM_CORE_WR;
        end else if (bridge_wr_pending) begin
          sram_a <= bridge_wr_sram_addr;
          sram_dq_out <= bridge_wr_latched[15:0];
          sram_dq_oe <= 1;
          sram_we_n <= 0;
          sram_state <= SRAM_BRIDGE_WR_LO;
        end else if (core_rd_pending_74a && rd_available) begin
          sram_base_addr <= {core_rd_chunk[14:0], 2'b00};
          sram_word_idx <= 2'd0;
          sram_a <= {core_rd_chunk[14:0], 2'b00};
          sram_oe_n <= 0;
          sram_dq_oe <= 0;
          sram_state <= SRAM_CORE_RD_SETUP;
        end else if (prefetch_pending) begin
          sram_a <= prefetch_sram_addr;
          sram_oe_n <= 0;
          sram_dq_oe <= 0;
          prefetch_pending <= 0;  // consumed here; bridge_rd re-arms for next word
          sram_state <= SRAM_PF_SETUP;
        end
      end

      // ----- Engine save chunk: 4 × 16-bit writes -----
      SRAM_CORE_WR: begin
        sram_we_n  <= 1;  // rising edge commits the write
        sram_state <= SRAM_CORE_WR_END;
      end

      SRAM_CORE_WR_END: begin
        if (sram_word_idx == 2'd3) begin
          sram_dq_oe <= 0;
          sram_wr_ack_toggle <= ~sram_wr_ack_toggle;
          core_wr_pending_74a <= 0;  // consumed
          save_words_in <= save_words_in + 19'd4;
          sram_state <= SRAM_IDLE;
        end else begin
          sram_word_idx <= sram_word_idx + 2'd1;
          sram_a <= sram_base_addr + {15'd0, sram_word_idx} + 17'd1;
          case (sram_word_idx)
            2'd0: sram_dq_out <= core_wr_word_1;
            2'd1: sram_dq_out <= core_wr_word_2;
            2'd2: sram_dq_out <= core_wr_word_3;
            default: sram_dq_out <= 16'd0;
          endcase
          sram_we_n <= 0;
          sram_state <= SRAM_CORE_WR;
        end
      end

      // ----- Engine load chunk: 1 setup + 4 captures -----
      SRAM_CORE_RD_SETUP: begin
        sram_state <= SRAM_CORE_RD_N;
      end

      SRAM_CORE_RD_N: begin
        case (sram_word_idx)
          2'd0: sram_rd_result[15:0] <= sram_dq_in;
          2'd1: sram_rd_result[31:16] <= sram_dq_in;
          2'd2: sram_rd_result[47:32] <= sram_dq_in;
          2'd3: sram_rd_result[63:48] <= sram_dq_in;
        endcase

        if (sram_word_idx == 2'd3) begin
          sram_oe_n <= 1;
          sram_rd_ack_toggle <= ~sram_rd_ack_toggle;
          core_rd_pending_74a <= 0;  // consumed
          sram_state <= SRAM_IDLE;
        end else begin
          sram_word_idx <= sram_word_idx + 2'd1;
          sram_a <= sram_base_addr + {15'd0, sram_word_idx} + 17'd1;
        end
      end

      // ----- Bridge load write: 2 × 16-bit writes -----
      SRAM_BRIDGE_WR_LO: begin
        sram_we_n  <= 1;
        sram_state <= SRAM_BRIDGE_WR_GAP;
      end

      SRAM_BRIDGE_WR_GAP: begin
        sram_a <= bridge_wr_sram_addr + 17'd1;
        sram_dq_out <= bridge_wr_latched[31:16];
        sram_we_n <= 0;
        sram_state <= SRAM_BRIDGE_WR_HI;
      end

      SRAM_BRIDGE_WR_HI: begin
        sram_we_n  <= 1;
        sram_state <= SRAM_BRIDGE_WR_END;
      end

      SRAM_BRIDGE_WR_END: begin
        sram_dq_oe <= 0;
        bridge_wr_pending <= 0;
        load_words_in <= load_words_in + 19'd2;
        sram_state <= SRAM_IDLE;
      end

      // ----- APF save-read prefetch: 2 × 16-bit reads -----
      SRAM_PF_SETUP: begin
        sram_state <= SRAM_PF_LO;
      end

      SRAM_PF_LO: begin
        prefetch_lo <= sram_dq_in;
        sram_a <= sram_a + 17'd1;  // sram_a holds LO addr from SRAM_IDLE; prefetch_sram_addr
                                   // may have advanced (bridge_rd mid-flight) so don't re-use it
        sram_state <= SRAM_PF_HI;
      end

      SRAM_PF_HI: begin
        save_state_bridge_read_data <= {sram_dq_in, prefetch_lo};
        // Do NOT clear prefetch_pending here — if bridge_rd re-armed it while this
        // prefetch was in flight (the bridge_rd trigger fires during PF_SETUP/LO/HI),
        // SRAM_IDLE will pick it up immediately on the next cycle and start the
        // next prefetch.  Clearing here would lose that queued trigger.
        sram_oe_n <= 1;
        sram_state <= SRAM_IDLE;
      end

      default: sram_state <= SRAM_IDLE;

    endcase
  end

endmodule
