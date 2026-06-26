// tb_ss_staging — focused, self-checking testbench for the Phase C
// savestate LOAD staging path:
//
//   APF bridge_wr stream → save_state_controller (32→64 dcfifo_mixed_widths
//   + staging FSM, clk_sys) → ss_sdram_arbiter (toggle CDC + clk_mem FSM)
//   → sdram.sv (MiSTer controller) → behavioral MT48LC16M16 chip model.
//
// Reproduces (or exonerates) the hardware bug where, for every 64-bit
// chunk, SDRAM words 0/1 land correctly but words 2/3 read back $0000.
//
// No SNES core is elaborated — staging runs with the CPU paused, so the
// CPU/PPU are irrelevant to this path.
//
// Run headless:  vsim -c -do simulate_staging.do   (from src/fpga/)

`timescale 1ps / 1ps

// ===========================================================================
// Behavioral SDRAM chip model (MT48LC16M16-ish, single-word accesses only)
//
// Decodes the exact command sequence sdram.sv emits:
//   STATE_START: CMD_ACTIVE, A=row (addr[13:1]), BA=addr[24:23]
//   STATE_CONT : CMD_WRITE (data on DQ, DQM=A[12:11], auto-precharge A[10])
//                or CMD_READ (CL=2)
//
// Linear 16-bit-word index reconstructed as {BA, col[8:0], row[12:0]} which
// equals byte_addr[24:1] under sdram.sv's mapping — so an addressing bug in
// sdram.sv shows up as data landing at the wrong reconstructed index.
//
// Commands are sampled on negedge clk (the chip's clock is inverted vs
// clk_mem via altddio_out, so the chip's rising edge = FPGA falling edge).
// ===========================================================================
module sdram_chip_model (
    input  wire        clk,        // clk_mem (FPGA side)
    inout  wire [15:0] dq,
    input  wire [12:0] a,
    input  wire [1:0]  ba,
    input  wire        ncs,
    input  wire        nras,
    input  wire        ncas,
    input  wire        nwe
);

  localparam CMD_ACTIVE  = 3'b011;
  localparam CMD_READ    = 3'b101;
  localparam CMD_WRITE   = 3'b100;
  localparam CMD_REFRESH = 3'b001;
  localparam CMD_LOADMOD = 3'b000;
  localparam CMD_PRECHRG = 3'b010;

  // Sparse memory + write-tracking, keyed by linear 16-bit-word index.
  logic [15:0] mem     [int unsigned];
  bit          written [int unsigned];

  reg  [12:0] row_addr [0:3];
  reg         row_open [0:3];

  // Instrumentation: per-word-class write counters relative to the staging
  // base (byte addr 25'h800000 → linear word index 24'h400000).  Class =
  // word index within a 64-bit chunk (0..3).  The non-saturating version of
  // the hardware's dbg_real_writes evidence gap.
  localparam int unsigned SS_LIN_BASE = 24'h40_0000;
  int wr_count_by_class [0:3];
  int total_writes  = 0;
  int total_reads   = 0;
  int log_remaining = 24;     // $display the first N writes

  // Read pipeline: drive dq for 3 chip-clock periods starting just after
  // the READ command edge — generously covers sdram.sv's last_data capture
  // at STATE_READY (CL=2).  Reads are widely spaced (toggle handshake), so
  // the window never collides with a following access.
  reg [2:0]  rd_sh  = 3'b000;
  reg [15:0] rd_val = 16'h0000;
  assign dq = (|rd_sh) ? rd_val : 16'hzzzz;

  wire [2:0] cmd = {nras, ncas, nwe};

  function automatic int unsigned lin_index(input [1:0] bank, input [8:0] col,
                                            input [12:0] row);
    lin_index = {bank, col, row};
  endfunction

  initial begin
    row_open[0] = 0; row_open[1] = 0; row_open[2] = 0; row_open[3] = 0;
  end

  always @(negedge clk) begin
    int unsigned lin;
    rd_sh <= {rd_sh[1:0], 1'b0};

    if (~ncs) begin
      case (cmd)
        CMD_ACTIVE: begin
          row_addr[ba] <= a[12:0];
          row_open[ba] <= 1'b1;
        end

        CMD_WRITE: begin
          if (!row_open[ba])
            $error("[chip] WRITE to bank %0d with no open row at %0t", ba, $time);
          lin = lin_index(ba, a[8:0], row_addr[ba]);
          if (!mem.exists(lin)) mem[lin] = 16'h0000;
          if (!a[12]) mem[lin][15:8] = dq[15:8];  // DQMH
          if (!a[11]) mem[lin][7:0]  = dq[7:0];   // DQML
          written[lin] = 1'b1;
          total_writes++;
          if (lin >= SS_LIN_BASE && lin < SS_LIN_BASE + 24'h10_0000)
            wr_count_by_class[lin[1:0]]++;
          if (log_remaining > 0) begin
            $display("[chip] WR lin=%h (class %0d) data=%h dqm=%b t=%0t",
                     lin, lin[1:0], dq, a[12:11], $time);
            log_remaining--;
          end
          if (a[10]) row_open[ba] <= 1'b0;  // auto-precharge
        end

        CMD_READ: begin
          if (!row_open[ba])
            $error("[chip] READ from bank %0d with no open row at %0t", ba, $time);
          lin    = lin_index(ba, a[8:0], row_addr[ba]);
          rd_val <= mem.exists(lin) ? mem[lin] : 16'hDEAD;
          rd_sh  <= 3'b001;
          total_reads++;
          if (a[10]) row_open[ba] <= 1'b0;
        end

        default: ;  // NOP / refresh / precharge / load-mode — ignored
      endcase
    end
  end

endmodule


// ===========================================================================
// Top-level testbench
// ===========================================================================
module tb_ss_staging #(
    // Overridable from the vsim command line via -g for stress sweeps.
    parameter int NUM_BRIDGE_WORDS = 512,       // 256 chunks = 2 KB staged
    parameter int BRIDGE_GAP_CYC   = 60,        // clk_74a cycles between writes
                                                // (~808 ns/word — keeps the
                                                // 512-entry FIFO from filling,
                                                // matching APF's real pacing)
    parameter int CLKMEM_PHASE_PS  = 3777,      // clk_mem initial phase offset
    parameter int RUN_SAVE         = 0,         // 1 = run the SAVE-path test instead of LOAD
    parameter int SAVE_CHUNKS      = 8,         // # 64-bit chunks the firmware "saves"
    parameter int SAVE_PAUSE_AT    = 0,         // chunk index to pause APF reads at (0=off)
    parameter int SAVE_PAUSE_CYC   = 4000,      // clk_sys cycles to pause (> idle_max=2000)
    parameter int SAVE_PRESTART_DLY = 0,        // clk_sys cycles to delay APF's FIRST read
    parameter int USE_REAL_STA     = 0,         // 1 = stream a real .sta payload (smw_payload.hex)
    parameter int REAL_STA_CHUNKS  = 4096       // # chunks of the real payload to round-trip test
);
  localparam [24:0] STAGING_BASE  = 25'h800000; // byte address (matches DUT)

  // Clock periods (ps).  clk_mem is deliberately NOT an exact multiple of
  // clk_sys and starts phase-offset, so the CDC alignment sweeps.
  localparam CLK74_PER  = 13468;   // 74.25 MHz
  localparam CLKSYS_PER = 46560;   // 21.477 MHz
  localparam CLKMEM_PER = 11630;   // ~85.99 MHz

  reg clk_74a = 0;
  reg clk_sys = 0;
  reg clk_mem = 0;

  always #(CLK74_PER  / 2) clk_74a = ~clk_74a;
  always #(CLKSYS_PER / 2) clk_sys = ~clk_sys;
  initial begin
    #(CLKMEM_PHASE_PS);         // phase offset
    forever #(CLKMEM_PER / 2) clk_mem = ~clk_mem;
  end

  // -------------------------------------------------------------------
  // Bridge / APF stimulus signals
  // -------------------------------------------------------------------
  reg         bridge_wr      = 0;
  reg         bridge_rd      = 0;
  reg  [31:0] bridge_addr    = 32'h0;
  reg  [31:0] bridge_wr_data = 32'h0;
  reg         savestate_load = 0;
  wire [31:0] save_state_bridge_read_data;

  // Firmware-side SAVE drivers (model savestates.sv producing the state).
  // Safe defaults reproduce the old tie-offs so the LOAD test is unaffected.
  reg         fw_savestate_start = 0;
  reg  [63:0] fw_ss_din          = 64'h0;
  reg  [16:0] fw_ss_addr         = 17'h0;
  reg         fw_ss_rnw          = 1'b1;
  reg         fw_ss_req          = 1'b0;
  reg         fw_ss_busy         = 1'b0;
  wire        ss_ack_w;
  wire [63:0] ss_dout_w;
  wire        ss_load_w;

  // Bridge data pattern for word n: both 16-bit halves nonzero & unique.
  function automatic [31:0] vpat(input int n);
    vpat = {16'hC000 | n[13:0], 16'h4000 | n[13:0]};
  endfunction

  // Byte-swap a 32-bit word (matches fifo_save.q → bridge_rd_data wiring).
  function automatic [31:0] bswap32(input [31:0] d);
    bswap32 = {d[7:0], d[15:8], d[23:16], d[31:24]};
  endfunction

  // Byte-reverse (replicates bridge_wr_swapped in save_state_controller).
  function automatic [31:0] bswap(input [31:0] x);
    bswap = {x[7:0], x[15:8], x[23:16], x[31:24]};
  endfunction

  // Build the 64-bit chunk the LOAD firmware reads, from the two file words.
  // smw_payload.hex stores word w0=0xB0B1B2B3 (file bytes b0..b3, b0=MSB) and
  // w1=0xB4B5B6B7.  The controller stages file bytes little-endian (b0 in
  // [7:0]) so ss_dout = {b7..b0}.  Thus want = {bswap(w1), bswap(w0)}.
  function automatic [63:0] bswap_chunk(input [31:0] w0, input [31:0] w1);
    bswap_chunk = {bswap(w1), bswap(w0)};
  endfunction

  // Expected 16-bit SDRAM word for global staged word index k.
  //   chunk N = bridge words 2N (→ q[31:0]) and 2N+1 (→ q[63:32])
  function automatic [15:0] expected_word(input int k);
    logic [31:0] src;
    src = bswap(vpat(2 * (k / 4) + ((k % 4) / 2)));
    expected_word = (k % 2) ? src[31:16] : src[15:0];
  endfunction

  // -------------------------------------------------------------------
  // DUT 1: save_state_controller (clk_74a / clk_sys)
  // -------------------------------------------------------------------
  wire        ss_sdram_wr_req;
  wire [24:0] ss_sdram_wr_addr;
  wire [15:0] ss_sdram_wr_data;
  wire        ss_sdram_wr_ack;
  wire        ss_sdram_rd_req;
  wire [24:0] ss_sdram_rd_addr;
  wire [15:0] ss_sdram_rd_data;
  wire        ss_sdram_rd_ack;
  wire        ss_loading;
  wire        ss_pause_cpu;

  // 2026-06-22: PSRAM staging interface (controller drives, ss_psram_arbiter
  // CDCs into clk_mem, psram_arbiter Port B drives the psram core and chip).
  wire        ss_psram_wr_req;
  wire [18:0] ss_psram_wr_addr;
  wire [63:0] ss_psram_wr_data;
  wire        ss_psram_wr_ack;
  wire        ss_psram_rd_req;
  wire [18:0] ss_psram_rd_addr;
  wire [63:0] ss_psram_rd_data;
  wire        ss_psram_rd_ack;

  // Shorten the serve idle-completion watchdog so the SAVE test can verify
  // the no-freeze release in a few hundred clk_sys instead of ~1M.
  save_state_controller #(.SAVE_SERVE_IDLE_MAX(21'd2000)) ssc (
      .clk_74a(clk_74a),
      .clk_sys(clk_sys),

      .bridge_wr(bridge_wr),
      .bridge_rd(bridge_rd),
      .bridge_endian_little(1'b0),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),
      .save_state_bridge_read_data(save_state_bridge_read_data),

      .savestate_load(savestate_load),
      .savestate_load_ack_s(),
      .savestate_load_busy_s(),
      .savestate_load_ok_s(),
      .savestate_load_err_s(),

      .savestate_start(fw_savestate_start),
      .savestate_start_ack_s(),
      .savestate_start_busy_s(),
      .savestate_start_ok_s(),
      .savestate_start_err_s(),

      .ss_save(),
      .ss_load(ss_load_w),

      .ss_din(fw_ss_din),
      .ss_dout(ss_dout_w),
      .ss_addr(fw_ss_addr),
      .ss_rnw(fw_ss_rnw),
      .ss_req(fw_ss_req),
      .ss_be(8'hFF),
      .ss_ack(ss_ack_w),
      .ss_busy(fw_ss_busy),

      .ss_sdram_wr_req (ss_sdram_wr_req),
      .ss_sdram_wr_addr(ss_sdram_wr_addr),
      .ss_sdram_wr_data(ss_sdram_wr_data),
      .ss_sdram_wr_ack (ss_sdram_wr_ack),
      .ss_sdram_rd_req (ss_sdram_rd_req),
      .ss_sdram_rd_addr(ss_sdram_rd_addr),
      .ss_sdram_rd_data(ss_sdram_rd_data),
      .ss_sdram_rd_ack (ss_sdram_rd_ack),
      .ss_loading      (ss_loading),
      .ss_pause_cpu    (ss_pause_cpu),

      // PSRAM staging interface — drives the new bank-1 path under test.
      .ss_psram_wr_req (ss_psram_wr_req),
      .ss_psram_wr_addr(ss_psram_wr_addr),
      .ss_psram_wr_data(ss_psram_wr_data),
      .ss_psram_wr_ack (ss_psram_wr_ack),
      .ss_psram_rd_req (ss_psram_rd_req),
      .ss_psram_rd_addr(ss_psram_rd_addr),
      .ss_psram_rd_data(ss_psram_rd_data),
      .ss_psram_rd_ack (ss_psram_rd_ack)
  );

  // -------------------------------------------------------------------
  // DUT 2: ss_sdram_arbiter (clk_sys ↔ clk_mem CDC + access FSM)
  // -------------------------------------------------------------------
  wire        ss_loading_mem;
  wire        ss_mem_rd, ss_mem_wr;
  wire [24:0] ss_mem_addr;
  wire [15:0] ss_mem_din;
  wire [15:0] sdram_dout;
  wire        sdram_busy;

  ss_sdram_arbiter arbiter (
      .clk_sys(clk_sys),
      .clk_mem(clk_mem),

      .ss_sdram_wr_req (ss_sdram_wr_req),
      .ss_sdram_wr_addr(ss_sdram_wr_addr),
      .ss_sdram_wr_data(ss_sdram_wr_data),
      .ss_sdram_wr_ack (ss_sdram_wr_ack),
      .ss_sdram_rd_req (ss_sdram_rd_req),
      .ss_sdram_rd_addr(ss_sdram_rd_addr),
      .ss_sdram_rd_data(ss_sdram_rd_data),
      .ss_sdram_rd_ack (ss_sdram_rd_ack),
      .ss_loading      (ss_loading),

      .ss_loading_mem(ss_loading_mem),
      .ss_mem_rd     (ss_mem_rd),
      .ss_mem_wr     (ss_mem_wr),
      .ss_mem_addr   (ss_mem_addr),
      .ss_mem_din    (ss_mem_din),
      .sdram_dout    (sdram_dout),
      .sdram_busy    (sdram_busy)
  );

  // -------------------------------------------------------------------
  // DUT 3: sdram.sv — replicating the SNES.sv input mux with the
  // non-savestate paths tied off (cart_download=0, ROM side idle).
  // -------------------------------------------------------------------
  wire [15:0] dram_dq;
  wire [12:0] dram_a;
  wire [1:0]  dram_ba;
  wire        dram_dqml, dram_dqmh;
  wire        dram_ncs, dram_nwe, dram_nras, dram_ncas;
  wire        dram_clk, dram_cke;
  wire [15:0] dbg_real_writes_unused;  // = first_w2_data in v60
  wire [15:0] dbg_w2_info;             // = {cnt_wr_hi[17:10], first_w2_addr_lo}

  sdram sdram (
      .init(1'b0),
      .clk (clk_mem),

      .addr(ss_loading_mem ? ss_mem_addr : 25'h0),
      .din (ss_loading_mem ? ss_mem_din  : 16'h0),
      .dout(sdram_dout),
      .rd  (ss_loading_mem ? ss_mem_rd   : 1'b0),
      .wr  (ss_loading_mem ? ss_mem_wr   : 1'b0),
      .word(1'b1),
      .busy(sdram_busy),

      .dbg_real_writes(dbg_real_writes_unused),
      .dbg_w2_info    (dbg_w2_info),

      .SDRAM_DQ(dram_dq),
      .SDRAM_A(dram_a),
      .SDRAM_DQML(dram_dqml),
      .SDRAM_DQMH(dram_dqmh),
      .SDRAM_BA(dram_ba),
      .SDRAM_nCS(dram_ncs),
      .SDRAM_nWE(dram_nwe),
      .SDRAM_nRAS(dram_nras),
      .SDRAM_nCAS(dram_ncas),
      .SDRAM_CLK(dram_clk),
      .SDRAM_CKE(dram_cke)
  );

  sdram_chip_model chip (
      .clk (clk_mem),
      .dq  (dram_dq),
      .a   (dram_a),
      .ba  (dram_ba),
      .ncs (dram_ncs),
      .nras(dram_nras),
      .ncas(dram_ncas),
      .nwe (dram_nwe)
  );

  // -------------------------------------------------------------------
  // DUT 4: ss_psram_arbiter + psram_arbiter + psram core + chip model.
  // The new SAVE path under test.  Port A (ARAM) is tied to idle so
  // contention testing happens in a later TB pass; this run validates
  // only Port B (savestate) end-to-end.
  // -------------------------------------------------------------------
  wire        ss_psram_b_write_en;
  wire        ss_psram_b_read_en;
  wire [21:0] ss_psram_b_addr;
  wire [15:0] ss_psram_b_data_in;
  wire        ss_psram_b_write_high_byte;
  wire        ss_psram_b_write_low_byte;
  wire        ss_psram_b_bank_sel;
  wire [15:0] ss_psram_b_data_out;
  wire        ss_psram_b_read_avail;
  wire        ss_psram_b_busy;

  ss_psram_arbiter ss_psram_arb (
      .clk_sys(clk_sys),
      .clk_mem(clk_mem),

      .ss_psram_wr_req (ss_psram_wr_req),
      .ss_psram_wr_addr(ss_psram_wr_addr),
      .ss_psram_wr_data(ss_psram_wr_data),
      .ss_psram_wr_ack (ss_psram_wr_ack),
      .ss_psram_rd_req (ss_psram_rd_req),
      .ss_psram_rd_addr(ss_psram_rd_addr),
      .ss_psram_rd_data(ss_psram_rd_data),
      .ss_psram_rd_ack (ss_psram_rd_ack),

      .b_write_en       (ss_psram_b_write_en),
      .b_read_en        (ss_psram_b_read_en),
      .b_addr           (ss_psram_b_addr),
      .b_data_in        (ss_psram_b_data_in),
      .b_write_high_byte(ss_psram_b_write_high_byte),
      .b_write_low_byte (ss_psram_b_write_low_byte),
      .b_bank_sel       (ss_psram_b_bank_sel),
      .b_data_out       (ss_psram_b_data_out),
      .b_read_avail     (ss_psram_b_read_avail),
      .b_busy           (ss_psram_b_busy)
  );

  // CRAM1 pin bus (single chip model).
  wire [21:16] cram1_a;
  wire [15:0]  cram1_dq;
  wire         cram1_wait;
  wire         cram1_clk, cram1_adv_n, cram1_cre;
  wire         cram1_ce0_n, cram1_ce1_n, cram1_oe_n, cram1_we_n;
  wire         cram1_ub_n, cram1_lb_n;

  // Port A activity model — replicate the SPC700/ARAM access pattern that
  // the silicon SAVE-on-PSRAM build exposed:
  //
  //   - Each access asserts a_read_en for ONE clk_mem cycle only.
  //   - On the very next cycle a_read_en drops AND a_addr changes (the
  //     consumer has "moved on" to its next access setup or back to idle).
  //
  // This is the realistic model: the arbiter MUST capture the address at
  // the request edge, not at grant time, or it will service A's deferred
  // requests with the wrong address.
  reg [6:0]  a_traffic_cnt = 7'd0;
  reg        a_traffic_active = 1'b0;
  reg [15:0] a_seq = 16'h1000;       // monotonically incrementing test addr
  reg        a_pulse_read_r = 1'b0;
  reg [15:0] a_addr_at_edge = 16'h0;
  always @(posedge clk_mem) begin
    if (a_traffic_cnt == 7'd83) a_traffic_cnt <= 7'd0;
    else                        a_traffic_cnt <= a_traffic_cnt + 7'd1;
    if (a_traffic_active & (a_traffic_cnt == 7'd0)) begin
      a_pulse_read_r <= 1'b1;
      a_addr_at_edge <= a_seq;
      a_seq          <= a_seq + 16'd1;
    end else begin
      a_pulse_read_r <= 1'b0;
    end
  end
  // The consumer's address is only "valid" during the single pulse cycle.
  // Outside the pulse, we deliberately drive a_addr to a CHANGED value to
  // expose any arbiter logic that samples a_addr after the edge.
  wire [21:0] a_addr_wire = a_pulse_read_r ? {6'b0, a_addr_at_edge}
                                           : 22'h3F_FFFF;

  // Track every Port A access for later verification: what addr did the
  // chip actually see (vs. what we asked for at the edge)?
  int a_serviced_count = 0;
  int a_addr_mismatch  = 0;

  psram_arbiter #(
      .CLOCK_SPEED(85.9)
  ) cram1_arb (
      .clk(clk_mem),

      // Port A — driven by the traffic model above when a_traffic_active=1.
      .a_bank_sel(1'b0),
      .a_addr(a_addr_wire),
      .a_write_en(1'b0),
      .a_data_in(16'd0),
      .a_write_high_byte(1'b0),
      .a_write_low_byte(1'b0),
      .a_read_en(a_pulse_read_r),
      .a_read_avail(),
      .a_data_out(),
      .a_busy(),

      // Port B — driven by ss_psram_arbiter.
      .b_bank_sel(ss_psram_b_bank_sel),
      .b_addr(ss_psram_b_addr),
      .b_write_en(ss_psram_b_write_en),
      .b_data_in(ss_psram_b_data_in),
      .b_write_high_byte(ss_psram_b_write_high_byte),
      .b_write_low_byte(ss_psram_b_write_low_byte),
      .b_read_en(ss_psram_b_read_en),
      .b_read_avail(ss_psram_b_read_avail),
      .b_data_out(ss_psram_b_data_out),
      .b_busy(ss_psram_b_busy),

      .cram_a(cram1_a),
      .cram_dq(cram1_dq),
      .cram_wait(cram1_wait),
      .cram_clk(cram1_clk),
      .cram_adv_n(cram1_adv_n),
      .cram_cre(cram1_cre),
      .cram_ce0_n(cram1_ce0_n),
      .cram_ce1_n(cram1_ce1_n),
      .cram_oe_n(cram1_oe_n),
      .cram_we_n(cram1_we_n),
      .cram_ub_n(cram1_ub_n),
      .cram_lb_n(cram1_lb_n)
  );

  psram_chip_model psram_chip (
      .clk(clk_mem),
      .cram_a(cram1_a),
      .cram_dq(cram1_dq),
      .cram_wait(cram1_wait),
      .cram_clk(cram1_clk),
      .cram_adv_n(cram1_adv_n),
      .cram_cre(cram1_cre),
      .cram_ce0_n(cram1_ce0_n),
      .cram_ce1_n(cram1_ce1_n),
      .cram_oe_n(cram1_oe_n),
      .cram_we_n(cram1_we_n),
      .cram_ub_n(cram1_ub_n),
      .cram_lb_n(cram1_lb_n)
  );

  // -------------------------------------------------------------------
  // Spies (hierarchical, need -novopt / +acc)
  // -------------------------------------------------------------------
  localparam SYS_STAGE_FIFO_LATCH = 5'd12;

  // v61: the load FIFO is now plain 32-bit; a chunk = two FIFO_LATCH
  // visits (stage_half 0 then 1).  Count assembled chunks and check both
  // halves carried nonzero data (the TB pattern guarantees nonzero).
  int fifo_latch_count       = 0;   // completed chunks
  int fifo_upper_zero_count  = 0;
  int fifo_lower_zero_count  = 0;
  logic [63:0] first_fifo_dout = 'x;

  always @(posedge clk_sys) begin
    if (ssc.sys_state == SYS_STAGE_FIFO_LATCH) begin
      if (!ssc.stage_half) begin
        if (ssc.fifo_load_dout == 32'h0) fifo_lower_zero_count++;
      end else begin
        fifo_latch_count++;
        if (fifo_latch_count == 1)
          first_fifo_dout = {ssc.fifo_load_dout, ssc.stage_buffer[31:0]};
        if (ssc.fifo_load_dout == 32'h0) fifo_upper_zero_count++;
      end
    end
  end

  // -------------------------------------------------------------------
  // Stimulus
  // -------------------------------------------------------------------
  int errors = 0;

  // v66 hardware proved bridge_wr is a clean 1-cycle pulse per 32-bit word
  // (addresses captured as +0,+4,+8,+C, no repeats).  The earlier 2-cycle
  // model was a wrong guess; drive a faithful single-cycle strobe.
  task automatic bridge_write(input [31:0] addr, input [31:0] data);
    @(posedge clk_74a);
    bridge_addr    <= addr;
    bridge_wr_data <= data;
    bridge_wr      <= 1'b1;
    @(posedge clk_74a);
    bridge_wr      <= 1'b0;
  endtask

  // ---- SAVE-path helpers ----
  // 64-bit chunk pattern: 4 distinct, nonzero, idx-dependent 16-bit words so
  // the staging word order and "varied (not fill)" serve can both be checked.
  function automatic [63:0] save_pat(input int idx);
    save_pat = { 16'hD000 | idx[11:0], 16'hC000 | idx[11:0],
                 16'hB000 | idx[11:0], 16'hA000 | idx[11:0] };
  endfunction

  // Model the firmware producing one save chunk: present data/addr, toggle
  // ss_req, wait for the controller's ss_ack toggle (after it commits the 4
  // SDRAM words).  Mirrors the real ddr_req/ddr_ack backpressure.
  task automatic fw_save_chunk(input [16:0] idx, input [63:0] data);
    logic ack_prev;
    @(posedge clk_sys);
    ack_prev   = ss_ack_w;
    fw_ss_din  <= data;
    fw_ss_addr <= idx;
    fw_ss_rnw  <= 1'b0;
    fw_ss_req  <= ~fw_ss_req;
    @(posedge clk_sys);
    while (ss_ack_w === ack_prev) @(posedge clk_sys);
  endtask

  // Model the LOAD firmware reading one chunk back: present chunk index with
  // ss_rnw=1, toggle ss_req, wait for ss_ack, capture ss_dout.  This exactly
  // matches the verified firmware behaviour (ss_addr = 0,1,2,... monotonic,
  // no region resets; chunk k = save-stream bytes [8k..8k+7]).
  task automatic fw_load_chunk(input [16:0] idx, output [63:0] data);
    logic ack_prev;
    @(posedge clk_sys);
    ack_prev   = ss_ack_w;
    fw_ss_addr <= idx;
    fw_ss_rnw  <= 1'b1;
    fw_ss_req  <= ~fw_ss_req;
    @(posedge clk_sys);
    while (ss_ack_w === ack_prev) @(posedge clk_sys);
    // ss_dout is registered in SYS_SERVE_ACK on the same edge ss_ack toggles.
    @(posedge clk_sys);
    data = ss_dout_w;
  endtask

  // APF reads one 32-bit word from the slot (showahead=ON: q is the head;
  // the rising-edge pop advances afterward).  Waits for data to be available
  // first — real APF reads (~75 cyc/word) are slower than the serve produces,
  // so gating on non-empty models a sustainable read pace and isolates the
  // serve *logic* from the TB-only over-read (the real throughput margin is
  // validated on hardware).
  task automatic bridge_read32(input [31:0] addr, output [31:0] data);
    int wait_cyc;
    // Bounded wait for FIFO non-empty.  If the serve has ABORTED (the bug),
    // the FIFO drains and stays empty forever — return 0 (as APF would read
    // past the served data) instead of hanging, so the self-check can flag
    // the resulting zero chunks as errors.
    wait_cyc = 0;
    while (ssc.fifo_save_rd_empty && wait_cyc < 100000) begin
      @(posedge clk_74a); wait_cyc++;
    end
    if (ssc.fifo_save_rd_empty) begin
      data = 32'h0;  // serve produced nothing more (aborted / underran)
      return;
    end
    @(posedge clk_74a);
    bridge_addr <= addr;
    bridge_rd   <= 1'b1;          // rising edge → handler issues rdreq
    // showahead=OFF: q presents the popped word a couple rdclk cycles after
    // rdreq, within APF's read latency.  Sample after it settles.
    repeat (3) @(posedge clk_74a);
    #1 data = save_state_bridge_read_data;
    bridge_rd <= 1'b0;
    repeat (4) @(posedge clk_74a);  // gap before next read
  endtask

  // Real .sta payload storage (loaded from smw_payload.hex when USE_REAL_STA=1).
  // Each entry is one 32-bit big-endian bridge word (file bytes [4n..4n+3]).
  logic [31:0] sta_words [0:131071];

  initial begin : main
    int n, k;
    int unsigned lin;
    logic [15:0] got, exp;

    // =====================================================================
    // REAL .sta ROUND-TRIP TEST (USE_REAL_STA=1): stream a real SMW savestate
    // payload through LOAD staging into PSRAM bank 1, then model the firmware
    // reading every chunk back and verify byte-exactness against the file.
    // This catches any DATA-DEPENDENT drift the synthetic vpat() can't.
    // =====================================================================
    if (USE_REAL_STA) begin : real_sta_test
      int c, serve_errs, nchunks;
      logic [63:0] got64, want64;
      $readmemh("core/tb/smw_payload.hex", sta_words);
      nchunks = REAL_STA_CHUNKS;
      serve_errs = 0;
      $display("=== REAL .sta test: streaming %0d chunks (%0d KB) ===",
               nchunks, nchunks*8/1024);

      #5_000_000;  // let any mem init settle

      // Stream the real payload as bridge writes (2 words per chunk).
      for (n = 0; n < nchunks*2; n++) begin
        bridge_write(32'h4000_0000 + 4*n, sta_words[n]);
        repeat (BRIDGE_GAP_CYC) @(posedge clk_74a);
        if (n == 4) savestate_load <= 1'b1;  // command arrives shortly after data starts
      end
      $display("[tb] real payload streamed, t=%0t", $time);
      #20_000_000;
      savestate_load <= 1'b0;

      // Wait for staging→serve kick.
      fork begin
        int wd; wd=0;
        while (ssc.sys_state !== 5'd20 && wd < 2000000) begin @(posedge clk_sys); wd++; end
      end join
      if (ssc.sys_state !== 5'd20) begin
        $display("*** REAL .sta FAIL — serve never kicked (sys_state=%0d) ***", ssc.sys_state);
        $finish;
      end
      $display("[tb] serve kicked; firmware reading %0d chunks back...", nchunks);

      fw_ss_busy <= 1'b1;
      for (c = 0; c < nchunks; c++) begin
        fw_load_chunk(c[16:0], got64);
        // Expected chunk c = file bytes [8c..8c+7].  sta_words[2c] = bytes
        // [8c..8c+3] big-endian, sta_words[2c+1] = [8c+4..8c+7].  The firmware
        // reads chunk as 64-bit little-endian word: byte0 in [7:0].  Build the
        // expected the same way save_state_controller serves it.
        want64 = bswap_chunk(sta_words[2*c], sta_words[2*c+1]);
        if (got64 !== want64) begin
          serve_errs++;
          if (serve_errs <= 24)
            $display("ERROR real chunk %0d: got %h want %h", c, got64, want64);
        end
      end
      fw_ss_busy <= 1'b0;
      $display("[tb] REAL .sta round-trip: %0d / %0d chunks wrong", serve_errs, nchunks);
      if (serve_errs == 0)
        $display("*** REAL .sta PASS — full payload round-trips byte-exact ***");
      else
        $display("*** REAL .sta FAIL — %0d chunks corrupted ***", serve_errs);
      $finish;
    end

    // =====================================================================
    // SAVE-path test (RUN_SAVE=1): stage SAVE_CHUNKS firmware chunks into
    // SDRAM, verify the SDRAM contents (staging exact), then read the slot
    // back via bridge_rd and confirm the serve delivers VARIED data (not the
    // constant fill the old streaming-FIFO save produced).
    // =====================================================================
    if (RUN_SAVE) begin : save_test
      int i, w, errs, nz;
      int unsigned slin;
      logic [15:0] mw, ew;
      logic [31:0] rdw;
      errs = 0; nz = 0;
      $display("=== SAVE test: %0d chunks ===", SAVE_CHUNKS);
      // Watchdog so a missed handshake reports state instead of hanging.
      // Generous (12 ms): the serve now PRE-FILLS the 1024-deep fifo_save
      // (reading staged + padding chunks from SDRAM) before asserting OK.
      fork : save_wd
        begin
          repeat (12000) #1_000_000;  // 12 ms
          $display("WATCHDOG: save hung. sys_state=%0d start_busy=%b ok=%b ss_ack=%b ss_loading=%b ss_req=%b sdram_wr_ack=%b sdram_rd_ack=%b prefilled=%b idle=%0d",
                   ssc.sys_state, ssc.savestate_start_busy, ssc.savestate_start_ok,
                   ss_ack_w, ssc.ss_loading, fw_ss_req, ss_sdram_wr_ack, ss_sdram_rd_ack,
                   ssc.serve_prefilled, ssc.save_serve_idle);
          $finish;
        end
      join_none

      #5_000_000;  // sdram init

      // Enable Port A traffic on the PSRAM arbiter — simulates SPC700/ARAM
      // hitting CRAM1 bank 0 in parallel with savestate's bank-1 writes.
      // The hardware failure mode this exposes: if Port B is starved by
      // Port A's ~a_req gate, bank-1 writes drop and the served data ends
      // up as the stale PSRAM contents (or wrong addresses).
      a_traffic_active = 1'b1;

      // Hold savestate_start wide (~270 ns) so the slower clk_sys synch_3
      // reliably catches the rising edge.
      @(posedge clk_74a); fw_savestate_start <= 1'b1;
      repeat (20) @(posedge clk_74a); fw_savestate_start <= 1'b0;
      fw_ss_busy <= 1'b1;
      wait (ssc.savestate_start_busy == 1'b1);
      $display("[tb] save started: start_busy=1, sys_state=%0d, t=%0t", ssc.sys_state, $time);

      for (i = 0; i < SAVE_CHUNKS; i++) begin
        $display("[tb] producing chunk %0d (sys_state=%0d)...", i, ssc.sys_state);
        fw_save_chunk(i[16:0], save_pat(i));
        $display("[tb]   chunk %0d acked, t=%0t", i, $time);
      end
      $display("[tb] all chunks produced; dropping ss_busy");

      @(posedge clk_sys); fw_ss_busy <= 1'b0;
      // OK is now asserted only AFTER the serve pre-fills fifo_save to full.
      wait (ssc.savestate_start_ok == 1'b1);
      $display("[tb] save staged + serve pre-filled, ok asserted, t=%0t (prefilled=%b)",
               $time, ssc.serve_prefilled);

      // Staging check: PSRAM bank-1 word-index = idx*4 + w (base 0, stride 1).
      // 2026-06-22 pivot: SAVE staging now lives in CRAM1 bank 1, not SDRAM.
      for (i = 0; i < SAVE_CHUNKS; i++)
        for (w = 0; w < 4; w++) begin
          logic [21:0] paddr;
          paddr = i*4 + w;
          mw = psram_chip.peek(1'b1, paddr);
          ew = save_pat(i) >> (w*16);
          if (mw !== ew) begin
            errs++;
            if (errs <= 8)
              $display("ERROR stage chunk %0d w%0d (paddr=%h): got %h exp %h",
                       i, w, paddr, mw, ew);
          end
        end
      $display("[tb] PSRAM staging check: %0d word errors (of %0d)", errs, SAVE_CHUNKS*4);
      $display("[tb] PSRAM chip: bank1 writes=%0d bank0 writes=%0d  bank1 reads=%0d bank0 reads=%0d",
               psram_chip.total_writes_bank1, psram_chip.total_writes_bank0,
               psram_chip.total_reads_bank1, psram_chip.total_reads_bank0);

      // Port A check for the DROP-ON-BUSY contract: Port A requests may be
      // dropped when the core is busy with Port B (matching the original
      // psram, whose DSP/ARAM consumer tolerates dropped reads).  So the
      // serviced addresses must be a MONOTONIC SUBSEQUENCE of the requested
      // sequence (0x1000, 0x1001, ...), not a strict 1:1 mapping.  A real
      // error is a serviced address that goes BACKWARD or was never asked
      // (== the arbiter latched a stale/garbage address).
      begin : port_a_check
        int  k;
        logic [21:0] got;
        logic [21:0] prev_got;
        prev_got = 22'h000FFF;  // one below the first expected (0x1000)
        $display("[tb] Port A bank-0 reads serviced: %0d (of up to %0d requested)",
                 psram_chip.bank0_read_log_count, a_seq - 16'h1000);
        for (k = 0; k < psram_chip.bank0_read_log_count; k++) begin
          got = psram_chip.bank0_read_addr_log[k];
          // Must be strictly increasing and within the requested range.
          if (got <= prev_got || got < 22'h001000 || got >= {6'b0, a_seq}) begin
            errs++;
            a_addr_mismatch++;
            if (a_addr_mismatch <= 8)
              $display("ERROR Port A read %0d: chip saw addr=%h (prev=%h) — not a valid in-order request",
                       k, got, prev_got);
          end else begin
            a_serviced_count++;
          end
          prev_got = got;
        end
        $display("[tb] Port A: %0d reads with correct addr, %0d address mismatches",
                 a_serviced_count, a_addr_mismatch);
      end

      // Serve check: read back the staged chunks (front of the pre-filled
      // FIFO) and confirm EXACT ordering against the staged pattern.  Each
      // 64-bit chunk = two 32-bit APF words: low word = save_pat[31:0],
      // high word = save_pat[63:32].
      begin : serve_check
        logic [31:0] elo, ehi;
        // Model APF being SLOW TO START reading after savestate_start_ok.
        // This is the real hardware failure: the serve fills the FIFO (1024
        // chunks) and then, before APF's first read, the idle watchdog
        // counted to SAVE_SERVE_IDLE_MAX and ABORTED — truncating the save to
        // exactly one FIFO depth.  The fix (apf_ever_read arming) must let the
        // serve wait through this pre-start gap.
        if (SAVE_PRESTART_DLY > 0) begin
          $display("[tb] APF delaying first read %0d clk_sys...", SAVE_PRESTART_DLY);
          repeat (SAVE_PRESTART_DLY) @(posedge clk_sys);
          $display("[tb] APF first read now. sys_state=%0d serve_aborted=%b idx_max=%0d ss_loading=%b",
                   ssc.sys_state, ssc.serve_aborted, ssc.save_serve_idx_max, ssc.ss_loading);
        end
        // APF sees bridge_rd_data = byteswap32(fifo_q); the FIFO holds the
        // staged 64-bit chunk (save_pat).  Expected per-word = byteswapped.
        //
        // 2026-06-25: inject a mid-stream APF PAUSE longer than
        // SAVE_SERVE_IDLE_MAX (TB override = 2000) once the serve has filled
        // the FIFO.  This reproduces the hardware failure mode: with the old
        // watchdog (reset only in the apf_reading branch), a real >FIFO-depth
        // save let save_serve_idle climb to MAX and ABORT the serve after one
        // FIFO depth.  With the fix (reset on every push), a pause only stalls
        // while there's genuinely no progress; reads resume cleanly after.
        for (i = 0; i < SAVE_CHUNKS; i++) begin
          // Pause partway through, long enough to exceed the idle watchdog.
          if (SAVE_PAUSE_AT > 0 && i == SAVE_PAUSE_AT) begin
            $display("[tb] APF pausing %0d clk_sys at chunk %0d (idle_max overridden=%0d)",
                     SAVE_PAUSE_CYC, i, 2000);
            repeat (SAVE_PAUSE_CYC) @(posedge clk_sys);
            $display("[tb] APF resuming reads, sys_state=%0d ss_loading=%b serve_aborted=%b idx_max=%0d",
                     ssc.sys_state, ssc.ss_loading, ssc.serve_aborted, ssc.save_serve_idx_max);
          end
          bridge_read32(32'h4000_0000 + (i*2  )*4, rdw); elo = rdw;
          bridge_read32(32'h4000_0000 + (i*2+1)*4, rdw); ehi = rdw;
          if (elo !== 32'h0 && ehi !== 32'h0) nz += 2;
          if (elo !== bswap32(save_pat(i)[31:0]) ||
              ehi !== bswap32(save_pat(i)[63:32])) begin
            errs++;
            if (errs <= 8)
              $display("ERROR serve chunk %0d: got %h_%h exp %h_%h",
                       i, ehi, elo, bswap32(save_pat(i)[63:32]), bswap32(save_pat(i)[31:0]));
          end
          if (i < 4) $display("[tb] serve chunk %0d = %h_%h", i, ehi, elo);
        end
        $display("[tb] serve read-back: %0d/%0d nonzero, %0d order errors",
                 nz, SAVE_CHUNKS*2, errs);
      end

      // No-freeze check: stop reading.  The serve keeps pushing padding until
      // fifo_save is full, then the idle watchdog (SAVE_SERVE_IDLE_MAX=2000)
      // must release ss_loading and return to IDLE — never hang.
      begin : idle_complete
        int wd; wd = 0;
        while (ssc.ss_loading === 1'b1 && wd < 200000) begin
          @(posedge clk_sys); wd++;
        end
        if (ssc.ss_loading === 1'b0)
          $display("[tb] serve idle-completed: ss_loading cleared after %0d clk_sys, sys_state=%0d",
                   wd, ssc.sys_state);
        else
          $display("ERROR serve never released ss_loading (FREEZE), sys_state=%0d", ssc.sys_state);
      end

      if (errs == 0 && ssc.ss_loading === 1'b0)
        $display("*** SAVE PASS — staging exact, serve ordered, idle-completes (no freeze) ***");
      else
        $display("*** SAVE FAIL (errs=%0d, ss_loading=%b) ***", errs, ssc.ss_loading);
      $finish;
    end

    $display("=== tb_ss_staging: %0d bridge words (%0d chunks) ===",
             NUM_BRIDGE_WORDS, NUM_BRIDGE_WORDS / 2);

    // Let sdram.sv's internal init sequence reach MODE_NORMAL (~190 clk_mem).
    #5_000_000;  // 5 us

    // Stream the bridge words; raise savestate_load partway through, as the
    // real APF does (data streams first, command follows).
    for (n = 0; n < NUM_BRIDGE_WORDS; n++) begin
      bridge_write(32'h4000_0000 + 4 * n, vpat(n));
      repeat (BRIDGE_GAP_CYC) @(posedge clk_74a);
      if (n == NUM_BRIDGE_WORDS / 2) begin
        savestate_load <= 1'b1;
        $display("[tb] savestate_load raised at word %0d, t=%0t", n, $time);
      end
    end
    $display("[tb] bridge stream complete, t=%0t", $time);

    // Drop savestate_load after a while (controller edge-detects it).
    #30_000_000;  // 30 us
    savestate_load <= 1'b0;

    // Wait for staging to finish and the controller's own 4-word probe to
    // complete (probe_done covers chunk-0 words 0..3 read back via SDRAM).
    fork
      wait (ssc.probe_done == 1'b1);
      begin
        repeat (2000) #10_000_000;  // 20 ms watchdog (2000 × 10 us)
        $display("FATAL: watchdog — probe_done never rose.");
        $display("  sys_state=%0d stage_entry_count=%0d fifo_latches=%0d",
                 ssc.sys_state, ssc.stage_entry_count, fifo_latch_count);
        $fatal(1);
      end
    join_any
    disable fork;

    // Allow the final ack CDC to settle.
    #2_000_000;  // 2 us

    // ---------------------------------------------------------------
    // Checks
    // ---------------------------------------------------------------
    $display("");
    $display("=== RESULTS ===");
    $display("[tb] fifo latches            : %0d (expect %0d)",
             fifo_latch_count, NUM_BRIDGE_WORDS / 2);
    $display("[tb] fifo dout upper==0 count: %0d", fifo_upper_zero_count);
    $display("[tb] fifo dout lower==0 count: %0d", fifo_lower_zero_count);
    $display("[tb] first fifo dout         : %h", first_fifo_dout);
    $display("[tb] fifo_or_checksum        : %h", ssc.fifo_or_checksum);
    $display("[tb] fifo_load_drop_cnt      : %0d  overflow=%b",
             ssc.fifo_load_drop_cnt, ssc.fifo_load_overflow);
    $display("[tb] PSRAM bank1 writes      : %0d (expect %0d chunks)",
             psram_chip.total_writes_bank1, NUM_BRIDGE_WORDS / 2);

    // 2026-06-25: LOAD now stages to PSRAM bank 1 (chunk N at word N*4),
    // mirroring SAVE.  Full content check of every staged 16-bit word read
    // directly from the PSRAM chip model.  PSRAM word k holds expected_word(k).
    begin
      int mism = 0;
      logic [15:0] pw;
      for (k = 0; k < NUM_BRIDGE_WORDS * 2; k++) begin
        exp = expected_word(k);
        pw  = psram_chip.peek(1'b1, k[21:0]);   // bank 1, word k
        if (pw !== exp) begin
          errors++;
          mism++;
          if (mism <= 16)
            $display("ERROR: staged word %0d (chunk %0d idx %0d): got %h expected %h",
                     k, k / 4, k % 4, pw, exp);
        end
      end
      if (mism > 16) $display("  ... and %0d more mismatches", mism - 16);
      $display("[tb] PSRAM content check     : %0d / %0d words wrong", mism,
               NUM_BRIDGE_WORDS * 2);
    end

    if (ssc.fifo_load_drop_cnt != 0) begin
      errors++;
      $display("ERROR: load FIFO dropped %0d words (TB pacing too fast?)",
               ssc.fifo_load_drop_cnt);
    end

    // ===================================================================
    // LOAD SERVE ROUND-TRIP CHECK (2026-06-26): model the firmware reading
    // each chunk back via ss_req/ss_rnw=1/ss_addr and verify ss_dout matches
    // what was staged.  This closes the §9 "drift" gap WITHOUT the SNES core:
    // it exercises the real LOAD serve FSM (PSRAM burst read → ss_dout) end to
    // end across the exact firmware interface.  Firmware model is the verified
    // sequence: ss_addr = 0,1,2,...,N monotonic, chunk k = staged words [4k..4k+3].
    begin : load_serve_roundtrip
      int c, nchunks, serve_errs;
      logic [63:0] got, want;
      serve_errs = 0;
      nchunks = NUM_BRIDGE_WORDS / 2;   // 2 bridge words = 1 chunk

      // Keep the firmware "busy" so the controller stays in the serve loop,
      // then wait for it to reach SERVE_WAIT_REQ (serve kicked after staging).
      fw_ss_busy <= 1'b1;
      fork
        begin
          int wd; wd = 0;
          while (ssc.sys_state !== 5'd20 /*SYS_SERVE_WAIT_REQ*/ && wd < 500000) begin
            @(posedge clk_sys); wd++;
          end
        end
      join
      if (ssc.sys_state !== 5'd20) begin
        errors++;
        $display("ERROR: serve never reached SERVE_WAIT_REQ (sys_state=%0d) — LOAD kick failed",
                 ssc.sys_state);
      end else begin
        $display("[tb] serve kicked (SERVE_WAIT_REQ); firmware reading %0d chunks back...", nchunks);
        for (c = 0; c < nchunks; c++) begin
          fw_load_chunk(c[16:0], got);
          want = {expected_word(4*c+3), expected_word(4*c+2),
                  expected_word(4*c+1), expected_word(4*c+0)};
          if (got !== want) begin
            serve_errs++;
            errors++;
            if (serve_errs <= 16)
              $display("ERROR: LOAD serve chunk %0d: got %h want %h", c, got, want);
          end
        end
        $display("[tb] LOAD serve round-trip   : %0d / %0d chunks wrong", serve_errs, nchunks);
      end
      fw_ss_busy <= 1'b0;
    end

    $display("");
    if (errors == 0)
      $display("*** PASS — staging path clean: all 4 words of every chunk landed ***");
    else
      $display("*** FAIL — %0d errors (see above) ***", errors);
    $display("");
    $finish;
  end

endmodule
