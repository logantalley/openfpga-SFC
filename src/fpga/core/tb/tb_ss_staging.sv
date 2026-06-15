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
    parameter int CLKMEM_PHASE_PS  = 3777       // clk_mem initial phase offset
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
  reg  [31:0] bridge_addr    = 32'h0;
  reg  [31:0] bridge_wr_data = 32'h0;
  reg         savestate_load = 0;

  // Bridge data pattern for word n: both 16-bit halves nonzero & unique.
  function automatic [31:0] vpat(input int n);
    vpat = {16'hC000 | n[13:0], 16'h4000 | n[13:0]};
  endfunction

  // Byte-reverse (replicates bridge_wr_swapped in save_state_controller).
  function automatic [31:0] bswap(input [31:0] x);
    bswap = {x[7:0], x[15:8], x[23:16], x[31:24]};
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

  save_state_controller ssc (
      .clk_74a(clk_74a),
      .clk_sys(clk_sys),

      .bridge_wr(bridge_wr),
      .bridge_rd(1'b0),
      .bridge_endian_little(1'b0),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),
      .save_state_bridge_read_data(),

      .savestate_load(savestate_load),
      .savestate_load_ack_s(),
      .savestate_load_busy_s(),
      .savestate_load_ok_s(),
      .savestate_load_err_s(),

      .savestate_start(1'b0),
      .savestate_start_ack_s(),
      .savestate_start_busy_s(),
      .savestate_start_ok_s(),
      .savestate_start_err_s(),

      .ss_save(),
      .ss_load(),

      .ss_din(64'h0),
      .ss_dout(),
      .ss_addr(17'h0),
      .ss_rnw(1'b1),
      .ss_req(1'b0),
      .ss_be(8'hFF),
      .ss_ack(),
      .ss_busy(1'b0),

      .ss_sdram_wr_req (ss_sdram_wr_req),
      .ss_sdram_wr_addr(ss_sdram_wr_addr),
      .ss_sdram_wr_data(ss_sdram_wr_data),
      .ss_sdram_wr_ack (ss_sdram_wr_ack),
      .ss_sdram_rd_req (ss_sdram_rd_req),
      .ss_sdram_rd_addr(ss_sdram_rd_addr),
      .ss_sdram_rd_data(ss_sdram_rd_data),
      .ss_sdram_rd_ack (ss_sdram_rd_ack),
      .ss_loading      (ss_loading),
      .ss_pause_cpu    (ss_pause_cpu)
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

  // APF holds bridge_wr HIGH for two clk_74a cycles per 32-bit word, with
  // bridge_wr_data valid only on the first (rising-edge) cycle and the bus
  // cleared to 0 on the second.  This is why the MiSTer data_loader.sv
  // edge-detects bridge_wr.  Model it faithfully so the TB reproduces the
  // hardware "every second word is zero" bug.
  task automatic bridge_write(input [31:0] addr, input [31:0] data);
    @(posedge clk_74a);
    bridge_addr    <= addr;
    bridge_wr_data <= data;
    bridge_wr      <= 1'b1;
    @(posedge clk_74a);
    bridge_wr_data <= 32'h0;   // APF drops data but holds strobe a 2nd cycle
    @(posedge clk_74a);
    bridge_wr      <= 1'b0;
  endtask

  initial begin : main
    int n, k;
    int unsigned lin;
    logic [15:0] got, exp;

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
    $display("[tb] chip total writes/reads : %0d / %0d",
             chip.total_writes, chip.total_reads);
    $display("[tb] writes by word class    : c0=%0d c1=%0d c2=%0d c3=%0d (expect %0d each)",
             chip.wr_count_by_class[0], chip.wr_count_by_class[1],
             chip.wr_count_by_class[2], chip.wr_count_by_class[3],
             NUM_BRIDGE_WORDS / 2);
    $display("[tb] probe_result_0..3       : %h %h %h %h",
             ssc.probe_result_0, ssc.probe_result_1,
             ssc.probe_result_2, ssc.probe_result_3);
    $display("[tb] v60 first_w2_data       : %h (expect %h)",
             dbg_real_writes_unused, expected_word(2));
    $display("[tb] v60 first_w2_addr_lo    : %h (expect 04)", dbg_w2_info[7:0]);
    $display("[tb] v62 dbg_w2_src (ctrl)   : %h (expect %h)",
             ssc.dbg_w2_src, expected_word(2));
    $display("[tb] v63 dbg_pop2 (raw FIFO) : %h (expect %h)",
             ssc.dbg_pop2, expected_word(2));
    $display("[tb] v64 dbg_wr1/wr2 (push)  : %h %h (expect %h %h)",
             ssc.dbg_wr1, ssc.dbg_wr2, bswap(vpat(0)) & 32'hFFFF,
             bswap(vpat(1)) & 32'hFFFF);

    // v64 write-side: 2nd word pushed into FIFO == bridge word 1's swapped lo.
    if (ssc.dbg_wr2 !== (bswap(vpat(1)) & 32'hFFFF)) begin
      errors++;
      $display("ERROR: dbg_wr2 = %h, expected %h",
               ssc.dbg_wr2, bswap(vpat(1)) & 32'hFFFF);
    end

    // v62 controller-side capture of stage_buffer[47:32] at word-2 WR_REQ.
    if (ssc.dbg_w2_src !== expected_word(2)) begin
      errors++;
      $display("ERROR: dbg_w2_src = %h, expected %h",
               ssc.dbg_w2_src, expected_word(2));
    end
    // v63 raw FIFO second-pop data feeding stage_buffer[63:32].
    if (ssc.dbg_pop2 !== expected_word(2)) begin
      errors++;
      $display("ERROR: dbg_pop2 = %h, expected %h",
               ssc.dbg_pop2, expected_word(2));
    end

    // v60 chip-boundary capture must match chunk0 word2.
    if (dbg_real_writes_unused !== expected_word(2)) begin
      errors++;
      $display("ERROR: first_w2_data = %h, expected %h",
               dbg_real_writes_unused, expected_word(2));
    end
    if (dbg_w2_info[7:0] !== 8'h04) begin
      errors++;
      $display("ERROR: first_w2_addr_lo = %h, expected 04", dbg_w2_info[7:0]);
    end

    // Per-class write counts — the saturated-counter evidence gap, resolved.
    for (k = 0; k < 4; k++) begin
      if (chip.wr_count_by_class[k] != NUM_BRIDGE_WORDS / 2) begin
        errors++;
        $display("ERROR: word class %0d got %0d chip writes, expected %0d",
                 k, chip.wr_count_by_class[k], NUM_BRIDGE_WORDS / 2);
      end
    end

    // Full content check of every staged word.
    begin
      int mism = 0;
      for (k = 0; k < NUM_BRIDGE_WORDS * 2; k++) begin
        lin = (STAGING_BASE >> 1) + k;
        exp = expected_word(k);
        got = chip.mem.exists(lin) ? chip.mem[lin] : 16'hDEAD;
        if (got !== exp) begin
          errors++;
          mism++;
          if (mism <= 16)
            $display("ERROR: staged word %0d (chunk %0d idx %0d, lin=%h): got %h expected %h",
                     k, k / 4, k % 4, lin, got, exp);
        end
      end
      if (mism > 16) $display("  ... and %0d more mismatches", mism - 16);
      $display("[tb] content check: %0d / %0d words wrong", mism,
               NUM_BRIDGE_WORDS * 2);
    end

    // Controller's own probe registers (chunk 0 words 0..3 via real reads).
    begin
      logic [15:0] pr[4];
      pr[0] = ssc.probe_result_0; pr[1] = ssc.probe_result_1;
      pr[2] = ssc.probe_result_2; pr[3] = ssc.probe_result_3;
      for (k = 0; k < 4; k++) begin
        if (pr[k] !== expected_word(k)) begin
          errors++;
          $display("ERROR: probe_result_%0d = %h, expected %h",
                   k, pr[k], expected_word(k));
        end
      end
    end

    if (ssc.fifo_load_drop_cnt != 0) begin
      errors++;
      $display("ERROR: load FIFO dropped %0d words (TB pacing too fast?)",
               ssc.fifo_load_drop_cnt);
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
