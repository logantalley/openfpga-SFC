// tb_ss_stream — full-stack testbench for the direct-stream savestate
// transport (save_state_stream.sv + SRAM ring) with the REAL savestates.sv
// engine on the (now ungated) system clock.
//
//   LOAD: an APF model writes the real .sta payload at the true bridge pace
//         (one 32-bit word per ~75 clk_74a cycles) WHILE the CPU model
//         consumes at DMA pace — walk armed on first data, ring occupancy
//         tracked (high-water mark must stay under 256 KB).
//   SAVE: the CPU model produces the same payload chunk-by-chunk with
//         STATUS polls (the firmware's real flow control); the APF model
//         starts reading at start_ok and the readout is byte-compared.
//
// CPU bus model carried over from tb_ss_serve (mirrors savestates.asm).
//
// Run headless:  vsim -c -do simulate_stream.do   (from src/fpga/)

`timescale 1ps / 1ps

// Behavioral async SRAM (IS62WV... style): combinational read, WE-pulse write.
module sram_chip_model_async (
    input  wire [16:0] a,
    inout  wire [15:0] dq,
    input  wire        oe_n,
    input  wire        we_n,
    input  wire        ub_n,
    input  wire        lb_n
);
  reg [15:0] mem[0:131071];
  assign dq = (~oe_n && we_n) ? mem[a] : 16'hzzzz;
  always @(posedge we_n) begin
    // Data/address captured at the rising WE edge (write completes).
    if (~ub_n) mem[a][15:8] <= dq[15:8];
    if (~lb_n) mem[a][7:0] <= dq[7:0];
  end
endmodule

module tb_ss_stream #(
    parameter int STA_CHUNKS     = 2048,  // 64-bit chunks round-tripped (16 KB)
    parameter int APF_WR_GAP     = 75,    // clk_74a cycles per bridge word (real pace)
    parameter int APF_RD_GAP     = 75,    // clk_74a cycles per bridge read
    parameter int RUN_SAVE       = 1,     // also exercise the save path
    parameter int WALK_START_DLY = 40000  // MCLK before the TB takes the NMI
                                          // (models hijack latency; stresses ring)
);
  localparam CLK74_PER = 13468;  // 74.25 MHz
  localparam CLKSYS_PER = 46560;  // 21.477 MHz

  reg clk_74a = 0;
  reg clk_sys = 0;
  always #(CLK74_PER / 2) clk_74a = ~clk_74a;
  always #(CLKSYS_PER / 2) clk_sys = ~clk_sys;

  wire mclk = clk_sys;  // direct-stream: engine on the raw system clock

  // -------------------------------------------------------------------
  // Bridge / APF stimulus
  // -------------------------------------------------------------------
  reg bridge_wr = 0;
  reg bridge_rd = 0;
  reg [31:0] bridge_addr = 0;
  reg [31:0] bridge_wr_data = 0;
  reg savestate_load = 0;
  reg savestate_start = 0;
  wire [31:0] save_state_bridge_read_data;

  wire savestate_load_ack, savestate_load_busy, savestate_load_ok, savestate_load_err;
  wire savestate_start_ack, savestate_start_busy, savestate_start_ok, savestate_start_err;

  // -------------------------------------------------------------------
  // Engine ↔ stream handshake
  // -------------------------------------------------------------------
  wire [63:0] ss_ddr_do;
  wire [63:0] ss_ddr_di;
  wire [16:0] ss_ddr_addr;
  wire ss_ddr_we;
  wire [7:0] ss_ddr_be;
  wire ss_ddr_req;
  wire ss_ddr_ack;
  wire ss_busy_w;
  wire ss_load_w;
  wire ss_save_w;

  // -------------------------------------------------------------------
  // DUT 1: save_state_stream + SRAM chip
  // -------------------------------------------------------------------
  wire [16:0] sram_a;
  wire [15:0] sram_dq;
  wire sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;

  save_state_stream dut (
      .clk_74a(clk_74a),
      .clk_sys(clk_sys),

      .bridge_wr(bridge_wr),
      .bridge_rd(bridge_rd),
      .bridge_endian_little(1'b0),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),
      .save_state_bridge_read_data(save_state_bridge_read_data),

      .savestate_load(savestate_load),
      .savestate_load_ack_s(savestate_load_ack),
      .savestate_load_busy_s(savestate_load_busy),
      .savestate_load_ok_s(savestate_load_ok),
      .savestate_load_err_s(savestate_load_err),

      .savestate_start(savestate_start),
      .savestate_start_ack_s(savestate_start_ack),
      .savestate_start_busy_s(savestate_start_busy),
      .savestate_start_ok_s(savestate_start_ok),
      .savestate_start_err_s(savestate_start_err),

      .ss_save(ss_save_w),
      .ss_load(ss_load_w),

      .ss_din(ss_ddr_do),
      .ss_dout(ss_ddr_di),
      .ss_addr(ss_ddr_addr),
      .ss_rnw(~ss_ddr_we),
      .ss_req(ss_ddr_req),
      .ss_be(ss_ddr_be),
      .ss_ack(ss_ddr_ack),

      .ss_busy(ss_busy_w),

      .sram_a(sram_a),
      .sram_dq(sram_dq),
      .sram_oe_n(sram_oe_n),
      .sram_we_n(sram_we_n),
      .sram_ub_n(sram_ub_n),
      .sram_lb_n(sram_lb_n)
  );

  sram_chip_model_async sram_chip (
      .a(sram_a),
      .dq(sram_dq),
      .oe_n(sram_oe_n),
      .we_n(sram_we_n),
      .ub_n(sram_ub_n),
      .lb_n(sram_lb_n)
  );

  // -------------------------------------------------------------------
  // DUT 2: the REAL savestates.sv engine
  // -------------------------------------------------------------------
  reg reset_n = 0;
  reg [23:0] ca = 24'h0;
  reg cpurd_n = 1;
  reg cpuwr_n = 1;
  reg [7:0] cpu_di_r = 8'h0;
  reg vblank_n = 1;

  reg [2:0] bus_phase = 3'd0;
  always @(posedge mclk) bus_phase <= bus_phase + 3'd1;
  wire sysclkr_ce = (bus_phase == 3'd1);
  wire sysclkf_ce = (bus_phase == 3'd5);

  wire [7:0] ss_do_w;
  wire ss_do_ovr_w;
  wire [15:0] fw_stall_cnt;

  savestates u_ss (
      .reset_n(reset_n),
      .clk(mclk),

      .save(ss_save_w),
      .load(ss_load_w),

      .ram_size(4'd0),
      .rom_type(8'd0),

      .sysclkf_ce(sysclkf_ce),
      .sysclkr_ce(sysclkr_ce),

      .romsel_n(1'b1),
      .rom_q(16'h0),

      .ca(ca),
      .cpurd_n(cpurd_n),
      .cpuwr_n(cpuwr_n),

      .pa(8'h0),
      .pard_n(1'b1),
      .pawr_n(1'b1),

      .di(cpu_di_r),
      .ss_do(ss_do_w),

      .rom_addr(),
      .ext_addr(),

      .spc_di(8'h0),

      .ddr_di(ss_ddr_di),
      .ddr_do(ss_ddr_do),
      .ddr_ack(ss_ddr_ack),
      .ddr_addr(ss_ddr_addr),
      .ddr_we(ss_ddr_we),
      .ddr_be(ss_ddr_be),
      .ddr_req(ss_ddr_req),

      .aram_sel(),
      .dsp_regs_sel(),
      .smp_regs_sel(),

      .ppu_di(8'h0),

      .bsram_sel(),
      .bsram_di(8'h0),

      .dspn_regs_sel(),
      .dspn_ram_sel(),
      .dspn_di(8'h0),

      .gsu_regs_sel(),
      .gsu_di(8'h0),

      .sa1_active(1'b0),
      .sa1_a(24'h0),
      .sa1_di(8'h0),
      .sa1_rd_n(1'b1),
      .sa1_wr_n(1'b1),
      .sa1_sa1_romsel(1'b1),
      .sa1_sns_romsel(1'b1),

      .vblank_n(vblank_n),

      .cpu_di(cpu_di_r),

      .ss_do_ovr(ss_do_ovr_w),
      .ss_rom_ovr(),
      .ss_busy(ss_busy_w),

      .dbg_rti_arms(),
      .dbg_vect_reentry(),
      .dbg_ddr_writes(),
      .dbg_save_end_writes(),
      .dbg_fw_entry(),
      .dbg_fw_nmidis(),
      .dbg_fw_at_8000(),
      .dbg_fw_at_8003(),
      .dbg_byte_at_8000(),
      .dbg_byte_at_8001(),
      .dbg_load_byte0(),
      .dbg_load_byte1(),
      .dbg_load_en_cnt(),
      .dbg_load_vect_cnt(),
      .dbg_load_busy_cnt(),
      .dbg_load_stall_cnt(fw_stall_cnt)
  );

  // -------------------------------------------------------------------
  // CPU bus model (from tb_ss_serve): one bus cycle = 8 MCLK = DMA pace
  // -------------------------------------------------------------------
  task automatic cpu_read(input [23:0] addr, output [7:0] data);
    @(posedge mclk);
    ca <= addr;
    @(posedge mclk);
    cpurd_n <= 1'b0;
    @(posedge mclk);
    @(posedge mclk);
    @(posedge mclk);
    #1 data = ss_do_w;
    cpurd_n <= 1'b1;
    @(posedge mclk);
    @(posedge mclk);
    @(posedge mclk);
  endtask

  task automatic cpu_write(input [23:0] addr, input [7:0] data);
    @(posedge mclk);
    ca <= addr;
    cpu_di_r <= data;
    @(posedge mclk);
    cpuwr_n <= 1'b0;
    repeat (4) @(posedge mclk);
    cpuwr_n <= 1'b1;
    repeat (2) @(posedge mclk);
  endtask

  // Firmware save data write: must overlap sysclkf_ce while cpuwr_n low
  // (savestates.sv data-write block requires ~cpuwr_n & sysclkf_ce).
  task automatic cpu_write_data(input [7:0] data);
    @(posedge mclk);
    ca <= 24'hC06000;
    cpu_di_r <= data;
    @(posedge mclk);
    cpuwr_n <= 1'b0;
    repeat (8) @(posedge mclk);  // spans one full bus cycle → one sysclkf_ce
    cpuwr_n <= 1'b1;
    repeat (2) @(posedge mclk);
  endtask

  // -------------------------------------------------------------------
  // Payload + expected data
  // -------------------------------------------------------------------
  logic [31:0] sta_words[0:131071];

  function automatic [7:0] sta_byte(input int i);
    sta_byte = sta_words[i/4] >> (8 * (3 - (i % 4)));
  endfunction

  int errors = 0;

  // Debug spy: first N interesting transitions on the serve chain
  int spy_left = 60;
  reg [2:0] prev_sys_state_spy = 0;
  reg [3:0] prev_sram_state_spy = 0;
  reg prev_req_spy = 0, prev_ack2_spy = 0, prev_bv_spy = 0;
  always @(posedge clk_sys) begin
    if (spy_left > 0) begin
      if (dut.sys_state !== prev_sys_state_spy) begin
        $display("[spy] sys_state %0d -> %0d  (req=%b ack=%b rnw=%b chunk=%0d) t=%0t",
                 prev_sys_state_spy, dut.sys_state, ss_ddr_req, ss_ddr_ack, ~ss_ddr_we,
                 ss_ddr_addr, $time);
        spy_left--;
      end
      prev_sys_state_spy <= dut.sys_state;
      if (ss_ddr_req !== prev_req_spy) begin
        $display("[spy] ddr_req toggle (we=%b addr=%0d) t=%0t", ss_ddr_we, ss_ddr_addr, $time);
        spy_left--;
      end
      prev_req_spy <= ss_ddr_req;
      if (ss_ddr_ack !== prev_ack2_spy) begin
        $display("[spy] ddr_ack toggle (dout=%h) t=%0t", ss_ddr_di, $time);
        spy_left--;
      end
      prev_ack2_spy <= ss_ddr_ack;
      if (u_ss.load_buf_valid !== prev_bv_spy) begin
        $display("[spy] load_buf_valid -> %b (buf=%h) t=%0t", u_ss.load_buf_valid, u_ss.load_buf,
                 $time);
        spy_left--;
      end
      prev_bv_spy <= u_ss.load_buf_valid;
    end
  end
  int spy74_left = 40;
  always @(posedge clk_74a) begin
    // Only the engine-read path (states 3,4) and pending/avail edges matter.
    if (spy74_left > 0 && dut.sram_state !== prev_sram_state_spy &&
        (dut.sram_state == 4'd3 || dut.sram_state == 4'd4 ||
         prev_sram_state_spy == 4'd3 || prev_sram_state_spy == 4'd4)) begin
      $display("[spy74] sram_state %0d -> %0d (rd_pend=%b avail=%b chunk=%0d words_in=%0d) t=%0t",
               prev_sram_state_spy, dut.sram_state, dut.core_rd_pending_74a, dut.rd_available,
               dut.core_rd_chunk, dut.load_words_in, $time);
      spy74_left--;
    end
    prev_sram_state_spy <= dut.sram_state;
  end
  reg prev_rd_pend_spy = 0;
  always @(posedge clk_74a) begin
    if (spy74_left > 0 && dut.core_rd_pending_74a !== prev_rd_pend_spy) begin
      $display("[spy74] rd_pending -> %b (avail=%b chunk=%0d needed=%0d words_in=%0d) t=%0t",
               dut.core_rd_pending_74a, dut.rd_available, dut.core_rd_chunk,
               dut.rd_words_needed, dut.load_words_in, $time);
      spy74_left--;
    end
    prev_rd_pend_spy <= dut.core_rd_pending_74a;
  end

  // Spy: start_ok rise — which path asserted it? Also dump SRAM[0..7] at that moment.
  reg prev_ok_spy = 0;
  always @(posedge clk_sys) begin
    if (dut.savestate_start_ok !== prev_ok_spy) begin
      $display("[spy] start_ok -> %b (sys=%0d busy=%b thr_s=%b words=%0d) t=%0t",
               dut.savestate_start_ok, dut.sys_state, ss_busy_w, dut.save_threshold_hit_s,
               dut.save_words_in, $time);
    end
    prev_ok_spy <= dut.savestate_start_ok;
  end

  // SRAM write commit spy: fires on the first few rising we_n edges during SAVE.
  // Gates on savestate_start_busy (clk_sys reg, safe to read here for diagnostics).
  int save_wr_spy_left = 24;
  reg prev_sram_we_n_spy = 1;
  always @(posedge clk_74a) begin
    if (sram_we_n && ~prev_sram_we_n_spy && dut.savestate_start_busy && save_wr_spy_left > 0) begin
      $display("[swr] we_n rise: a=%0d dq=%h (wr_data=%h lat=%h wrd1=%h) t=%0t",
               sram_a, sram_dq, dut.core_wr_data, dut.latched_core_wr_data,
               dut.core_wr_word_1, $time);
      save_wr_spy_left--;
    end
    prev_sram_we_n_spy <= sram_we_n;
  end
  // SRAM dump on start_ok rise (clk_74a so SRAM data is valid)
  reg prev_ok_74a_spy = 0;
  wire ok_74a_spy = dut.savestate_start_ok_s;
  always @(posedge clk_74a) begin
    if (ok_74a_spy && ~prev_ok_74a_spy) begin
      $display("[sram@ok] [0]=%h [1]=%h [2]=%h [3]=%h [4]=%h [5]=%h [6]=%h [7]=%h",
               sram_chip.mem[0], sram_chip.mem[1], sram_chip.mem[2], sram_chip.mem[3],
               sram_chip.mem[4], sram_chip.mem[5], sram_chip.mem[6], sram_chip.mem[7]);
    end
    prev_ok_74a_spy <= ok_74a_spy;
  end

  // Ring occupancy high-water mark (bytes in flight)
  int consumed_chunks = 0;
  int occ, occ_max = 0;
  reg prev_ack_spy = 0;
  always @(posedge clk_sys) begin
    if (ss_ddr_ack !== prev_ack_spy) begin
      prev_ack_spy <= ss_ddr_ack;
      consumed_chunks <= consumed_chunks + 1;
    end
    occ = (int'(dut.load_words_in) * 2) - (consumed_chunks * 8);
    if (occ > occ_max) occ_max = occ;
  end

  // -------------------------------------------------------------------
  // APF model: LOAD writer (runs concurrently with the CPU model)
  // -------------------------------------------------------------------
  task automatic apf_stream_load(input int nchunks);
    int n;
    for (n = 0; n < nchunks * 2; n++) begin
      @(posedge clk_74a);
      bridge_addr <= 32'h4000_0000 + 4 * n;
      bridge_wr_data <= sta_words[n];
      bridge_wr <= 1'b1;
      @(posedge clk_74a);
      bridge_wr <= 1'b0;
      repeat (APF_WR_GAP - 2) @(posedge clk_74a);
    end
    $display("[apf] load stream complete (%0d words), t=%0t", nchunks * 2, $time);
    // Command arrives after the data (Tamagotchi choreography)
    repeat (100) @(posedge clk_74a);
    savestate_load <= 1'b1;
    repeat (40) @(posedge clk_74a);
    savestate_load <= 1'b0;
  endtask

  // -------------------------------------------------------------------
  // CPU model: firmware LOAD walk (from tb_ss_serve, arming rework)
  // -------------------------------------------------------------------
  task automatic fw_load_walk(input int nchunks);
    int i, poll_cnt, total_bytes;
    logic [7:0] b;
    total_bytes = nchunks * 8;

    // Wait for the stream to arm the walk (first bridge word)
    begin : wait_arm
      int wd;
      wd = 0;
      while (u_ss.load_en !== 1'b1 && wd < 2000000) begin
        @(posedge clk_sys);
        wd++;
      end
      if (u_ss.load_en !== 1'b1) begin
        $display("*** STREAM FAIL — load_en never armed ***");
        $finish;
      end
    end
    $display("[tb] load_en armed at t=%0t", $time);

    // Model hijack latency (game runs until its next vblank NMI)
    repeat (WALK_START_DLY) @(posedge mclk);

    vblank_n = 1'b0;
    cpu_read(24'h00FFEA, b);
    if (b !== 8'h04) begin
      $display("ERROR: NMI vector low = %h, want 04", b);
      errors++;
    end
    cpu_read(24'h00FFEB, b);
    if (b !== 8'h80) begin
      $display("ERROR: NMI vector high = %h, want 80", b);
      errors++;
    end
    vblank_n = 1'b1;

    cpu_write(24'hC06001, 8'h00);  // SSADDR reset

    poll_cnt = 0;
    do begin
      cpu_read(24'hC0600F, b);
      poll_cnt++;
      if (poll_cnt > 200000) begin
        $display("*** STREAM FAIL — STATUS_BUSY never cleared ***");
        $finish;
      end
    end while (b & 8'h02);
    $display("[tb] STATUS ready after %0d polls; DMA readback of %0d bytes", poll_cnt,
             total_bytes);

    for (i = 0; i < total_bytes; i++) begin
      cpu_read(24'hC06000, b);
      if (b !== sta_byte(i)) begin
        errors++;
        if (errors <= 24)
          $display("ERROR load byte %0d: got %h want %h (occ=%0d)", i, b, sta_byte(i), occ);
      end
    end

    cpu_read(24'hC08008, b);  // deliberate RTI
    repeat (16) @(posedge mclk);
    if (u_ss.load_en !== 1'b0) begin
      $display("ERROR: load_en still set after RTI");
      errors++;
    end
  endtask

  // -------------------------------------------------------------------
  // CPU model: firmware SAVE walk (chunked writes + STATUS polls)
  // -------------------------------------------------------------------
  task automatic fw_save_walk(input int nchunks);
    int i, poll_cnt;
    logic [7:0] b;

    vblank_n = 1'b0;
    cpu_read(24'h00FFEA, b);
    if (b !== 8'h00) begin
      $display("ERROR: SAVE vector low = %h, want 00 ($8000)", b);
      errors++;
    end
    cpu_read(24'h00FFEB, b);
    vblank_n = 1'b1;

    cpu_write(24'hC06001, 8'h00);  // SSADDR reset

    for (i = 0; i < nchunks * 8; i++) begin
      cpu_write_data(sta_byte(i));
      if ((i % 8) == 7) begin
        // Firmware polls STATUS (ddr_busy) after each chunk
        poll_cnt = 0;
        do begin
          cpu_read(24'hC0600F, b);
          poll_cnt++;
          if (poll_cnt > 200000) begin
            $display("*** STREAM FAIL — save STATUS busy stuck at chunk %0d ***", i / 8);
            $finish;
          end
        end while (b & 8'h02);
      end
    end

    cpu_write(24'hC0600E, 8'hFF);  // SS_END
    cpu_read(24'hC08008, b);  // RTI
    repeat (16) @(posedge mclk);
  endtask

  // APF model: SAVE reader — starts at start_ok, reads at bridge pace
  task automatic apf_read_save(input int nchunks, output int rd_errors);
    int n;
    logic [31:0] got, want;
    rd_errors = 0;
    begin : wait_ok
      int wd;
      wd = 0;
      while (!savestate_start_ok && wd < 8000000) begin
        @(posedge clk_74a);
        wd++;
      end
      if (!savestate_start_ok) begin
        $display("*** STREAM FAIL — start_ok never asserted ***");
        $finish;
      end
    end
    $display("[apf] start_ok at t=%0t — reading %0d words", $time, nchunks * 2);

    for (n = 0; n < nchunks * 2; n++) begin
      @(posedge clk_74a);
      bridge_addr <= 32'h4000_0000 + 4 * n;
      bridge_rd <= 1'b1;
      repeat (3) @(posedge clk_74a);
      #1 got = save_state_bridge_read_data;
      bridge_rd <= 1'b0;
      want = sta_words[n];
      if (got !== want) begin
        rd_errors++;
        if (rd_errors <= 24) $display("ERROR save word %0d: got %h want %h", n, got, want);
      end
      repeat (APF_RD_GAP - 4) @(posedge clk_74a);
    end
  endtask

  // -------------------------------------------------------------------
  // Main
  // -------------------------------------------------------------------
  initial begin : main
    int save_errs;

    $readmemh("core/tb/smw_payload.hex", sta_words);
    $display("=== STREAM test: %0d chunks (%0d KB), APF pace %0d cyc/word ===", STA_CHUNKS,
             STA_CHUNKS * 8 / 1024, APF_WR_GAP);

    fork
      begin
        repeat (600) #1_000_000_000;  // 600 ms watchdog
        $display("WATCHDOG: sys=%0d sram=%0d busy=%b load_en=%b words_in=%0d occ=%0d",
                 dut.sys_state, dut.sram_state, ss_busy_w, u_ss.load_en, dut.load_words_in, occ);
        $display("*** STREAM FAIL — watchdog ***");
        $finish;
      end
    join_none

    #2_000_000 reset_n = 1;
    #2_000_000;

    // ================= LOAD: concurrent stream + walk =================
    fork
      apf_stream_load(STA_CHUNKS);
      fw_load_walk(STA_CHUNKS);
    join

    $display("[tb] LOAD done: %0d errors, ring high-water %0d bytes (%0d KB), stalls=%0d",
             errors, occ_max, occ_max / 1024, fw_stall_cnt);
    if (occ_max > 262144) begin
      $display("ERROR: ring high-water exceeds 256 KB SRAM");
      errors++;
    end

    // Wait for load ok (walk done + command seen)
    begin : wait_load_ok
      int wd;
      wd = 0;
      while (!savestate_load_ok && wd < 1000000) begin
        @(posedge clk_74a);
        wd++;
      end
      if (!savestate_load_ok) begin
        $display("ERROR: savestate_load_ok never asserted");
        errors++;
      end
    end

    // ================= SAVE: engine-paced stream out =================
    if (RUN_SAVE) begin
      repeat (2000) @(posedge clk_74a);
      savestate_start <= 1'b1;
      repeat (40) @(posedge clk_74a);
      savestate_start <= 1'b0;

      fork
        fw_save_walk(STA_CHUNKS);
        apf_read_save(STA_CHUNKS, save_errs);
      join
      errors += save_errs;
      $display("[tb] SAVE done: %0d word errors", save_errs);
    end

    if (errors == 0)
      $display("*** STREAM PASS — %0d chunks round-trip byte-exact through the SRAM ring ***",
               STA_CHUNKS);
    else
      $display("*** STREAM FAIL — %0d errors ***", errors);
    $finish;
  end

endmodule
