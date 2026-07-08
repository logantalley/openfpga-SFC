// tb_ss_serve — full-stack, self-checking testbench for the savestate LOAD
// SERVE path, including the REAL savestates.sv engine on the gated MCLK:
//
//   APF bridge_wr stream → save_state_controller staging (PSRAM bank 1)
//   → SYS_SERVE FSM → ss_dout/ss_ack → savestates.sv (ddr_di/ddr_ack,
//   MCLK = clk_sys & clk_sys_en) → load_buf prefetch/swap → ss_do
//   → CPU bus model reading SSDATA at DMA pace.
//
// This covers the layer the staging TB stops short of: the engine's
// load_buf double-buffer, the byte-7 swap race against DMA pacing, and the
// walk completion (RTI) handshake.  The CPU model mirrors savestates.asm
// Load_start exactly: ONE STATUS_BUSY poll before the header, then the
// entire payload read back-to-back with NO flow control (SetupDMA + $420B),
// one byte per 8 MCLK.  If the chunk prefetch ever loses that 64-cycle
// race, ss_do serves stale load_buf data and the byte compare fails —
// the mid-stream "drift" failure shape.
//
// Mirrors the silicon overlay probes: computes the same rotl-xor stream
// checksum as chk_load (chunk 0 excluded) and cross-checks the DUT's
// register, so a hardware overlay reading can be compared 1:1 against sim.
//
// Adapted patterns (behavioral model discipline, fake-engine pacing) from
// timothy redaelli's openFPGA-SNES save_state_mem_tb / save_state_controller_tb.
//
// Run headless:  vsim -c -do simulate_serve.do   (from src/fpga/)

`timescale 1ps / 1ps

module tb_ss_serve #(
    parameter int BRIDGE_GAP_CYC  = 60,     // clk_74a cycles between bridge words
    parameter int CLKMEM_PHASE_PS = 3777,   // clk_mem initial phase offset
    parameter int STA_CHUNKS      = 512,    // # 64-bit chunks to round-trip (4 KB)
    parameter int ARAM_TRAFFIC    = 1,      // Port A (ARAM) traffic during serve
    parameter int SPURIOUS_RD     = 0       // emit a 2nd read strobe per bus cycle
                                            // (models silicon 68ab774: ~2.3 strobes
                                            // per consumed byte; the bus-cycle gate
                                            // must absorb it byte-exactly)
);
  // Clock periods (ps).  clk_mem deliberately not an exact multiple of
  // clk_sys and phase-offset, so CDC alignment sweeps (same as staging TB).
  localparam CLK74_PER  = 13468;   // 74.25 MHz
  localparam CLKSYS_PER = 46560;   // 21.477 MHz
  localparam CLKMEM_PER = 11630;   // ~85.99 MHz

  reg clk_74a = 0;
  reg clk_sys = 0;
  reg clk_mem = 0;

  always #(CLK74_PER  / 2) clk_74a = ~clk_74a;
  always #(CLKSYS_PER / 2) clk_sys = ~clk_sys;
  initial begin
    #(CLKMEM_PHASE_PS);
    forever #(CLKMEM_PER / 2) clk_mem = ~clk_mem;
  end

  // Gated MCLK — replica of SNES.sv: clk_sys_en <= ~ss_pause_any, and the
  // engine + CPU run on clk_sys & clk_sys_en.  The CPU model runs on mclk so
  // it freezes through staging exactly like the real console.
  wire ss_pause_cpu;
  reg  clk_sys_en = 1'b1;
  always @(posedge clk_sys) clk_sys_en <= ~ss_pause_cpu;
  wire mclk = clk_sys & clk_sys_en;

  // -------------------------------------------------------------------
  // Bridge / APF stimulus
  // -------------------------------------------------------------------
  reg         bridge_wr      = 0;
  reg         bridge_rd      = 0;
  reg  [31:0] bridge_addr    = 32'h0;
  reg  [31:0] bridge_wr_data = 32'h0;
  reg         savestate_load = 0;
  wire [31:0] save_state_bridge_read_data;

  // -------------------------------------------------------------------
  // Engine ↔ controller handshake (REAL savestates.sv drives this side)
  // -------------------------------------------------------------------
  wire [63:0] ss_ddr_do;    // engine → controller (save data; unused in load)
  wire [63:0] ss_ddr_di;    // controller → engine (serve data)
  wire [16:0] ss_ddr_addr;
  wire        ss_ddr_we;
  wire [7:0]  ss_ddr_be;
  wire        ss_ddr_req;
  wire        ss_ddr_ack;
  wire        ss_busy_w;
  wire        ss_load_w;
  wire        ss_save_w;

  // -------------------------------------------------------------------
  // DUT 1: save_state_controller
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

  wire        ss_psram_wr_req;
  wire [18:0] ss_psram_wr_addr;
  wire [63:0] ss_psram_wr_data;
  wire        ss_psram_wr_ack;
  wire        ss_psram_rd_req;
  wire [18:0] ss_psram_rd_addr;
  wire [63:0] ss_psram_rd_data;
  wire        ss_psram_rd_ack;

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

      .savestate_start(1'b0),
      .savestate_start_ack_s(),
      .savestate_start_busy_s(),
      .savestate_start_ok_s(),
      .savestate_start_err_s(),

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
  // SDRAM stack (controller probe reads need it) — same as staging TB
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

  wire [15:0] dram_dq;
  wire [12:0] dram_a;
  wire [1:0]  dram_ba;
  wire        dram_dqml, dram_dqmh;
  wire        dram_ncs, dram_nwe, dram_nras, dram_ncas;
  wire        dram_clk, dram_cke;

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

      .dbg_real_writes(),
      .dbg_w2_info    (),

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
  // PSRAM stack — same as staging TB, with the Port A (ARAM) traffic
  // model active during serve when ARAM_TRAFFIC=1 (audio keeps running
  // during a real load, so the arbiter must interleave).
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
  wire        ss_psram_b_grant;

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
      .b_busy           (ss_psram_b_busy),
      .b_grant          (ss_psram_b_grant)
  );

  wire [21:16] cram1_a;
  wire [15:0]  cram1_dq;
  wire         cram1_wait;
  wire         cram1_clk, cram1_adv_n, cram1_cre;
  wire         cram1_ce0_n, cram1_ce1_n, cram1_oe_n, cram1_we_n;
  wire         cram1_ub_n, cram1_lb_n;

  // Port A (ARAM) traffic model — single-cycle request pulses with the
  // address invalidated immediately after the edge (from the staging TB).
  reg        a_traffic_active = 1'b0;
  reg [6:0]  a_traffic_cnt = 7'd0;
  reg [15:0] a_seq = 16'h1000;
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
  wire [21:0] a_addr_wire = a_pulse_read_r ? {6'b0, a_addr_at_edge}
                                           : 22'h3F_FFFF;

  psram_arbiter #(
      .CLOCK_SPEED(85.9)
  ) cram1_arb (
      .clk(clk_mem),

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
      .b_grant(ss_psram_b_grant),

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
  // DUT 2: the REAL savestates.sv engine on the gated MCLK
  // -------------------------------------------------------------------
  reg         reset_n  = 0;
  reg  [23:0] ca       = 24'h0;
  reg         cpurd_n  = 1;
  reg         cpuwr_n  = 1;
  reg  [7:0]  cpu_di_r = 8'h0;
  reg         vblank_n = 1;

  // Free-running SNES bus-phase CEs: one bus cycle = 8 MCLK (≈2.68 MHz,
  // the DMA byte rate).  sysclkr early in the cycle, sysclkf late.
  // SPURIOUS_RD stretches the cycle to 16 MCLK so two read strobes fit
  // cleanly inside one sysclkf window.
  localparam [3:0] BUS_LAST  = SPURIOUS_RD ? 4'd15 : 4'd7;
  localparam [3:0] CLKF_AT   = SPURIOUS_RD ? 4'd13 : 4'd5;
  reg [3:0] bus_phase = 4'd0;
  always @(posedge mclk) bus_phase <= (bus_phase == BUS_LAST) ? 4'd0 : bus_phase + 4'd1;
  wire sysclkr_ce = (bus_phase == 4'd1);
  wire sysclkf_ce = (bus_phase == CLKF_AT);

  wire [7:0]  ss_do_w;
  wire        ss_do_ovr_w;
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
  // CPU bus model — one bus cycle per task call = 8 MCLK = DMA byte pace
  // -------------------------------------------------------------------
  task automatic cpu_read(input [23:0] addr, output [7:0] data);
    if (SPURIOUS_RD) begin
      // Align to the bus cycle so both strobes land in ONE sysclkf window
      // (that is the silicon pattern the gate is designed for).
      @(posedge mclk);
      while (bus_phase != 4'd0) @(posedge mclk);
      ca <= addr;
      @(posedge mclk);                     // 1
      @(posedge mclk); cpurd_n <= 1'b0;    // low 2..4
      @(posedge mclk);
      @(posedge mclk); #1 data = ss_do_w;  // sample at 4 (first strobe = real)
      cpurd_n <= 1'b1;                     // high at 5
      @(posedge mclk);
      @(posedge mclk); cpurd_n <= 1'b0;    // spurious repeat, low 7..8
      @(posedge mclk);
      @(posedge mclk); cpurd_n <= 1'b1;    // high at 9, well before CLKF at 13
      @(posedge mclk);
    end else begin
      @(posedge mclk); ca <= addr;
      @(posedge mclk); cpurd_n <= 1'b0;   // cpurd_ce fires next MCLK
      @(posedge mclk);
      @(posedge mclk);
      @(posedge mclk); #1 data = ss_do_w; // sample late in the low window
      cpurd_n <= 1'b1;                    // cpurd_ce_n fires next MCLK
      @(posedge mclk);
      @(posedge mclk);
      @(posedge mclk);
    end
  endtask

  task automatic cpu_write(input [23:0] addr, input [7:0] data);
    @(posedge mclk); ca <= addr; cpu_di_r <= data;
    @(posedge mclk); cpuwr_n <= 1'b0;
    repeat (4) @(posedge mclk);
    cpuwr_n <= 1'b1;
    repeat (2) @(posedge mclk);
  endtask

  // -------------------------------------------------------------------
  // Stimulus / expected data
  // -------------------------------------------------------------------
  task automatic bridge_write(input [31:0] addr, input [31:0] data);
    @(posedge clk_74a);
    bridge_addr    <= addr;
    bridge_wr_data <= data;
    bridge_wr      <= 1'b1;
    @(posedge clk_74a);
    bridge_wr      <= 1'b0;
  endtask

  // Real .sta payload: sta_words[n] = file bytes [4n..4n+3], big-endian.
  logic [31:0] sta_words [0:131071];

  function automatic [7:0] sta_byte(input int i);
    sta_byte = sta_words[i/4] >> (8 * (3 - (i % 4)));
  endfunction

  int errors = 0;

  initial begin : main
    int n, i, total_bytes, poll_cnt;
    logic [7:0] b;
    logic [7:0] chk_tb;

    $readmemh("core/tb/smw_payload.hex", sta_words);
    total_bytes = STA_CHUNKS * 8;
    $display("=== SERVE test: %0d chunks (%0d KB) through the real engine ===",
             STA_CHUNKS, STA_CHUNKS * 8 / 1024);

    // Global watchdog: report state instead of hanging.
    fork begin
      repeat (400) #1_000_000_000;  // 400 ms sim time
      $display("WATCHDOG: hung. sys_state=%0d ss_busy=%b load_en=%b buf_valid=%b pf_ready=%b ddr_state=%0d stalls=%0d",
               ssc.sys_state, ss_busy_w, u_ss.load_en, u_ss.load_buf_valid,
               u_ss.load_pf_ready, u_ss.ddr_state, fw_stall_cnt);
      $display("*** SERVE FAIL — watchdog ***");
      $finish;
    end join_none

    #2_000_000 reset_n = 1;
    #5_000_000;  // mem init settle

    // ---- Phase 1: stage the payload (CPU freezes under ss_pause_cpu) ----
    for (n = 0; n < STA_CHUNKS * 2; n++) begin
      bridge_write(32'h4000_0000 + 4 * n, sta_words[n]);
      repeat (BRIDGE_GAP_CYC) @(posedge clk_74a);
      if (n == 4) savestate_load <= 1'b1;
    end
    $display("[tb] payload staged, t=%0t", $time);
    #20_000_000;
    savestate_load <= 1'b0;

    // ---- Phase 2: wait for the controller to kick ss_load ----
    begin : wait_kick
      int wd;
      wd = 0;
      while (u_ss.load_en !== 1'b1 && wd < 4000000) begin
        @(posedge clk_sys); wd++;
      end
      if (u_ss.load_en !== 1'b1) begin
        $display("*** SERVE FAIL — load_en never set (sys_state=%0d) ***", ssc.sys_state);
        $finish;
      end
    end
    $display("[tb] load_en set; entering NMI vector hijack, t=%0t", $time);

    if (ARAM_TRAFFIC) a_traffic_active = 1'b1;

    // ---- Phase 3: NMI vector hijack (vblank) → firmware entry ----
    vblank_n = 1'b0;
    cpu_read(24'h00FFEA, b);
    if (b !== 8'h04) begin
      $display("ERROR: NMI vector low byte = %h, want 04 (load entry $8004)", b);
      errors++;
    end
    cpu_read(24'h00FFEB, b);
    if (b !== 8'h80) begin
      $display("ERROR: NMI vector high byte = %h, want 80", b);
      errors++;
    end
    if (ss_busy_w !== 1'b1) begin
      $display("ERROR: ss_busy not set after vector hijack");
      errors++;
    end
    vblank_n = 1'b1;

    // ---- Phase 4: firmware model (mirrors savestates.asm Load_start) ----
    // sta SSADDR — reset stream address, kicks the first chunk prefetch.
    cpu_write(24'hC06001, 8'h00);

    // Single STATUS_BUSY poll before the header (the ONLY poll in the asm).
    poll_cnt = 0;
    do begin
      cpu_read(24'hC0600F, b);
      poll_cnt++;
      if (poll_cnt > 100000) begin
        $display("*** SERVE FAIL — STATUS_BUSY never cleared ***");
        $finish;
      end
    end while (b & 8'h02);
    $display("[tb] STATUS ready after %0d polls; DMA readback of %0d bytes...",
             poll_cnt, total_bytes);

    // Whole stream at DMA pace, no flow control (header + payload).
    // Mirror the silicon chk_load: rotl-xor over bytes with addr[19:3] != 0.
    chk_tb = 8'h00;
    for (i = 0; i < total_bytes; i++) begin
      cpu_read(24'hC06000, b);
      if (b !== sta_byte(i)) begin
        errors++;
        if (errors <= 24)
          $display("ERROR byte %0d (chunk %0d, lane %0d): got %h want %h (stalls so far=%0d)",
                   i, i / 8, i % 8, b, sta_byte(i), fw_stall_cnt);
      end
      if (i >= 8) chk_tb = {chk_tb[6:0], chk_tb[7]} ^ b;
    end

    // ---- Phase 5: RTI handshake → walk completion ----
    cpu_read(24'hC08008, b);
    repeat (16) @(posedge mclk);
    if (u_ss.load_en !== 1'b0) begin
      $display("ERROR: load_en still set after RTI read");
      errors++;
    end
    if (ss_busy_w !== 1'b0) begin
      $display("ERROR: ss_busy still set after RTI read");
      errors++;
    end

    // Cross-check the DUT's silicon-overlay checksum against the TB's.
    if (u_ss.chk_load !== chk_tb) begin
      $display("ERROR: DUT chk_load=%h != TB chk=%h (probe hook mismatch)",
               u_ss.chk_load, chk_tb);
      errors++;
    end

    // Strobe census: raw strobes stay counted ungated (SPURIOUS_RD doubles
    // them); the byte-exact compare above proves the bus-cycle gate advanced
    // the stream exactly once per byte regardless.
    if (u_ss.cnt_load_rd !== total_bytes * (SPURIOUS_RD ? 2 : 1)) begin
      $display("ERROR: cnt_load_rd=%0d != %0d (strobe/byte ratio %f)",
               u_ss.cnt_load_rd, total_bytes * (SPURIOUS_RD ? 2 : 1),
               real'(u_ss.cnt_load_rd) / total_bytes);
      errors++;
    end
    if (u_ss.cnt_load_wr !== 8'd0) begin
      $display("ERROR: cnt_load_wr=%0d != 0 (write strobes during load DMA)",
               u_ss.cnt_load_wr);
      errors++;
    end

    $display("[tb] done: %0d byte errors, %0d prefetch stalls, chk=%h, t=%0t",
             errors, fw_stall_cnt, chk_tb, $time);
    if (fw_stall_cnt != 0 && errors == 0)
      $display("NOTE: %0d stalls with zero byte errors — stall counter fires without data loss?",
               fw_stall_cnt);

    if (errors == 0)
      $display("*** SERVE PASS — %0d bytes byte-exact through the real engine at DMA pace ***",
               total_bytes);
    else
      $display("*** SERVE FAIL — %0d errors ***", errors);
    $finish;
  end

endmodule
