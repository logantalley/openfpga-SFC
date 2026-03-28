module MAIN_SNES (
    input wire clk_mem_85_9,
    input wire clk_sys_21_48,

    input wire core_reset,

    input [64:0] rtc,

    // Settings
    input wire cpu_turbo_enabled,
    input wire gsu_turbo_enabled,

    input wire multitap_enabled,
    input wire lightgun_enabled,
    input wire lightgun_type,
    input wire [7:0] dpad_aim_speed,
    input wire mouse_enabled,

    input wire blend_enabled,

    // Inputs
    input wire p1_button_a,
    input wire p1_button_b,
    input wire p1_button_x,
    input wire p1_button_y,
    input wire p1_button_trig_l,
    input wire p1_button_trig_r,
    input wire p1_button_start,
    input wire p1_button_select,
    input wire p1_dpad_up,
    input wire p1_dpad_down,
    input wire p1_dpad_left,
    input wire p1_dpad_right,

    input wire [7:0] p1_lstick_x,
    input wire [7:0] p1_lstick_y,

    input wire p2_button_a,
    input wire p2_button_b,
    input wire p2_button_x,
    input wire p2_button_y,
    input wire p2_button_trig_l,
    input wire p2_button_trig_r,
    input wire p2_button_start,
    input wire p2_button_select,
    input wire p2_dpad_up,
    input wire p2_dpad_down,
    input wire p2_dpad_left,
    input wire p2_dpad_right,

    input wire p3_button_a,
    input wire p3_button_b,
    input wire p3_button_x,
    input wire p3_button_y,
    input wire p3_button_trig_l,
    input wire p3_button_trig_r,
    input wire p3_button_start,
    input wire p3_button_select,
    input wire p3_dpad_up,
    input wire p3_dpad_down,
    input wire p3_dpad_left,
    input wire p3_dpad_right,

    input wire p4_button_a,
    input wire p4_button_b,
    input wire p4_button_x,
    input wire p4_button_y,
    input wire p4_button_trig_l,
    input wire p4_button_trig_r,
    input wire p4_button_start,
    input wire p4_button_select,
    input wire p4_dpad_up,
    input wire p4_dpad_down,
    input wire p4_dpad_left,
    input wire p4_dpad_right,

    // ROM loading
    input wire ioctl_download,
    input wire ioctl_wr,
    input wire [24:0] ioctl_addr,
    input wire [15:0] ioctl_dout,

    input wire [7:0] rom_type,
    input wire [3:0] rom_size,
    input wire [3:0] ram_size,
    input wire PAL,

    // Saves
    input wire save_download,
    input wire sd_rd,
    input wire sd_wr,
    input wire [16:0] sd_buff_addr,
    output wire [15:0] sd_buff_din,
    input wire [15:0] sd_buff_dout,

    output reg [3:0] sram_size,

    // SDRAM
    output wire [12:0] dram_a,
    output wire [ 1:0] dram_ba,
    inout  wire [15:0] dram_dq,
    output wire [ 1:0] dram_dqm,
    output wire        dram_clk,
    output wire        dram_cke,
    output wire        dram_ras_n,
    output wire        dram_cas_n,
    output wire        dram_we_n,

    // PSRAM
    output wire [21:16] cram0_a,
    inout wire [15:0] cram0_dq,
    input wire cram0_wait,
    output wire cram0_clk,
    output wire cram0_adv_n,
    output wire cram0_cre,
    output wire cram0_ce0_n,
    output wire cram0_ce1_n,
    output wire cram0_oe_n,
    output wire cram0_we_n,
    output wire cram0_ub_n,
    output wire cram0_lb_n,

    output wire [21:16] cram1_a,
    inout wire [15:0] cram1_dq,
    input wire cram1_wait,
    output wire cram1_clk,
    output wire cram1_adv_n,
    output wire cram1_cre,
    output wire cram1_ce0_n,
    output wire cram1_ce1_n,
    output wire cram1_oe_n,
    output wire cram1_we_n,
    output wire cram1_ub_n,
    output wire cram1_lb_n,

    // Video
    output wire vblank,
    output wire hblank,
    output wire vsync,
    output wire hsync,

    output wire [7:0] video_r,
    output wire [7:0] video_g,
    output wire [7:0] video_b,

    // Audio
    output wire [15:0] audio_l,
    output wire [15:0] audio_r,

    // ---------- Save state ports (NEW) ----------
    input  wire        clk_74a,
    input  wire        bram_wr,
    input  wire [19:0] bram_wr_addr,
    input  wire  [7:0] bram_wr_data,
    output wire  [7:0] bram_rd_data,
    input  wire [19:0] bram_rd_addr,
    input  wire        ss_save,
    input  wire        ss_load,
    output wire        ss_busy
    // --------------------------------------------
);
  parameter USE_CX4 = 1'b0;
  parameter USE_SDD1 = 1'b0;
  parameter USE_GSU = 1'b0;
  parameter USE_SA1 = 1'b0;
  parameter USE_DSPn = 1'b0;
  parameter USE_SPC7110 = 1'b0;
  parameter USE_BSX = 1'b0;
  parameter USE_MSU = 1'b0;

  initial begin
    $info("Selected chips");
    $info("CX4 %d", USE_CX4);
    $info("SDD1 %d", USE_SDD1);
    $info("GSU %d", USE_GSU);
    $info("SA1 %d", USE_SA1);
    $info("DSPn %d", USE_DSPn);
    $info("SPC7110 %d", USE_SPC7110);
    $info("BSX %d", USE_BSX);
    $info("MSU %d", USE_MSU);
  end

  // Hardcoded wires
  wire [63:0] status = 0;
  wire [5:0] ioctl_index = 0;  // TODO
  wire GUN_BTN = status[27];
  wire [1:0] GUN_MODE = lightgun_enabled ? 2'd1 : 0;
  wire [1:0] mouse_mode = status[6:5];
  wire joy_swap = status[7] | piano;

  wire [6:0] USER_IN = 0;
  wire [6:0] USER_OUT;

  reg [128:0] gg_code = 0;
  wire gg_available;

  wire OSD_STATUS = 0;

  wire piano = 0;
  wire piano_joypad_do = 0;

  // Renamed wires
  wire clk_sys = clk_sys_21_48;
  wire clk_mem = clk_mem_85_9;

  wire code_index = &ioctl_index;
  wire code_download = ioctl_download & code_index;
  wire cart_download = ioctl_download & ioctl_index[5:0] == 0;
  // wire spc_download = ioctl_download & ioctl_index[5:0] == 6'h01;
  wire spc_download = 0;

  reg new_vmode;
  always @(posedge clk_sys) begin
    reg old_pal;
    int to;

    if (~reset) begin
      old_pal <= PAL;
      if (old_pal != PAL) to <= 2000000;
    end

    if (to) begin
      to <= to - 1;
      if (to == 1) new_vmode <= ~new_vmode;
    end
  end

  //////////////////////////  ROM DETECT  /////////////////////////////////

  reg [23:0] rom_mask, ram_mask;

  reg prev_cart_download;
  reg [2:0] parser_delay = 7;

  always @(posedge clk_sys) begin
    prev_cart_download <= cart_download;
    if (prev_cart_download && ~cart_download) begin
      parser_delay <= 6;
    end

    if (parser_delay != 7 && parser_delay != 0) begin
      parser_delay <= parser_delay - 1;
    end

    if (parser_delay == 1) begin
      // Set up masks before core starts
      rom_mask  <= (24'd1024 << rom_size) - 1'd1;
      ram_mask  <= ram_size > 0 ? (24'd1024 << ram_size) - 1'd1 : 24'd0;

      sram_size <= ram_size;
    end
  end

  reg spc_mode = 0;
  always @(posedge clk_sys) begin
    if (ioctl_wr) begin
      spc_mode <= spc_download;
    end
  end

  ////////////////////////////  SYSTEM  ///////////////////////////////////

  wire turbo_allow;

  reg [15:0] main_audio_l;
  reg [15:0] main_audio_r;

  wire vblank_n;
  wire hblank_n;
  wire dotclk;

  assign vblank = ~vblank_n;
  assign hblank = ~hblank_n;

  wire [7:0] R;
  wire [7:0] G;
  wire [7:0] B;

  // ---------- Save state internal wires (NEW) ----------
  // Signals savestates needs from the main CPU bus
  wire [23:0] SS_CA;
  wire        SS_CPURD_N;
  wire        SS_CPUWR_N;
  wire  [7:0] SS_PA;
  wire        SS_PARD_N;
  wire        SS_PAWR_N;
  wire  [7:0] SS_DO;        // CPU data-out (what CPU is writing)
  wire        SS_ROMSEL_N;
  wire        SS_SYSCLKF_CE;
  wire        SS_SYSCLKR_CE;

  // Outputs from savestates back into the bus
  wire  [7:0] ss_do_out;
  wire        ss_do_ovr;
  wire [23:0] ss_rom_addr;
  wire        ss_rom_ovr;
  wire [19:0] ss_ext_addr;

  // ROM bus with save-state address/data override applied
  wire [23:0] ROM_ADDR;
  wire        ROM_OE_N;
  wire        ROM_WE_N;
  wire        ROM_WORD;
  wire [15:0] ROM_D;
  wire [15:0] ROM_Q_raw;   // raw data back from SDRAM

  // When savestates is driving the CPU, replace ROM_Q low byte with ss_do
  wire [15:0] ROM_Q = ss_do_ovr ? {ROM_Q_raw[15:8], ss_do_out} : ROM_Q_raw;

  // When savestates needs to redirect ROM access (to savestates.bin at $FF:8000)
  wire [23:0] sdram_addr = cart_download ? {1'b0, ioctl_addr}
                         : ss_rom_ovr    ? ss_rom_addr
                         :                ROM_ADDR;
  // -----------------------------------------------------

  main #(
      .USE_CX4(USE_CX4),
      .USE_SDD1(USE_SDD1),
      .USE_GSU(USE_GSU),
      .USE_SA1(USE_SA1),
      .USE_DSPn(USE_DSPn),
      .USE_SPC7110(USE_SPC7110),
      .USE_BSX(USE_BSX),
      .USE_MSU(USE_MSU)
  ) main (
      .RESET_N(RESET_N),

      .MCLK(clk_sys),  // 21.47727 / 21.28137
      .ACLK(clk_sys),

      // .GSU_ACTIVE(GSU_ACTIVE),
      .GSU_TURBO(gsu_turbo_enabled),

      .ROM_TYPE(rom_type),
      .ROM_MASK(rom_mask),
      .RAM_MASK(ram_mask),
      .PAL(PAL),
      .BLEND(blend_enabled),

      .ROM_ADDR(ROM_ADDR),
      .ROM_D(ROM_D),
      .ROM_Q(ROM_Q),       // <-- now carries ss_do override when active
      .ROM_OE_N(ROM_OE_N),
      .ROM_WE_N(ROM_WE_N),
      .ROM_WORD(ROM_WORD),

      .BSRAM_ADDR(BSRAM_ADDR),
      .BSRAM_D(BSRAM_D),
      .BSRAM_Q(BSRAM_Q),
      .BSRAM_CE_N(BSRAM_CE_N),
      .BSRAM_OE_N(BSRAM_OE_N),
      .BSRAM_WE_N(BSRAM_WE_N),

      .WRAM_ADDR(WRAM_ADDR),
      .WRAM_D(WRAM_D),
      .WRAM_Q(WRAM_Q),
      .WRAM_CE_N(WRAM_CE_N),
      .WRAM_OE_N(WRAM_OE_N),
      .WRAM_WE_N(WRAM_WE_N),

      .VRAM1_ADDR(VRAM1_ADDR),
      .VRAM1_DI  (VRAM1_Q),
      .VRAM1_DO  (VRAM1_D),
      .VRAM1_WE_N(VRAM1_WE_N),

      .VRAM2_ADDR(VRAM2_ADDR),
      .VRAM2_DI  (VRAM2_Q),
      .VRAM2_DO  (VRAM2_D),
      .VRAM2_WE_N(VRAM2_WE_N),

      .ARAM_ADDR(ARAM_ADDR),
      .ARAM_D(ARAM_D),
      .ARAM_Q(ARAM_Q),
      .ARAM_CE_N(ARAM_CE_N),
      .ARAM_OE_N(ARAM_OE_N),
      .ARAM_WE_N(ARAM_WE_N),

      .R(R),
      .G(G),
      .B(B),

      // .FIELD(FIELD), // TODO
      // .INTERLACE(INTERLACE),
      // .HIGH_RES(HIGH_RES),
      .DOTCLK(dotclk),

      .HBLANKn(hblank_n),
      .VBLANKn(vblank_n),
      .HSYNC  (hsync),
      .VSYNC  (vsync),

      .JOY1_DI(JOY1_DI),
      .JOY2_DI(GUN_MODE ? LG_DO : JOY2_DI),
      .JOY_STRB(JOY_STRB),
      .JOY1_CLK(JOY1_CLK),
      .JOY2_CLK(JOY2_CLK),
      .JOY1_P6(JOY1_P6),
      .JOY2_P6(JOY2_P6),
      .JOY2_P6_in(JOY2_P6_DI),

      .EXT_RTC(rtc),

      .GG_EN(status[24]),
      .GG_CODE(gg_code),
      .GG_RESET((code_download && ioctl_wr && !ioctl_addr) || cart_download),
      .GG_AVAILABLE(gg_available),

      .SPC_MODE(spc_mode),

      .IO_ADDR(ioctl_addr[16:0]),
      .IO_DAT (ioctl_dout),
      .IO_WR  (spc_download & ioctl_wr),

      .TURBO(cpu_turbo_enabled & turbo_allow),
      .TURBO_ALLOW(turbo_allow),

`ifdef DEBUG_BUILD
      .DBG_BG_EN (DBG_BG_EN),
      .DBG_CPU_EN(DBG_CPU_EN),
`else
      .DBG_BG_EN (5'b11111),
      .DBG_CPU_EN(1'b1),
`endif

      .MSU_ENABLE(0),  // TODO

      .AUDIO_L(audio_l),
      .AUDIO_R(audio_r),

      // ---------- CPU bus outputs for savestates (NEW) ----------
      .CA         (SS_CA),
      .CPURD_N    (SS_CPURD_N),
      .CPUWR_N    (SS_CPUWR_N),
      .PA         (SS_PA),
      .PARD_N     (SS_PARD_N),
      .PAWR_N     (SS_PAWR_N),
      .DO         (SS_DO),
      .ROMSEL_N   (SS_ROMSEL_N),
      .SYSCLKF_CE (SS_SYSCLKF_CE),
      .SYSCLKR_CE (SS_SYSCLKR_CE)
      // ----------------------------------------------------------
  );

  wire reset = core_reset | cart_download | spc_download | bk_loading | clearing_ram | msu_data_download | parser_delay != 0;

  reg RESET_N = 0;
  reg RFSH = 0;
  always @(posedge clk_sys) begin
    reg [1:0] div;

    div  <= div + 1'd1;
    RFSH <= !div;

    if (div == 2) RESET_N <= ~reset;
  end

  always @(posedge clk_sys) begin
    video_r <= (LG_TARGET && lightgun_enabled) ? {8{LG_TARGET[0]}} : R;
    video_g <= (LG_TARGET && lightgun_enabled) ? {8{LG_TARGET[1]}} : G;
    video_b <= (LG_TARGET && lightgun_enabled) ? {8{LG_TARGET[2]}} : B;
  end

  ////////////////////////////  MEMORY  ///////////////////////////////////

  reg [16:0] mem_fill_addr;
  // Slowed down for PSRAM
  reg [1:0] clear_div = 0;
  reg clearing_ram = 0;
  always @(posedge clk_sys) begin
    if (~old_downloading & cart_download) clearing_ram <= 1'b1;

    if (&mem_fill_addr) clearing_ram <= 0;

    clear_div <= clear_div + 1;

    if (clearing_ram) begin
      if (clear_div == 0) begin
        mem_fill_addr <= mem_fill_addr + 1'b1;
      end
    end else mem_fill_addr <= 0;
  end

  reg [7:0] wram_fill_data;
  always @* begin
    case (status[22:21])
      0: wram_fill_data = (mem_fill_addr[8] ^ mem_fill_addr[2]) ? 8'h66 : 8'h99;
      1: wram_fill_data = (mem_fill_addr[9] ^ mem_fill_addr[0]) ? 8'hFF : 8'h00;
      2: wram_fill_data = 8'h55;
      3: wram_fill_data = 8'hFF;
    endcase
  end

  reg [7:0] aram_fill_data;
  always @* begin
    case (status[45:44])
      0: aram_fill_data = (mem_fill_addr[8] ^ mem_fill_addr[2]) ? 8'h66 : 8'h99;
      1: aram_fill_data = (mem_fill_addr[9] ^ mem_fill_addr[0]) ? 8'hFF : 8'h00;
      2: aram_fill_data = 8'h55;
      3: aram_fill_data = 8'hFF;
    endcase
  end

  // ROM_ADDR, ROM_OE_N, ROM_WE_N, ROM_WORD, ROM_D declared above with
  // the save state wires. ROM_Q_raw declared there too; ROM_Q is the
  // override-capable version fed back into main.

  sdram sdram (
      .init(0),  //~clock_locked),
      .clk(clk_mem),

      // Use sdram_addr so savestates can redirect to $FF:8000 (savestates.bin)
      .addr(sdram_addr),
      .din (cart_download ? ioctl_dout : ROM_D),
      .dout(ROM_Q_raw),    // raw output; ROM_Q mux applied above
      .rd  (~cart_download & (RESET_N ? ~ROM_OE_N : RFSH)),
      .wr  (cart_download ? ioctl_wr : ~ROM_WE_N),
      .word(cart_download | ROM_WORD),
      .busy(),

      // Actual SDRAM interface
      .SDRAM_DQ(dram_dq),
      .SDRAM_A(dram_a),
      .SDRAM_DQML(dram_dqm[0]),
      .SDRAM_DQMH(dram_dqm[1]),
      .SDRAM_BA(dram_ba),
      .SDRAM_nWE(dram_we_n),
      .SDRAM_nRAS(dram_ras_n),
      .SDRAM_nCAS(dram_cas_n),
      .SDRAM_CLK(dram_clk),
      .SDRAM_CKE(dram_cke)
  );

  wire [16:0] WRAM_ADDR;
  wire        WRAM_CE_N;
  wire        WRAM_OE_N;
  wire        WRAM_WE_N;
  wire [7:0] WRAM_Q, WRAM_D;

  wire [16:0] psram_wram_addr = clearing_ram ? mem_fill_addr[16:0] : WRAM_ADDR;
  wire [15:0] wram_data_in = clearing_ram ? {wram_fill_data, wram_fill_data} : // TODO: This isn't correct
  // Data either goes in high or low byte
  psram_wram_addr[0] ? {WRAM_D, 8'h0} : {8'h0, WRAM_D};
  wire [15:0] wram_data_out;

  assign WRAM_Q = psram_wram_addr[0] ? wram_data_out[15:8] : wram_data_out[7:0];

  psram #(
      .CLOCK_SPEED(85.9)
  ) wram (
      .clk(clk_mem_85_9),

      .bank_sel(0),
      // Remove bottom most bit, since this is a 8bit address and the RAM wants a 16bit address
      .addr(psram_wram_addr[16:1]),

      .write_en(clearing_ram ? 1'b1 : ~WRAM_CE_N & ~WRAM_WE_N),
      .data_in(wram_data_in),
      .write_high_byte(psram_wram_addr[0]),
      .write_low_byte(~psram_wram_addr[0]),

      .read_en (clearing_ram ? 1'b0 : ~WRAM_CE_N & ~WRAM_OE_N),
      .data_out(wram_data_out),

      // Actual PSRAM interface
      .cram_a(cram0_a),
      .cram_dq(cram0_dq),
      .cram_wait(cram0_wait),
      .cram_clk(cram0_clk),
      .cram_adv_n(cram0_adv_n),
      .cram_cre(cram0_cre),
      .cram_ce0_n(cram0_ce0_n),
      .cram_ce1_n(cram0_ce1_n),
      .cram_oe_n(cram0_oe_n),
      .cram_we_n(cram0_we_n),
      .cram_ub_n(cram0_ub_n),
      .cram_lb_n(cram0_lb_n)
  );

  wire [15:0] VRAM1_ADDR;
  wire        VRAM1_WE_N;
  wire [7:0] VRAM1_D, VRAM1_Q;
  dpram #(15) vram1 (
      .clock(clk_sys),
      .address_a(VRAM1_ADDR[14:0]),
      .data_a(VRAM1_D),
      .wren_a(~VRAM1_WE_N),
      .q_a(VRAM1_Q),

      // clear the RAM on loading
      .address_b(mem_fill_addr[14:0]),
      .wren_b(clearing_ram)
  );

  wire [15:0] VRAM2_ADDR;
  wire        VRAM2_WE_N;
  wire [7:0] VRAM2_D, VRAM2_Q;
  dpram #(15) vram2 (
      .clock(clk_sys),
      .address_a(VRAM2_ADDR[14:0]),
      .data_a(VRAM2_D),
      .wren_a(~VRAM2_WE_N),
      .q_a(VRAM2_Q),

      // clear the RAM on loading
      .address_b(mem_fill_addr[14:0]),
      .wren_b(clearing_ram)
  );

  wire [15:0] ARAM_ADDR;
  wire        ARAM_CE_N;
  wire        ARAM_OE_N;
  wire        ARAM_WE_N;
  wire [7:0] ARAM_Q, ARAM_D;

  wire [15:0] aram_16_out;

  assign ARAM_Q = psram_aram_addr[0] ? aram_16_out[15:8] : aram_16_out[7:0];

  wire [15:0] psram_aram_addr = clearing_ram ? mem_fill_addr[15:0] : ARAM_ADDR;

  wire [ 7:0] aram_data = clearing_ram ? aram_fill_data : ARAM_D;
  wire [15:0] aram_16_data = psram_aram_addr[0] ? {aram_data, 8'h0} : {8'h0, aram_data};

  psram #(
      .CLOCK_SPEED(85.9)
  ) aram (
      .clk(clk_mem_85_9),

      .bank_sel(0),
      // Remove bottom most bit, since this is a 8bit address and the RAM wants a 16bit address
      .addr(psram_aram_addr[15:1]),

      .write_en(clearing_ram ? 1'b1 : ~ARAM_CE_N & ~ARAM_WE_N),
      .data_in(aram_16_data),
      .write_high_byte(psram_aram_addr[0]),
      .write_low_byte(~psram_aram_addr[0]),

      .read_en (~ARAM_CE_N & ~ARAM_OE_N),
      .data_out(aram_16_out),

      // Actual PSRAM interface
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

  localparam BSRAM_BITS = 17;  // 1Mbits
  wire [19:0] BSRAM_ADDR;
  wire        BSRAM_CE_N;
  wire        BSRAM_OE_N;
  wire        BSRAM_WE_N;
  wire [7:0] BSRAM_Q, BSRAM_D;

  // savestates drives bsram_sel when it needs to stream BSRAM over the
  // peripheral bus.  Gate the normal mapper write-enable so the two
  // drivers don't fight.
  wire ss_bsram_sel;
  wire bsram_write_en = clearing_ram ? 1'b1
                      : ss_bsram_sel ? 1'b0          // ss owns the bus
                      : ~BSRAM_CE_N & ~BSRAM_WE_N;

  dpram_dif #(BSRAM_BITS, 8, BSRAM_BITS - 1, 16) bsram (
      .clock(clk_sys),

      //Thrash the BSRAM upon ROM loading
      .address_a(clearing_ram ? mem_fill_addr[BSRAM_BITS-1:0] : BSRAM_ADDR[BSRAM_BITS-1:0]),
      .data_a(clearing_ram ? 8'hFF : BSRAM_D),
      .wren_a(bsram_write_en),
      .q_a(BSRAM_Q),

      .address_b(sd_buff_addr),
      .data_b(sd_buff_dout),
      .wren_b(sd_wr),
      .q_b(sd_buff_din)
  );

  ////////////////////////////  SAVE STATES  //////////////////////////////

  savestates savestates (
      .reset_n     (RESET_N),
      .clk         (clk_sys),

      // APF triggers (already in clk_sys domain — caller synchronises)
      .ss_save     (ss_save),
      .ss_load     (ss_load),

      // BRAM bridge (clk_74a domain)
      .clk_74a     (clk_74a),
      .bram_wr     (bram_wr),
      .bram_wr_addr(bram_wr_addr),
      .bram_wr_data(bram_wr_data),
      .bram_rd_data(bram_rd_data),
      .bram_rd_addr(bram_rd_addr),

      // ROM config
      .ram_size    (ram_size),
      .rom_type    (rom_type),

      // SNES clock enables
      .sysclkf_ce  (SS_SYSCLKF_CE),
      .sysclkr_ce  (SS_SYSCLKR_CE),

      // SNES CPU bus
      .romsel_n    (SS_ROMSEL_N),
      .ca          (SS_CA),
      .cpurd_n     (SS_CPURD_N),
      .cpuwr_n     (SS_CPUWR_N),

      // SNES peripheral bus
      .pa          (SS_PA),
      .pard_n      (SS_PARD_N),
      .pawr_n      (SS_PAWR_N),

      // Data bus
      .di          (SS_DO),        // what the CPU is writing
      .ss_do       (ss_do_out),    // override data back to CPU

      // ROM address override
      .rom_addr    (ss_rom_addr),
      .ext_addr    (ss_ext_addr),

      // SPC700 / APU
      .spc_di      (ARAM_Q),

      // PPU register readback — ROM_Q_raw low byte is valid during $C1:21xx
      // reads because savestates itself drives the address; use raw here so
      // we don't create a combinational loop through the ROM_Q mux.
      .ppu_di      (ROM_Q_raw[7:0]),

      // BSRAM
      .bsram_sel   (ss_bsram_sel),
      .bsram_di    (BSRAM_Q),

      // DSPn — tie off when not present; savestates_map handles gating
      .dspn_regs_sel(),
      .dspn_ram_sel (),
      .dspn_di     (8'h00),

      // GSU (SuperFX)
      .gsu_regs_sel(),
      .gsu_di      (8'h00),

      // SA-1 — disabled for now
      .sa1_active      (1'b0),
      .sa1_a           (24'h0),
      .sa1_di          (8'h00),
      .sa1_rd_n        (1'b1),
      .sa1_wr_n        (1'b1),
      .sa1_sa1_romsel  (1'b1),
      .sa1_sns_romsel  (1'b1),

      // Bus override qualifiers
      .ss_do_ovr   (ss_do_ovr),
      .ss_rom_ovr  (ss_rom_ovr),
      .ss_busy     (ss_busy)
  );

  ////////////////////////////  I/O PORTS  ////////////////////////////////

  wire [11:0] joy0 = {
    p1_button_start,
    p1_button_select,
    p1_button_trig_r,
    p1_button_trig_l,
    p1_button_y,
    p1_button_x,
    p1_button_b,
    p1_button_a,
    p1_dpad_up,
    p1_dpad_down,
    p1_dpad_left,
    p1_dpad_right
  };

  wire [11:0] joy1 = {
    p2_button_start,
    p2_button_select,
    p2_button_trig_r,
    p2_button_trig_l,
    p2_button_y,
    p2_button_x,
    p2_button_b,
    p2_button_a,
    p2_dpad_up,
    p2_dpad_down,
    p2_dpad_left,
    p2_dpad_right
  };

  wire [11:0] joy2 = {
    p3_button_start,
    p3_button_select,
    p3_button_trig_r,
    p3_button_trig_l,
    p3_button_y,
    p3_button_x,
    p3_button_b,
    p3_button_a,
    p3_dpad_up,
    p3_dpad_down,
    p3_dpad_left,
    p3_dpad_right
  };

  wire [11:0] joy3 = {
    p4_button_start,
    p4_button_select,
    p4_button_trig_r,
    p4_button_trig_l,
    p4_button_y,
    p4_button_x,
    p4_button_b,
    p4_button_a,
    p4_dpad_up,
    p4_dpad_down,
    p4_dpad_left,
    p4_dpad_right
  };

  wire [1:0] JOY1_DO = piano ? {1'b1, piano_joypad_do} : JOY1_DO_t;

  wire JOY_STRB;

  wire [1:0] JOY1_DO_t;
  wire JOY1_CLK;
  wire JOY1_P6;
  ioport port1 (
      .CLK(clk_sys),

      .PORT_LATCH(JOY_STRB),
      .PORT_CLK(JOY1_CLK),
      .PORT_P6(JOY1_P6),
      .PORT_DO(JOY1_DO_t),

      .JOYSTICK1(joy_swap ? joy1 : joy0),
      .JOY_X(p1_lstick_x),
      .JOY_Y(p1_lstick_y),

      .DPAD_AIM_SPEED(dpad_aim_speed),
      .MOUSE_EN(mouse_enabled)
  );

  wire [1:0] JOY2_DO;
  wire       JOY2_CLK;
  wire       JOY2_P6;
  ioport port2 (
      .CLK(clk_sys),

      .MULTITAP(multitap_enabled),

      .PORT_LATCH(JOY_STRB),
      .PORT_CLK(JOY2_CLK),
      .PORT_P6(JOY2_P6),
      .PORT_DO(JOY2_DO),

      .JOYSTICK1((joy_swap ^ raw_serial) ? joy0 : joy1),
      .JOYSTICK2(joy2),
      .JOYSTICK3(joy3),
      // .JOYSTICK4(joy4),

      // .MOUSE(ps2_mouse),
      // .MOUSE_EN(mouse_mode[1])
  );

  wire LG_P6_out;
  wire [1:0] LG_DO;
  wire [2:0] LG_TARGET;

  lightgun lightgun (
      .CLK  (clk_sys),
      .RESET(reset),

      .JOY_X(p1_lstick_x),
      .JOY_Y(p1_lstick_y),

      .F(p1_button_a),
      .C(p1_button_b),
      .T(p1_button_x),
      .P(p1_button_y),

      .UP(p1_dpad_up),
      .DOWN(p1_dpad_down),
      .LEFT(p1_dpad_left),
      .RIGHT(p1_dpad_right),
      .DPAD_AIM_SPEED(dpad_aim_speed),

      .HDE(hblank_n),
      .VDE(vblank_n),
      .CLKPIX(dotclk),

      .TARGET(LG_TARGET),
      .SIZE(0),
      .GUN_TYPE(lightgun_type),

      .PORT_LATCH(JOY_STRB),
      .PORT_CLK(JOY2_CLK),
      .PORT_P6(LG_P6_out),
      .PORT_DO(LG_DO)
  );

  wire raw_serial = status[8];
  reg                          snac_p2 = 0;

  assign USER_OUT[2] = 1'b1;
  assign USER_OUT[5] = 1'b1;
  assign USER_OUT[6] = 1'b1;

  wire [1:0] datajoy0_DI = snac_p2 ? {1'b1, USER_IN[6]} : JOY1_DO;
  wire [1:0] datajoy1_DI = snac_p2 ? {USER_IN[2], USER_IN[6]} : JOY2_DO;

  // JOYX_DO[0] is P4, JOYX_DO[1] is P5
  wire [1:0] JOY1_DI;
  wire [1:0] JOY2_DI;
  wire JOY2_P6_DI;

  always @(posedge clk_sys) begin
    if (raw_serial) begin
      if (~USER_IN[6]) snac_p2 <= 1;
    end else begin
      snac_p2 <= 0;
    end
  end

  always_comb begin
    if (raw_serial) begin
      USER_OUT[0] = JOY_STRB;
      USER_OUT[1] = joy_swap ? ~JOY2_CLK : ~JOY1_CLK;
      USER_OUT[3] = joy_swap ? ~JOY1_CLK : ~JOY2_CLK;
      USER_OUT[4] = joy_swap ? JOY2_P6 : snac_p2 ? JOY2_P6 : JOY1_P6;
      JOY1_DI = joy_swap ? datajoy0_DI : snac_p2 ? {1'b1, USER_IN[5]} : {USER_IN[2], USER_IN[5]};
      JOY2_DI = joy_swap ? {USER_IN[2], USER_IN[5]} : datajoy1_DI;
      JOY2_P6_DI = joy_swap ? USER_IN[4] : snac_p2 ? USER_IN[4] : (LG_P6_out | !GUN_MODE);
    end else begin
      USER_OUT[0] = 1'b1;
      USER_OUT[1] = 1'b1;
      USER_OUT[3] = 1'b1;
      USER_OUT[4] = 1'b1;
      JOY1_DI = JOY1_DO;
      JOY2_DI = JOY2_DO;
      JOY2_P6_DI = (LG_P6_out | !GUN_MODE);
    end
  end

  /////////////////////////  STATE SAVE/LOAD  /////////////////////////////

  reg bk_ena = 0;
  reg old_downloading = 0;
  always @(posedge clk_sys) begin
    old_downloading <= cart_download;
    if (~old_downloading & cart_download) bk_ena <= 0;
  end

  reg bk_loading = 0;
  reg bk_state = 0;

  ///////////////////////////  MSU1  ///////////////////////////////////

  wire msu_data_download = 0;


endmodule
