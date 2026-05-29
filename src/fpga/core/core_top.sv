//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

    //
    // physical connections
    //

    ///////////////////////////////////////////////////
    // clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

    input wire clk_74a,  // mainclk1
    input wire clk_74b,  // mainclk1 

    ///////////////////////////////////////////////////
    // cartridge interface
    // switches between 3.3v and 5v mechanically
    // output enable for multibit translators controlled by pic32

    // GBA AD[15:8]
    inout  wire [7:0] cart_tran_bank2,
    output wire       cart_tran_bank2_dir,

    // GBA AD[7:0]
    inout  wire [7:0] cart_tran_bank3,
    output wire       cart_tran_bank3_dir,

    // GBA A[23:16]
    inout  wire [7:0] cart_tran_bank1,
    output wire       cart_tran_bank1_dir,

    // GBA [7] PHI#
    // GBA [6] WR#
    // GBA [5] RD#
    // GBA [4] CS1#/CS#
    //     [3:0] unwired
    inout  wire [7:4] cart_tran_bank0,
    output wire       cart_tran_bank0_dir,

    // GBA CS2#/RES#
    inout  wire cart_tran_pin30,
    output wire cart_tran_pin30_dir,
    // when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
    // the goal is that when unconfigured, the FPGA weak pullups won't interfere.
    // thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
    // and general IO drive this pin.
    output wire cart_pin30_pwroff_reset,

    // GBA IRQ/DRQ
    inout  wire cart_tran_pin31,
    output wire cart_tran_pin31_dir,

    // infrared
    input  wire port_ir_rx,
    output wire port_ir_tx,
    output wire port_ir_rx_disable,

    // GBA link port
    inout  wire port_tran_si,
    output wire port_tran_si_dir,
    inout  wire port_tran_so,
    output wire port_tran_so_dir,
    inout  wire port_tran_sck,
    output wire port_tran_sck_dir,
    inout  wire port_tran_sd,
    output wire port_tran_sd_dir,

    ///////////////////////////////////////////////////
    // cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

    output wire [21:16] cram0_a,
    inout  wire [ 15:0] cram0_dq,
    input  wire         cram0_wait,
    output wire         cram0_clk,
    output wire         cram0_adv_n,
    output wire         cram0_cre,
    output wire         cram0_ce0_n,
    output wire         cram0_ce1_n,
    output wire         cram0_oe_n,
    output wire         cram0_we_n,
    output wire         cram0_ub_n,
    output wire         cram0_lb_n,

    output wire [21:16] cram1_a,
    inout  wire [ 15:0] cram1_dq,
    input  wire         cram1_wait,
    output wire         cram1_clk,
    output wire         cram1_adv_n,
    output wire         cram1_cre,
    output wire         cram1_ce0_n,
    output wire         cram1_ce1_n,
    output wire         cram1_oe_n,
    output wire         cram1_we_n,
    output wire         cram1_ub_n,
    output wire         cram1_lb_n,

    ///////////////////////////////////////////////////
    // sdram, 512mbit 16bit

    output wire [12:0] dram_a,
    output wire [ 1:0] dram_ba,
    inout  wire [15:0] dram_dq,
    output wire [ 1:0] dram_dqm,
    output wire        dram_clk,
    output wire        dram_cke,
    output wire        dram_ras_n,
    output wire        dram_cas_n,
    output wire        dram_we_n,

    ///////////////////////////////////////////////////
    // sram, 1mbit 16bit

    output wire [16:0] sram_a,
    inout  wire [15:0] sram_dq,
    output wire        sram_oe_n,
    output wire        sram_we_n,
    output wire        sram_ub_n,
    output wire        sram_lb_n,

    ///////////////////////////////////////////////////
    // vblank driven by dock for sync in a certain mode

    input wire vblank,

    ///////////////////////////////////////////////////
    // i/o to 6515D breakout usb uart

    output wire dbg_tx,
    input  wire dbg_rx,

    ///////////////////////////////////////////////////
    // i/o pads near jtag connector user can solder to

    output wire user1,
    input  wire user2,

    ///////////////////////////////////////////////////
    // RFU internal i2c bus 

    inout  wire aux_sda,
    output wire aux_scl,

    ///////////////////////////////////////////////////
    // RFU, do not use
    output wire vpll_feed,


    //
    // logical connections
    //

    ///////////////////////////////////////////////////
    // video, audio output to scaler
    output wire [23:0] video_rgb,
    output wire        video_rgb_clock,
    output wire        video_rgb_clock_90,
    output wire        video_de,
    output wire        video_skip,
    output wire        video_vs,
    output wire        video_hs,

    output wire audio_mclk,
    input  wire audio_adc,
    output wire audio_dac,
    output wire audio_lrck,

    ///////////////////////////////////////////////////
    // bridge bus connection
    // synchronous to clk_74a
    output wire        bridge_endian_little,
    input  wire [31:0] bridge_addr,
    input  wire        bridge_rd,
    output reg  [31:0] bridge_rd_data,
    input  wire        bridge_wr,
    input  wire [31:0] bridge_wr_data,

    ///////////////////////////////////////////////////
    // controller data
    // 
    // key bitmap:
    //   [0]    dpad_up
    //   [1]    dpad_down
    //   [2]    dpad_left
    //   [3]    dpad_right
    //   [4]    face_a
    //   [5]    face_b
    //   [6]    face_x
    //   [7]    face_y
    //   [8]    trig_l1
    //   [9]    trig_r1
    //   [10]   trig_l2
    //   [11]   trig_r2
    //   [12]   trig_l3
    //   [13]   trig_r3
    //   [14]   face_select
    //   [15]   face_start
    // joy values - unsigned
    //   [ 7: 0] lstick_x
    //   [15: 8] lstick_y
    //   [23:16] rstick_x
    //   [31:24] rstick_y
    // trigger values - unsigned
    //   [ 7: 0] ltrig
    //   [15: 8] rtrig
    //
    input wire [15:0] cont1_key,
    input wire [15:0] cont2_key,
    input wire [15:0] cont3_key,
    input wire [15:0] cont4_key,
    input wire [31:0] cont1_joy,
    input wire [31:0] cont2_joy,
    input wire [31:0] cont3_joy,
    input wire [31:0] cont4_joy,
    input wire [15:0] cont1_trig,
    input wire [15:0] cont2_trig,
    input wire [15:0] cont3_trig,
    input wire [15:0] cont4_trig

);

  // not using the IR port, so turn off both the LED, and
  // disable the receive circuit to save power
  assign port_ir_tx              = 0;
  assign port_ir_rx_disable      = 1;

  // bridge endianness
  assign bridge_endian_little    = 0;

  // cart is unused, so set all level translators accordingly
  // directions are 0:IN, 1:OUT
  assign cart_tran_bank3         = 8'hzz;
  assign cart_tran_bank3_dir     = 1'b0;
  assign cart_tran_bank2         = 8'hzz;
  assign cart_tran_bank2_dir     = 1'b0;
  assign cart_tran_bank1         = 8'hzz;
  assign cart_tran_bank1_dir     = 1'b0;
  assign cart_tran_bank0         = 4'hf;
  assign cart_tran_bank0_dir     = 1'b1;
  assign cart_tran_pin30         = 1'b0;  // reset or cs2, we let the hw control it by itself
  assign cart_tran_pin30_dir     = 1'bz;
  assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
  assign cart_tran_pin31         = 1'bz;  // input
  assign cart_tran_pin31_dir     = 1'b0;  // input

  // link port is input only
  assign port_tran_so            = 1'bz;
  assign port_tran_so_dir        = 1'b0;  // SO is output only
  assign port_tran_si            = 1'bz;
  assign port_tran_si_dir        = 1'b0;  // SI is input only
  assign port_tran_sck           = 1'bz;
  assign port_tran_sck_dir       = 1'b0;  // clock direction can change
  assign port_tran_sd            = 1'bz;
  assign port_tran_sd_dir        = 1'b0;  // SD is input and not used

  // tie off the rest of the pins we are not using
  //   assign cram0_a                 = 'h0;
  //   assign cram0_dq                = {16{1'bZ}};
  //   assign cram0_clk               = 0;
  //   assign cram0_adv_n             = 1;
  //   assign cram0_cre               = 0;
  //   assign cram0_ce0_n             = 1;
  //   assign cram0_ce1_n             = 1;
  //   assign cram0_oe_n              = 1;
  //   assign cram0_we_n              = 1;
  //   assign cram0_ub_n              = 1;
  //   assign cram0_lb_n              = 1;

  //   assign cram1_a                 = 'h0;
  //   assign cram1_dq                = {16{1'bZ}};
  //   assign cram1_clk               = 0;
  //   assign cram1_adv_n             = 1;
  //   assign cram1_cre               = 0;
  //   assign cram1_ce0_n             = 1;
  //   assign cram1_ce1_n             = 1;
  //   assign cram1_oe_n              = 1;
  //   assign cram1_we_n              = 1;
  //   assign cram1_ub_n              = 1;
  //   assign cram1_lb_n              = 1;

  //   assign dram_a                  = 'h0;
  //   assign dram_ba                 = 'h0;
  //   assign dram_dq                 = {16{1'bZ}};
  //   assign dram_dqm                = 'h0;
  //   assign dram_clk                = 'h0;
  //   assign dram_cke                = 'h0;
  //   assign dram_ras_n              = 'h1;
  //   assign dram_cas_n              = 'h1;
  //   assign dram_we_n               = 'h1;

  // Phase C: on-board SRAM no longer used (save: streaming FIFO; load: SDRAM).
  // Drive the pins to safe defaults so the chip stays idle.
  assign sram_a    = 17'h0;
  assign sram_dq   = 16'hZZZZ;
  assign sram_oe_n = 1'b1;
  assign sram_we_n = 1'b1;
  assign sram_ub_n = 1'b1;
  assign sram_lb_n = 1'b1;

  assign dbg_tx                  = 1'bZ;
  assign user1                   = 1'bZ;
  assign aux_scl                 = 1'bZ;
  assign vpll_feed               = 1'bZ;


  // for bridge write data, we just broadcast it to all bus devices
  // for bridge read data, we have to mux it
  // add your own devices here
  always @(*) begin
    casex (bridge_addr)
      default: begin
        bridge_rd_data <= 0;
      end
      32'h10xxxxxx: begin
        // example
        bridge_rd_data <= 0;
      end
      32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
      end
    endcase

    if (bridge_addr[31:28] == 4'h2) begin
      bridge_rd_data <= sd_read_data;
    end

    if (bridge_addr[31:28] == 4'h4) begin
      bridge_rd_data <= save_state_bridge_read_data;
    end
  end

  always @(posedge clk_74a) begin
    if (reset_delay > 0) begin
      reset_delay <= reset_delay - 1;
    end

    if (bridge_wr) begin
      casex (bridge_addr)
        32'h0: begin
          ioctl_download <= bridge_wr_data[0];
        end
        32'h4: begin
          rom_size <= bridge_wr_data[3:0];
        end
        32'h8: begin
          rom_type <= bridge_wr_data[7:0];
        end
        32'hC: begin
          ram_size <= bridge_wr_data[3:0];
        end
        32'h10: begin
          PAL <= bridge_wr_data[0];
        end
        32'h50: begin
          reset_delay <= 32'h100000;
        end
        32'h80: begin
          cpu_turbo_enabled <= bridge_wr_data[0];
        end
        32'h84: begin
          gsu_turbo_enabled <= bridge_wr_data[0];
        end
        32'h100: begin
          multitap_enabled <= bridge_wr_data[0];
        end
        32'h104: begin
          lightgun_enabled <= bridge_wr_data[0];
          lightgun_type    <= bridge_wr_data[1];
          mouse_enabled    <= bridge_wr_data[2];
        end
        32'h00000108: begin
          dpad_aim_speed <= bridge_wr_data[7:0];
        end
        32'h0000010C: begin
          joystick_deadzone <= bridge_wr_data[7:0];
        end
        32'h200: begin
          use_square_pixels <= bridge_wr_data[0];
        end
        32'h204: begin
          blend_enabled <= bridge_wr_data[0];
        end
      endcase
    end
  end


  //
  // host/target command handler
  //
  wire reset_n;  // driven by host commands, can be used as core-wide reset
  wire [31:0] cmd_bridge_rd_data;

  // bridge host commands
  // synchronous to clk_74a
  wire status_boot_done = pll_core_locked;
  wire status_setup_done = pll_core_locked;  // rising edge triggers a target command
  wire status_running = reset_n;  // we are running as soon as reset_n goes high

  wire dataslot_requestread;
  wire [15:0] dataslot_requestread_id;
  wire dataslot_requestread_ack = 1;
  wire dataslot_requestread_ok = 1;

  wire dataslot_requestwrite;
  wire [15:0] dataslot_requestwrite_id;
  wire dataslot_requestwrite_ack = 1;
  wire dataslot_requestwrite_ok = 1;

  wire dataslot_allcomplete;

  wire savestate_supported = 1;
  wire [31:0] savestate_addr = 32'h40000000;
  // Phase C: streaming save + SDRAM-staged load.  The actual payload with
  // ARAM enabled is ~307KB; SDRAM staging area has room for many MB so
  // declare 512KB to give APF margin without overcommitting.
  wire [31:0] savestate_size        = 32'h80000;  // 512KB
  wire [31:0] savestate_maxloadsize = 32'h80000;  // 512KB

  wire savestate_start;
  wire savestate_start_ack;
  wire savestate_start_busy;
  wire savestate_start_ok;
  wire savestate_start_err;

  wire savestate_load;
  wire savestate_load_ack;
  wire savestate_load_busy;
  wire savestate_load_ok;
  wire savestate_load_err;

  wire osnotify_inmenu;

  wire [31:0] rtc_date;
  wire [31:0] rtc_time;

  // bridge target commands
  // synchronous to clk_74a


  // bridge data slot access

  reg [9:0] datatable_addr;
  reg datatable_wren;
  reg [31:0] datatable_data;
  wire [31:0] datatable_q;

  core_bridge_cmd icb (

      .clk    (clk_74a),
      .reset_n(reset_n),

      .bridge_endian_little(bridge_endian_little),
      .bridge_addr         (bridge_addr),
      .bridge_rd           (bridge_rd),
      .bridge_rd_data      (cmd_bridge_rd_data),
      .bridge_wr           (bridge_wr),
      .bridge_wr_data      (bridge_wr_data),

      .status_boot_done (status_boot_done),
      .status_setup_done(status_setup_done),
      .status_running   (status_running),

      .dataslot_requestread    (dataslot_requestread),
      .dataslot_requestread_id (dataslot_requestread_id),
      .dataslot_requestread_ack(dataslot_requestread_ack),
      .dataslot_requestread_ok (dataslot_requestread_ok),

      .dataslot_requestwrite    (dataslot_requestwrite),
      .dataslot_requestwrite_id (dataslot_requestwrite_id),
      .dataslot_requestwrite_ack(dataslot_requestwrite_ack),
      .dataslot_requestwrite_ok (dataslot_requestwrite_ok),

      .dataslot_allcomplete(dataslot_allcomplete),

      .rtc_date(rtc_date),
      .rtc_time(rtc_time),

      .savestate_supported  (savestate_supported),
      .savestate_addr       (savestate_addr),
      .savestate_size       (savestate_size),
      .savestate_maxloadsize(savestate_maxloadsize),

      .savestate_start     (savestate_start),
      .savestate_start_ack (savestate_start_ack),
      .savestate_start_busy(savestate_start_busy),
      .savestate_start_ok  (savestate_start_ok),
      .savestate_start_err (savestate_start_err),

      .savestate_load     (savestate_load),
      .savestate_load_ack (savestate_load_ack),
      .savestate_load_busy(savestate_load_busy),
      .savestate_load_ok  (savestate_load_ok),
      .savestate_load_err (savestate_load_err),

      .osnotify_inmenu(osnotify_inmenu),

      .datatable_addr(datatable_addr),
      .datatable_wren(datatable_wren),
      .datatable_data(datatable_data),
      .datatable_q   (datatable_q)
  );

  // ===== Save State Controller =====
  wire ss_busy;
  wire [63:0] ss_din;
  wire [63:0] ss_dout;
  wire [16:0] ss_ddr_addr;
  wire ss_rnw;
  wire ss_req;
  wire [7:0] ss_be;
  wire ss_ack;
  wire ss_we;

  wire ss_save;
  wire ss_load;

  // Phase B: SDRAM staging interface between save_state_controller and SNES.sv.
  // Controller drives wr/rd req + addr/data in clk_sys; SNES.sv handles CDC
  // into clk_mem and muxes onto the cart-ROM sdram instance.  Phase B leaves
  // all these tied to 0 by the controller — no functional change.
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

  // bridge_rd_data for the savestate region (0x4xxxxxxx) is produced by
  // a data_unloader instance (see below), wired directly to bridge_rd_data
  // via the existing mux in the always_comb block at the top of this file.
  wire [31:0] save_state_bridge_read_data;

  // Debug taps from save_state_controller (clk_sys domain)
  wire [3:0] debug_sys_state;
  wire       debug_ss_busy_seen;
  wire       debug_ss_busy_ever;
  wire       debug_ss_save_ever;
  wire [3:0] debug_ss_save_count;
  wire [3:0] debug_ss_busy_rises;
  wire       debug_ss_req_ever;
  wire [3:0] debug_ss_req_toggles;
  wire       debug_core_wr_ever;
  wire       debug_sram_wr_ack_ever;
  wire [7:0] debug_bridge_wr_count_lo;
  wire [7:0] debug_bridge_wr_count_hi;
  wire [7:0] debug_first_wr_data_b0;
  wire [7:0] debug_first_wr_data_b1;
  wire [7:0] debug_first_wr_addr_lo;
  wire [7:0] debug_first_wr_addr_hi;
  wire [7:0] debug_first_save_byte0;
  wire [7:0] debug_first_save_byte1;
  wire [7:0] debug_first_save_addr_lo;
  wire [7:0] debug_first_save_addr_hi;
  wire [7:0] debug_first_pf_addr_lo;
  wire [7:0] debug_first_pf_addr_hi;
  wire [7:0] debug_first_sram_w0_lo;
  wire [7:0] debug_first_sram_w0_hi;
  wire [7:0] debug_first_sram_w1_lo;
  wire [7:0] debug_first_sram_w1_hi;
  wire [7:0] debug_max_sram_base_lo;
  wire [7:0] debug_max_sram_base_hi;
  wire [7:0] debug_save_wr_count_lo;
  wire [7:0] debug_save_wr_count_hi;
  wire [7:0] debug_ss_addr_overflow;
  wire [7:0] debug_ss_addr_max_hi;
  wire [7:0] debug_pf_at_first_rd_lo;
  wire [7:0] debug_pf_at_first_rd_hi;
  wire [7:0] debug_bridge_rd_count_lo;
  wire [7:0] debug_bridge_rd_count_hi;
  wire [7:0] debug_first_rd_addr_lo;
  wire [7:0] debug_first_rd_addr_hi;
  wire [7:0] debug_last_w0_data_lo;
  wire [7:0] debug_last_w0_data_hi;
  wire [7:0] debug_w0_wr_count;

  // Debug taps from savestates.sv (SNES side)
  wire [3:0] dbg_rti_arms;
  wire [3:0] dbg_vect_reentry;
  wire [3:0] dbg_ddr_writes;
  wire [3:0] dbg_save_end_writes;
  wire [3:0] dbg_fw_entry;
  wire [3:0] dbg_fw_nmidis;
  wire [3:0] dbg_fw_at_8000;
  wire [3:0] dbg_fw_at_8003;
  wire [7:0] dbg_byte_at_8000;
  wire [7:0] dbg_byte_at_8001;
  wire [7:0] dbg_load_byte0;
  wire [7:0] dbg_load_byte1;
  wire [3:0] dbg_load_en_cnt;
  wire [3:0] dbg_load_vect_cnt;
  wire [3:0] dbg_load_busy_cnt;

  save_state_controller save_state_controller (
      .clk_74a(clk_74a),
      .clk_sys(clk_sys_21_48),

      // APF Bridge — writes (load) and reads (save FIFO drain)
      .bridge_wr(bridge_wr),
      .bridge_rd(bridge_rd),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),
      .save_state_bridge_read_data(save_state_bridge_read_data),

      // APF Save State Handshake
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

      // Core-side savestate control
      .ss_save(ss_save),
      .ss_load(ss_load),

      .ss_din (ss_din),
      .ss_dout(ss_dout),
      .ss_addr(ss_ddr_addr),
      .ss_rnw (~ss_we),
      .ss_req (ss_req),
      .ss_be  (ss_be),
      .ss_ack (ss_ack),

      .ss_busy(ss_busy),

      // Debug taps for on-screen overlay
      .debug_sys_state       (debug_sys_state),
      .debug_ss_busy_seen    (debug_ss_busy_seen),
      .debug_ss_busy_ever    (debug_ss_busy_ever),
      .debug_ss_save_ever    (debug_ss_save_ever),
      .debug_ss_save_count   (debug_ss_save_count),
      .debug_ss_busy_rises   (debug_ss_busy_rises),
      .debug_ss_req_ever     (debug_ss_req_ever),
      .debug_ss_req_toggles  (debug_ss_req_toggles),
      .debug_core_wr_ever    (debug_core_wr_ever),
      .debug_sram_wr_ack_ever(debug_sram_wr_ack_ever),
      .debug_bridge_wr_count_lo(debug_bridge_wr_count_lo),
      .debug_bridge_wr_count_hi(debug_bridge_wr_count_hi),
      .debug_first_wr_data_b0  (debug_first_wr_data_b0),
      .debug_first_wr_data_b1  (debug_first_wr_data_b1),
      .debug_first_wr_addr_lo  (debug_first_wr_addr_lo),
      .debug_first_wr_addr_hi  (debug_first_wr_addr_hi),
      .debug_first_save_byte0  (debug_first_save_byte0),
      .debug_first_save_byte1  (debug_first_save_byte1),
      .debug_first_save_addr_lo(debug_first_save_addr_lo),
      .debug_first_save_addr_hi(debug_first_save_addr_hi),
      .debug_first_pf_addr_lo  (debug_first_pf_addr_lo),
      .debug_first_pf_addr_hi  (debug_first_pf_addr_hi),
      .debug_first_sram_w0_lo  (debug_first_sram_w0_lo),
      .debug_first_sram_w0_hi  (debug_first_sram_w0_hi),
      .debug_first_sram_w1_lo  (debug_first_sram_w1_lo),
      .debug_first_sram_w1_hi  (debug_first_sram_w1_hi),
      .debug_max_sram_base_lo  (debug_max_sram_base_lo),
      .debug_max_sram_base_hi  (debug_max_sram_base_hi),
      .debug_save_wr_count_lo  (debug_save_wr_count_lo),
      .debug_save_wr_count_hi  (debug_save_wr_count_hi),
      .debug_ss_addr_overflow  (debug_ss_addr_overflow),
      .debug_ss_addr_max_hi    (debug_ss_addr_max_hi),
      .debug_pf_at_first_rd_lo (debug_pf_at_first_rd_lo),
      .debug_pf_at_first_rd_hi (debug_pf_at_first_rd_hi),
      .debug_bridge_rd_count_lo(debug_bridge_rd_count_lo),
      .debug_bridge_rd_count_hi(debug_bridge_rd_count_hi),
      .debug_first_rd_addr_lo  (debug_first_rd_addr_lo),
      .debug_first_rd_addr_hi  (debug_first_rd_addr_hi),
      .debug_last_w0_data_lo   (debug_last_w0_data_lo),
      .debug_last_w0_data_hi   (debug_last_w0_data_hi),
      .debug_w0_wr_count       (debug_w0_wr_count),

      // Phase C: SDRAM staging interface (replaces SRAM pin set entirely)
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

  reg ioctl_download = 0;
  wire ioctl_wr;
  wire [24:0] ioctl_addr;
  wire [15:0] ioctl_dout;

  reg save_download = 0;
  reg dataslot_allcomplete_prev;

  always @(posedge clk_74a) begin
    dataslot_allcomplete_prev <= dataslot_allcomplete;

    // if (dataslot_requestwrite) ioctl_download <= 1;
    // else if (dataslot_allcomplete) ioctl_download <= 0;

    if (dataslot_requestread || dataslot_requestwrite) save_download <= 1;
    else if (dataslot_allcomplete && ~dataslot_allcomplete_prev) save_download <= 0;
  end

  reg [7:0] rom_type;
  reg [3:0] rom_size;
  reg [3:0] ram_size;
  reg PAL;

  wire save_download_s;

  synch_3 save_s (
      save_download,
      save_download_s,
      clk_sys_21_48
  );

  data_loader #(
      .ADDRESS_MASK_UPPER_4(4'h1),
      .ADDRESS_SIZE(25),
      .WRITE_MEM_CLOCK_DELAY(7),
      .OUTPUT_WORD_SIZE(2)
  ) data_loader (
      .clk_74a(clk_74a),
      .clk_memory(clk_sys_21_48),

      .bridge_wr(bridge_wr),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),

      .write_en  (ioctl_wr),
      .write_addr(ioctl_addr),
      .write_data(ioctl_dout)
  );

  data_loader #(
      .ADDRESS_MASK_UPPER_4(4'h2),
      .ADDRESS_SIZE(17),
      .WRITE_MEM_CLOCK_DELAY(7),
      .OUTPUT_WORD_SIZE(2)
  ) save_data_loader (
      .clk_74a(clk_74a),
      .clk_memory(clk_sys_21_48),

      .bridge_wr(bridge_wr),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_wr_data(bridge_wr_data),

      .write_en  (sd_wr),
      .write_addr(sd_buff_addr_in),
      .write_data(sd_buff_dout)
  );

  wire [31:0] sd_read_data;

  wire sd_rd;
  wire sd_wr;

  wire [16:0] sd_buff_addr_in;
  wire [16:0] sd_buff_addr_out;

  // Lowest bit is for byte addressing
  wire [15:0] sd_buff_addr = sd_wr ? sd_buff_addr_in[16:1] : sd_buff_addr_out[16:1];

  wire [15:0] sd_buff_din;
  wire [15:0] sd_buff_dout;

  data_unloader #(
      .ADDRESS_MASK_UPPER_4(4'h2),
      .ADDRESS_SIZE(17),
      .READ_MEM_CLOCK_DELAY(7),
      .INPUT_WORD_SIZE(2)
  ) data_unloader (
      .clk_74a(clk_74a),
      .clk_memory(clk_sys_21_48),

      .bridge_rd(bridge_rd),
      .bridge_endian_little(bridge_endian_little),
      .bridge_addr(bridge_addr),
      .bridge_rd_data(sd_read_data),

      .read_en  (sd_rd),
      .read_addr(sd_buff_addr_out),
      .read_data(sd_buff_din)
  );

  // (Phase A: save-state bridge READ path is now handled internally by
  // save_state_controller via a save FIFO.  No data_unloader needed.)

  always @(posedge clk_74a or negedge pll_core_locked) begin
    if (~pll_core_locked) begin
      datatable_addr <= 0;
      datatable_data <= 0;
      datatable_wren <= 0;
    end else begin
      // Write sram size half of the time
      datatable_wren <= 1;
      // sram_size is the size of the config value in the ROM. Convert to actual size
      datatable_data <= sram_size ? 32'd1024 << sram_size : 32'h0;
      // Data slot index 1, not id 1
      datatable_addr <= 1 * 2 + 1;
    end
  end

  wire [15:0] audio_l;
  wire [15:0] audio_r;

  wire [3:0] sram_size;

  wire [15:0] cont1_key_s;
  wire [15:0] cont2_key_s;
  wire [15:0] cont3_key_s;
  wire [15:0] cont4_key_s;
  wire [31:0] cont1_joy_s;

  wire [15:0] cont1_joy_x = cont1_joy_s[7:0];
  wire [15:0] cont1_joy_y = cont1_joy_s[15:8];
  wire [15:0] cont1_joy_dx = cont1_joy_x[7] ? cont1_joy_x[6:0] : 8'd128 - cont1_joy_x[6:0];
  wire [15:0] cont1_joy_dy = cont1_joy_y[7] ? cont1_joy_y[6:0] : 8'd128 - cont1_joy_y[6:0];
  wire [16:0] cont1_joy_total = cont1_joy_dx + cont1_joy_dy;
  wire [15:0] cont1_joy_x_calibrated = cont1_joy_total > joystick_deadzone ? cont1_joy_x : 8'd128;
  wire [15:0] cont1_joy_y_calibrated = cont1_joy_total > joystick_deadzone ? cont1_joy_y : 8'd128;

  synch_3 #(
      .WIDTH(32)
  ) cont1_s (
      cont1_key,
      cont1_key_s,
      clk_sys_21_48
  );

  synch_3 #(
      .WIDTH(32)
  ) cont2_s (
      cont2_key,
      cont2_key_s,
      clk_sys_21_48
  );

  synch_3 #(
      .WIDTH(32)
  ) cont3_s (
      cont3_key,
      cont3_key_s,
      clk_sys_21_48
  );

  synch_3 #(
      .WIDTH(32)
  ) cont4_s (
      cont4_key,
      cont4_key_s,
      clk_sys_21_48
  );

  synch_3 #(
      .WIDTH(32)
  ) joy1_s (
      cont1_joy,
      cont1_joy_s,
      clk_sys_21_48
  );

  // Settings
  reg [31:0] reset_delay = 0;
  wire reset_button = reset_delay > 0;

  reg cpu_turbo_enabled = 0;
  reg gsu_turbo_enabled = 0;

  reg multitap_enabled = 0;
  reg lightgun_enabled = 0;
  reg lightgun_type = 0;
  reg [7:0] dpad_aim_speed = 0;
  reg [7:0] joystick_deadzone;
  reg mouse_enabled;

  reg use_square_pixels = 0;
  reg blend_enabled = 0;

  // Settings sync
  wire reset_button_s;

  wire cpu_turbo_enabled_s;
  wire gsu_turbo_enabled_s;

  wire multitap_enabled_s;
  wire lightgun_enabled_s;
  wire lightgun_type_s;
  wire [7:0] dpad_aim_speed_s;
  wire [7:0] joystick_deadzone_s;
  wire mouse_enabled_s;

  wire use_square_pixels_s;
  wire blend_enabled_s;

  synch_3 #(
      .WIDTH(25)
  ) settings_s (
      {
        reset_button,
        cpu_turbo_enabled,
        gsu_turbo_enabled,
        multitap_enabled,
        lightgun_enabled,
        lightgun_type,
        dpad_aim_speed,
        joystick_deadzone,
        mouse_enabled,
        use_square_pixels,
        blend_enabled
      },
      {
        reset_button_s,
        cpu_turbo_enabled_s,
        gsu_turbo_enabled_s,
        multitap_enabled_s,
        lightgun_enabled_s,
        lightgun_type_s,
        dpad_aim_speed_s,
        joystick_deadzone_s,
        mouse_enabled_s,
        use_square_pixels_s,
        blend_enabled_s
      },
      clk_sys_21_48
  );

  reg new_rtc = 0;
  reg [31:0] prev_time = 0;

  always @(posedge clk_74a) begin
    if (rtc_time != prev_time) begin
      prev_time <= rtc_time;
      new_rtc   <= ~new_rtc;
    end
  end

  wire [64:0] rtc = {
    new_rtc,
    8'b0,  // Empty
    8'b1,  // Week day (not supported)
    rtc_date[23:16],  // Year (lower byte)
    rtc_date[15:8],  // Month
    rtc_date[7:0],  // Day
    rtc_time[23:16],  // Hour
    rtc_time[15:8],  // Minute
    rtc_time[7:0]  // Second
  };

  MAIN_SNES snes (
      .clk_mem_85_9 (clk_mem_85_9),
      .clk_sys_21_48(clk_sys_21_48),

      .core_reset(~pll_core_locked || reset_button_s),

      .rtc(rtc),

      // Settings
      .cpu_turbo_enabled(cpu_turbo_enabled_s),
      .gsu_turbo_enabled(gsu_turbo_enabled_s),

      .multitap_enabled(multitap_enabled_s),
      .lightgun_enabled(lightgun_enabled_s),
      .lightgun_type(lightgun_type_s),
      .dpad_aim_speed(dpad_aim_speed_s),
      .mouse_enabled(mouse_enabled_s),

      .blend_enabled(blend_enabled_s),

      // Save states — APF bridge interface
      .ss_save(ss_save),
      .ss_load(ss_load),
      .ss_din(ss_dout),         // save_state_controller output → SNES input (load)
      .ss_dout(ss_din),         // SNES output → save_state_controller input (save)
      .ss_ack(ss_ack),
      .ss_ddr_addr(ss_ddr_addr),
      .ss_we(ss_we),
      .ss_be(ss_be),
      .ss_req(ss_req),
      .ss_busy_out(ss_busy),

      // Phase B: SDRAM staging interface (controller drives, SNES.sv CDCs)
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

      .dbg_rti_arms       (dbg_rti_arms),
      .dbg_vect_reentry   (dbg_vect_reentry),
      .dbg_ddr_writes     (dbg_ddr_writes),
      .dbg_save_end_writes(dbg_save_end_writes),
      .dbg_fw_entry       (dbg_fw_entry),
      .dbg_fw_nmidis      (dbg_fw_nmidis),
      .dbg_fw_at_8000     (dbg_fw_at_8000),
      .dbg_fw_at_8003     (dbg_fw_at_8003),
      .dbg_byte_at_8000   (dbg_byte_at_8000),
      .dbg_byte_at_8001   (dbg_byte_at_8001),
      .dbg_load_byte0     (dbg_load_byte0),
      .dbg_load_byte1     (dbg_load_byte1),
      .dbg_load_en_cnt    (dbg_load_en_cnt),
      .dbg_load_vect_cnt  (dbg_load_vect_cnt),
      .dbg_load_busy_cnt  (dbg_load_busy_cnt),

      // Input
      .p1_button_a(cont1_key_s[4]),
      .p1_button_b(cont1_key_s[5]),
      .p1_button_x(cont1_key_s[6]),
      .p1_button_y(cont1_key_s[7]),
      .p1_button_trig_l(cont1_key_s[8]),
      .p1_button_trig_r(cont1_key_s[9]),
      .p1_button_start(cont1_key_s[15]),
      .p1_button_select(cont1_key_s[14]),
      .p1_dpad_up(cont1_key_s[0]),
      .p1_dpad_down(cont1_key_s[1]),
      .p1_dpad_left(cont1_key_s[2]),
      .p1_dpad_right(cont1_key_s[3]),

      .p1_lstick_x(cont1_joy_x_calibrated),
      .p1_lstick_y(cont1_joy_y_calibrated),

      .p2_button_a(cont2_key_s[4]),
      .p2_button_b(cont2_key_s[5]),
      .p2_button_x(cont2_key_s[6]),
      .p2_button_y(cont2_key_s[7]),
      .p2_button_trig_l(cont2_key_s[8]),
      .p2_button_trig_r(cont2_key_s[9]),
      .p2_button_start(cont2_key_s[15]),
      .p2_button_select(cont2_key_s[14]),
      .p2_dpad_up(cont2_key_s[0]),
      .p2_dpad_down(cont2_key_s[1]),
      .p2_dpad_left(cont2_key_s[2]),
      .p2_dpad_right(cont2_key_s[3]),

      .p3_button_a(cont3_key_s[4]),
      .p3_button_b(cont3_key_s[5]),
      .p3_button_x(cont3_key_s[6]),
      .p3_button_y(cont3_key_s[7]),
      .p3_button_trig_l(cont3_key_s[8]),
      .p3_button_trig_r(cont3_key_s[9]),
      .p3_button_start(cont3_key_s[15]),
      .p3_button_select(cont3_key_s[14]),
      .p3_dpad_up(cont3_key_s[0]),
      .p3_dpad_down(cont3_key_s[1]),
      .p3_dpad_left(cont3_key_s[2]),
      .p3_dpad_right(cont3_key_s[3]),

      .p4_button_a(cont4_key_s[4]),
      .p4_button_b(cont4_key_s[5]),
      .p4_button_x(cont4_key_s[6]),
      .p4_button_y(cont4_key_s[7]),
      .p4_button_trig_l(cont4_key_s[8]),
      .p4_button_trig_r(cont4_key_s[9]),
      .p4_button_start(cont4_key_s[15]),
      .p4_button_select(cont4_key_s[14]),
      .p4_dpad_up(cont4_key_s[0]),
      .p4_dpad_down(cont4_key_s[1]),
      .p4_dpad_left(cont4_key_s[2]),
      .p4_dpad_right(cont4_key_s[3]),

      // ROM loading
      .ioctl_download(ioctl_download),
      .ioctl_wr(ioctl_wr),
      .ioctl_addr(ioctl_addr),
      .ioctl_dout(ioctl_dout),

      .rom_type(rom_type),
      .rom_size(rom_size),
      .ram_size(ram_size),
      .PAL(PAL),

      // Save input/output
      .save_download(save_download_s),
      .sd_rd(sd_rd),
      .sd_wr(sd_wr),
      .sd_buff_addr(sd_buff_addr),
      .sd_buff_din(sd_buff_din),
      .sd_buff_dout(sd_buff_dout),

      .sram_size(sram_size),

      // SDRAM
      .dram_a(dram_a),
      .dram_ba(dram_ba),
      .dram_dq(dram_dq),
      .dram_dqm(dram_dqm),
      .dram_clk(dram_clk),
      .dram_cke(dram_cke),
      .dram_ras_n(dram_ras_n),
      .dram_cas_n(dram_cas_n),
      .dram_we_n(dram_we_n),

      // PSRAM
      .cram0_a(cram0_a),
      .cram0_dq(cram0_dq),
      .cram0_wait(cram0_wait),
      .cram0_clk(cram0_clk),
      .cram0_adv_n(cram0_adv_n),
      .cram0_cre(cram0_cre),
      .cram0_ce0_n(cram0_ce0_n),
      .cram0_ce1_n(cram0_ce1_n),
      .cram0_oe_n(cram0_oe_n),
      .cram0_we_n(cram0_we_n),
      .cram0_ub_n(cram0_ub_n),
      .cram0_lb_n(cram0_lb_n),

      .cram1_a(cram1_a),
      .cram1_dq(cram1_dq),
      .cram1_wait(cram1_wait),
      .cram1_clk(cram1_clk),
      .cram1_adv_n(cram1_adv_n),
      .cram1_cre(cram1_cre),
      .cram1_ce0_n(cram1_ce0_n),
      .cram1_ce1_n(cram1_ce1_n),
      .cram1_oe_n(cram1_oe_n),
      .cram1_we_n(cram1_we_n),
      .cram1_ub_n(cram1_ub_n),
      .cram1_lb_n(cram1_lb_n),

      // Video
      .hblank (h_blank),
      .vblank (v_blank),
      .hsync  (video_hs_snes),
      .vsync  (video_vs_snes),
      .video_r(video_rgb_snes[23:16]),
      .video_g(video_rgb_snes[15:8]),
      .video_b(video_rgb_snes[7:0]),

      // Audio
      .audio_l(audio_l),
      .audio_r(audio_r)
  );

  // Video

  wire h_blank;
  wire v_blank;
  wire video_hs_snes;
  wire video_vs_snes;
  wire [23:0] video_rgb_snes;

  assign video_rgb_clock = clk_video_5_37;
  assign video_rgb_clock_90 = clk_video_5_37_90deg;
  assign video_rgb = rgb;
  assign video_de = de;

  reg de;
  reg [23:0] rgb;
  wire [7:0] snap_index;
  wire [23:0] rgb_out;
  wire de_out;

  scanline_filler #(
      .SNAP_COUNT (2),
      .SNAP_POINTS('{240, 224}),
      .HSYNC_DELAY(1)
  ) scanline_filler (
      .clk(clk_video_5_37),

      .hsync_in(video_hs_snes),
      .vsync_in(video_vs_snes),

      .vblank_in(v_blank),
      .hblank_in(h_blank),
      .rgb_in(video_rgb_snes),

      .hsync(video_hs),
      .vsync(video_vs),

      .de (de_out),
      .rgb(rgb_out),

      .snap_index(snap_index)
  );

  reg prev_de;
  reg prev_vs;
  reg [7:0] latched_snap_index;

  // --------------------------------------------------------------------
  // Debug overlay: once ANYTHING save-state-related has happened, replace
  // the active video with a color that encodes controller + SNES-side state.
  // Stays on permanently so we can read the state even after the controller
  // returned to IDLE.  Helps diagnose the freeze without SignalTap.
  //
  // Encoding:
  //   R[7:4] = sys_state[3:0]
  //     0=IDLE 1=SAVE_ACTIVE 2=SAVE_WAIT_SRAM 3=LOAD_ACTIVE 4=LOAD_WAIT_SRAM
  //   R[3:0] = ss_save_count[3:0] (mod 16) — how many times controller fired ss_save
  //   G[7]   = ss_busy_seen (current, cleared on return to IDLE)
  //   G[6]   = ss_busy_ever (sticky — set on FIRST ss_busy rise, never cleared)
  //   G[5:4] = 0
  //   G[3:0] = ss_busy_rises[3:0] (mod 16)
  //   B[7]   = ss_busy_video (current SNES-side ss_busy)
  //   B[6]   = ss_save_ever (sticky — set on first savestate_start, never cleared)
  //   B[5:0] = 0x3F constant so overlay is always distinctly visible
  // --------------------------------------------------------------------
  reg [1:0] ss_busy_video_sync;
  reg [1:0] ss_busy_seen_video_sync;
  reg [1:0] ss_busy_ever_video_sync;
  reg [1:0] ss_save_ever_video_sync;
  reg [1:0] ss_req_ever_video_sync;
  reg [1:0] core_wr_ever_video_sync;
  reg [1:0] sram_wr_ack_ever_video_sync;
  reg [3:0] debug_sys_state_video_sync_0;
  reg [3:0] debug_sys_state_video_sync_1;
  reg [3:0] debug_ss_req_toggles_video_sync_0;
  reg [3:0] debug_ss_req_toggles_video_sync_1;
  reg [3:0] dbg_rti_arms_sync_0;
  reg [3:0] dbg_rti_arms_sync_1;
  reg [3:0] dbg_vect_reentry_sync_0;
  reg [3:0] dbg_vect_reentry_sync_1;
  reg [3:0] dbg_ddr_writes_sync_0;
  reg [3:0] dbg_ddr_writes_sync_1;
  reg [3:0] dbg_save_end_writes_sync_0;
  reg [3:0] dbg_save_end_writes_sync_1;
  reg [3:0] dbg_fw_entry_sync_0;
  reg [3:0] dbg_fw_entry_sync_1;
  reg [3:0] dbg_fw_nmidis_sync_0;
  reg [3:0] dbg_fw_nmidis_sync_1;
  reg [3:0] dbg_fw_at_8000_sync_0;
  reg [3:0] dbg_fw_at_8000_sync_1;
  reg [3:0] dbg_fw_at_8003_sync_0;
  reg [3:0] dbg_fw_at_8003_sync_1;
  reg [7:0] dbg_byte_at_8000_sync_0;
  reg [7:0] dbg_byte_at_8000_sync_1;
  reg [7:0] dbg_byte_at_8001_sync_0;
  reg [7:0] dbg_byte_at_8001_sync_1;
  reg [7:0] dbg_load_byte0_sync_0;
  reg [7:0] dbg_load_byte0_sync_1;
  reg [7:0] dbg_load_byte1_sync_0;
  reg [7:0] dbg_load_byte1_sync_1;
  reg [3:0] dbg_load_en_cnt_sync_0,   dbg_load_en_cnt_sync_1;
  reg [3:0] dbg_load_vect_cnt_sync_0, dbg_load_vect_cnt_sync_1;
  reg [3:0] dbg_load_busy_cnt_sync_0, dbg_load_busy_cnt_sync_1;
  reg [7:0] dbg_bridge_wr_lo_sync_0;
  reg [7:0] dbg_bridge_wr_lo_sync_1;
  reg [7:0] dbg_bridge_wr_hi_sync_0;
  reg [7:0] dbg_bridge_wr_hi_sync_1;
  reg [7:0] dbg_first_data_b0_sync_0;
  reg [7:0] dbg_first_data_b0_sync_1;
  reg [7:0] dbg_first_data_b1_sync_0;
  reg [7:0] dbg_first_data_b1_sync_1;
  reg [7:0] dbg_first_addr_lo_sync_0;
  reg [7:0] dbg_first_addr_lo_sync_1;
  reg [7:0] dbg_first_addr_hi_sync_0;
  reg [7:0] dbg_first_addr_hi_sync_1;
  reg [7:0] dbg_first_save_b0_sync_0;
  reg [7:0] dbg_first_save_b0_sync_1;
  reg [7:0] dbg_first_save_b1_sync_0;
  reg [7:0] dbg_first_save_b1_sync_1;
  reg [7:0] dbg_first_save_addr_lo_sync_0;
  reg [7:0] dbg_first_save_addr_lo_sync_1;
  reg [7:0] dbg_first_save_addr_hi_sync_0;
  reg [7:0] dbg_first_save_addr_hi_sync_1;
  reg [7:0] dbg_first_pf_addr_lo_sync_0;
  reg [7:0] dbg_first_pf_addr_lo_sync_1;
  reg [7:0] dbg_first_pf_addr_hi_sync_0;
  reg [7:0] dbg_first_pf_addr_hi_sync_1;
  reg [7:0] dbg_first_sram_w0_lo_sync_0;
  reg [7:0] dbg_first_sram_w0_lo_sync_1;
  reg [7:0] dbg_first_sram_w0_hi_sync_0;
  reg [7:0] dbg_first_sram_w0_hi_sync_1;
  reg [7:0] dbg_first_sram_w1_lo_sync_0;
  reg [7:0] dbg_first_sram_w1_lo_sync_1;
  reg [7:0] dbg_first_sram_w1_hi_sync_0;
  reg [7:0] dbg_first_sram_w1_hi_sync_1;
  reg [7:0] dbg_max_sram_base_lo_sync_0;
  reg [7:0] dbg_max_sram_base_lo_sync_1;
  reg [7:0] dbg_max_sram_base_hi_sync_0;
  reg [7:0] dbg_max_sram_base_hi_sync_1;
  reg [7:0] dbg_save_wr_count_lo_sync_0;
  reg [7:0] dbg_save_wr_count_lo_sync_1;
  reg [7:0] dbg_save_wr_count_hi_sync_0;
  reg [7:0] dbg_save_wr_count_hi_sync_1;
  reg [7:0] dbg_ss_addr_overflow_sync_0;
  reg [7:0] dbg_ss_addr_overflow_sync_1;
  reg [7:0] dbg_ss_addr_max_hi_sync_0;
  reg [7:0] dbg_ss_addr_max_hi_sync_1;
  reg [7:0] dbg_pf_at_first_rd_lo_sync_0;
  reg [7:0] dbg_pf_at_first_rd_lo_sync_1;
  reg [7:0] dbg_pf_at_first_rd_hi_sync_0;
  reg [7:0] dbg_pf_at_first_rd_hi_sync_1;
  reg [7:0] dbg_bridge_rd_lo_sync_0;
  reg [7:0] dbg_bridge_rd_lo_sync_1;
  reg [7:0] dbg_bridge_rd_hi_sync_0;
  reg [7:0] dbg_bridge_rd_hi_sync_1;
  reg [7:0] dbg_first_rd_lo_sync_0;
  reg [7:0] dbg_first_rd_lo_sync_1;
  reg [7:0] dbg_first_rd_hi_sync_0;
  reg [7:0] dbg_first_rd_hi_sync_1;
  reg [7:0] dbg_last_w0_lo_sync_0;
  reg [7:0] dbg_last_w0_lo_sync_1;
  reg [7:0] dbg_last_w0_hi_sync_0;
  reg [7:0] dbg_last_w0_hi_sync_1;
  reg [7:0] dbg_w0_wr_count_sync_0;
  reg [7:0] dbg_w0_wr_count_sync_1;

  always @(posedge clk_video_5_37) begin
    ss_busy_video_sync           <= {ss_busy_video_sync[0],           ss_busy};
    ss_busy_seen_video_sync      <= {ss_busy_seen_video_sync[0],      debug_ss_busy_seen};
    ss_busy_ever_video_sync      <= {ss_busy_ever_video_sync[0],      debug_ss_busy_ever};
    ss_save_ever_video_sync      <= {ss_save_ever_video_sync[0],      debug_ss_save_ever};
    ss_req_ever_video_sync       <= {ss_req_ever_video_sync[0],       debug_ss_req_ever};
    core_wr_ever_video_sync      <= {core_wr_ever_video_sync[0],      debug_core_wr_ever};
    sram_wr_ack_ever_video_sync  <= {sram_wr_ack_ever_video_sync[0],  debug_sram_wr_ack_ever};
    debug_sys_state_video_sync_0 <= debug_sys_state;
    debug_sys_state_video_sync_1 <= debug_sys_state_video_sync_0;
    debug_ss_req_toggles_video_sync_0 <= debug_ss_req_toggles;
    debug_ss_req_toggles_video_sync_1 <= debug_ss_req_toggles_video_sync_0;
    dbg_rti_arms_sync_0        <= dbg_rti_arms;
    dbg_rti_arms_sync_1        <= dbg_rti_arms_sync_0;
    dbg_vect_reentry_sync_0    <= dbg_vect_reentry;
    dbg_vect_reentry_sync_1    <= dbg_vect_reentry_sync_0;
    dbg_ddr_writes_sync_0      <= dbg_ddr_writes;
    dbg_ddr_writes_sync_1      <= dbg_ddr_writes_sync_0;
    dbg_save_end_writes_sync_0 <= dbg_save_end_writes;
    dbg_save_end_writes_sync_1 <= dbg_save_end_writes_sync_0;
    dbg_fw_entry_sync_0   <= dbg_fw_entry;
    dbg_fw_entry_sync_1   <= dbg_fw_entry_sync_0;
    dbg_fw_nmidis_sync_0  <= dbg_fw_nmidis;
    dbg_fw_nmidis_sync_1  <= dbg_fw_nmidis_sync_0;
    dbg_fw_at_8000_sync_0 <= dbg_fw_at_8000;
    dbg_fw_at_8000_sync_1 <= dbg_fw_at_8000_sync_0;
    dbg_fw_at_8003_sync_0 <= dbg_fw_at_8003;
    dbg_fw_at_8003_sync_1 <= dbg_fw_at_8003_sync_0;
    dbg_byte_at_8000_sync_0 <= dbg_byte_at_8000;
    dbg_byte_at_8000_sync_1 <= dbg_byte_at_8000_sync_0;
    dbg_byte_at_8001_sync_0 <= dbg_byte_at_8001;
    dbg_byte_at_8001_sync_1 <= dbg_byte_at_8001_sync_0;
    dbg_load_byte0_sync_0   <= dbg_load_byte0;
    dbg_load_byte0_sync_1   <= dbg_load_byte0_sync_0;
    dbg_load_byte1_sync_0   <= dbg_load_byte1;
    dbg_load_byte1_sync_1   <= dbg_load_byte1_sync_0;
    dbg_load_en_cnt_sync_0   <= dbg_load_en_cnt;
    dbg_load_en_cnt_sync_1   <= dbg_load_en_cnt_sync_0;
    dbg_load_vect_cnt_sync_0 <= dbg_load_vect_cnt;
    dbg_load_vect_cnt_sync_1 <= dbg_load_vect_cnt_sync_0;
    dbg_load_busy_cnt_sync_0 <= dbg_load_busy_cnt;
    dbg_load_busy_cnt_sync_1 <= dbg_load_busy_cnt_sync_0;
    dbg_bridge_wr_lo_sync_0 <= debug_bridge_wr_count_lo;
    dbg_bridge_wr_lo_sync_1 <= dbg_bridge_wr_lo_sync_0;
    dbg_bridge_wr_hi_sync_0 <= debug_bridge_wr_count_hi;
    dbg_bridge_wr_hi_sync_1 <= dbg_bridge_wr_hi_sync_0;
    dbg_first_data_b0_sync_0 <= debug_first_wr_data_b0;
    dbg_first_data_b0_sync_1 <= dbg_first_data_b0_sync_0;
    dbg_first_data_b1_sync_0 <= debug_first_wr_data_b1;
    dbg_first_data_b1_sync_1 <= dbg_first_data_b1_sync_0;
    dbg_first_addr_lo_sync_0 <= debug_first_wr_addr_lo;
    dbg_first_addr_lo_sync_1 <= dbg_first_addr_lo_sync_0;
    dbg_first_addr_hi_sync_0 <= debug_first_wr_addr_hi;
    dbg_first_addr_hi_sync_1 <= dbg_first_addr_hi_sync_0;
    dbg_first_save_b0_sync_0 <= debug_first_save_byte0;
    dbg_first_save_b0_sync_1 <= dbg_first_save_b0_sync_0;
    dbg_first_save_b1_sync_0 <= debug_first_save_byte1;
    dbg_first_save_b1_sync_1 <= dbg_first_save_b1_sync_0;
    dbg_bridge_rd_lo_sync_0  <= debug_bridge_rd_count_lo;
    dbg_bridge_rd_lo_sync_1  <= dbg_bridge_rd_lo_sync_0;
    dbg_bridge_rd_hi_sync_0  <= debug_bridge_rd_count_hi;
    dbg_bridge_rd_hi_sync_1  <= dbg_bridge_rd_hi_sync_0;
    dbg_first_rd_lo_sync_0   <= debug_first_rd_addr_lo;
    dbg_first_rd_lo_sync_1   <= dbg_first_rd_lo_sync_0;
    dbg_first_rd_hi_sync_0   <= debug_first_rd_addr_hi;
    dbg_first_rd_hi_sync_1   <= dbg_first_rd_hi_sync_0;
    dbg_last_w0_lo_sync_0    <= debug_last_w0_data_lo;
    dbg_last_w0_lo_sync_1    <= dbg_last_w0_lo_sync_0;
    dbg_last_w0_hi_sync_0    <= debug_last_w0_data_hi;
    dbg_last_w0_hi_sync_1    <= dbg_last_w0_hi_sync_0;
    dbg_w0_wr_count_sync_0   <= debug_w0_wr_count;
    dbg_w0_wr_count_sync_1   <= dbg_w0_wr_count_sync_0;
    dbg_first_save_addr_lo_sync_0 <= debug_first_save_addr_lo;
    dbg_first_save_addr_lo_sync_1 <= dbg_first_save_addr_lo_sync_0;
    dbg_first_save_addr_hi_sync_0 <= debug_first_save_addr_hi;
    dbg_first_save_addr_hi_sync_1 <= dbg_first_save_addr_hi_sync_0;
    dbg_first_pf_addr_lo_sync_0   <= debug_first_pf_addr_lo;
    dbg_first_pf_addr_lo_sync_1   <= dbg_first_pf_addr_lo_sync_0;
    dbg_first_pf_addr_hi_sync_0   <= debug_first_pf_addr_hi;
    dbg_first_pf_addr_hi_sync_1   <= dbg_first_pf_addr_hi_sync_0;
    dbg_first_sram_w0_lo_sync_0   <= debug_first_sram_w0_lo;
    dbg_first_sram_w0_lo_sync_1   <= dbg_first_sram_w0_lo_sync_0;
    dbg_first_sram_w0_hi_sync_0   <= debug_first_sram_w0_hi;
    dbg_first_sram_w0_hi_sync_1   <= dbg_first_sram_w0_hi_sync_0;
    dbg_first_sram_w1_lo_sync_0   <= debug_first_sram_w1_lo;
    dbg_first_sram_w1_lo_sync_1   <= dbg_first_sram_w1_lo_sync_0;
    dbg_first_sram_w1_hi_sync_0   <= debug_first_sram_w1_hi;
    dbg_first_sram_w1_hi_sync_1   <= dbg_first_sram_w1_hi_sync_0;
    dbg_max_sram_base_lo_sync_0   <= debug_max_sram_base_lo;
    dbg_max_sram_base_lo_sync_1   <= dbg_max_sram_base_lo_sync_0;
    dbg_max_sram_base_hi_sync_0   <= debug_max_sram_base_hi;
    dbg_max_sram_base_hi_sync_1   <= dbg_max_sram_base_hi_sync_0;
    dbg_save_wr_count_lo_sync_0   <= debug_save_wr_count_lo;
    dbg_save_wr_count_lo_sync_1   <= dbg_save_wr_count_lo_sync_0;
    dbg_save_wr_count_hi_sync_0   <= debug_save_wr_count_hi;
    dbg_save_wr_count_hi_sync_1   <= dbg_save_wr_count_hi_sync_0;
    dbg_ss_addr_overflow_sync_0   <= debug_ss_addr_overflow;
    dbg_ss_addr_overflow_sync_1   <= dbg_ss_addr_overflow_sync_0;
    dbg_ss_addr_max_hi_sync_0     <= debug_ss_addr_max_hi;
    dbg_ss_addr_max_hi_sync_1     <= dbg_ss_addr_max_hi_sync_0;
    dbg_pf_at_first_rd_lo_sync_0  <= debug_pf_at_first_rd_lo;
    dbg_pf_at_first_rd_lo_sync_1  <= dbg_pf_at_first_rd_lo_sync_0;
    dbg_pf_at_first_rd_hi_sync_0  <= debug_pf_at_first_rd_hi;
    dbg_pf_at_first_rd_hi_sync_1  <= dbg_pf_at_first_rd_hi_sync_0;
  end

  wire ss_busy_video          = ss_busy_video_sync[1];
  wire ss_busy_seen_video     = ss_busy_seen_video_sync[1];
  wire ss_busy_ever_video     = ss_busy_ever_video_sync[1];
  wire ss_save_ever_video     = ss_save_ever_video_sync[1];
  wire ss_req_ever_video      = ss_req_ever_video_sync[1];
  wire core_wr_ever_video     = core_wr_ever_video_sync[1];
  wire sram_wr_ack_ever_video = sram_wr_ack_ever_video_sync[1];
  wire [3:0] debug_sys_state_video     = debug_sys_state_video_sync_1;
  wire [3:0] debug_ss_req_toggles_video = debug_ss_req_toggles_video_sync_1;
  wire [3:0] dbg_rti_arms_video         = dbg_rti_arms_sync_1;
  wire [3:0] dbg_vect_reentry_video     = dbg_vect_reentry_sync_1;
  wire [3:0] dbg_ddr_writes_video       = dbg_ddr_writes_sync_1;
  wire [3:0] dbg_save_end_writes_video  = dbg_save_end_writes_sync_1;
  wire [3:0] dbg_fw_entry_video         = dbg_fw_entry_sync_1;
  wire [3:0] dbg_fw_nmidis_video        = dbg_fw_nmidis_sync_1;
  wire [3:0] dbg_fw_at_8000_video       = dbg_fw_at_8000_sync_1;
  wire [3:0] dbg_fw_at_8003_video       = dbg_fw_at_8003_sync_1;
  wire [7:0] dbg_byte_at_8000_video     = dbg_byte_at_8000_sync_1;
  wire [7:0] dbg_byte_at_8001_video     = dbg_byte_at_8001_sync_1;
  wire [7:0] dbg_load_byte0_video       = dbg_load_byte0_sync_1;
  wire [7:0] dbg_load_byte1_video       = dbg_load_byte1_sync_1;
  wire [3:0] dbg_load_en_cnt_video      = dbg_load_en_cnt_sync_1;
  wire [3:0] dbg_load_vect_cnt_video    = dbg_load_vect_cnt_sync_1;
  wire [3:0] dbg_load_busy_cnt_video    = dbg_load_busy_cnt_sync_1;
  wire [7:0] dbg_bridge_wr_lo_video     = dbg_bridge_wr_lo_sync_1;
  wire [7:0] dbg_bridge_wr_hi_video     = dbg_bridge_wr_hi_sync_1;
  wire [7:0] dbg_first_data_b0_video    = dbg_first_data_b0_sync_1;
  wire [7:0] dbg_first_data_b1_video    = dbg_first_data_b1_sync_1;
  wire [7:0] dbg_first_addr_lo_video    = dbg_first_addr_lo_sync_1;
  wire [7:0] dbg_first_addr_hi_video    = dbg_first_addr_hi_sync_1;
  wire [7:0] dbg_first_save_b0_video    = dbg_first_save_b0_sync_1;
  wire [7:0] dbg_first_save_b1_video    = dbg_first_save_b1_sync_1;
  wire [7:0] dbg_first_save_addr_lo_video = dbg_first_save_addr_lo_sync_1;
  wire [7:0] dbg_first_save_addr_hi_video = dbg_first_save_addr_hi_sync_1;
  wire [7:0] dbg_first_pf_addr_lo_video   = dbg_first_pf_addr_lo_sync_1;
  wire [7:0] dbg_first_pf_addr_hi_video   = dbg_first_pf_addr_hi_sync_1;
  wire [7:0] dbg_first_sram_w0_lo_video   = dbg_first_sram_w0_lo_sync_1;
  wire [7:0] dbg_first_sram_w0_hi_video   = dbg_first_sram_w0_hi_sync_1;
  wire [7:0] dbg_first_sram_w1_lo_video   = dbg_first_sram_w1_lo_sync_1;
  wire [7:0] dbg_first_sram_w1_hi_video   = dbg_first_sram_w1_hi_sync_1;
  wire [7:0] dbg_max_sram_base_lo_video   = dbg_max_sram_base_lo_sync_1;
  wire [7:0] dbg_max_sram_base_hi_video   = dbg_max_sram_base_hi_sync_1;
  wire [7:0] dbg_save_wr_count_lo_video   = dbg_save_wr_count_lo_sync_1;
  wire [7:0] dbg_save_wr_count_hi_video   = dbg_save_wr_count_hi_sync_1;
  wire [7:0] dbg_last_w0_lo_video         = dbg_last_w0_lo_sync_1;
  wire [7:0] dbg_last_w0_hi_video         = dbg_last_w0_hi_sync_1;
  wire [7:0] dbg_w0_wr_count_video        = dbg_w0_wr_count_sync_1;
  wire [7:0] dbg_ss_addr_overflow_video   = dbg_ss_addr_overflow_sync_1;
  wire [7:0] dbg_ss_addr_max_hi_video     = dbg_ss_addr_max_hi_sync_1;
  wire [7:0] dbg_pf_at_first_rd_lo_video  = dbg_pf_at_first_rd_lo_sync_1;
  wire [7:0] dbg_pf_at_first_rd_hi_video  = dbg_pf_at_first_rd_hi_sync_1;
  wire [7:0] dbg_bridge_rd_lo_video     = dbg_bridge_rd_lo_sync_1;
  wire [7:0] dbg_bridge_rd_hi_video     = dbg_bridge_rd_hi_sync_1;
  wire [7:0] dbg_first_rd_lo_video      = dbg_first_rd_lo_sync_1;
  wire [7:0] dbg_first_rd_hi_video      = dbg_first_rd_hi_sync_1;

  wire overlay_enable = ss_busy_video | ss_busy_ever_video | ss_save_ever_video;

  // ====================================================================
  // Text overlay — displays 4 hex digits stacked vertically.
  //
  //   Row 0 (red marker)    : dbg_rti_arms        (RTI arm count)
  //   Row 1 (green marker)  : dbg_vect_reentry    (NMI re-entry count)
  //   Row 2 (yellow marker) : dbg_save_end_writes (Save_end count)
  //   Row 3 (cyan marker)   : ss_req_toggles      (chunk transfer count)
  //
  // Each digit is rendered from an 8x8 bitmap, drawn at 2x scale (16x16).
  // Rows are placed at the top-left of the visible video, one per line block.
  // ====================================================================

  reg [10:0] h_pixel_count;
  reg [10:0] v_line_count;
  reg        prev_de_overlay;
  reg        prev_vs_overlay;
  always @(posedge clk_video_5_37) begin
    prev_de_overlay <= de_out;
    prev_vs_overlay <= video_vs;

    // Reset line count at vsync
    if (video_vs && ~prev_vs_overlay)
      v_line_count <= 11'd0;
    // Increment line count on each de_out falling edge (end of active line)
    else if (~de_out && prev_de_overlay)
      v_line_count <= v_line_count + 11'd1;

    // Reset pixel count at start of each active line
    if (de_out && ~prev_de_overlay)
      h_pixel_count <= 11'd0;
    else if (de_out)
      h_pixel_count <= h_pixel_count + 11'd1;
  end

  // 8x8 font ROM for hex digits 0..F.  Each row is one byte (MSB = leftmost
  // pixel).  Adapted from a standard 8x8 font.
  function [7:0] font_rom (input [3:0] digit, input [2:0] row);
    case ({digit, row})
      // 0
      7'h00: font_rom = 8'b00111100;
      7'h01: font_rom = 8'b01100110;
      7'h02: font_rom = 8'b01101110;
      7'h03: font_rom = 8'b01110110;
      7'h04: font_rom = 8'b01100110;
      7'h05: font_rom = 8'b01100110;
      7'h06: font_rom = 8'b00111100;
      7'h07: font_rom = 8'b00000000;
      // 1
      7'h08: font_rom = 8'b00011000;
      7'h09: font_rom = 8'b00111000;
      7'h0A: font_rom = 8'b00011000;
      7'h0B: font_rom = 8'b00011000;
      7'h0C: font_rom = 8'b00011000;
      7'h0D: font_rom = 8'b00011000;
      7'h0E: font_rom = 8'b01111110;
      7'h0F: font_rom = 8'b00000000;
      // 2
      7'h10: font_rom = 8'b00111100;
      7'h11: font_rom = 8'b01100110;
      7'h12: font_rom = 8'b00000110;
      7'h13: font_rom = 8'b00001100;
      7'h14: font_rom = 8'b00110000;
      7'h15: font_rom = 8'b01100000;
      7'h16: font_rom = 8'b01111110;
      7'h17: font_rom = 8'b00000000;
      // 3
      7'h18: font_rom = 8'b00111100;
      7'h19: font_rom = 8'b01100110;
      7'h1A: font_rom = 8'b00000110;
      7'h1B: font_rom = 8'b00011100;
      7'h1C: font_rom = 8'b00000110;
      7'h1D: font_rom = 8'b01100110;
      7'h1E: font_rom = 8'b00111100;
      7'h1F: font_rom = 8'b00000000;
      // 4
      7'h20: font_rom = 8'b00001100;
      7'h21: font_rom = 8'b00011100;
      7'h22: font_rom = 8'b00111100;
      7'h23: font_rom = 8'b01101100;
      7'h24: font_rom = 8'b01111110;
      7'h25: font_rom = 8'b00001100;
      7'h26: font_rom = 8'b00001100;
      7'h27: font_rom = 8'b00000000;
      // 5
      7'h28: font_rom = 8'b01111110;
      7'h29: font_rom = 8'b01100000;
      7'h2A: font_rom = 8'b01111100;
      7'h2B: font_rom = 8'b00000110;
      7'h2C: font_rom = 8'b00000110;
      7'h2D: font_rom = 8'b01100110;
      7'h2E: font_rom = 8'b00111100;
      7'h2F: font_rom = 8'b00000000;
      // 6
      7'h30: font_rom = 8'b00111100;
      7'h31: font_rom = 8'b01100110;
      7'h32: font_rom = 8'b01100000;
      7'h33: font_rom = 8'b01111100;
      7'h34: font_rom = 8'b01100110;
      7'h35: font_rom = 8'b01100110;
      7'h36: font_rom = 8'b00111100;
      7'h37: font_rom = 8'b00000000;
      // 7
      7'h38: font_rom = 8'b01111110;
      7'h39: font_rom = 8'b00000110;
      7'h3A: font_rom = 8'b00001100;
      7'h3B: font_rom = 8'b00011000;
      7'h3C: font_rom = 8'b00110000;
      7'h3D: font_rom = 8'b00110000;
      7'h3E: font_rom = 8'b00110000;
      7'h3F: font_rom = 8'b00000000;
      // 8
      7'h40: font_rom = 8'b00111100;
      7'h41: font_rom = 8'b01100110;
      7'h42: font_rom = 8'b01100110;
      7'h43: font_rom = 8'b00111100;
      7'h44: font_rom = 8'b01100110;
      7'h45: font_rom = 8'b01100110;
      7'h46: font_rom = 8'b00111100;
      7'h47: font_rom = 8'b00000000;
      // 9
      7'h48: font_rom = 8'b00111100;
      7'h49: font_rom = 8'b01100110;
      7'h4A: font_rom = 8'b01100110;
      7'h4B: font_rom = 8'b00111110;
      7'h4C: font_rom = 8'b00000110;
      7'h4D: font_rom = 8'b01100110;
      7'h4E: font_rom = 8'b00111100;
      7'h4F: font_rom = 8'b00000000;
      // A
      7'h50: font_rom = 8'b00111100;
      7'h51: font_rom = 8'b01100110;
      7'h52: font_rom = 8'b01100110;
      7'h53: font_rom = 8'b01111110;
      7'h54: font_rom = 8'b01100110;
      7'h55: font_rom = 8'b01100110;
      7'h56: font_rom = 8'b01100110;
      7'h57: font_rom = 8'b00000000;
      // B
      7'h58: font_rom = 8'b01111100;
      7'h59: font_rom = 8'b01100110;
      7'h5A: font_rom = 8'b01100110;
      7'h5B: font_rom = 8'b01111100;
      7'h5C: font_rom = 8'b01100110;
      7'h5D: font_rom = 8'b01100110;
      7'h5E: font_rom = 8'b01111100;
      7'h5F: font_rom = 8'b00000000;
      // C
      7'h60: font_rom = 8'b00111100;
      7'h61: font_rom = 8'b01100110;
      7'h62: font_rom = 8'b01100000;
      7'h63: font_rom = 8'b01100000;
      7'h64: font_rom = 8'b01100000;
      7'h65: font_rom = 8'b01100110;
      7'h66: font_rom = 8'b00111100;
      7'h67: font_rom = 8'b00000000;
      // D
      7'h68: font_rom = 8'b01111000;
      7'h69: font_rom = 8'b01101100;
      7'h6A: font_rom = 8'b01100110;
      7'h6B: font_rom = 8'b01100110;
      7'h6C: font_rom = 8'b01100110;
      7'h6D: font_rom = 8'b01101100;
      7'h6E: font_rom = 8'b01111000;
      7'h6F: font_rom = 8'b00000000;
      // E
      7'h70: font_rom = 8'b01111110;
      7'h71: font_rom = 8'b01100000;
      7'h72: font_rom = 8'b01100000;
      7'h73: font_rom = 8'b01111000;
      7'h74: font_rom = 8'b01100000;
      7'h75: font_rom = 8'b01100000;
      7'h76: font_rom = 8'b01111110;
      7'h77: font_rom = 8'b00000000;
      // F
      7'h78: font_rom = 8'b01111110;
      7'h79: font_rom = 8'b01100000;
      7'h7A: font_rom = 8'b01100000;
      7'h7B: font_rom = 8'b01111000;
      7'h7C: font_rom = 8'b01100000;
      7'h7D: font_rom = 8'b01100000;
      7'h7E: font_rom = 8'b01100000;
      7'h7F: font_rom = 8'b00000000;
      default: font_rom = 8'b00000000;
    endcase
  endfunction

  // Layout: 4 rows × 2 digits per row.  Each digit drawn at 2x scale = 16x16.
  // Row height = 18 lines (16 + 2 px gap).
  // Each row also has a 16x16 colored "marker" tile on its left to identify it.
  //
  //   x:0..15       = colored row marker
  //   x:16..31      = digit hi-nibble
  //   x:32..47      = digit lo-nibble
  //
  //   Row 0 (lines 0..15)  : RED    marker, rti_arms       (4-bit)
  //   Row 1 (lines 18..33) : GREEN  marker, vect_reentry   (4-bit)
  //   Row 2 (lines 36..51) : YELLOW marker, save_end_wr    (4-bit)
  //   Row 3 (lines 54..69) : CYAN   marker, ss_req_toggles (4-bit)

  // 4 rows of 16 lines each, stacked at the top of the screen.
  wire [1:0] text_row    = v_line_count[5:4];          // 0..3 (each block is 16 lines)
  wire [3:0] text_row_y  = v_line_count[3:0];          // y within the block
  wire       text_active = (v_line_count < 11'd64);    // first 64 lines

  // x position breaks into 4 columns of 16 px each
  wire [1:0] text_col    = h_pixel_count[5:4];         // 0..3 (column)
  wire [3:0] text_col_x  = h_pixel_count[3:0];         // x within column
  wire       text_col_active = (h_pixel_count < 11'd64);

  // Pick the value for this row (full 8 bits — displayed as two hex digits)
  reg [7:0] row_value;
  reg [23:0] row_marker_rgb;
  always @(*) begin
    case (text_row)
      // Overflow was fixed (Red=$00) but the file still shows 'SNES' at
      // payload offset 0x3F806 — meaning APF reads SRAM starting from
      // SRAM byte ~0x7FA (= word 0x3FD = 1021), wrapping.  Determine
      // whether the controller's pf_next_addr was AT 0 when APF's first
      // bridge_rd arrived (= correct) or had advanced (= our bug).
      //
      // Row 0 (RED)    : pf_at_first_rd_hi — pf_next_addr bits [9:8] at
      //                  first bridge_rd.  Expect $00 (= addr was 0..255).
      // Row 1 (GREEN)  : pf_at_first_rd_lo — pf_next_addr bits [7:0] at
      //                  first bridge_rd.  Expect $02 (= we just fetched
      //                  word 0 and advanced to 2).  If $00, FSM hadn't
      //                  finished its first prefetch yet.  If much larger
      //                  (e.g. $FA), spurious advances happened.
      // Row 2 (YELLOW) : ss_addr_max_hi (sanity, should still be $61)
      // Row 3 (CYAN)   : save_wr_count_lo (sanity, should still be $D0)
      // Inspect what APF writes to SRAM during the LOAD phase.  If the
      // first bridge_wr at 4xxxxxxx has data $53 4E ... (file 'SNES'
      // header) at address 0, then APF is doing the right thing and the
      // bug is in our bridge_wr SRAM handler.  If the data or address
      // looks different, APF is using a different protocol than we expect.
      //
      // ARAM-enabled diagnostic: find out why SRAM word 0 is no longer 'SNES'.
      //
      // Row 0 (RED)    : dbg_ss_addr_overflow — sticky flag, non-zero iff
      //                  ss_addr bits [16:15] ever went high during save
      //                  (= save > 256KB, chunks wrapped onto addr 0).
      //                  Expect $00 if save fits.  Non-zero = WRAP happened.
      // Row 1 (GREEN)  : ss_addr_max_hi — max value of ss_addr[16:8] seen.
      //                  If overflow=0, this stays under $80 (15-bit fit).
      //                  If >= $80, ss_addr[15] went high (= overflow).
      // Row 2 (YELLOW) : debug_first_save_byte0 — first byte SNES wrote to
      //                  SRAM during save.  Expect $53='S'.  If $53,
      //                  firmware wrote header correctly and a LATER wrap
      //                  overwrote it.  If not $53, firmware itself misbehaved.
      // Row 3 (CYAN)   : save_wr_count_lo — low byte of # of SRAM core writes.
      //                  Each write = 8 bytes.  $4000 writes = 128KB. $8000
      //                  writes = 256KB.  This rolls over modulo 256.
      // Phase C serve-diagnostic overlay v4 — FSM action counters:
      //   RED   : cnt_ss_load_pulses (= dbg_first_wr_addr_hi).  Should be
      //           $01 if we transitioned to serve exactly once.
      //   GREEN : cnt_serve_wait_entries (= dbg_first_wr_data_b0).
      //           Should match RED.
      //   YELLOW: cnt_serve_rd_entries (= dbg_first_wr_data_b1).  # of
      //           times firmware ss_req fired new_ddr_req && ss_rnw.
      //           $00 = firmware never asked for a chunk.
      //   CYAN  : cnt_serve_ack_entries (= dbg_first_wr_addr_lo).
      //           # of full 4-word reads completed.  Should equal yellow.
      // Phase C overlay v18 — capture FIRST served chunk (proves capture
      // mechanism) + chunk-serve depth.  Expected first chunk bytes:
      // payload +0x00 = "SNES-SS\0" = 53 4E 45 53 2D 53 53 00.
      //   RED   : chunk0 byte0 — want $53 ('S')
      //   GREEN : chunk0 byte1 — want $4E ('N')
      //   YELLOW: cnt_serve_ack_entries — # full 4-word serves completed
      //           ($FF saturated => served >=255 chunks => deep load went
      //           through).
      //   CYAN  : chunk0 byte2 — want $45 ('E')
      // If all four reasonable => serve mechanism works deep; corruption
      // is in completion handshake or a specific block handler.
      2'd0: begin row_value = dbg_first_sram_w0_lo_video; row_marker_rgb = 24'hFF0000; end
      2'd1: begin row_value = dbg_first_sram_w0_hi_video; row_marker_rgb = 24'h00FF00; end
      2'd2: begin row_value = dbg_first_addr_lo_video;    row_marker_rgb = 24'hFFFF00; end
      2'd3: begin row_value = dbg_first_sram_w1_lo_video; row_marker_rgb = 24'h00FFFF; end
    endcase
  end

  // Display two hex digits: high nibble in column 1, low nibble in column 2.
  wire [3:0] digit_hi = row_value[7:4];
  wire [3:0] digit_lo = row_value[3:0];
  wire [3:0] digit_to_show = (text_col == 2'd1) ? digit_hi : digit_lo;
  wire [2:0] font_row = text_row_y[3:1];   // 16-line row / 2 = 8-line font row
  wire [2:0] font_col = text_col_x[3:1];   // 16-px column / 2 = 8-px font col
  wire [7:0] font_line = font_rom(digit_to_show, font_row);
  wire       font_pixel = font_line[3'd7 - font_col];

  reg [23:0] text_overlay_rgb;
  reg        text_overlay_hit;
  always @(*) begin
    text_overlay_hit = 0;
    text_overlay_rgb = 24'h000000;
    if (text_active && text_col_active) begin
      text_overlay_hit = 1;
      case (text_col)
        2'd0: text_overlay_rgb = row_marker_rgb;                           // colored marker block
        2'd1: text_overlay_rgb = font_pixel ? 24'hFFFFFF : 24'h000000;     // hex digit hi
        2'd2: text_overlay_rgb = font_pixel ? 24'hFFFFFF : 24'h000000;     // hex digit lo
        default: text_overlay_rgb = 24'h000000;
      endcase
    end
  end

  // (text_overlay_rgb / text_overlay_hit are used directly in the rgb mux below)

  always @(posedge clk_video_5_37) begin
    prev_de <= de_out;
    prev_vs <= video_vs;

    de <= 0;

    if (video_vs && ~prev_vs) begin
      latched_snap_index <= snap_index;
    end

    if (~de_out && prev_de) begin
      // Write video slot
      rgb <= {9'b0, ~latched_snap_index[0], use_square_pixels_s, 10'b0, 3'b0};
    end else if (de_out) begin
      de  <= 1;
      // Only override the small text-overlay region; leave the rest of the
      // frame as the game's output.  This way we can see both the debug
      // counters AND whether the game/firmware is still drawing anything.
      rgb <= (overlay_enable && text_overlay_hit) ? text_overlay_rgb : rgb_out;
    end
  end

  sound_i2s #(
      .CHANNEL_WIDTH(16),
      .SIGNED_INPUT (1)
  ) sound_i2s (
      .clk_74a  (clk_74a),
      .clk_audio(clk_sys_21_48),

      .audio_l(audio_l),
      .audio_r(audio_r),

      .audio_mclk(audio_mclk),
      .audio_lrck(audio_lrck),
      .audio_dac (audio_dac)
  );

  ///////////////////////////////////////////////

  wire clk_mem_85_9;
  wire clk_sys_21_48;
  wire clk_video_5_37;
  wire clk_video_5_37_90deg;

  wire pll_core_locked;

  parameter PAL_PLL = 1'b0;

  generate
    if (PAL_PLL) begin
      mf_pllbase_pal mp1 (
          .refclk(clk_74a),

          .outclk_0(clk_mem_85_9),
          .outclk_1(clk_sys_21_48),
          .outclk_2(clk_video_5_37),
          .outclk_3(clk_video_5_37_90deg),

          .locked(pll_core_locked)
      );
    end else begin
      mf_pllbase mp1 (
          .refclk(clk_74a),

          .outclk_0(clk_mem_85_9),
          .outclk_1(clk_sys_21_48),
          .outclk_2(clk_video_5_37),
          .outclk_3(clk_video_5_37_90deg),

          .locked(pll_core_locked)
      );
    end
  endgenerate

endmodule
