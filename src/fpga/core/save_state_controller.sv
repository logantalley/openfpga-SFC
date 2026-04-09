// Save State Controller
//
// Bridges the APF host savestate protocol to the MiSTer-style toggle-based
// DDR interface used by savestates.sv.
//
// APF side (clk_74a):
//   - Bridge writes (0x4xxxxxxx) fill the load FIFO (32→64 bit)
//   - Bridge reads (0x4xxxxxxx) drain the save FIFO (64→32 bit)
//   - savestate_start / savestate_load handshake with core_bridge_cmd
//
// Core side (clk_sys):
//   - ss_save / ss_load pulses trigger save/load in savestates.sv
//   - Toggle-based ddr_req/ddr_ack interface for 64-bit data streaming
//

module save_state_controller (
    input wire clk_74a,
    input wire clk_sys,

    // APF Bridge
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
    output wire [63:0] ss_dout,   // Data to core (load)
    input wire [16:0] ss_addr,    // Address (for ordering, not used for FIFO)
    input wire ss_rnw,            // Read/not-write (0=write/save, 1=read/load)
    input wire ss_req,            // Toggle request from savestates.sv
    input wire [7:0] ss_be,       // Byte enable
    output reg ss_ack,            // Toggle acknowledge to savestates.sv

    input wire ss_busy
);

  // ===== Clock Domain Crossing: APF handshake =====
  wire savestate_load_s;
  wire savestate_start_s;

  synch_3 #(
      .WIDTH(2)
  ) savestate_in (
      {savestate_load, savestate_start},
      {savestate_load_s, savestate_start_s},
      clk_sys
  );

  reg savestate_load_ack;
  reg savestate_load_busy;
  reg savestate_load_ok;
  reg savestate_load_err;

  reg savestate_start_ack;
  reg savestate_start_busy;
  reg savestate_start_ok;
  reg savestate_start_err;

  synch_3 #(
      .WIDTH(8)
  ) savestate_out (
      {
        savestate_load_ack,
        savestate_load_busy,
        savestate_load_ok,
        savestate_load_err,
        savestate_start_ack,
        savestate_start_busy,
        savestate_start_ok,
        savestate_start_err
      },
      {
        savestate_load_ack_s,
        savestate_load_busy_s,
        savestate_load_ok_s,
        savestate_load_err_s,
        savestate_start_ack_s,
        savestate_start_busy_s,
        savestate_start_ok_s,
        savestate_start_err_s
      },
      clk_74a
  );

  // ===== Load FIFO: APF bridge writes (32-bit) → core reads (64-bit) =====
  wire fifo_load_empty;
  reg fifo_load_read_req = 0;
  wire [63:0] fifo_load_dout;
  reg fifo_load_clr = 0;

  // Byte-swap for APF big-endian ↔ core little-endian
  assign ss_dout = {
    fifo_load_dout[39:32],
    fifo_load_dout[47:40],
    fifo_load_dout[55:48],
    fifo_load_dout[63:56],
    fifo_load_dout[7:0],
    fifo_load_dout[15:8],
    fifo_load_dout[23:16],
    fifo_load_dout[31:24]
  };

  dcfifo_mixed_widths fifo_load (
      .data(bridge_wr_data),
      .rdclk(clk_sys),
      .rdreq(fifo_load_read_req),
      .wrclk(clk_74a),
      .wrreq(bridge_wr && bridge_addr[31:28] == 4'h4),
      .q(fifo_load_dout),
      .rdempty(fifo_load_empty),
      .aclr(fifo_load_clr)
  );
  defparam fifo_load.intended_device_family = "Cyclone V",
      fifo_load.lpm_numwords = 4096,
      fifo_load.lpm_showahead = "OFF",
      fifo_load.lpm_type = "dcfifo_mixed_widths",
      fifo_load.lpm_width = 32,
      fifo_load.lpm_widthu = 12,
      fifo_load.lpm_widthu_r = 11,
      fifo_load.lpm_width_r = 64,
      fifo_load.overflow_checking = "OFF",
      fifo_load.rdsync_delaypipe = 5,
      fifo_load.underflow_checking = "ON",
      fifo_load.use_eab = "ON",
      fifo_load.wrsync_delaypipe = 5,
      fifo_load.write_aclr_synch = "ON";

  // ===== Save FIFO: core writes (64-bit) → APF bridge reads (32-bit) =====
  reg  fifo_save_write_req;
  reg  fifo_save_read_req;
  wire fifo_save_rd_empty;
  wire fifo_save_wr_empty;

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
      .aclr(1'b0)
  );
  defparam fifo_save.intended_device_family = "Cyclone V",
      fifo_save.lpm_numwords = 4,
      fifo_save.lpm_showahead = "ON",
      fifo_save.lpm_type = "dcfifo_mixed_widths",
      fifo_save.lpm_width = 64,
      fifo_save.lpm_widthu = 2,
      fifo_save.lpm_widthu_r = 3,
      fifo_save.lpm_width_r = 32,
      fifo_save.overflow_checking = "ON",
      fifo_save.rdsync_delaypipe = 5,
      fifo_save.underflow_checking = "ON",
      fifo_save.use_eab = "ON",
      fifo_save.wrsync_delaypipe = 5;

  // ===== APF bridge read side (clk_74a domain) =====
  reg prev_bridge_rd;
  reg [1:0] save_read_state = 0;
  reg [20:0] last_unloader_addr = 21'h1FFFFF;

  wire [27:0] bridge_save_addr = bridge_addr[27:0];

  localparam SAVE_READ_REQ = 1;

  always @(posedge clk_74a) begin
    prev_bridge_rd <= bridge_rd;

    if (bridge_rd && ~prev_bridge_rd && bridge_addr[31:28] == 4'h4) begin
      if (~fifo_save_rd_empty && bridge_save_addr[22:2] != last_unloader_addr) begin
        save_read_state <= SAVE_READ_REQ;
        fifo_save_read_req <= 1;
        last_unloader_addr <= bridge_save_addr[22:2];
      end
    end

    case (save_read_state)
      SAVE_READ_REQ: begin
        save_read_state <= 0;
        fifo_save_read_req <= 0;
      end
    endcase
  end

  // ===== Core-side state machine (clk_sys domain) =====
  // Handles toggle-based DDR req/ack conversion

  localparam NONE = 0;

  localparam SAVE_BUSY = 1;
  localparam SAVE_WAIT_REQ = 2;
  localparam SAVE_WAIT_REQ_DELAY = 3;
  localparam SAVE_WAIT_ACK = 4;

  localparam LOAD_WAIT_REQ = 20;
  localparam LOAD_READ_REQ = 21;
  localparam LOAD_WAIT_APF_START = 22;
  localparam LOAD_APF_COMPLETE = 23;

  reg [7:0] state = NONE;

  reg load_ack_pending = 0;
  reg save_state_loading = 0;
  reg did_req = 0;

  reg prev_savestate_start = 0;
  reg prev_savestate_load = 0;
  reg prev_ss_busy = 0;
  reg prev_ss_req = 0;

  // Detect toggle change on ss_req (MiSTer DDR style)
  wire new_ddr_req = (ss_req != prev_ss_req);

  always @(posedge clk_sys) begin
    prev_ss_busy <= ss_busy;
    prev_savestate_start <= savestate_start_s;
    prev_savestate_load <= savestate_load_s;
    prev_ss_req <= ss_req;

    ss_load <= 0;
    ss_save <= 0;
    fifo_save_write_req <= 0;

    // Detect load FIFO data arriving — begin loading into core
    if (~fifo_load_empty && ~save_state_loading && state == NONE) begin
      state <= LOAD_WAIT_REQ;
      save_state_loading <= 1;
      ss_load <= 1;
    end

    // APF triggers save
    if (savestate_start_s && ~prev_savestate_start) begin
      state <= SAVE_BUSY;

      savestate_start_ack <= 1;
      savestate_start_ok <= 0;
      savestate_start_err <= 0;

      savestate_load_ok <= 0;
      savestate_load_err <= 0;

      ss_save <= 1;
    end else if (savestate_load_s && ~prev_savestate_load) begin
      // APF signals load is ready (data already copied into FIFO)
      load_ack_pending <= 1;

      savestate_load_ack <= 1;
      savestate_load_ok <= 0;
      savestate_load_err <= 0;

      savestate_start_ok <= 0;
      savestate_start_err <= 0;
    end

    case (state)
      // ===== Saving =====
      SAVE_BUSY: begin
        savestate_start_ack <= 0;
        savestate_start_busy <= 1;

        if (new_ddr_req) begin
          // First request from savestates.sv — data available
          state <= SAVE_WAIT_REQ_DELAY;
          fifo_save_write_req <= 1;
        end else if (prev_ss_busy && ~ss_busy) begin
          // Save ended before any DDR stream request completed
          state <= NONE;
          savestate_start_busy <= 0;
          savestate_start_err <= 1;
        end
      end

      SAVE_WAIT_REQ: begin
        // Wait for savestates.sv to send more data (toggle ddr_req)
        if (new_ddr_req) begin
          state <= SAVE_WAIT_REQ_DELAY;
          fifo_save_write_req <= 1;
        end else if (prev_ss_busy && ~ss_busy) begin
          // Core finished saving
          state <= NONE;
          savestate_start_busy <= 0;
          savestate_start_ok <= 1;
        end
      end

      SAVE_WAIT_REQ_DELAY: begin
        // Delay for FIFO empty signal to update
        state <= SAVE_WAIT_ACK;
      end

      SAVE_WAIT_ACK: begin
        // Wait for bridge to drain save FIFO word
        if (fifo_save_wr_empty) begin
          state <= SAVE_WAIT_REQ;
          ss_ack <= ~ss_ack;  // Toggle ack back to savestates.sv
        end
      end

      // ===== Loading =====
      LOAD_WAIT_REQ: begin
        if (prev_ss_busy && ~ss_busy) begin
          // Core finished loading. Wait for APF ack.
          state <= LOAD_WAIT_APF_START;
        end else if (~fifo_load_empty) begin
          if (did_req || new_ddr_req) begin
            // savestates.sv wants data and FIFO has it
            state <= LOAD_READ_REQ;
            fifo_load_read_req <= 1;
            did_req <= 0;
          end
        end else if (new_ddr_req) begin
          // savestates.sv wants data but FIFO is empty — remember request
          did_req <= 1;
        end
      end

      LOAD_READ_REQ: begin
        // FIFO data should be available after 1 cycle
        state <= LOAD_WAIT_REQ;
        fifo_load_read_req <= 0;
        ss_ack <= ~ss_ack;  // Toggle ack — data available on ss_dout
      end

      LOAD_WAIT_APF_START: begin
        if (load_ack_pending) begin
          // APF acknowledged load
          state <= LOAD_APF_COMPLETE;
          fifo_load_clr <= 1;

          savestate_load_ack <= 0;
          savestate_load_busy <= 1;

          load_ack_pending <= 0;
        end
      end

      LOAD_APF_COMPLETE: begin
        state <= NONE;
        fifo_load_clr <= 0;

        savestate_load_busy <= 0;
        savestate_load_ok <= 1;

        save_state_loading <= 0;
      end
    endcase
  end

endmodule
