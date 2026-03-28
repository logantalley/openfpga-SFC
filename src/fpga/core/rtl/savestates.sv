// savestates.sv — Analogue Pocket adaptation of MiSTer SNES_MiSTer savestates.sv
//
// Replaces the DDR/HPS backend with a byte-addressable BRAM that the APF
// bridge can read/write directly through main.v.
//
// Interface contract (matches what main.v already expects from MiSTer minus
// the DDR signals):
//
//   SAVE path:  APF triggers ss_save pulse → savestates.sv waits for the
//               next NMI/IRQ vector read → hijacks it to $8000 (Save_start)
//               → SNES CPU runs savestates.asm → each byte written to
//               $C06000 lands in bram[] at bram_wr_addr → when $C0600E
//               (SS_END) is written the module de-asserts ss_busy.
//               APF can then DMA the BRAM out to SD.
//
//   LOAD path:  APF fills BRAM via bram_wr / bram_wr_addr / bram_wr_data
//               (bridge clock domain), then pulses ss_load → savestates.sv
//               waits for next NMI/IRQ vector read → hijacks it to $8004
//               (Load_start) → SNES CPU runs load path → each $C06000 read
//               returns bram[bram_rd_addr] → RTI at $8008 ends the session.
//
// BRAM size:  The largest save state (WRAM 128 KB + VRAM 64 KB + …) fits
//             comfortably in 512 KB.  We allocate 20-bit addresses (1 MB
//             address space) so the same BRAM can be used for all mappers.
//             Actual data written never exceeds ~200 KB in practice.
//
// Clock domains:
//   clk        — master SNES clock (21.477 MHz), all SNES bus logic
//   clk_74a    — APF bridge clock (74.25 MHz), BRAM write port from APF
//
// -----------------------------------------------------------------------------

module savestates
(
    input  wire        reset_n,
    input  wire        clk,           // SNES master clock

    // APF-side triggers (already synchronised to clk by caller)
    input  wire        ss_save,       // pulse: begin save
    input  wire        ss_load,       // pulse: begin load

    // BRAM bridge — APF writes here to stage a load, reads here after a save.
    // These signals are in clk_74a domain; the BRAM is true dual-port.
    input  wire        clk_74a,
    input  wire        bram_wr,
    input  wire [19:0] bram_wr_addr,
    input  wire  [7:0] bram_wr_data,
    output wire  [7:0] bram_rd_data,  // APF read-back after save
    input  wire [19:0] bram_rd_addr,

    // ROM config (driven by mapper detection in main.v)
    input  wire  [3:0] ram_size,
    input  wire  [7:0] rom_type,

    // SNES clock enables
    input  wire        sysclkf_ce,
    input  wire        sysclkr_ce,

    // SNES CPU bus
    input  wire        romsel_n,
    input  wire [23:0] ca,
    input  wire        cpurd_n,
    input  wire        cpuwr_n,

    // SNES peripheral bus
    input  wire  [7:0] pa,
    input  wire        pard_n,
    input  wire        pawr_n,

    // Data bus
    input  wire  [7:0] di,           // CPU data in (what CPU is writing)
    output reg   [7:0] ss_do,        // data override to CPU data bus

    // ROM address override (points into the savestates.bin image in ROM)
    output reg  [23:0] rom_addr,

    // External (SPC/BSRAM/DSPn) address counter — passed to SPC/BSRAM
    output wire [19:0] ext_addr,

    // SPC700 / APU DI mux (savestates.asm reads SPC via $2184–$2188)
    input  wire  [7:0] spc_di,

    // PPU register readback ($C1:21xx)
    input  wire  [7:0] ppu_di,

    // BSRAM
    output wire        bsram_sel,
    input  wire  [7:0] bsram_di,

    // DSPn (optional coprocessor)
    output wire        dspn_regs_sel,
    output wire        dspn_ram_sel,
    input  wire  [7:0] dspn_di,

    // GSU (SuperFX)
    output wire        gsu_regs_sel,
    input  wire  [7:0] gsu_di,

    // SA-1
    input  wire        sa1_active,
    input  wire [23:0] sa1_a,
    input  wire  [7:0] sa1_di,
    input  wire        sa1_rd_n,
    input  wire        sa1_wr_n,
    input  wire        sa1_sa1_romsel,
    input  wire        sa1_sns_romsel,

    // Output overrides
    output wire        ss_do_ovr,    // 1 = replace CPU DI with ss_do
    output wire        ss_rom_ovr,   // 1 = replace ROM address with rom_addr
    output reg         ss_busy       // 1 = save/load session in progress
);

// ---------------------------------------------------------------------------
// Edge detectors for CPU and peripheral bus
// ---------------------------------------------------------------------------
reg cpurd_n_old, cpuwr_n_old;
reg pard_n_old,  pawr_n_old;
reg ss_save_old, ss_load_old;

always @(posedge clk or negedge reset_n) begin
    if (~reset_n) begin
        cpurd_n_old <= 1'b1;
        cpuwr_n_old <= 1'b1;
        pard_n_old  <= 1'b1;
        pawr_n_old  <= 1'b1;
        ss_save_old <= 1'b0;
        ss_load_old <= 1'b0;
    end else begin
        cpurd_n_old <= cpurd_n;
        cpuwr_n_old <= cpuwr_n;
        pard_n_old  <= pard_n;
        pawr_n_old  <= pawr_n;
        ss_save_old <= ss_save;
        ss_load_old <= ss_load;
    end
end

wire cpurd_ce   =  cpurd_n_old & ~cpurd_n;
wire cpurd_ce_n = ~cpurd_n_old &  cpurd_n;
wire cpuwr_ce   =  cpuwr_n_old & ~cpuwr_n;
wire cpuwr_ce_n = ~cpuwr_n_old &  cpuwr_n;

wire pard_ce    =  pard_n_old  & ~pard_n;
wire pard_ce_n  = ~pard_n_old  &  pard_n;
wire pawr_ce    =  pawr_n_old  & ~pawr_n;
wire pawr_ce_n  = ~pawr_n_old  &  pawr_n;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
reg save_en, load_en;
reg rd_rti;
reg save_end;
reg load_ready;   // BRAM has been validated as containing a "SNES" header

// ---------------------------------------------------------------------------
// Address decode — mirrors savestates.asm constants
//
//   $C0:6000   SSDATA   — byte stream read/write
//   $C0:6001   SSADDR   — write resets internal address counter to 8
//   $C0:6002   SS_EXT_ADDR — write resets ext counter
//   $C0:6003   SS_BSRAMSIZE
//   $C0:6004   SS_ROMTYPE
//   $C0:600E   SS_END   — write signals end of save
//   $C0:600F   SS_STATUS — read: bit1=busy, bit0=save_en
//   $C0:61xx   DSPn register space
//   $C0:62xx   GSU register space
//   $C1:21xx   PPU shadow registers
// ---------------------------------------------------------------------------
wire ss_reg_sel    = (ca[23:16] == 8'hC0);

wire ss_data_sel   = ss_reg_sel & (ca[15:0] == 16'h6000);
wire ss_addr_sel   = ss_reg_sel & (ca[15:0] == 16'h6001);
wire ss_ext_sel    = ss_reg_sel & (ca[15:0] == 16'h6002);
wire ss_ramsize_sel= ss_reg_sel & (ca[15:0] == 16'h6003);
wire ss_romtype_sel= ss_reg_sel & (ca[15:0] == 16'h6004);
wire ss_end_sel    = ss_reg_sel & (ca[15:0] == 16'h600E);
wire ss_status_sel = ss_reg_sel & (ca[15:0] == 16'h600F);

assign dspn_regs_sel = ss_reg_sel & (ca[15:8] == 8'h61);
assign gsu_regs_sel  = ss_reg_sel & (ca[15:8] == 8'h62);

wire ppu_sel = (ca[23:16] == 8'hC1) & (ca[15:8] == 8'h21);

// RTI at $00:8008 — end-of-session marker
wire rti_sel = (ca[23:0] == 24'h00_8008);

// NMI vector $00:FFEA/B, IRQ vector $00:FFEE/F
wire nmi_vect   = ({ca[23:1], 1'b0} == 24'h00FFEA);
wire nmi_vect_l = nmi_vect & ~ca[0];
wire nmi_vect_h = nmi_vect &  ca[0];

wire irq_vect   = ({ca[23:1], 1'b0} == 24'h00FFEE);
wire irq_vect_l = irq_vect & ~ca[0];
wire irq_vect_h = irq_vect &  ca[0];

// ---------------------------------------------------------------------------
// NMI usage detector — same logic as MiSTer original.
// Keeps a shift-register of whether the NMI vector was read recently.
// Games that don't use NMI get intercepted via IRQ instead.
// ---------------------------------------------------------------------------
reg [15:0] nmi_cycle_cnt, nmi_read_sr;
wire ss_use_nmi = |nmi_read_sr;

always @(posedge clk) begin
    if (~reset_n) begin
        nmi_cycle_cnt <= 16'd0;
        nmi_read_sr   <= 16'd0;
    end else if (sysclkf_ce) begin
        nmi_cycle_cnt <= nmi_cycle_cnt + 1'b1;
        if (&nmi_cycle_cnt | (~cpurd_n & nmi_vect_l)) begin
            nmi_read_sr   <= {nmi_read_sr[14:0], nmi_vect_l};
            nmi_cycle_cnt <= 16'd0;
        end
    end
end

// ---------------------------------------------------------------------------
// BRAM — true dual-port, 20-bit address, 8-bit data
//   Port A : SNES CPU clock domain (read during load, write during save)
//   Port B : APF bridge clock domain (write during load staging, read back)
//
// We infer a simple dual-port RAM here.  Quartus will implement it in M10K.
// Size: 2^20 = 1 MB.  For Cyclone V this costs ~512 M10K blocks; if that is
// too large reduce to [17:0] (256 KB) — savestates never exceed ~200 KB.
// ---------------------------------------------------------------------------
localparam BRAM_AW = 18;  // 256 KB — savestates never exceed ~200 KB

// Port A — SNES side (clk domain)
reg  [BRAM_AW-1:0] bram_a_addr;
reg  [7:0]         bram_a_wdata;
reg                bram_a_we;
wire [7:0]         bram_a_rdata;   // ← changed from reg to wire

// Port B — APF/bridge side (clk_74a domain)
wire [7:0]         bram_b_rdata;   // ← changed from reg to wire

dpram_difclk #(
    .addr_width_a (BRAM_AW),
    .data_width_a (8),
    .addr_width_b (BRAM_AW),
    .data_width_b (8)
) ss_bram (
    .clock0    (clk),
    .clock1    (clk_74a),

    // Port A — SNES side
    .address_a (bram_a_addr),
    .data_a    (bram_a_wdata),
    .wren_a    (bram_a_we),
    .enable_a  (1'b1),
    .q_a       (bram_a_rdata),
    .cs_a      (1'b1),

    // Port B — APF bridge side
    .address_b (bram_wr ? bram_wr_addr[BRAM_AW-1:0] : bram_rd_addr[BRAM_AW-1:0]),
    .data_b    (bram_wr_data),
    .wren_b    (bram_wr),
    .enable_b  (1'b1),
    .q_b       (bram_b_rdata),
    .cs_b      (1'b1)
);

assign bram_rd_data = bram_b_rdata;

// ---------------------------------------------------------------------------
// SNES-side address counter
//   ss_data_addr — byte pointer into BRAM for save (write) / load (read)
//   Starts at 8 (bytes 0–7 are the 8-byte header written by savestates.asm)
// ---------------------------------------------------------------------------
reg [BRAM_AW-1:0] ss_data_addr;
reg               ss_data_addr_inc;

// ss_data_size captured at SS_END write — used to tell APF how many bytes
// to pull from BRAM.
reg [BRAM_AW-1:0] ss_data_size;

// ---------------------------------------------------------------------------
// External address counter (SPC/BSRAM/DSPn streaming)
// ---------------------------------------------------------------------------
reg [19:0] ss_ext_addr;
reg        ss_ext_addr_inc;

assign ext_addr = ss_ext_addr;

// ---------------------------------------------------------------------------
// Load validation — check that BRAM[8..11] == "SNES" before accepting load.
// We do this synchronously on the clk domain after load is pulsed, by
// reading four bytes from BRAM port A.
//
// State machine: VALIDATE_0..3 → either set load_ready or abort.
// ---------------------------------------------------------------------------
localparam [2:0]
    VAL_IDLE  = 3'd0,
    VAL_B0    = 3'd1,
    VAL_B1    = 3'd2,
    VAL_B2    = 3'd3,
    VAL_B3    = 3'd4,
    VAL_DONE  = 3'd5;

reg [2:0] val_state;
reg [31:0] val_buf;

// ---------------------------------------------------------------------------
// APF data-ready signal (exported so main.v / save_state_controller knows
// the save is complete and BRAM is ready to DMA)
// ---------------------------------------------------------------------------
// ss_busy goes low at end of session — that is the "done" signal.
// save_data_size is valid while ~ss_busy & ~save_en.

// We also export the final byte count so the APF layer knows how large the
// file is.  Wire it out via a dedicated port if needed; for now it is
// readable as the first 4 bytes of BRAM written by savestates.asm itself
// (the "SNES" magic + reserved bytes occupy bytes 0–7; data follows at 8).

// ---------------------------------------------------------------------------
// Main control FSM
// ---------------------------------------------------------------------------
always @(posedge clk or negedge reset_n) begin
    if (~reset_n) begin
        ss_busy          <= 1'b0;
        save_en          <= 1'b0;
        load_en          <= 1'b0;
        rd_rti           <= 1'b0;
        save_end         <= 1'b0;
        load_ready       <= 1'b0;
        ss_data_addr     <= {BRAM_AW{1'b0}};
        ss_data_addr_inc <= 1'b0;
        ss_data_size     <= {BRAM_AW{1'b0}};
        ss_ext_addr      <= 20'd0;
        ss_ext_addr_inc  <= 1'b0;
        bram_a_we        <= 1'b0;
        val_state        <= VAL_IDLE;
        val_buf          <= 32'd0;
    end else begin

        bram_a_we <= 1'b0;  // default: no write

        // ------------------------------------------------------------------
        // Accept new save/load request (only when idle)
        // ------------------------------------------------------------------
        if (~(save_en | load_en)) begin
            if (~ss_save_old & ss_save) begin
                save_en <= 1'b1;
            end else if (~ss_load_old & ss_load) begin
                load_en    <= 1'b1;
                load_ready <= 1'b0;
                val_state  <= VAL_B0;   // start header validation
            end
        end

        // ------------------------------------------------------------------
        // Load header validation (reads BRAM port A sequentially)
        // We stall here until we know the data is valid before letting
        // the CPU intercept happen.
        // ------------------------------------------------------------------
        case (val_state)
            VAL_B0: begin
                // Set address, wait one cycle for BRAM read
                bram_a_addr <= {{(BRAM_AW-4){1'b0}}, 4'd8};
                val_state   <= VAL_B1;
            end
            VAL_B1: begin
                val_buf[7:0] <= bram_a_rdata;
                bram_a_addr  <= {{(BRAM_AW-4){1'b0}}, 4'd9};
                val_state    <= VAL_B2;
            end
            VAL_B2: begin
                val_buf[15:8] <= bram_a_rdata;
                bram_a_addr   <= {{(BRAM_AW-4){1'b0}}, 4'd10};
                val_state     <= VAL_B3;
            end
            VAL_B3: begin
                val_buf[23:16] <= bram_a_rdata;
                bram_a_addr    <= {{(BRAM_AW-4){1'b0}}, 4'd11};
                val_state      <= VAL_DONE;
            end
            VAL_DONE: begin
                val_buf[31:24] <= bram_a_rdata;
                val_state      <= VAL_IDLE;
                // "SNES" in little-endian: 'S'=53, 'N'=4E, 'E'=45, 'S'=53
                if ({bram_a_rdata, val_buf[23:0]} == 32'h53_45_4E_53) begin
                    load_ready <= 1'b1;
                end else begin
                    load_en <= 1'b0;   // no valid state — abort
                end
            end
            default: ; // VAL_IDLE — nothing
        endcase

        // ------------------------------------------------------------------
        // CPU vector intercept: hook NMI (or IRQ if game doesn't use NMI)
        // ------------------------------------------------------------------
        if (cpurd_ce) begin
            if (nmi_vect_l | (~ss_use_nmi & irq_vect_l)) begin
                if (~ss_busy & (save_en | (load_en & load_ready))) begin
                    ss_busy <= 1'b1;
                    // Reset data address to 8 (header already at 0–7 in ASM)
                    ss_data_addr <= {{(BRAM_AW-4){1'b0}}, 4'd8};
                    ss_ext_addr  <= 20'd0;
                end
            end

            // Detect RTI fetch at $00:8008 — end of session
            if (ss_busy & rti_sel) begin
                rd_rti <= 1'b1;
            end
        end

        // RTI address de-asserted → session over
        if (cpurd_ce_n & rd_rti) begin
            ss_busy  <= 1'b0;
            rd_rti   <= 1'b0;
            load_en  <= 1'b0;
            save_en  <= 1'b0;
            save_end <= 1'b0;
        end

        // ------------------------------------------------------------------
        // SNES CPU writes to $C0:6001 (SSADDR) — reset stream address
        // SNES CPU writes to $C0:6002 (SS_EXT_ADDR) — reset ext counter
        // SNES CPU writes to $C0:600E (SS_END) — save done
        // ------------------------------------------------------------------
        if (cpuwr_ce & ss_busy) begin
            if (ss_addr_sel) begin
                ss_data_addr <= {{(BRAM_AW-4){1'b0}}, 4'd8};
            end

            if (ss_ext_sel) begin
                ss_ext_addr <= 20'd0;
            end

            if (ss_end_sel) begin
                save_end     <= 1'b1;
                ss_data_size <= ss_data_addr;
                // No DDR flush needed — BRAM is already written byte-by-byte
            end
        end

        // ------------------------------------------------------------------
        // Address increment — fires on rising edge of cpuwr_n or cpurd_n
        // after a $C0:6000 access
        // ------------------------------------------------------------------
        if (cpuwr_ce | cpurd_ce) begin
            if (ss_data_sel & ss_busy)
                ss_data_addr_inc <= 1'b1;
        end

        if (cpuwr_ce_n | cpurd_ce_n) begin
            if (ss_data_addr_inc) begin
                ss_data_addr     <= ss_data_addr + 1'b1;
                ss_data_addr_inc <= 1'b0;
            end
        end

        // ------------------------------------------------------------------
        // External (SPC / BSRAM / DSPn) address increment
        // ------------------------------------------------------------------
        if (pawr_ce | pard_ce) begin
            if (spc_sel | bsram_sel | dspn_ram_sel)
                ss_ext_addr_inc <= 1'b1;
        end

        if (pawr_ce_n | pard_ce_n) begin
            if (ss_ext_addr_inc) begin
                ss_ext_addr     <= ss_ext_addr + 1'b1;
                ss_ext_addr_inc <= 1'b0;
            end
        end

        // ------------------------------------------------------------------
        // SAVE — CPU writes byte to $C0:6000 → store in BRAM
        // ------------------------------------------------------------------
        if (~cpuwr_n & sysclkf_ce & ss_busy & ss_data_sel & save_en) begin
            bram_a_addr  <= ss_data_addr[BRAM_AW-1:0];
            bram_a_wdata <= bram_data_mux;
            bram_a_we    <= 1'b1;
        end

        // ------------------------------------------------------------------
        // LOAD — CPU reads $C0:6000 → set BRAM read address so rdata is
        // ready next cycle (BRAM is registered-output).
        // We pre-fetch by setting address on cpurd_ce so data is valid when
        // the CPU captures it.
        // ------------------------------------------------------------------
        if (cpurd_ce & ss_busy & ss_data_sel & load_en) begin
            bram_a_addr <= ss_data_addr[BRAM_AW-1:0];
        end

    end // ~reset
end

// ---------------------------------------------------------------------------
// Data mux: what actually gets written to BRAM during save
//   Normal CPU writes → di
//   SPC DMA reads (pa == $84/$85/$86) → spc_di
//   BSRAM reads (pa == $87) → bsram_di
//   DSPn RAM reads (pa == $88) → dspn_di
// ---------------------------------------------------------------------------
wire aram_sel;
wire dsp_regs_sel;
wire smp_regs_sel;
wire spc_sel     = aram_sel | dsp_regs_sel | smp_regs_sel;
wire spc_read    = spc_sel  & ~pard_n;
wire bsram_read  = bsram_sel & ~pard_n;
wire dspn_ram_read = dspn_ram_sel & ~pard_n;

reg [7:0] bram_data_mux;
always @(*) begin
    bram_data_mux = di;
    if (spc_read)      bram_data_mux = spc_di;
    if (bsram_read)    bram_data_mux = bsram_di;
    if (dspn_ram_read) bram_data_mux = dspn_di;
end

// ---------------------------------------------------------------------------
// savestates_regs sub-module (unchanged from MiSTer)
// Handles $C0:10xx – $C0:3xxx shadow register readback
// ---------------------------------------------------------------------------
wire [7:0] ssr_do;
wire       ssr_oe;

savestates_regs ss_regs (
    .reset_n     (reset_n),
    .clk         (clk),
    .ss_busy     (ss_busy),
    .save_en     (save_en),
    .ss_reg_sel  (ss_reg_sel),
    .sysclkf_ce  (sysclkf_ce),
    .sysclkr_ce  (sysclkr_ce),
    .romsel_n    (romsel_n),
    .ca          (ca),
    .cpurd_ce    (cpurd_ce),
    .cpurd_ce_n  (cpurd_ce_n),
    .cpuwr_ce    (cpuwr_ce),
    .pa          (pa),
    .pard_ce     (pard_ce),
    .pawr_ce     (pawr_ce),
    .di          (di),
    .ssr_do      (ssr_do),
    .ssr_oe      (ssr_oe)
);

// ---------------------------------------------------------------------------
// savestates_map sub-module (unchanged from MiSTer)
// Handles mapper-specific ROM overlay address for SA-1 / GSU etc.
// ---------------------------------------------------------------------------
wire [7:0]  map_ss_do;
wire        map_ss_oe;
wire [15:0] map_rom_addr;
wire        map_rom_ovr;
wire        map_active;

savestates_map ss_map (
    .reset_n        (reset_n),
    .clk            (clk),
    .ss_busy        (ss_busy),
    .save_en        (save_en),
    .ss_reg_sel     (ss_reg_sel),
    .sysclkf_ce     (sysclkf_ce),
    .sysclkr_ce     (sysclkr_ce),
    .ca             (ca),
    .cpurd_n        (cpurd_n),
    .cpuwr_n        (cpuwr_n),
    .cpuwr_ce       (cpuwr_ce),
    .pa             (pa),
    .pard_n         (pard_n),
    .pawr_n         (pawr_n),
    .di             (di),
    .sa1_active     (sa1_active),
    .sa1_a          (sa1_a),
    .sa1_rd_n       (sa1_rd_n),
    .sa1_wr_n       (sa1_wr_n),
    .sa1_di         (sa1_di),
    .sa1_sa1_romsel (sa1_sa1_romsel),
    .sa1_sns_romsel (sa1_sns_romsel),
    .map_active     (map_active),
    .rom_addr       (map_rom_addr),
    .rom_ovr        (map_rom_ovr),
    .ss_do          (map_ss_do),
    .ss_oe          (map_ss_oe)
);

// ---------------------------------------------------------------------------
// ss_do output mux — what the CPU reads from the save-state address space
// ---------------------------------------------------------------------------
wire [15:0] nmi_vect_addr = save_en ? 16'h8000 : 16'h8004;

wire ss_oe = ss_data_sel | ss_status_sel | nmi_vect | irq_vect |
             ss_ramsize_sel | ss_romtype_sel | ssr_oe | map_ss_oe |
             ppu_sel | dspn_regs_sel | gsu_regs_sel;

always @(posedge clk) begin
    ss_do <= 8'h00;
    // LOAD: byte from BRAM (registered read, address set by cpurd_ce logic)
    if (ss_data_sel & load_en)  ss_do <= bram_a_rdata;
    // STATUS: bit1=not_ready(during validation), bit0=save_en
    if (ss_status_sel)          ss_do <= {6'd0, (load_en & ~load_ready), save_en};
    if (nmi_vect_l | irq_vect_l) ss_do <= nmi_vect_addr[7:0];
    if (nmi_vect_h | irq_vect_h) ss_do <= nmi_vect_addr[15:8];
    if (ss_ramsize_sel)          ss_do <= {4'd0, ram_size};
    if (ss_romtype_sel)          ss_do <= rom_type;
    if (ssr_oe)                  ss_do <= ssr_do;
    if (map_ss_oe)               ss_do <= map_ss_do;
    if (ppu_sel)                 ss_do <= ppu_di;
    if (dspn_regs_sel)           ss_do <= dspn_di;
    if (gsu_regs_sel)            ss_do <= gsu_di;
end

// ---------------------------------------------------------------------------
// ROM address override — points into savestates.bin embedded in ROM space.
// savestates.bin is mapped at $FF:8000 in the SNES ROM image.
// ca[16] selects between save ($8000) and load ($8004) entry points.
// ---------------------------------------------------------------------------
always @(*) begin
    rom_addr[23:16] = {2'b11, 6'b11_1111};   // bank $FF
    rom_addr[15:0]  = {ca[16], ca[14:0]};
    if (map_rom_ovr)
        rom_addr[15:0] = map_rom_addr;
end

// ---------------------------------------------------------------------------
// Output qualifiers
// ---------------------------------------------------------------------------
assign ss_do_ovr  = ss_busy & ss_oe;
assign ss_rom_ovr = map_active ? map_rom_ovr : ss_busy;

assign aram_sel      = ss_busy & (pa == 8'h84);
assign dsp_regs_sel  = ss_busy & (pa == 8'h85);
assign smp_regs_sel  = ss_busy & (pa == 8'h86);
assign bsram_sel     = ss_busy & (pa == 8'h87);
assign dspn_ram_sel  = ss_busy & (pa == 8'h88);


endmodule
