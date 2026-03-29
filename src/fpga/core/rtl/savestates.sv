// savestates.sv — Analogue Pocket adaptation of MiSTer SNES_MiSTer savestates.sv
//
// Uses the on-board 128 KB async SRAM chip instead of on-chip M10K BRAM to
// avoid FPGA fitment errors on the Cyclone V 5CEBA4.
//
// Interface contract:
//
//   SAVE path:  APF triggers ss_save pulse → savestates.sv waits for the
//               next NMI/IRQ vector read → hijacks it to $8000 (Save_start)
//               → SNES CPU runs savestates.asm → each byte written to
//               $C06000 lands in the external SRAM → when $C0600E
//               (SS_END) is written the module de-asserts ss_busy.
//               APF can then DMA the SRAM out to SD via the bridge SRAM
//               access ports (sram_* pins driven by core_top when ~ss_busy).
//
//   LOAD path:  APF fills SRAM via bridge SRAM access ports (core_top drives
//               sram_* when ~ss_busy), then pulses ss_load → savestates.sv
//               waits for next NMI/IRQ vector read → hijacks it to $8004
//               (Load_start) → SNES CPU runs load path → each $C06000 read
//               returns SRAM[addr] → RTI at $8008 ends the session.
//
// SRAM size:  128 KB (17-bit byte address space).  Savestates for most
//             mappers fit within this.  Saves that would exceed 128 KB are
//             silently truncated — preferable to the design not fitting at all.
//
// Clock domain:
//   clk        — master SNES clock (21.477 MHz), all SNES bus logic and
//                SRAM access during ss_busy.
//
// SRAM access notes:
//   The on-board SRAM is 16-bit wide.  Byte access uses sram_a[16:1] as the
//   word address and sram_ub_n/sram_lb_n to select the byte lane.
//   Byte address N → word addr = N[16:1], upper byte when N[0]=1, lower when
//   N[0]=0.  Write data is replicated on both sram_dq halves.
//   Read data is sampled (registered) one clk cycle after address/oe_n change,
//   matching the former one-cycle BRAM read latency.
//
// -----------------------------------------------------------------------------

module savestates
(
    input  wire        reset_n,
    input  wire        clk,           // SNES master clock

    // APF-side triggers (already synchronised to clk by caller)
    input  wire        ss_save,       // pulse: begin save
    input  wire        ss_load,       // pulse: begin load

    // External async SRAM interface (active during ss_busy and header validation).
    // core_top muxes these with its bridge-side SRAM drivers based on ss_busy.
    output wire [16:0] sram_a,        // word address; bit[0] always 0
    output wire [15:0] sram_dq_o,     // write data (byte replicated on both halves)
    input  wire [15:0] sram_dq_i,     // SRAM data output (read path)
    output wire        sram_oe_n,     // output enable (0 = read)
    output wire        sram_we_n,     // write enable (0 = write)
    output wire        sram_ub_n,     // upper byte select (DQ[15:8]) — odd addr
    output wire        sram_lb_n,     // lower byte select (DQ[7:0])  — even addr

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
// External SRAM — 128 KB byte-addressable via 16-bit wide async SRAM chip.
//   BRAM_AW = 17 → 2^17 = 128 KB address space.
//   Word address = byte_addr[16:1]; byte lane selected by ub_n/lb_n.
//
// Port A — SNES side (clk domain): reads during load, writes during save.
// Bridge access is handled externally by core_top when ~ss_busy.
// ---------------------------------------------------------------------------
localparam BRAM_AW = 17;  // 128 KB — on-board SRAM size

// Port A — SNES side (clk domain)
reg  [BRAM_AW-1:0] bram_a_addr;
reg  [7:0]         bram_a_wdata;
reg                bram_a_we;

// Registered SRAM read-data (1-cycle latency, matches former BRAM behaviour)
reg  [7:0]         sram_rd_reg;
always @(posedge clk)
    sram_rd_reg <= bram_a_addr[0] ? sram_dq_i[15:8] : sram_dq_i[7:0];

// SRAM combinatorial control driven by the SNES-side registers.
// core_top overrides these with its bridge-side drivers when ~ss_busy.
//   Word address: byte_addr >> 1 (SRAM has 16-bit words)
//   Upper byte (DQ[15:8]) selected when byte address is odd (bit[0]=1)
//   Lower byte (DQ[7:0])  selected when byte address is even (bit[0]=0)
assign sram_a    = {bram_a_addr[BRAM_AW-1:1], 1'b0};
assign sram_dq_o = {bram_a_wdata, bram_a_wdata};   // replicate byte on both lanes
// Byte lane select (active-low enables):
//   even byte address (bit[0]=0) → lower byte DQ[7:0]  → lb_n=0, ub_n=1
//   odd  byte address (bit[0]=1) → upper byte DQ[15:8] → ub_n=0, lb_n=1
assign sram_ub_n = ~bram_a_addr[0];  // 0 when odd addr  → enables upper byte
assign sram_lb_n =  bram_a_addr[0];  // 0 when even addr → enables lower byte
assign sram_oe_n = ~load_en;         // assert during header validation + load
assign sram_we_n = ~bram_a_we;       // assert for one clk cycle per save byte

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
// Main control FSM
// ---------------------------------------------------------------------------
always @(posedge clk or negedge reset_n) begin
    if (~reset_n) begin
        ss_busy          <= 1'b0;
        save_en          <= 1'b0;
        load_en          <= 1'b0;
        rd_rti           <= 1'b0;
        save_end         <= 1'b0;
        ss_data_addr     <= {BRAM_AW{1'b0}};
        ss_data_addr_inc <= 1'b0;
        ss_data_size     <= {BRAM_AW{1'b0}};
        ss_ext_addr      <= 20'd0;
        ss_ext_addr_inc  <= 1'b0;
        bram_a_we        <= 1'b0;
    end else begin

        bram_a_we <= 1'b0;  // default: no write

        // ------------------------------------------------------------------
        // Accept new save/load request (only when idle)
        // ------------------------------------------------------------------
        if (~(save_en | load_en)) begin
            if (~ss_save_old & ss_save) begin
                save_en <= 1'b1;
            end else if (~ss_load_old & ss_load) begin
                load_en <= 1'b1;
            end
        end

        // ------------------------------------------------------------------
        // CPU vector intercept: hook NMI (or IRQ if game doesn't use NMI)
        // ------------------------------------------------------------------
        if (cpurd_ce) begin
            if (nmi_vect_l | (~ss_use_nmi & irq_vect_l)) begin
                if (~ss_busy & (save_en | load_en)) begin
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
    if (ss_data_sel & load_en)  ss_do <= sram_rd_reg;
    // STATUS: bit1=busy (high while save/load is in progress), bit0=save_en
    if (ss_status_sel)          ss_do <= {6'd0, ss_busy, save_en};
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
