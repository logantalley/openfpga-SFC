// Embedded ROM for the savestates.bin firmware.
//
// The savestates module redirects CPU fetches in bank $FF to a firmware ROM
// that drives the save/load state machine.  Historically this was supposed
// to be loaded into SDRAM at offset $FF0000 via an APF data slot, but that
// path was unreliable on this build (the CPU was reading $AA instead of
// the firmware bytes — confirmed via on-screen debug overlay).
//
// To remove the dependency on the data-slot load path entirely, the
// firmware is embedded directly in FPGA block RAM here.  Quartus infers
// this as an M10K-block ROM.
//
// The .hex file is generated from dist/Assets/snes/common/savestates.bin
// by tools/build_savestate_rom_hex.py — re-run that script if the firmware
// is rebuilt.

module savestate_rom (
    input  wire        clk,
    input  wire [10:0] addr,    // word address (0..2047), 16 bits per word = 4 KB
    output reg  [15:0] q
);

    (* ramstyle = "M10K" *) reg [15:0] mem [0:2047];

    // Path is relative to the Quartus project root (src/fpga/).
    initial $readmemh("core/rtl/savestate_rom.hex", mem);

    always @(posedge clk) begin
        q <= mem[addr];
    end

endmodule
