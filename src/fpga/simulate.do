# simulate.do — ModelSim/QuestaSim script for snes_savestate_tb
#
# Run from the src/fpga/ directory:
#   vsim -do simulate.do
# or from the ModelSim GUI:
#   File > Change Directory → src/fpga/
#   File > Do...            → simulate.do
#
# Requirements:
#   ModelSim-Intel FPGA Edition (ships with Quartus Prime Lite).
#   The Quartus altera_mf simulation library must be pre-compiled.
#   Quartus Prime Lite includes a script to do this:
#     Tools > Launch Simulation Library Compiler
#   Then set ALTERA_MF_LIB below to the output directory.
#
# ---------------------------------------------------------------------------

# ============================================================================
# 0. Paths — adjust MODELSIM_ASE_ROOT to match your Quartus installation
# ============================================================================
set MODELSIM_ASE_ROOT "C:/intelFPGA/20.1/modelsim_ase"
set ALTERA_MF_LIB     "$MODELSIM_ASE_ROOT/altera/verilog/altera_mf"
set ASE_INI           "$MODELSIM_ASE_ROOT/modelsim.ini"

# Sanity check
if { ![file isdirectory $MODELSIM_ASE_ROOT] } {
    puts ""
    puts "ERROR: ModelSim ASE root not found: $MODELSIM_ASE_ROOT"
    puts "  Update MODELSIM_ASE_ROOT in simulate.do to match your install."
    puts ""
    quit -code 1
}
if { ![file isdirectory $ALTERA_MF_LIB] } {
    puts ""
    puts "WARNING: altera_mf library not found at: $ALTERA_MF_LIB"
    puts "  mf_datatable (dual-port BRAM) will not simulate correctly."
    puts "  Compile simulation libraries via:"
    puts "    Quartus > Tools > Launch Simulation Library Compiler"
    puts ""
}

# ============================================================================
# 1. Working library
#
# Copy the ASE's own modelsim.ini into the working directory so that ieee,
# std, altera_mf, etc. are already mapped.  Without this, vcom cannot find
# the IEEE libraries and fails immediately on any VHDL file.
# ============================================================================
if { [file exists work] } {
    vdel -lib work -all
}

# Inherit all library mappings from the ASE global ini
if { [file exists $ASE_INI] } {
    file copy -force $ASE_INI modelsim.ini
} else {
    puts "WARNING: $ASE_INI not found — IEEE libraries may not resolve"
}

vlib work
vmap work work
vmap altera_mf $ALTERA_MF_LIB

# ============================================================================
# 2. Compile helpers
#    Uses eval + list to safely handle flags and filenames in all Tcl versions.
#    {*} expansion is Tcl 8.5+ only; ModelSim ASE ships with Tcl 8.4.
# ============================================================================
proc vcom_file { flags file } {
    set cmd [concat vcom $flags [list $file]]
    if { [catch { eval $cmd } msg] } {
        puts "ERROR compiling $file:\n$msg"
        quit -code 1
    }
}
proc vlog_file { flags file } {
    set cmd [concat vlog $flags [list $file]]
    if { [catch { eval $cmd } msg] } {
        puts "ERROR compiling $file:\n$msg"
        quit -code 1
    }
}

# ============================================================================
# 3. Common compile flags (plain lists, no string-with-spaces tricks)
# ============================================================================
set VCOM  [list -93 -work work]
set VLOG  [list -sv -work work \
    +incdir+core/rtl \
    +incdir+core/rtl/mister_top \
    +incdir+apf]
set VLOGP [list -work work]   ;# plain Verilog, no SV or incdir needed

# ============================================================================
# 4. APF support
#    common.v       → synch_3, used by save_state_controller
#    mf_datatable.v → dual-port BRAM wrapper (needs altera_mf)
# ============================================================================
vlog_file $VLOGP apf/common.v
vlog_file $VLOGP apf/mf_datatable.v

# ============================================================================
# 5. VHDL packages (must precede any VHDL that uses them)
# ============================================================================
vcom_file $VCOM core/rtl/PPU_PKG.vhd
vcom_file $VCOM core/rtl/DSP_PKG.vhd
vcom_file $VCOM core/rtl/chip/GSU/GSU_PKG.vhd
vcom_file $VCOM core/rtl/SPC700/SPC700_pkg.vhd
vcom_file $VCOM core/rtl/65C816/P65816_pkg.vhd
vcom_file $VCOM core/rtl/chip/SPC7110/SPC7110_DEC_PKG.vhd

# ============================================================================
# 6. VHDL RTL (dependency order within each subsystem)
# ============================================================================

# 65C816 CPU
vcom_file $VCOM core/rtl/65C816/BCDAdder.vhd
vcom_file $VCOM core/rtl/65C816/AddSubBCD.vhd
vcom_file $VCOM core/rtl/65C816/AddrGen.vhd
vcom_file $VCOM core/rtl/65C816/ALU.vhd
vcom_file $VCOM core/rtl/65C816/MCode.vhd
vcom_file $VCOM core/rtl/65C816/P65C816.vhd

# SPC700 APU
vcom_file $VCOM core/rtl/SPC700/AddrGen.vhd
vcom_file $VCOM core/rtl/SPC700/AddSub.vhd
vcom_file $VCOM core/rtl/SPC700/BCDAdj.vhd
vcom_file $VCOM core/rtl/SPC700/ALU.vhd
vcom_file $VCOM core/rtl/SPC700/MulDiv.vhd
vcom_file $VCOM core/rtl/SPC700/MCode.vhd
vcom_file $VCOM core/rtl/SPC700/SPC700.vhd

# PPU / BRAM / CEGen
vcom_file $VCOM core/rtl/bram.vhd
vcom_file $VCOM core/rtl/CEGen.vhd
vcom_file $VCOM core/rtl/PPU.vhd

# DSP
vcom_file $VCOM core/rtl/DSP.vhd
vcom_file $VCOM core/rtl/chip/DSP/DSPn.vhd
vcom_file $VCOM core/rtl/chip/DSP/DSP_LHRomMap.vhd
vcom_file $VCOM core/rtl/chip/DSP/OBC1.vhd

# SWRAM / SMP / CPU / top VHDL wrappers
vcom_file $VCOM core/rtl/SWRAM.vhd
vcom_file $VCOM core/rtl/SMP.vhd
vcom_file $VCOM core/rtl/CPU.vhd
vcom_file $VCOM core/rtl/SNES.vhd

# Coprocessor chips
vcom_file $VCOM core/rtl/chip/CX4/cx4cache.vhd
vcom_file $VCOM core/rtl/chip/CX4/CX4.vhd
vcom_file $VCOM core/rtl/chip/CX4/CX4Map.vhd

vcom_file $VCOM core/rtl/chip/SDD1/InputMgr.vhd
vcom_file $VCOM core/rtl/chip/SDD1/Decoder.vhd
vcom_file $VCOM core/rtl/chip/SDD1/SDD1.vhd
vcom_file $VCOM core/rtl/chip/SDD1/SDD1Map.vhd

vcom_file $VCOM core/rtl/chip/GSU/GSU.vhd
vcom_file $VCOM core/rtl/chip/GSU/GSUMap.vhd

vcom_file $VCOM core/rtl/chip/SA1/SA1DIV.vhd
vcom_file $VCOM core/rtl/chip/SA1/SA1MULT.vhd
vcom_file $VCOM core/rtl/chip/SA1/SA1.vhd
vcom_file $VCOM core/rtl/chip/SA1/SA1Map.vhd

vcom_file $VCOM core/rtl/chip/SPC7110/SPC7110_DEC.vhd
vcom_file $VCOM core/rtl/chip/SPC7110/SPC7110_FIFO.vhd
vcom_file $VCOM core/rtl/chip/SPC7110/SPC7110_MULDIV.vhd
vcom_file $VCOM core/rtl/chip/SPC7110/SPC7110.vhd
vcom_file $VCOM core/rtl/chip/SPC7110/SPC7110Map.vhd

vcom_file $VCOM core/rtl/chip/BSX/BSX_MCC.vhd
vcom_file $VCOM core/rtl/chip/BSX/BSX_DP.vhd
vcom_file $VCOM core/rtl/chip/BSX/BSX_BS.vhd
vcom_file $VCOM core/rtl/chip/BSX/BSXMap.vhd

vcom_file $VCOM core/rtl/chip/RTC4513.vhd
vcom_file $VCOM core/rtl/chip/SRTC.vhd

# ============================================================================
# 7. SystemVerilog / Verilog RTL
# ============================================================================

# MSU-1 audio expansion
vlog_file $VLOGP core/rtl/chip/MSU1/msu_fifo.v
vlog_file $VLOGP core/rtl/chip/MSU1/msu_audio.v
vlog_file $VLOG  core/rtl/chip/MSU1/msu_data_store.sv
vlog_file $VLOG  core/rtl/chip/MSU1/MSU.sv

# Core RTL
vlog_file $VLOGP core/rtl/hps_ext.v
vlog_file $VLOG  core/rtl/ioport.sv
vlog_file $VLOG  core/rtl/lightgun.sv
vlog_file $VLOG  core/rtl/cheatcodes.sv
vlog_file $VLOG  core/rtl/miracle.sv
vlog_file $VLOG  core/rtl/sdram.sv
vlog_file $VLOG  core/rtl/savestates_regs.sv
vlog_file $VLOG  core/rtl/savestates_map.sv
vlog_file $VLOG  core/rtl/savestates_sa1.sv
vlog_file $VLOG  core/rtl/savestates.sv
vlog_file $VLOGP core/rtl/main.v

# MiSTer-top wrappers
vlog_file $VLOG core/rtl/mister_top/sync_fifo.sv
vlog_file $VLOG core/rtl/mister_top/scanline_filler.sv
vlog_file $VLOG core/rtl/mister_top/sound_i2s.sv
vlog_file $VLOG core/rtl/mister_top/rom_parser.sv
vlog_file $VLOG core/rtl/mister_top/data_loader.sv
vlog_file $VLOG core/rtl/mister_top/data_unloader.sv
vlog_file $VLOG core/rtl/mister_top/psram.sv
vlog_file $VLOG core/rtl/mister_top/ss_sdram_arbiter.sv
vlog_file $VLOG core/rtl/mister_top/SNES.sv

# APF controller + save state
vlog_file $VLOGP core/core_bridge_cmd.v
vlog_file $VLOG  core/save_state_controller.sv

# ============================================================================
# 8. Testbench (contains behavioral SDRAM/SRAM/PSRAM models + stimulus)
# ============================================================================
vlog_file $VLOG core/rtl/mister_top/snes_savestate_tb.sv

# ============================================================================
# 9. Elaborate
#    -novopt  : keep all signals visible (no optimisation pruning)
#    -t 1ps   : simulation time resolution
# ============================================================================
vsim -t 1ps \
     -novopt \
     -lib work \
     work.snes_savestate_tb

# ============================================================================
# 10. Waveform setup
# ============================================================================
add wave -divider "--- Clocks & Reset ---"
add wave -label clk_74a        sim:/snes_savestate_tb/clk_74a
add wave -label clk_sys        sim:/snes_savestate_tb/clk_sys
add wave -label reset_n        sim:/snes_savestate_tb/reset_n

add wave -divider "--- APF Bridge ---"
add wave -label bridge_wr           sim:/snes_savestate_tb/bridge_wr
add wave -label bridge_rd           sim:/snes_savestate_tb/bridge_rd
add wave -hex -label bridge_addr    sim:/snes_savestate_tb/bridge_addr
add wave -hex -label bridge_wr_data sim:/snes_savestate_tb/bridge_wr_data
add wave -hex -label save_state_bridge_read_data \
    sim:/snes_savestate_tb/save_state_bridge_read_data

add wave -divider "--- APF Save/Load Handshake (clk_74a domain) ---"
add wave -label savestate_start      sim:/snes_savestate_tb/savestate_start
add wave -label savestate_start_ack  sim:/snes_savestate_tb/savestate_start_ack
add wave -label savestate_start_busy sim:/snes_savestate_tb/savestate_start_busy
add wave -label savestate_start_ok   sim:/snes_savestate_tb/savestate_start_ok
add wave -label savestate_start_err  sim:/snes_savestate_tb/savestate_start_err
add wave -label savestate_load       sim:/snes_savestate_tb/savestate_load
add wave -label savestate_load_ack   sim:/snes_savestate_tb/savestate_load_ack
add wave -label savestate_load_busy  sim:/snes_savestate_tb/savestate_load_busy
add wave -label savestate_load_ok    sim:/snes_savestate_tb/savestate_load_ok
add wave -label savestate_load_err   sim:/snes_savestate_tb/savestate_load_err

add wave -divider "--- Core-Side CDC Toggle Interface (clk_sys domain) ---"
add wave -label ss_save          sim:/snes_savestate_tb/ss_save
add wave -label ss_load          sim:/snes_savestate_tb/ss_load
add wave -label ss_req           sim:/snes_savestate_tb/ss_req
add wave -label ss_ack           sim:/snes_savestate_tb/ss_ack
add wave -label ss_we            sim:/snes_savestate_tb/ss_we
add wave -hex -label ss_ddr_addr sim:/snes_savestate_tb/ss_ddr_addr
add wave -hex -label ss_dout     sim:/snes_savestate_tb/ss_dout
add wave -hex -label ss_din      sim:/snes_savestate_tb/ss_din
add wave -label ss_busy          sim:/snes_savestate_tb/ss_busy

add wave -divider "--- save_state_controller FSM ---"
add wave -label ssc_sys_state  sim:/snes_savestate_tb/ssc/sys_state
add wave -label ssc_sram_state sim:/snes_savestate_tb/ssc/sram_state
add wave -label ss_busy_seen   sim:/snes_savestate_tb/ssc/ss_busy_seen

add wave -divider "--- SRAM Bus ---"
add wave -hex -label sram_a sim:/snes_savestate_tb/sram_a
add wave -hex -label sram_dq sim:/snes_savestate_tb/sram_dq
add wave -label sram_oe_n    sim:/snes_savestate_tb/sram_oe_n
add wave -label sram_we_n    sim:/snes_savestate_tb/sram_we_n

add wave -divider "--- ROM Load ---"
add wave -label ioctl_download  sim:/snes_savestate_tb/ioctl_download
add wave -label ioctl_wr        sim:/snes_savestate_tb/ioctl_wr
add wave -hex -label ioctl_addr sim:/snes_savestate_tb/ioctl_addr
add wave -hex -label ioctl_dout sim:/snes_savestate_tb/ioctl_dout

add wave -divider "--- Video (CPU liveness) ---"
add wave -label vblank sim:/snes_savestate_tb/vblank
add wave -label vsync  sim:/snes_savestate_tb/vsync

# ============================================================================
# 11. Run
# ============================================================================
run -all
