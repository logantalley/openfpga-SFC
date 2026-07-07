# simulate_serve.do — full-stack ModelSim run for the savestate LOAD SERVE
# path including the REAL savestates.sv engine on the gated MCLK, consumed
# at firmware DMA pace (tb_ss_serve.sv).  Self-checking; PASS/FAIL banner.
#
# Headless:   vsim -c -do simulate_serve.do     (from src/fpga/)
# GUI:        vsim -do simulate_serve.do
#
# Sweep example (more chunks): add -gSTA_CHUNKS=2048 to the vsim line.

set MODELSIM_ASE_ROOT "C:/intelFPGA/20.1/modelsim_ase"
set ALTERA_MF_LIB     "$MODELSIM_ASE_ROOT/altera/verilog/altera_mf"
set ASE_INI           "$MODELSIM_ASE_ROOT/modelsim.ini"

if { ![file isdirectory $MODELSIM_ASE_ROOT] } {
    puts "ERROR: ModelSim ASE root not found: $MODELSIM_ASE_ROOT"
    quit -code 1
}
if { ![file isdirectory $ALTERA_MF_LIB] } {
    puts "ERROR: altera_mf library not found at: $ALTERA_MF_LIB"
    quit -code 1
}

if { [file exists work] } { vdel -lib work -all }
if { [file exists $ASE_INI] } {
    file copy -force $ASE_INI modelsim.ini
    file attributes modelsim.ini -readonly 0
}

vlib work
vmap work work
vmap altera_mf $ALTERA_MF_LIB

proc vlog_file { flags file } {
    set cmd [concat vlog $flags [list $file]]
    if { [catch { eval $cmd } msg] } {
        puts "ERROR compiling $file:\n$msg"
        quit -code 1
    }
}

set VLOGP [list -work work]
set VLOG  [list -sv -work work -suppress 2244]

vlog_file $VLOGP apf/common.v
vlog_file $VLOG  core/rtl/sdram.sv
vlog_file $VLOG  core/rtl/mister_top/ss_sdram_arbiter.sv
vlog_file $VLOG  core/rtl/mister_top/psram.sv
vlog_file $VLOG  core/rtl/mister_top/psram_arbiter.sv
vlog_file $VLOG  core/rtl/mister_top/ss_psram_arbiter.sv
vlog_file $VLOG  core/tb/psram_chip_model.sv
vlog_file $VLOG  core/save_state_controller.sv
# Real engine under test (needs rom_addr as output reg — fixed for vlog).
vlog_file $VLOG  core/rtl/savestates_regs.sv
vlog_file $VLOG  core/rtl/savestates_sa1.sv
vlog_file $VLOG  core/rtl/savestates_map.sv
vlog_file $VLOG  core/rtl/savestates.sv
# tb_ss_staging.sv provides sdram_chip_model (its own top stays unelaborated).
vlog_file $VLOG  core/tb/tb_ss_staging.sv
vlog_file $VLOG  core/tb/tb_ss_serve.sv

# +acc keeps hierarchy visible for the TB's spies (ssc.*, u_ss.*).
vsim -t 1ps -voptargs=+acc -L altera_mf -lib work work.tb_ss_serve

run -all
quit -code 0
