# simulate_staging.do — minimal, fast ModelSim run for the savestate
# LOAD staging path only (no SNES core).  Self-checking; prints a
# PASS/FAIL banner.
#
# Headless:   vsim -c -do simulate_staging.do     (from src/fpga/)
# GUI:        vsim -do simulate_staging.do
#
# Compiles only: synch_3 (apf/common.v), sdram.sv, ss_sdram_arbiter.sv,
# save_state_controller.sv, and the testbench.  dcfifo_mixed_widths and
# altddio_out come from the precompiled altera_mf library.

set MODELSIM_ASE_ROOT "C:/intelFPGA/20.1/modelsim_ase"
set ALTERA_MF_LIB     "$MODELSIM_ASE_ROOT/altera/verilog/altera_mf"
set ASE_INI           "$MODELSIM_ASE_ROOT/modelsim.ini"

if { ![file isdirectory $MODELSIM_ASE_ROOT] } {
    puts "ERROR: ModelSim ASE root not found: $MODELSIM_ASE_ROOT"
    quit -code 1
}
if { ![file isdirectory $ALTERA_MF_LIB] } {
    puts "ERROR: altera_mf library not found at: $ALTERA_MF_LIB"
    puts "  Compile it via Quartus > Tools > Launch Simulation Library Compiler"
    quit -code 1
}

# Fresh work library.  Copy the ASE master ini and STRIP THE READ-ONLY
# ATTRIBUTE — the master is read-only and `file copy` preserves that,
# which made every previous vmap fail with EACCES.
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
vlog_file $VLOG  core/tb/tb_ss_staging.sv

# +acc keeps hierarchy visible for the TB's spies (ssc.*, chip.*).
vsim -t 1ps -voptargs=+acc -L altera_mf -lib work work.tb_ss_staging

run -all
quit -code 0
