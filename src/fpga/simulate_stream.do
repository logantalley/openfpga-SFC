# simulate_stream.do — ModelSim run for the direct-stream savestate
# transport: save_state_stream.sv (SRAM ring) + the real savestates.sv
# engine on the ungated clock, at real APF bridge pacing (tb_ss_stream.sv).
#
# Headless:   vsim -c -do simulate_stream.do     (from src/fpga/)

set MODELSIM_ASE_ROOT "C:/intelFPGA/20.1/modelsim_ase"
set ASE_INI           "$MODELSIM_ASE_ROOT/modelsim.ini"

if { ![file isdirectory $MODELSIM_ASE_ROOT] } {
    puts "ERROR: ModelSim ASE root not found: $MODELSIM_ASE_ROOT"
    quit -code 1
}

if { [file exists work] } { vdel -lib work -all }
if { [file exists $ASE_INI] } {
    file copy -force $ASE_INI modelsim.ini
    file attributes modelsim.ini -readonly 0
}

vlib work
vmap work work

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
vlog_file $VLOG  core/save_state_stream.sv
vlog_file $VLOG  core/rtl/savestates_regs.sv
vlog_file $VLOG  core/rtl/savestates_sa1.sv
vlog_file $VLOG  core/rtl/savestates_map.sv
vlog_file $VLOG  core/rtl/savestates.sv
vlog_file $VLOG  core/tb/tb_ss_stream.sv

vsim -t 1ps -voptargs=+acc -lib work work.tb_ss_stream

run -all
quit -code 0
